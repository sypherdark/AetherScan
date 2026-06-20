"""
Cascading PID flight controller — position outer loop, quaternion attitude inner loop.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional, Tuple

import numpy as np

from core.math3d import quat_from_euler, quat_normalize, quat_error, quat_to_euler, quat_to_rotation_matrix
from core.navigation import TrajectorySample
from core.physics import QuadcopterDynamics, QuadcopterParams, RigidBodyState

if TYPE_CHECKING:
    from core.collision import MeshCollisionSolver


@dataclass
class PIDGains:
    kp: float
    ki: float
    kd: float
    i_limit: float = 2.0
    out_limit: float = 50.0


@dataclass
class FlightGains:
    # XY position loop — snappier response without overshoot
    # Z position PID is intentionally disabled (kp=kd=ki=0) because the nav's
    # _altitude_velocity already provides proportional altitude correction via
    # err*1.15 gain.  Using a cascaded pos→vel PID for Z caused pos_kd derivative
    # spikes (-kd*vz per substep at 500 Hz) that flip-flopped vel_cmd sign each
    # substep, triggering large vel_kd kicks and sustained ceiling overshoot.
    pos_kp: Tuple[float, float, float] = (2.2, 2.2, 0.0)
    pos_ki: Tuple[float, float, float] = (0.04, 0.04, 0.0)
    pos_kd: Tuple[float, float, float] = (0.65, 0.65, 0.0)
    vel_kp: Tuple[float, float, float] = (2.8, 2.8, 2.5)
    # Horizontal: 2.5 m/s indoor cruise.  Vertical: capped at 0.42 m/s —
    # matches the nav's _altitude_velocity max so momentum never exceeds what
    # the nav intended; prevents overshoot even if wind adds a burst.
    max_vel: Tuple[float, float, float] = (2.5, 2.5, 0.42)
    # Attitude PID — roll/pitch gains tuned so motor saturation never occurs
    # during normal ±27° flight (max_tilt_rad).  With att_kp=7.5 the attitude
    # torque contribution per motor exceeds t0=3.56 N at q_err>0.26 rad,
    # saturating two diagonal motors and starving yaw authority.  At att_kp=3.5
    # the per-motor torque contribution (1.22 N) stays well below t0 even at
    # max tilt, so alpha=1.0 and all three torque axes get full authority.
    att_kp: Tuple[float, float, float] = (3.5, 3.5, 4.5)
    att_kd: Tuple[float, float, float] = (0.5, 0.5, 0.65)
    max_tilt_accel: float = 5.5
    max_accel: float = 6.5


class PIDAxis:
    def __init__(self, gains: PIDGains):
        self.g = gains
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0
        self.integral = float(np.clip(self.integral + error * dt, -self.g.i_limit, self.g.i_limit))
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        output = self.g.kp * error + self.g.ki * self.integral + self.g.kd * derivative
        return float(np.clip(output, -self.g.out_limit, self.g.out_limit))


class CascadingFlightController:
    """Outer position/velocity loop + inner quaternion attitude loop."""

    def __init__(
        self,
        params: QuadcopterParams | None = None,
        gains: FlightGains | None = None,
    ):
        self.params = params or QuadcopterParams()
        self.gains = gains or FlightGains()
        self.dynamics = QuadcopterDynamics(self.params)

        def mk(kp, ki, kd, lim=50.0):
            return PIDAxis(PIDGains(kp, ki, kd, out_limit=lim))

        self._pid_px = mk(self.gains.pos_kp[0], self.gains.pos_ki[0], self.gains.pos_kd[0])
        self._pid_py = mk(self.gains.pos_kp[1], self.gains.pos_ki[1], self.gains.pos_kd[1])
        self._pid_pz = mk(self.gains.pos_kp[2], self.gains.pos_ki[2], self.gains.pos_kd[2])
        self._pid_vx = mk(self.gains.vel_kp[0], 0.0, 0.4)
        self._pid_vy = mk(self.gains.vel_kp[1], 0.0, 0.4)
        # Smaller kd for vertical velocity: the nav switches vel_cmd sign
        # sharply when crossing the altitude target, so a large derivative
        # gain amplifies that transition into ±6.5 m/s² spikes each substep.
        self._pid_vz = mk(self.gains.vel_kp[2], 0.0, 0.15)
        self._pid_qx = mk(self.gains.att_kp[0], 0.0, self.gains.att_kd[0], 8.0)
        self._pid_qy = mk(self.gains.att_kp[1], 0.0, self.gains.att_kd[1], 8.0)
        # Yaw torque is deliberately capped low (1.8 Nm).  With kappa=0.05 the motor
        # allocation needs ±5 N of per-motor differential thrust per 1 Nm of yaw, so a
        # 5 Nm yaw command demands ±25 N — far beyond the ~3.5 N per-motor hover budget.
        # The thrust-priority allocator then scales ALL torques (roll, pitch, yaw) down
        # together to stay within motor limits, starving attitude AND thrust authority,
        # so the drone tips over and drops altitude during an aggressive spin.  Capping
        # yaw torque keeps roll/pitch/thrust authority intact; the only cost is a
        # slightly slower yaw slew, which the gentle spin/scan rates already assume.
        self._pid_qz = mk(self.gains.att_kp[2], 0.0, self.gains.att_kd[2], 1.8)

        self.autonomous: bool = True
        self.manual_velocity: np.ndarray = np.zeros(3)
        self.manual_yaw_rate: float = 0.0
        self.target_yaw: float = 0.0

    @property
    def state(self) -> RigidBodyState:
        return self.dynamics.state

    def reset(self, position: np.ndarray, yaw: float = 0.0) -> None:
        self.dynamics.reset(position, yaw)
        self.target_yaw = yaw
        for pid in (
            self._pid_px, self._pid_py, self._pid_pz,
            self._pid_vx, self._pid_vy, self._pid_vz,
            self._pid_qx, self._pid_qy, self._pid_qz,
        ):
            pid.reset()

    def set_floor_height_fn(self, fn) -> None:
        self.dynamics.set_floor_height_fn(fn)

    def step(
        self,
        trajectory: TrajectorySample | None,
        dt: float,
        avoidance_accel: Optional[np.ndarray] = None,
        collision_solver: Optional[MeshCollisionSolver] = None,
    ) -> np.ndarray:
        s = self.dynamics.state

        if self.autonomous and trajectory is not None:
            pos_des = trajectory.position
            vel_des = trajectory.velocity
            self.target_yaw = trajectory.yaw
        else:
            pos_des = s.position.copy()
            vel_des = self.manual_velocity.copy()
            self.target_yaw += self.manual_yaw_rate * dt

        pos_err = pos_des - s.position
        vel_des_pid = np.array([
            self._pid_px.update(pos_err[0], dt),
            self._pid_py.update(pos_err[1], dt),
            self._pid_pz.update(pos_err[2], dt),
        ])
        vel_cmd = vel_des + vel_des_pid
        vel_cmd = np.clip(
            vel_cmd,
            [-self.gains.max_vel[0], -self.gains.max_vel[1], -self.gains.max_vel[2]],
            list(self.gains.max_vel),
        )

        vel_err = vel_cmd - s.velocity
        ax = float(np.clip(self._pid_vx.update(vel_err[0], dt), -self.gains.max_accel, self.gains.max_accel))
        ay = float(np.clip(self._pid_vy.update(vel_err[1], dt), -self.gains.max_accel, self.gains.max_accel))
        az = float(np.clip(self._pid_vz.update(vel_err[2], dt), -self.gains.max_accel, self.gains.max_accel))

        if avoidance_accel is not None:
            rep = np.asarray(avoidance_accel, dtype=np.float64)
            ax += float(rep[0])
            ay += float(rep[1])
            az += float(rep[2])
            ax = float(np.clip(ax, -self.gains.max_accel, self.gains.max_accel))
            ay = float(np.clip(ay, -self.gains.max_accel, self.gains.max_accel))
            az = float(np.clip(az, -self.gains.max_accel, self.gains.max_accel))

        # Map the DESIRED WORLD horizontal acceleration (ax, ay) to a body tilt.
        # Two things must be right or the drone flies perpendicular to its target:
        #   1. ax,ay are world-frame but tilt is body-frame → rotate by yaw first.
        #   2. With body-Z = [cosφ·sinθ, -sinφ, cosφ·cosθ] (ZYX euler), forward
        #      acceleration comes from PITCH (ax_body ∝ +θ) and lateral from ROLL
        #      (ay_body ∝ -φ).  The old code used θ←ay and φ←-ax (axes swapped, no
        #      yaw rotation), so a "go north" command drove the drone east, etc.
        cy, sy = float(np.cos(self.target_yaw)), float(np.sin(self.target_yaw))
        ax_b = ax * cy + ay * sy   # forward (body +X)
        ay_b = -ax * sy + ay * cy  # left    (body +Y)
        mt = self.params.max_tilt_rad
        theta_des = float(np.clip(ax_b / self.gains.max_tilt_accel, -mt, mt))   # pitch ← forward accel
        phi_des = float(np.clip(-ay_b / self.gains.max_tilt_accel, -mt, mt))    # roll  ← lateral accel
        q_des = quat_from_euler(phi_des, theta_des, self.target_yaw)

        T_hover = self.params.hover_thrust
        # Tilt compensation: when the drone is tilted the vertical component of
        # thrust is T*cos(tilt).  Boost T_cmd so that the vertical component
        # still equals m*(g+az) regardless of attitude.  Uses the actual rotation
        # matrix column R[:,2] (body Z in world frame) — its Z-component R[2,2]
        # equals cos(tilt) exactly.  Clamped to 0.5 so a near-horizontal drone
        # doesn't produce an unbounded boost.
        R = quat_to_rotation_matrix(s.quaternion)
        cos_tilt = float(max(R[2, 2], 0.5))
        T_cmd = self.params.mass * (self.params.gravity + az) / cos_tilt
        T_cmd = float(np.clip(T_cmd, 0.35 * T_hover, 1.85 * T_hover))

        q_err_vec = quat_error(q_des, s.quaternion)
        tau_x = self._pid_qx.update(q_err_vec[0], dt)
        tau_y = self._pid_qy.update(q_err_vec[1], dt)
        tau_z = self._pid_qz.update(q_err_vec[2], dt)
        body_torque = np.array([tau_x, tau_y, tau_z], dtype=np.float64)

        motor_thrusts = self.dynamics.allocate_motors(T_cmd, body_torque)
        self.dynamics.step_rk4(motor_thrusts, dt, collision_solver=collision_solver)
        return motor_thrusts

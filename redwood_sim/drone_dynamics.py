"""
6-DoF quadcopter rigid-body dynamics and cascading PID flight controller.

State (world frame unless noted):
  position p = [x, y, z]
  velocity v = [vx, vy, vz]
  euler angles [phi, theta, psi]  (roll, pitch, yaw)
  body angular rates omega_b = [p, q, r]

Thrust T acts along body +Z_b. Motor thrusts map through a standard X-configuration mixer.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple

import numpy as np


# ---------------------------------------------------------------------------
# Rotation utilities (ZYX yaw-pitch-roll intrinsic: R = Rz(psi) Ry(theta) Rx(phi))
# ---------------------------------------------------------------------------


def rotation_matrix_from_euler(phi: float, theta: float, psi: float) -> np.ndarray:
    """Return R_wb (body -> world) for roll phi, pitch theta, yaw psi."""
    cph, sph = np.cos(phi), np.sin(phi)
    cth, sth = np.cos(theta), np.sin(theta)
    cps, sps = np.cos(psi), np.sin(psi)

    Rx = np.array([[1, 0, 0], [0, cph, -sph], [0, sph, cph]])
    Ry = np.array([[cth, 0, sth], [0, 1, 0], [-sth, 0, cth]])
    Rz = np.array([[cps, -sps, 0], [sps, cps, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def euler_rates_from_body_rates(
    phi: float, theta: float, omega_b: np.ndarray
) -> np.ndarray:
    """
    Map body angular velocity to Euler angle derivatives.
    [phi_dot, theta_dot, psi_dot]^T = W(phi,theta) * [p,q,r]^T
    """
    p, q, r = omega_b
    sph, cph = np.sin(phi), np.cos(phi)
    tth, cth = np.tan(theta), np.cos(theta)
    if abs(cth) < 1e-3:
        cth = 1e-3 * np.sign(cth)
    W = np.array(
        [
            [1, sph * tth, cph * tth],
            [0, cph, -sph],
            [0, sph / cth, cph / cth],
        ]
    )
    return W @ omega_b


# ---------------------------------------------------------------------------
# PID
# ---------------------------------------------------------------------------


@dataclass
class PIDGains:
    kp: float
    ki: float
    kd: float
    i_limit: float = 2.0
    out_limit: float = 1e9


class PIDController:
    """Single-axis PID with anti-windup."""

    def __init__(self, gains: PIDGains):
        self.g = gains
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0
        self.integral = np.clip(
            self.integral + error * dt, -self.g.i_limit, self.g.i_limit
        )
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        out = self.g.kp * error + self.g.ki * self.integral + self.g.kd * derivative
        return float(np.clip(out, -self.g.out_limit, self.g.out_limit))


# ---------------------------------------------------------------------------
# Vehicle parameters
# ---------------------------------------------------------------------------


@dataclass
class QuadcopterParams:
    mass: float = 1.45  # kg
    gravity: float = 9.80665
    # Diagonal inertia tensor (kg·m²) for ~300mm frame
    Ixx: float = 0.012
    Iyy: float = 0.012
    Izz: float = 0.022
    arm_length: float = 0.18  # motor to center (m)
    k_thrust: float = 1.0e-5  # thrust coefficient (N/(rad/s)²) — used in mixer scaling
    max_motor_speed: float = 900.0  # rad/s equivalent command
    min_motor_speed: float = 0.0
    linear_drag: float = 0.35  # N·s/m per axis (approximate)
    angular_drag: float = 0.04  # N·m·s/rad
    max_tilt_rad: float = 0.45  # safety cap on commanded roll/pitch

    @property
    def inertia(self) -> np.ndarray:
        return np.diag([self.Ixx, self.Iyy, self.Izz])

    @property
    def hover_thrust(self) -> float:
        return self.mass * self.gravity


# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------


@dataclass
class DroneState:
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    euler: np.ndarray = field(default_factory=lambda: np.zeros(3))  # phi, theta, psi
    omega_b: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def copy(self) -> "DroneState":
        return DroneState(
            position=self.position.copy(),
            velocity=self.velocity.copy(),
            euler=self.euler.copy(),
            omega_b=self.omega_b.copy(),
        )


# ---------------------------------------------------------------------------
# Dynamics
# ---------------------------------------------------------------------------


class QuadcopterDynamics:
    """
    Rigid-body quadcopter with thrust along body Z and linear/angular drag.
    """

    def __init__(self, params: QuadcopterParams | None = None):
        self.p = params or QuadcopterParams()
        self.state = DroneState()

    def reset(self, position: np.ndarray | None = None, yaw: float = 0.0) -> None:
        self.state = DroneState()
        if position is not None:
            self.state.position = np.asarray(position, dtype=np.float64)
        self.state.euler[2] = yaw

    def body_thrust_vector(self, total_thrust: float) -> np.ndarray:
        """Thrust along body +Z, magnitude total_thrust (N)."""
        return np.array([0.0, 0.0, total_thrust])

    def motor_thrusts_to_wrench(
        self, motor_thrusts: np.ndarray
    ) -> Tuple[float, np.ndarray]:
        """
        Convert four motor thrusts [T_fl, T_fr, T_rr, T_rl] to total thrust and body torque.

        Layout (X configuration, looking down):
          FL (1)     FR (2)
              X
          RL (4)     RR (3)
        """
        t = np.asarray(motor_thrusts, dtype=np.float64)
        T = float(np.sum(t))
        L = self.p.arm_length
        # Roll moment: (T_fr + T_rr) - (T_fl + T_rl) with lever arm
        tau_x = L * (t[1] + t[2] - t[0] - t[3])
        # Pitch moment: (T_fl + T_fr) - (T_rl + T_rr)
        tau_y = L * (t[0] + t[1] - t[2] - t[3])
        # Yaw moment: reaction torque kappa*(w_fl - w_fr + w_rr - w_rl); approximate from thrust diff
        kappa = 0.02
        tau_z = kappa * (t[0] - t[1] + t[2] - t[3])
        return T, np.array([tau_x, tau_y, tau_z])

    def allocate_motors(self, total_thrust: float, body_torque: np.ndarray) -> np.ndarray:
        """
        Pseudo-inverse mixer: [T, tau_x, tau_y, tau_z] -> four motor thrusts.
        """
        L = self.p.arm_length
        kappa = 0.02
        # Allocation matrix A: thrust = A @ [T, tx, ty, tz]
        A = np.array(
            [
                [0.25, 0.25 / L, 0.25 / L, 0.25 / kappa],
                [0.25, -0.25 / L, 0.25 / L, -0.25 / kappa],
                [0.25, -0.25 / L, -0.25 / L, 0.25 / kappa],
                [0.25, 0.25 / L, -0.25 / L, -0.25 / kappa],
            ]
        )
        wrench = np.array([total_thrust, body_torque[0], body_torque[1], body_torque[2]])
        thrusts = A @ wrench
        thrusts = np.clip(thrusts, 0.0, self.p.hover_thrust * 0.8)
        return thrusts

    def derivatives(
        self,
        state: DroneState,
        total_thrust: float,
        body_torque: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Time derivatives (p_dot, v_dot, euler_dot, omega_dot).
        """
        phi, theta, psi = state.euler
        R = rotation_matrix_from_euler(phi, theta, psi)

        # Gravity in world frame; thrust in world frame
        F_gravity = np.array([0.0, 0.0, -self.p.mass * self.p.gravity])
        F_thrust_w = R @ self.body_thrust_vector(total_thrust)

        # Linear aerodynamic drag: F_d = -c * v (world frame, simplified)
        F_drag = -self.p.linear_drag * state.velocity

        a = (F_gravity + F_thrust_w + F_drag) / self.p.mass
        p_dot = state.velocity
        v_dot = a

        euler_dot = euler_rates_from_body_rates(phi, theta, state.omega_b)

        # Euler's rotation equation: I * omega_dot + omega x (I*omega) = tau
        I = self.p.inertia
        omega = state.omega_b
        tau_drag = -self.p.angular_drag * omega
        omega_dot = np.linalg.solve(
            I, body_torque + tau_drag - np.cross(omega, I @ omega)
        )

        return p_dot, v_dot, euler_dot, omega_dot

    def step(
        self,
        motor_thrusts: np.ndarray,
        dt: float,
        integrator: str = "rk4",
    ) -> DroneState:
        """
        Advance state by dt using motor thrust commands (N per motor).
        """
        T, tau = self.motor_thrusts_to_wrench(motor_thrusts)

        if integrator == "euler":
            return self._step_euler(T, tau, dt)
        return self._step_rk4(T, tau, dt)

    def _pack_deriv(self, s: DroneState, T: float, tau: np.ndarray) -> np.ndarray:
        pd, vd, ed, od = self.derivatives(s, T, tau)
        return np.concatenate([pd, vd, ed, od])

    def _unpack_state(self, s: DroneState, vec: np.ndarray) -> DroneState:
        out = s.copy()
        out.position = vec[0:3]
        out.velocity = vec[3:6]
        out.euler = vec[6:9]
        out.omega_b = vec[9:12]
        return out

    def _state_vector(self, s: DroneState) -> np.ndarray:
        return np.concatenate([s.position, s.velocity, s.euler, s.omega_b])

    def _step_rk4(self, T: float, tau: np.ndarray, dt: float) -> DroneState:
        s0 = self.state
        y0 = self._state_vector(s0)

        k1 = self._pack_deriv(s0, T, tau)

        s1 = self._unpack_state(s0, y0 + 0.5 * dt * k1)
        k2 = self._pack_deriv(s1, T, tau)

        s2 = self._unpack_state(s0, y0 + 0.5 * dt * k2)
        k3 = self._pack_deriv(s2, T, tau)

        s3 = self._unpack_state(s0, y0 + dt * k3)
        k4 = self._pack_deriv(s3, T, tau)

        y1 = y0 + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        self.state = self._unpack_state(s0, y1)
        return self.state

    def _step_euler(self, T: float, tau: np.ndarray, dt: float) -> DroneState:
        pd, vd, ed, od = self.derivatives(self.state, T, tau)
        self.state.position += pd * dt
        self.state.velocity += vd * dt
        self.state.euler += ed * dt
        self.state.omega_b += od * dt
        return self.state


# ---------------------------------------------------------------------------
# Cascading flight controller
# ---------------------------------------------------------------------------


@dataclass
class FlightControllerGains:
    # Position loop -> desired velocity / tilt
    pos_kp: Tuple[float, float, float] = (1.8, 1.8, 2.2)
    pos_ki: Tuple[float, float, float] = (0.05, 0.05, 0.15)
    pos_kd: Tuple[float, float, float] = (0.9, 0.9, 1.0)
    max_vel: Tuple[float, float, float] = (1.5, 1.5, 1.0)

    # Attitude loop -> body torques
    att_kp: Tuple[float, float, float] = (5.5, 5.5, 3.0)
    att_ki: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    att_kd: Tuple[float, float, float] = (0.8, 0.8, 0.4)

    # Yaw
    yaw_kp: float = 2.5
    yaw_kd: float = 0.3


class CascadingFlightController:
    """
    Outer position loop -> desired roll/pitch/thrust.
    Inner attitude loop -> body torques -> motor allocation.
    """

    def __init__(
        self,
        params: QuadcopterParams | None = None,
        gains: FlightControllerGains | None = None,
    ):
        self.params = params or QuadcopterParams()
        self.gains = gains or FlightControllerGains()
        self.dynamics = QuadcopterDynamics(self.params)

        self._pid_px = self._make_pid(self.gains.pos_kp[0], self.gains.pos_ki[0], self.gains.pos_kd[0])
        self._pid_py = self._make_pid(self.gains.pos_kp[1], self.gains.pos_ki[1], self.gains.pos_kd[1])
        self._pid_pz = self._make_pid(self.gains.pos_kp[2], self.gains.pos_ki[2], self.gains.pos_kd[2])

        self._pid_phi = self._make_pid(self.gains.att_kp[0], self.gains.att_ki[0], self.gains.att_kd[0], 3.0)
        self._pid_theta = self._make_pid(self.gains.att_kp[1], self.gains.att_ki[1], self.gains.att_kd[1], 3.0)
        self._pid_psi = self._make_pid(self.gains.yaw_kp, 0.0, self.gains.yaw_kd, 2.0)

        self.target_position: np.ndarray | None = None
        self.target_velocity: np.ndarray = np.zeros(3)
        self.target_yaw: float = 0.0
        self.mode: str = "velocity"  # "position" | "velocity"

    @staticmethod
    def _make_pid(kp: float, ki: float, kd: float, out_limit: float = 1.5) -> PIDController:
        return PIDController(PIDGains(kp, ki, kd, out_limit=out_limit))

    @property
    def state(self) -> DroneState:
        return self.dynamics.state

    def reset(self, position: np.ndarray, yaw: float = 0.0) -> None:
        self.dynamics.reset(position, yaw)
        for pid in (
            self._pid_px, self._pid_py, self._pid_pz,
            self._pid_phi, self._pid_theta, self._pid_psi,
        ):
            pid.reset()
        self.target_yaw = yaw

    def set_position_target(self, target: np.ndarray) -> None:
        self.target_position = np.asarray(target, dtype=np.float64)
        self.mode = "position"

    def set_velocity_target(self, vel: np.ndarray) -> None:
        self.target_velocity = np.asarray(vel, dtype=np.float64)
        self.mode = "velocity"

    def step_control(self, dt: float) -> np.ndarray:
        """
        Compute motor thrusts for one control tick, then integrate physics.
        Returns motor thrusts [N] used.
        """
        s = self.dynamics.state
        phi, theta, psi = s.euler

        # --- Outer loop: position / velocity ---
        if self.mode == "position" and self.target_position is not None:
            err_p = self.target_position - s.position
            v_des = np.array([
                self._pid_px.update(err_p[0], dt),
                self._pid_py.update(err_p[1], dt),
                self._pid_pz.update(err_p[2], dt),
            ])
        else:
            v_des = self.target_velocity.copy()

        v_des = np.clip(
            v_des,
            [-self.gains.max_vel[0], -self.gains.max_vel[1], -self.gains.max_vel[2]],
            self.gains.max_vel,
        )

        # Map desired horizontal velocity to roll/pitch commands (small-angle)
        # World velocity error drives tilt: pitch for forward (x), roll for lateral (y)
        v_err = v_des - s.velocity
        desired_pitch = np.clip(-0.25 * v_err[0], -self.params.max_tilt_rad, self.params.max_tilt_rad)
        desired_roll = np.clip(0.25 * v_err[1], -self.params.max_tilt_rad, self.params.max_tilt_rad)
        desired_yaw = self.target_yaw

        # Thrust from altitude / vertical velocity
        T_hover = self.params.hover_thrust
        T_z = T_hover + self.params.mass * (self._pid_pz.update(v_err[2], dt) if self.mode == "position" else 4.0 * v_err[2])
        T_z = float(np.clip(T_z, 0.2 * T_hover, 2.2 * T_hover))

        # --- Inner loop: attitude ---
        tau_x = self._pid_phi.update(desired_roll - phi, dt)
        tau_y = self._pid_theta.update(desired_pitch - theta, dt)
        tau_z = self._pid_psi.update(self._wrap_angle(desired_yaw - psi), dt)
        body_torque = np.array([tau_x, tau_y, tau_z])

        motor_thrusts = self.dynamics.allocate_motors(T_z, body_torque)
        self.dynamics.step(motor_thrusts, dt, integrator="rk4")
        return motor_thrusts

    @staticmethod
    def _wrap_angle(a: float) -> float:
        return float((a + np.pi) % (2 * np.pi) - np.pi)

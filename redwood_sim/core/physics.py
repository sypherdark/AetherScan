"""
6-DoF quadcopter rigid-body dynamics with RK4 integration.

Collision resolution runs after every RK4 micro-step (physics_dt) when a
MeshCollisionSolver is provided — prevents tunneling between control steps.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Callable, Optional, Tuple

import numpy as np

from core.math3d import (
    omega_quat_derivative,
    quat_normalize,
    quat_to_euler,
    quat_to_rotation_matrix,
)

if TYPE_CHECKING:
    from core.collision import MeshCollisionSolver


@dataclass
class QuadcopterParams:
    mass: float = 1.45
    gravity: float = 9.80665
    Ixx: float = 0.014
    Iyy: float = 0.014
    Izz: float = 0.026
    arm_length: float = 0.18
    # Realistic aerodynamic drag (DJI Phantom-class)
    linear_drag: float = 0.55
    angular_drag: float = 0.075
    ground_effect_height: float = 0.6
    ground_effect_gain: float = 0.28
    max_tilt_rad: float = 0.48
    # Motor reaction-torque coefficient (Nm per N of thrust).
    # Larger kappa → more yaw torque per unit differential thrust.
    # kappa=0.018 (original) required ±69 N differential thrust per motor for
    # 5 Nm of yaw torque, starving the thrust-priority allocator.  kappa=0.05
    # reduces this to ±25 N, giving adequate yaw authority (alpha≈0.14 at hover).
    kappa: float = 0.05
    # Low-amplitude wind turbulence (Dryden model approximation).
    # Reduced from 0.12 to 0.04 — high values continuously perturb hover states
    # (SCAN_HOLD, STUCK_SPIN) and cause roll to accumulate beyond what the
    # attitude PID can correct, preventing stable spinning and altitude hold.
    wind_turbulence_std: float = 0.04

    @property
    def inertia(self) -> np.ndarray:
        return np.diag([self.Ixx, self.Iyy, self.Izz]).astype(np.float64)

    @property
    def hover_thrust(self) -> float:
        return self.mass * self.gravity


@dataclass
class RigidBodyState:
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    quaternion: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))
    omega_b: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def copy(self) -> RigidBodyState:
        return RigidBodyState(
            position=self.position.copy(),
            velocity=self.velocity.copy(),
            quaternion=self.quaternion.copy(),
            omega_b=self.omega_b.copy(),
        )

    @property
    def euler(self) -> np.ndarray:
        return quat_to_euler(self.quaternion)

    @property
    def rotation_wb(self) -> np.ndarray:
        return quat_to_rotation_matrix(self.quaternion)


class QuadcopterDynamics:
    """Continuous rigid-body model integrated with fourth-order Runge-Kutta."""

    def __init__(self, params: QuadcopterParams | None = None):
        self.p = params or QuadcopterParams()
        self.state = RigidBodyState()
        self._floor_height_fn: Optional[Callable[[np.ndarray], float]] = None

    def set_floor_height_fn(self, fn: Callable[[np.ndarray], float]) -> None:
        self._floor_height_fn = fn

    def reset(self, position: np.ndarray, yaw: float = 0.0) -> None:
        self.state = RigidBodyState()
        self.state.position = np.asarray(position, dtype=np.float64)
        from core.math3d import quat_from_euler

        self.state.quaternion = quat_from_euler(0.0, 0.0, yaw)

    def ground_effect_multiplier(self, position: np.ndarray) -> float:
        floor_z = 0.0
        if self._floor_height_fn is not None:
            floor_z = float(self._floor_height_fn(position))
        height_agl = float(position[2] - floor_z)
        if height_agl >= self.p.ground_effect_height:
            return 1.0
        ratio = 1.0 - height_agl / max(self.p.ground_effect_height, 1e-3)
        return 1.0 + self.p.ground_effect_gain * ratio * ratio

    def motor_thrusts_to_wrench(self, motor_thrusts: np.ndarray) -> Tuple[float, np.ndarray]:
        t = np.asarray(motor_thrusts, dtype=np.float64)
        thrust = float(np.sum(t))
        L = self.p.arm_length
        kappa = self.p.kappa
        tau_x = L * (t[1] + t[2] - t[0] - t[3])
        tau_y = L * (t[0] + t[1] - t[2] - t[3])
        tau_z = kappa * (t[0] - t[1] + t[2] - t[3])
        return thrust, np.array([tau_x, tau_y, tau_z], dtype=np.float64)

    def allocate_motors(self, total_thrust: float, body_torque: np.ndarray) -> np.ndarray:
        """
        Thrust-priority motor allocation.

        Guarantees that sum(motor_thrusts) == total_thrust by scaling attitude
        torques down (alpha ∈ [0,1]) until every motor stays within [0, T_max].
        Without this, large attitude torques saturate two diagonal motors at T_max
        each (total = 2*T_max) regardless of the altitude-PID's T_cmd, causing
        runaway climb even when the altitude loop commands descent.
        """
        L = self.p.arm_length
        kappa = self.p.kappa
        # SIGN-CORRECTED allocation matrix.
        # The tau_x column was previously [+1,-1,-1,+1]/L which, combined with
        # motor_thrusts_to_wrench (tau_x = L*(t1+t2-t0-t3)), recovers -tau_x_in.
        # Negating the tau_x column (now [-1,+1,+1,-1]/L) restores the correct sign:
        # tau_x_recovered = L*(t1+t2-t0-t3) = tau_x_in.
        A = np.array(
            [
                [0.25, -0.25 / L, 0.25 / L, 0.25 / kappa],
                [0.25,  0.25 / L, 0.25 / L, -0.25 / kappa],
                [0.25,  0.25 / L, -0.25 / L, 0.25 / kappa],
                [0.25, -0.25 / L, -0.25 / L, -0.25 / kappa],
            ],
            dtype=np.float64,
        )
        T_max = self.p.hover_thrust * 0.85
        wrench = np.array([total_thrust, body_torque[0], body_torque[1], body_torque[2]])
        thrusts = A @ wrench

        # Fast path: no saturation
        if np.all(thrusts >= 0.0) and np.all(thrusts <= T_max):
            return thrusts

        # Find largest alpha in [0,1] that keeps every motor in [0, T_max].
        # t0 = T_cmd/4 is the per-motor hover component (guaranteed reachable).
        # delta[i] = thrusts[i] - t0 is the attitude contribution for motor i.
        # sum(delta) == 0 (torque rows of A sum to zero), so
        # sum(t0 + alpha*delta) == T_cmd for any alpha.  Altitude is preserved.
        t0 = total_thrust / 4.0
        delta = thrusts - t0
        alpha = 1.0
        for i in range(4):
            if delta[i] > 1e-9 and t0 + delta[i] > T_max:
                alpha = min(alpha, (T_max - t0) / delta[i])
            elif delta[i] < -1e-9 and t0 + delta[i] < 0.0:
                alpha = min(alpha, (-t0) / delta[i])
        alpha = max(0.0, alpha)
        thrusts = t0 + alpha * delta
        return np.clip(thrusts, 0.0, T_max)  # floating-point safety

    def derivatives(
        self,
        state: RigidBodyState,
        total_thrust: float,
        body_torque: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        R = quat_to_rotation_matrix(state.quaternion)
        ge = self.ground_effect_multiplier(state.position)
        thrust_b = np.array([0.0, 0.0, total_thrust * ge], dtype=np.float64)

        F_gravity = np.array([0.0, 0.0, -self.p.mass * self.p.gravity])
        F_thrust_w = R @ thrust_b
        # Velocity-squared drag for realism at higher speeds (Cd * v^2 blended with linear)
        spd = float(np.linalg.norm(state.velocity))
        drag_coeff = self.p.linear_drag + 0.08 * spd
        F_drag = -drag_coeff * state.velocity
        # Low-frequency wind turbulence
        wind = np.random.normal(0.0, self.p.wind_turbulence_std, 3)
        wind[2] *= 0.3  # vertical turbulence is weaker indoors

        acceleration = (F_gravity + F_thrust_w + F_drag + wind) / self.p.mass
        p_dot = state.velocity.copy()
        v_dot = acceleration

        q = quat_normalize(state.quaternion)
        q_dot = omega_quat_derivative(q, state.omega_b)

        I = self.p.inertia
        omega = state.omega_b
        tau_drag = -self.p.angular_drag * omega
        omega_dot = np.linalg.solve(I, body_torque + tau_drag - np.cross(omega, I @ omega))

        return p_dot, v_dot, q_dot, omega_dot

    def _pack_deriv(
        self, state: RigidBodyState, total_thrust: float, body_torque: np.ndarray
    ) -> np.ndarray:
        pd, vd, qd, od = self.derivatives(state, total_thrust, body_torque)
        return np.concatenate([pd, vd, qd, od])

    def _state_vector(self, state: RigidBodyState) -> np.ndarray:
        return np.concatenate([state.position, state.velocity, state.quaternion, state.omega_b])

    def _unpack_state(self, base: RigidBodyState, vec: np.ndarray) -> RigidBodyState:
        out = base.copy()
        out.position = vec[0:3]
        out.velocity = vec[3:6]
        out.quaternion = quat_normalize(vec[6:10])
        out.omega_b = vec[10:13]
        return out

    def _apply_collision(
        self,
        prev_position: np.ndarray,
        collision_solver: MeshCollisionSolver,
    ) -> None:
        pos, vel = collision_solver.resolve(
            prev_position,
            self.state.position,
            self.state.velocity,
            self.state.quaternion,
        )
        self.state.position[:] = pos
        self.state.velocity[:] = vel

    def step_rk4(
        self,
        motor_thrusts: np.ndarray,
        dt: float,
        collision_solver: Optional[MeshCollisionSolver] = None,
    ) -> RigidBodyState:
        """
        Integrate one physics micro-step. When collision_solver is set, resolve
        contacts after the RK4 update (every physics_dt, e.g. 0.002 s).
        """
        total_thrust, body_torque = self.motor_thrusts_to_wrench(motor_thrusts)
        s0 = self.state
        prev_pos = s0.position.copy()
        y0 = self._state_vector(s0)

        k1 = self._pack_deriv(s0, total_thrust, body_torque)
        s1 = self._unpack_state(s0, y0 + 0.5 * dt * k1)
        k2 = self._pack_deriv(s1, total_thrust, body_torque)
        s2 = self._unpack_state(s0, y0 + 0.5 * dt * k2)
        k3 = self._pack_deriv(s2, total_thrust, body_torque)
        s3 = self._unpack_state(s0, y0 + dt * k3)
        k4 = self._pack_deriv(s3, total_thrust, body_torque)

        y1 = y0 + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        if not np.all(np.isfinite(y1)):
            return s0

        self.state = self._unpack_state(s0, y1)
        omega_lim = 12.0
        self.state.omega_b = np.clip(self.state.omega_b, -omega_lim, omega_lim)

        if collision_solver is not None:
            self._apply_collision(prev_pos, collision_solver)

        return self.state

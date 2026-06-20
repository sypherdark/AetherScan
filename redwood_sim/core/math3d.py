"""3D rotation math — quaternions preferred over Euler for control."""

from __future__ import annotations

import numpy as np


def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / n


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def quat_from_euler(phi: float, theta: float, psi: float) -> np.ndarray:
    cph, sph = np.cos(phi * 0.5), np.sin(phi * 0.5)
    cth, sth = np.cos(theta * 0.5), np.sin(theta * 0.5)
    cps, sps = np.cos(psi * 0.5), np.sin(psi * 0.5)
    return quat_normalize(
        np.array(
            [
                cph * cth * cps + sph * sth * sps,
                sph * cth * cps - cph * sth * sps,
                cph * sth * cps + sph * cth * sps,
                cph * cth * sps - sph * sth * cps,
            ],
            dtype=np.float64,
        )
    )


def quat_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    q = quat_normalize(q)
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def quat_to_euler(q: np.ndarray) -> np.ndarray:
    q = quat_normalize(q)
    w, x, y, z = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    phi = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    theta = np.copysign(np.pi / 2, sinp) if abs(sinp) >= 1 else np.arcsin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    psi = np.arctan2(siny_cosp, cosy_cosp)
    return np.array([phi, theta, psi], dtype=np.float64)


def quat_error(q_des: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Rotation error as vector part of q_err = q_des * q^{-1}."""
    q_err = quat_multiply(q_des, quat_conjugate(quat_normalize(q)))
    if q_err[0] < 0.0:
        q_err = -q_err
    return 2.0 * q_err[1:4]


def omega_quat_derivative(q: np.ndarray, omega_b: np.ndarray) -> np.ndarray:
    wx, wy, wz = omega_b
    omega_q = np.array([0.0, wx, wy, wz], dtype=np.float64)
    return 0.5 * quat_multiply(q, omega_q)


def skew(v: np.ndarray) -> np.ndarray:
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=np.float64)

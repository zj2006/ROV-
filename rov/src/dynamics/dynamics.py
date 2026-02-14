from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass
class ROVParams:
    mass_u: float = 20.0
    mass_v: float = 20.0
    mass_w: float = 24.0
    inertia_r: float = 8.0
    du: float = 9.0
    dv: float = 9.0
    dw: float = 10.0
    dr: float = 3.5
    thruster_limit: float = 28.0


@dataclass
class ROVState:
    x: float
    y: float
    z: float
    yaw: float
    u: float
    v: float
    w: float
    r: float


def wrap_pi(a: float) -> float:
    return (a + np.pi) % (2 * np.pi) - np.pi


class ROV4DOF:
    def __init__(self, params: ROVParams | None = None) -> None:
        self.params = params or ROVParams()
        # Thruster map: 6 thrusters -> [Fx, Fy, Fz, Mz]
        # t1..t4 horizontal, t5..t6 vertical
        c = 0.7
        self.B = np.array([
            [1, 1, -1, -1, 0, 0],
            [1, -1, -1, 1, 0, 0],
            [0, 0, 0, 0, 1, 1],
            [c, -c, c, -c, 0.15, -0.15],
        ], dtype=float)

    def allocate(self, tau: np.ndarray) -> np.ndarray:
        f = np.linalg.pinv(self.B) @ tau
        lim = self.params.thruster_limit
        return np.clip(f, -lim, lim)

    def forces_from_thrusters(self, f: np.ndarray) -> np.ndarray:
        return self.B @ f

    def step(self, s: ROVState, f: np.ndarray, current_world: np.ndarray, dt: float) -> ROVState:
        tau = self.forces_from_thrusters(f)
        fx, fy, fz, mz = tau

        pu = self.params
        du = pu.du * s.u * abs(s.u)
        dv = pu.dv * s.v * abs(s.v)
        dw = pu.dw * s.w * abs(s.w)
        dr = pu.dr * s.r * abs(s.r)

        u_dot = (fx - du) / pu.mass_u
        v_dot = (fy - dv) / pu.mass_v
        w_dot = (fz - dw) / pu.mass_w
        r_dot = (mz - dr) / pu.inertia_r

        u = s.u + u_dot * dt
        v = s.v + v_dot * dt
        w = s.w + w_dot * dt
        r = s.r + r_dot * dt

        cy = np.cos(s.yaw)
        sy = np.sin(s.yaw)
        vel_world = np.array([cy * u - sy * v, sy * u + cy * v, w]) + current_world

        x = s.x + vel_world[0] * dt
        y = s.y + vel_world[1] * dt
        z = s.z + vel_world[2] * dt
        yaw = wrap_pi(s.yaw + r * dt)

        return ROVState(x=x, y=y, z=z, yaw=yaw, u=u, v=v, w=w, r=r)

    def pose(self, s: ROVState) -> np.ndarray:
        return np.array([s.x, s.y, s.z, s.yaw])

    def body_speed(self, s: ROVState) -> np.ndarray:
        return np.array([s.u, s.v, s.w, s.r])

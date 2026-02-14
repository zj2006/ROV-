from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np

from rov.src.dynamics.dynamics import ROV4DOF, ROVState, wrap_pi


@dataclass
class TrackConfig:
    kp_pos: float = 15.0
    kd_vel: float = 8.0
    kp_yaw: float = 8.0
    kd_r: float = 3.0
    lookahead: int = 3


def compute_tau(
    rov: ROV4DOF,
    state: ROVState,
    waypoints: List[np.ndarray],
    wp_idx: int,
    cfg: TrackConfig,
) -> tuple[np.ndarray, int, float]:
    if wp_idx >= len(waypoints):
        target = waypoints[-1]
    else:
        target = waypoints[min(wp_idx + cfg.lookahead, len(waypoints) - 1)]

    pos = np.array([state.x, state.y, state.z])
    e_world = target[:3] - pos

    dist_to_wp = float(np.linalg.norm(waypoints[wp_idx][:3] - pos)) if wp_idx < len(waypoints) else 0.0
    if wp_idx < len(waypoints) - 1 and dist_to_wp < 1.2:
        wp_idx += 1

    cy = np.cos(state.yaw)
    sy = np.sin(state.yaw)
    r_wb = np.array([[cy, sy, 0], [-sy, cy, 0], [0, 0, 1]])
    e_body = r_wb @ e_world

    vel = np.array([state.u, state.v, state.w])
    f_cmd = cfg.kp_pos * e_body - cfg.kd_vel * vel

    desired_yaw = np.arctan2(e_world[1], e_world[0])
    e_yaw = wrap_pi(desired_yaw - state.yaw)
    mz_cmd = cfg.kp_yaw * e_yaw - cfg.kd_r * state.r

    tau = np.array([f_cmd[0], f_cmd[1], f_cmd[2], mz_cmd])
    return tau, wp_idx, dist_to_wp

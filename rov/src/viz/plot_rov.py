from __future__ import annotations

from pathlib import Path
from typing import Dict

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def _save(fig, path: str) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=170)
    plt.close(fig)


def plot_env_topdown(occupancy: np.ndarray, path_xy: np.ndarray, traj_xy: np.ndarray, out: str) -> None:
    occ2 = occupancy.max(axis=2).T
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.imshow(occ2, origin="lower", cmap="bone_r", alpha=0.85)
    ax.plot(path_xy[:, 0], path_xy[:, 1], "c--", lw=2.0, label="Global Path")
    ax.plot(traj_xy[:, 0], traj_xy[:, 1], color="#ff2d55", lw=2.2, label="ROV Trajectory")
    ax.scatter(path_xy[0, 0], path_xy[0, 1], c="lime", s=90, marker="o", label="Start")
    ax.scatter(path_xy[-1, 0], path_xy[-1, 1], c="gold", s=110, marker="*", label="Goal")
    ax.set_title("Top View: Obstacles, Planned Path, and Executed Trajectory")
    ax.set_xlabel("X grid")
    ax.set_ylabel("Y grid")
    ax.legend(loc="upper right")
    ax.grid(alpha=0.25)
    _save(fig, out)


def plot_3d_path(path_xyz: np.ndarray, traj_xyz: np.ndarray, out: str) -> None:
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(path_xyz[:, 0], path_xyz[:, 1], path_xyz[:, 2], "--", color="deepskyblue", lw=2, label="Planned")
    ax.plot(traj_xyz[:, 0], traj_xyz[:, 1], traj_xyz[:, 2], color="orangered", lw=2.3, label="Executed")
    ax.scatter(path_xyz[0, 0], path_xyz[0, 1], path_xyz[0, 2], c="lime", s=70)
    ax.scatter(path_xyz[-1, 0], path_xyz[-1, 1], path_xyz[-1, 2], c="gold", s=80)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("3D Path Tracking")
    ax.legend()
    _save(fig, out)


def plot_timeseries(log: Dict[str, np.ndarray], out: str) -> None:
    t = log["t"]
    fig, axs = plt.subplots(2, 2, figsize=(12, 7))

    axs[0, 0].plot(t, log["x"], label="x")
    axs[0, 0].plot(t, log["y"], label="y")
    axs[0, 0].plot(t, log["z"], label="z")
    axs[0, 0].set_title("Position")
    axs[0, 0].legend(); axs[0, 0].grid(alpha=0.3)

    axs[0, 1].plot(t, log["yaw"], color="purple")
    axs[0, 1].set_title("Yaw (rad)")
    axs[0, 1].grid(alpha=0.3)

    axs[1, 0].plot(t, log["u"], label="u")
    axs[1, 0].plot(t, log["v"], label="v")
    axs[1, 0].plot(t, log["w"], label="w")
    axs[1, 0].plot(t, log["r"], label="r")
    axs[1, 0].set_title("Body Velocities")
    axs[1, 0].legend(); axs[1, 0].grid(alpha=0.3)

    axs[1, 1].plot(t, log["err"], color="red")
    axs[1, 1].set_title("Distance-to-Waypoint Error")
    axs[1, 1].grid(alpha=0.3)

    for a in axs.flat:
        a.set_xlabel("Time (s)")
    _save(fig, out)


def plot_thrusters(log: Dict[str, np.ndarray], out: str) -> None:
    t = log["t"]
    f = log["thrusters"]
    fig, ax = plt.subplots(figsize=(11, 6))
    colors = ["#ff4d4f", "#fa8c16", "#fadb14", "#52c41a", "#1890ff", "#722ed1"]
    for i in range(6):
        ax.plot(t, f[:, i], lw=1.7, color=colors[i], label=f"T{i+1}")
    ax.set_title("Thruster Commands (6 Thrusters: 4 horizontal + 2 vertical)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Force command")
    ax.legend(ncol=3)
    ax.grid(alpha=0.3)
    _save(fig, out)


def plot_current_field_sample(current_xy: np.ndarray, out: str) -> None:
    fig, ax = plt.subplots(figsize=(8, 7))
    x = current_xy[:, 0]
    y = current_xy[:, 1]
    u = current_xy[:, 2]
    v = current_xy[:, 3]
    ax.quiver(x, y, u, v, color="#0050b3", alpha=0.85)
    ax.set_title("Ocean Current Field Sample (XY plane)")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(alpha=0.3)
    _save(fig, out)

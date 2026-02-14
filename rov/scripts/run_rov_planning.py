from __future__ import annotations

from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rov.src.controller.path_tracker import TrackConfig, compute_tau
from rov.src.env.ocean_env import OceanEnv
from rov.src.planner.grid_planner import astar_3d
from rov.src.dynamics.dynamics import ROV4DOF, ROVState
from rov.src.viz.plot_rov import (
    plot_3d_path,
    plot_current_field_sample,
    plot_env_topdown,
    plot_thrusters,
    plot_timeseries,
)


def main() -> None:
    out = Path("output/rov")
    out.mkdir(parents=True, exist_ok=True)

    env = OceanEnv(nx=50, ny=50, nz=20, cell_size=1.0)
    env.add_default_scene()

    start = (2, 2, 3)
    goal = (46, 45, 8)
    path_idx = astar_3d(env, start, goal)
    if path_idx is None:
        raise RuntimeError("No path found for given map")

    waypoints = [env.grid_to_world_center(i) for i in path_idx]
    waypoints4 = [np.array([w[0], w[1], w[2], 0.0]) for w in waypoints]

    rov = ROV4DOF()
    cfg = TrackConfig()
    s = ROVState(x=waypoints4[0][0], y=waypoints4[0][1], z=waypoints4[0][2], yaw=0.0, u=0.0, v=0.0, w=0.0, r=0.0)

    dt = 0.08
    t_final = 150.0
    steps = int(t_final / dt)

    wp_i = 1

    log = {k: [] for k in ["t", "x", "y", "z", "yaw", "u", "v", "w", "r", "err", "thrusters"]}

    for k in range(steps):
        t = k * dt
        tau, wp_i, dist_err = compute_tau(rov, s, waypoints4, wp_i, cfg)
        thr = rov.allocate(tau)
        cur = env.current(np.array([s.x, s.y, s.z]), t)
        s = rov.step(s, thr, cur, dt)

        log["t"].append(t)
        log["x"].append(s.x)
        log["y"].append(s.y)
        log["z"].append(s.z)
        log["yaw"].append(s.yaw)
        log["u"].append(s.u)
        log["v"].append(s.v)
        log["w"].append(s.w)
        log["r"].append(s.r)
        log["err"].append(dist_err)
        log["thrusters"].append(thr.copy())

        pos = np.array([s.x, s.y, s.z])
        if np.linalg.norm(pos - waypoints4[-1][:3]) < 1.2 and wp_i >= len(waypoints4) - 1:
            break

    for key in ["t", "x", "y", "z", "yaw", "u", "v", "w", "r", "err"]:
        log[key] = np.array(log[key], dtype=float)
    log["thrusters"] = np.array(log["thrusters"], dtype=float)

    traj_xyz = np.column_stack([log["x"], log["y"], log["z"]])
    path_xyz = np.array([w[:3] for w in waypoints4])

    plot_env_topdown(env.occupancy, path_xyz[:, :2], traj_xyz[:, :2], str(out / "01_topdown_path.png"))
    plot_3d_path(path_xyz, traj_xyz, str(out / "02_3d_tracking.png"))
    plot_timeseries(log, str(out / "03_state_timeseries.png"))
    plot_thrusters(log, str(out / "04_thruster_commands.png"))

    # sample current field on mid-depth plane
    sample = []
    z_mid = env.nz * 0.5
    for xi in np.linspace(2, env.nx - 2, 16):
        for yi in np.linspace(2, env.ny - 2, 16):
            c = env.current(np.array([xi, yi, z_mid]), 0.0)
            sample.append([xi, yi, c[0], c[1]])
    plot_current_field_sample(np.array(sample), str(out / "05_ocean_current_field.png"))

    final_err = float(np.linalg.norm(traj_xyz[-1] - path_xyz[-1]))
    reached = final_err < 1.6
    energy = float(np.sum(np.abs(log["thrusters"])) * dt)

    txt = out / "result.txt"
    with txt.open("w", encoding="utf-8") as f:
        f.write("ROV Path Planning and Control Result\n")
        f.write("====================================\n")
        f.write(f"Grid size: {env.nx}x{env.ny}x{env.nz}\n")
        f.write(f"DOF model: x,y,z + yaw\n")
        f.write(f"Thrusters: 6 (4 horizontal + 2 vertical)\n")
        f.write(f"Planned waypoints: {len(path_xyz)}\n")
        f.write(f"Executed steps: {len(log['t'])}\n")
        f.write(f"Reached goal: {reached}\n")
        f.write(f"Final distance error: {final_err:.4f} m\n")
        f.write(f"Control effort integral: {energy:.2f}\n")

    print(f"Reached goal: {reached}")
    print(f"Final distance error: {final_err:.4f}")
    print(f"Saved: {txt}")
    for p in sorted(out.glob("*.png")):
        print(f"Saved: {p}")


if __name__ == "__main__":
    main()

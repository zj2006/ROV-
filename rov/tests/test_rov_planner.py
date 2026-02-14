from __future__ import annotations

import numpy as np

from rov.src.env.ocean_env import OceanEnv
from rov.src.planner.grid_planner import astar_3d
from rov.src.dynamics.dynamics import ROV4DOF


def test_astar_find_path() -> None:
    env = OceanEnv(nx=30, ny=30, nz=10)
    env.add_default_scene()
    p = astar_3d(env, (1, 1, 2), (25, 25, 6))
    assert p is not None
    assert len(p) > 3


def test_thruster_allocation_shape() -> None:
    rov = ROV4DOF()
    tau = np.array([10.0, -8.0, 6.0, 2.0])
    f = rov.allocate(tau)
    assert f.shape == (6,)
    assert np.max(np.abs(f)) <= rov.params.thruster_limit + 1e-9

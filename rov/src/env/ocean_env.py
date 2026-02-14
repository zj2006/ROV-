from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


@dataclass
class ObstacleBox:
    x0: int
    x1: int
    y0: int
    y1: int
    z0: int
    z1: int


@dataclass
class OceanEnv:
    nx: int = 50
    ny: int = 50
    nz: int = 20
    cell_size: float = 1.0

    def __post_init__(self) -> None:
        self.occupancy = np.zeros((self.nx, self.ny, self.nz), dtype=np.uint8)
        self.obstacles: List[ObstacleBox] = []

    def add_box(self, box: ObstacleBox) -> None:
        self.obstacles.append(box)
        self.occupancy[box.x0:box.x1, box.y0:box.y1, box.z0:box.z1] = 1

    def add_default_scene(self) -> None:
        self.add_box(ObstacleBox(8, 16, 10, 20, 0, 9))
        self.add_box(ObstacleBox(18, 30, 28, 36, 0, 13))
        self.add_box(ObstacleBox(32, 42, 8, 16, 0, 12))
        self.add_box(ObstacleBox(22, 28, 16, 25, 8, 17))
        self.add_box(ObstacleBox(38, 46, 32, 44, 0, 7))

    def in_bounds(self, idx: Tuple[int, int, int]) -> bool:
        x, y, z = idx
        return 0 <= x < self.nx and 0 <= y < self.ny and 0 <= z < self.nz

    def is_free(self, idx: Tuple[int, int, int]) -> bool:
        x, y, z = idx
        return self.in_bounds(idx) and self.occupancy[x, y, z] == 0

    def world_to_grid(self, p: np.ndarray) -> Tuple[int, int, int]:
        q = np.floor(p / self.cell_size).astype(int)
        return int(q[0]), int(q[1]), int(q[2])

    def grid_to_world_center(self, idx: Tuple[int, int, int]) -> np.ndarray:
        x, y, z = idx
        return np.array([(x + 0.5) * self.cell_size, (y + 0.5) * self.cell_size, (z + 0.5) * self.cell_size])

    def current(self, p: np.ndarray, t: float) -> np.ndarray:
        x, y, z = p
        cx = 0.25 + 0.12 * np.sin(0.11 * y + 0.35 * t) + 0.05 * np.cos(0.2 * z)
        cy = 0.08 * np.cos(0.09 * x + 0.27 * t) + 0.03 * np.sin(0.14 * z)
        cz = 0.04 * np.sin(0.17 * x + 0.13 * y + 0.4 * t)
        return np.array([cx, cy, cz])

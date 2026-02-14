from __future__ import annotations

import heapq
from typing import Dict, List, Optional, Tuple

import numpy as np

from rov.src.env.ocean_env import OceanEnv

Grid = Tuple[int, int, int]


def _heuristic(a: Grid, b: Grid) -> float:
    return float(np.linalg.norm(np.array(a) - np.array(b)))


def _neighbors(n: Grid) -> List[Grid]:
    x, y, z = n
    out: List[Grid] = []
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                if dx == dy == dz == 0:
                    continue
                out.append((x + dx, y + dy, z + dz))
    return out


def astar_3d(env: OceanEnv, start: Grid, goal: Grid) -> Optional[List[Grid]]:
    if not env.is_free(start) or not env.is_free(goal):
        return None

    pq: List[Tuple[float, Grid]] = []
    heapq.heappush(pq, (0.0, start))

    g: Dict[Grid, float] = {start: 0.0}
    parent: Dict[Grid, Grid] = {}

    while pq:
        _, cur = heapq.heappop(pq)
        if cur == goal:
            path = [cur]
            while cur in parent:
                cur = parent[cur]
                path.append(cur)
            path.reverse()
            return path

        for nb in _neighbors(cur):
            if not env.is_free(nb):
                continue
            step = _heuristic(cur, nb)
            cand = g[cur] + step
            if nb not in g or cand < g[nb]:
                g[nb] = cand
                parent[nb] = cur
                f = cand + _heuristic(nb, goal)
                heapq.heappush(pq, (f, nb))
    return None

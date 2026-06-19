"""City obstacle field — single source of truth shared by the offline sim,
the interceptor's collision-aware guidance, and (for drawing) the game manager.

Footprints mirror the buildings in worlds/vtol_world.sdf and the minimap in
game_manager.py. Each obstacle is modelled as a vertical cylinder: a horizontal
footprint radius plus a top height. An aircraft flying above `height` clears it.

World frame: +X = North, +Y = West, +Z = up (metres).
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class Obstacle:
    x: float
    y: float
    radius: float      # horizontal footprint radius (m)
    height: float      # top altitude (m); above this the aircraft is clear
    name: str = ""

    @property
    def center(self) -> np.ndarray:
        return np.array([self.x, self.y])


# (wx, wy, half_x, half_y, height_m, name) — footprints from _WORLD_BUILDINGS,
# heights chosen to match the SDF's relative scale (tower tallest).
_BUILDINGS = [
    (-35,  0,    6,    4,   18, "office_a"),
    ( 30,  15,   4,    5,   20, "office_b"),
    ( 25, -20,   3,    3,   32, "tower"),
    (-20,  35,  12.5,  6,   12, "warehouse"),
    (-15, -30,   5,    5,   14, "apartment"),
    ( 40,   5,   4,    3,   10, "shop_a"),
    (  5,  42,   3,    4,   10, "shop_b"),
    (-40, -22,  11,    7.5, 13, "factory"),
]

_CHIMNEY = (-33, -18, 0.8, 28, "chimney")


def city() -> list[Obstacle]:
    """The default city obstacle field used by v2."""
    obs = [
        Obstacle(x, y, radius=math.hypot(hx, hy), height=h, name=n)
        for (x, y, hx, hy, h, n) in _BUILDINGS
    ]
    obs.append(Obstacle(_CHIMNEY[0], _CHIMNEY[1], _CHIMNEY[2], _CHIMNEY[3], _CHIMNEY[4]))
    return obs

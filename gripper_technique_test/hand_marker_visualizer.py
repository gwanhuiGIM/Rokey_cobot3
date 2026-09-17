# hand_marker_visualizer.py

from __future__ import annotations

from typing import Callable, Optional, Sequence, Tuple

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualSphere


HandGetter = Callable[
    [],
    Tuple[
        Optional[Tuple[float, float, float]],
        int,
    ],
]


class HandMarkerVisualizer:
    """손 raw 좌표를 Isaac Sim의 VisualSphere로 표시한다."""

    def __init__(
        self,
        world: World,
        *,
        coordinate_getter: HandGetter,
        prim_path: str,
        object_name: str,
        color: Sequence[float],
        initial_position: Optional[Sequence[float]] = None,
        radius: float = 0.03,
        label: str = "HAND",
    ) -> None:
        self._coordinate_getter = coordinate_getter
        self._last_sequence = -1
        self._label = str(label)

        if initial_position is None:
            initial_position = (
                0.0,
                0.25,
                1.0,
            )

        marker_color = np.asarray(
            color,
            dtype=np.float64,
        )

        if marker_color.shape != (3,):
            raise ValueError(
                "color must have shape (3,)"
            )

        existing = world.scene.get_object(
            object_name
        )

        if existing is not None:
            self._marker = existing
        else:
            self._marker = world.scene.add(
                VisualSphere(
                    prim_path=prim_path,
                    name=object_name,
                    position=np.asarray(
                        initial_position,
                        dtype=np.float64,
                    ),
                    radius=float(radius),
                    color=marker_color,
                )
            )

    def update(self) -> None:
        hand_raw, sequence = self._coordinate_getter()

        if hand_raw is None:
            return

        if sequence == self._last_sequence:
            return

        position = np.asarray(
            hand_raw,
            dtype=np.float64,
        )

        if position.shape != (3,):
            print(
                f"[{self._label} Marker] "
                f"잘못된 좌표 형태: {position}",
                flush=True,
            )
            return

        if not np.all(np.isfinite(position)):
            print(
                f"[{self._label} Marker] "
                f"유효하지 않은 좌표: {position}",
                flush=True,
            )
            return

        self._marker.set_world_pose(
            position=position,
        )
        self._last_sequence = sequence

    def reset(self) -> None:
        self._last_sequence = -1
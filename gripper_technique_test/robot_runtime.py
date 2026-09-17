# robot_runtime.py
from __future__ import annotations

from dataclasses import dataclass
from typing import FrozenSet, Sequence

import numpy as np


@dataclass(frozen=True)
class RobotTask:
    """매니저가 상태 머신에 전달하는 실행 단위."""

    tray_id: int
    route_id: str

    # 트레이를 집은 뒤 이동할 로봇 전용 경유지.
    transit_position: np.ndarray
    transit_orientation: np.ndarray

    # 경유지 도착 후 TRACKING 방향으로 회전할 joint1 상대각.
    joint1_delta_rad: float = 0.0

    # 신규 트레이 PICK/PLACE 구역 락 사용 여부.
    uses_shared_zone: bool = False

    @classmethod
    def create(
        cls,
        *,
        tray_id: int,
        route_id: str,
        transit_position: Sequence[float],
        transit_orientation: Sequence[float],
        joint1_delta_rad: float = 0.0,
        uses_shared_zone: bool = False,
    ) -> "RobotTask":
        position = np.asarray(
            transit_position,
            dtype=np.float64,
        ).copy()

        orientation = np.asarray(
            transit_orientation,
            dtype=np.float64,
        ).copy()

        if position.shape != (3,):
            raise ValueError(
                "transit_position must have shape (3,)"
            )

        if orientation.shape != (4,):
            raise ValueError(
                "transit_orientation must have shape (4,)"
            )

        if not np.all(np.isfinite(position)):
            raise ValueError(
                "transit_position must be finite"
            )

        if not np.all(np.isfinite(orientation)):
            raise ValueError(
                "transit_orientation must be finite"
            )

        delta = float(joint1_delta_rad)

        if not np.isfinite(delta):
            raise ValueError(
                "joint1_delta_rad must be finite"
            )

        return cls(
            tray_id=int(tray_id),
            route_id=str(route_id).strip().upper(),
            transit_position=position,
            transit_orientation=orientation,
            joint1_delta_rad=delta,
            uses_shared_zone=bool(uses_shared_zone),
        )


@dataclass(frozen=True)
class RobotProfile:
    """
    로봇 선택과 고정 경유지 생성을 위한 정적 설정.

    경로 선택, fallback 경로, 경로 락은 더 이상 사용하지 않는다.
    각 로봇은 자기 전용 경유지 하나만 사용한다.
    """

    robot_id: str
    reachable_trays: FrozenSet[int]

    route_id: str
    transit_position: np.ndarray
    transit_orientation: np.ndarray
    tracking_joint1_delta_rad: float

    # 트레이까지 거리 계산 기준점.
    selection_position: np.ndarray

    @classmethod
    def create(
        cls,
        *,
        robot_id: str,
        reachable_trays,
        route_id: str,
        transit_position: Sequence[float],
        transit_orientation: Sequence[float],
        tracking_joint1_delta_rad: float,
        selection_position: Sequence[float],
    ) -> "RobotProfile":
        transit_position_array = np.asarray(
            transit_position,
            dtype=np.float64,
        ).copy()

        transit_orientation_array = np.asarray(
            transit_orientation,
            dtype=np.float64,
        ).copy()

        selection_position_array = np.asarray(
            selection_position,
            dtype=np.float64,
        ).copy()

        if transit_position_array.shape != (3,):
            raise ValueError(
                "transit_position must have shape (3,)"
            )

        if transit_orientation_array.shape != (4,):
            raise ValueError(
                "transit_orientation must have shape (4,)"
            )

        if selection_position_array.shape != (3,):
            raise ValueError(
                "selection_position must have shape (3,)"
            )

        if not np.all(np.isfinite(transit_position_array)):
            raise ValueError(
                "transit_position must be finite"
            )

        if not np.all(np.isfinite(transit_orientation_array)):
            raise ValueError(
                "transit_orientation must be finite"
            )

        if not np.all(np.isfinite(selection_position_array)):
            raise ValueError(
                "selection_position must be finite"
            )

        joint_delta = float(
            tracking_joint1_delta_rad
        )

        if not np.isfinite(joint_delta):
            raise ValueError(
                "tracking_joint1_delta_rad must be finite"
            )

        return cls(
            robot_id=str(robot_id).strip().upper(),
            reachable_trays=frozenset(
                int(value)
                for value in reachable_trays
            ),
            route_id=str(route_id).strip().upper(),
            transit_position=transit_position_array,
            transit_orientation=transit_orientation_array,
            tracking_joint1_delta_rad=joint_delta,
            selection_position=selection_position_array,
        )
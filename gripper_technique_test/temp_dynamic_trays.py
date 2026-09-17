# temp_dynamic_trays.py

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Mapping, Sequence, Tuple

import numpy as np

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid


def yaw_degrees_to_quaternion(
    yaw_degrees: float,
) -> np.ndarray:
    """Z축 yaw를 Isaac Sim quaternion [w, x, y, z]로 변환한다."""
    half = np.deg2rad(
        float(yaw_degrees)
    ) * 0.5

    return np.array(
        [
            np.cos(half),
            0.0,
            0.0,
            np.sin(half),
        ],
        dtype=np.float64,
    )


def quaternion_to_yaw(
    quaternion: Sequence[float],
) -> float:
    """Isaac Sim quaternion [w, x, y, z]에서 yaw를 추출한다."""
    w, x, y, z = np.asarray(
        quaternion,
        dtype=np.float64,
    )

    return float(
        np.arctan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
    )


def downward_tool_orientation_for_tray(
    tray_orientation: Sequence[float],
) -> np.ndarray:
    """
    수평 트레이의 yaw에 맞춰 그리퍼가 아래를 보도록 한다.

    팀원 코드와 동일하게:
        ee_yaw = tray_yaw + 90도
        quaternion = [0, cos(ee_yaw/2), sin(ee_yaw/2), 0]
    """
    tray_yaw = quaternion_to_yaw(
        tray_orientation
    )
    ee_yaw = tray_yaw + np.pi / 2.0
    half = ee_yaw * 0.5

    return np.array(
        [
            0.0,
            np.cos(half),
            np.sin(half),
            0.0,
        ],
        dtype=np.float64,
    )


@dataclass
class TrayPoseSnapshot:
    tray_id: int

    # 동적 물체의 현재 중심 pose
    current_center_position: np.ndarray
    current_orientation: np.ndarray

    # 현재 상단 접촉 위치
    pick_position: np.ndarray
    pick_orientation: np.ndarray

    # 생성 당시 원복 기준 pose
    spawn_reference_position: np.ndarray
    spawn_orientation: np.ndarray


@dataclass
class TrayRecord:
    tray_id: int
    rigid_object: DynamicCuboid

    # 기존 트레이 좌표와 같은 바닥/생성 기준 위치
    spawn_reference_position: np.ndarray
    spawn_orientation: np.ndarray

    size: np.ndarray


class DynamicTrayRegistry:
    """
    동적으로 생성된 트레이 또는 테스트 큐브를 ID별로 관리한다.

    실제 트레이 생성 코드로 교체할 때도 register()만 호출하면
    상태 머신 코드는 그대로 사용할 수 있다.
    """

    def __init__(self) -> None:
        self._records: Dict[int, TrayRecord] = {}

    def register(
        self,
        *,
        tray_id: int,
        rigid_object,
        spawn_reference_position: Sequence[float],
        spawn_orientation: Sequence[float],
        size: Sequence[float],
    ) -> None:
        tray_id = int(tray_id)

        self._records[tray_id] = TrayRecord(
            tray_id=tray_id,
            rigid_object=rigid_object,
            spawn_reference_position=np.asarray(
                spawn_reference_position,
                dtype=np.float64,
            ).copy(),
            spawn_orientation=np.asarray(
                spawn_orientation,
                dtype=np.float64,
            ).copy(),
            size=np.asarray(
                size,
                dtype=np.float64,
            ).copy(),
        )

    def has_tray(
        self,
        tray_id: int,
    ) -> bool:
        return int(tray_id) in self._records

    def get_snapshot(
        self,
        tray_id: int,
    ) -> TrayPoseSnapshot:
        tray_id = int(tray_id)

        if tray_id not in self._records:
            raise KeyError(
                f"등록되지 않은 트레이입니다: {tray_id}"
            )

        record = self._records[tray_id]

        current_center, current_orientation = (
            record.rigid_object.get_world_pose()
        )

        current_center = np.asarray(
            current_center,
            dtype=np.float64,
        )
        current_orientation = np.asarray(
            current_orientation,
            dtype=np.float64,
        )

        # 트레이는 항상 수평이라고 가정한다.
        # DynamicCuboid pose는 중심 기준이므로 절반 높이를 더하면 상단이다.
        pick_position = (
            current_center
            + np.array(
                [
                    0.0,
                    0.0,
                    record.size[2] * 0.5,
                ],
                dtype=np.float64,
            )
        )

        return TrayPoseSnapshot(
            tray_id=tray_id,
            current_center_position=(
                current_center.copy()
            ),
            current_orientation=(
                current_orientation.copy()
            ),
            pick_position=pick_position,
            pick_orientation=(
                downward_tool_orientation_for_tray(
                    current_orientation
                )
            ),
            spawn_reference_position=(
                record.spawn_reference_position.copy()
            ),
            spawn_orientation=(
                record.spawn_orientation.copy()
            ),
        )

    def reset_to_spawn(self) -> None:
        """테스트 재시작 시 모든 큐브를 생성 위치로 되돌린다."""
        for record in self._records.values():
            center_position = (
                record.spawn_reference_position
                + np.array(
                    [
                        0.0,
                        0.0,
                        record.size[2] * 0.5,
                    ],
                    dtype=np.float64,
                )
            )

            record.rigid_object.set_world_pose(
                position=center_position,
                orientation=record.spawn_orientation,
            )

            set_velocity = getattr(
                record.rigid_object,
                "set_linear_velocity",
                None,
            )
            if callable(set_velocity):
                set_velocity(
                    np.zeros(
                        3,
                        dtype=np.float64,
                    )
                )

            set_angular_velocity = getattr(
                record.rigid_object,
                "set_angular_velocity",
                None,
            )
            if callable(set_angular_velocity):
                set_angular_velocity(
                    np.zeros(
                        3,
                        dtype=np.float64,
                    )
                )


def create_temp_dynamic_trays(
    *,
    world: World,
    spawn_positions: Mapping[int, Sequence[float]],
    yaw_degrees: Mapping[int, float],
    tray_size: Sequence[float],
    mass: float,
) -> DynamicTrayRegistry:
    """
    흡착 테스트용 DynamicCuboid를 생성한다.

    삭제 방법:
        1. main.py의 이 함수 호출부 삭제
        2. temp_dynamic_trays.py 삭제
        3. 실제 생성 코드에서 동일 registry.register() 사용
    """
    registry = DynamicTrayRegistry()

    size = np.asarray(
        tray_size,
        dtype=np.float64,
    )

    colors = {
        4: np.array([0.8, 0.2, 0.2]),
        5: np.array([0.2, 0.8, 0.2]),
        6: np.array([0.2, 0.4, 0.9]),
        7: np.array([0.9, 0.7, 0.2]),
    }

    for tray_id, reference_position in spawn_positions.items():
        tray_id = int(tray_id)

        reference_position = np.asarray(
            reference_position,
            dtype=np.float64,
        )

        orientation = yaw_degrees_to_quaternion(
            yaw_degrees.get(
                tray_id,
                0.0,
            )
        )

        # DynamicCuboid는 중심 위치를 받으므로 반 높이를 더한다.
        center_position = (
            reference_position
            + np.array(
                [
                    0.0,
                    0.0,
                    size[2] * 0.5,
                ],
                dtype=np.float64,
            )
        )

        rigid_object = world.scene.add(
            DynamicCuboid(
                prim_path=(
                    f"/World/TempDynamicTrays/"
                    f"tray_{tray_id}"
                ),
                name=f"temp_tray_{tray_id}",
                position=center_position,
                orientation=orientation,
                scale=size,
                size=1.0,
                color=colors.get(
                    tray_id,
                    np.array(
                        [0.6, 0.6, 0.6]
                    ),
                ),
                mass=float(mass),
            )
        )

        registry.register(
            tray_id=tray_id,
            rigid_object=rigid_object,
            spawn_reference_position=(
                reference_position
            ),
            spawn_orientation=orientation,
            size=size,
        )

        print(
            f"[TempTray] tray={tray_id}, "
            f"spawn={reference_position.tolist()}, "
            f"yaw={yaw_degrees.get(tray_id, 0.0)} deg",
            flush=True,
        )

    return registry
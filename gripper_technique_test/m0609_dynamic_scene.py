# m0609_dynamic_scene.py
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Mapping, Sequence

import numpy as np
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.stage import add_reference_to_stage


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


def multiply_quaternions(
    lhs: Sequence[float],
    rhs: Sequence[float],
) -> np.ndarray:
    """Isaac Sim 순서 [w, x, y, z] quaternion 곱."""
    lw, lx, ly, lz = np.asarray(
        lhs,
        dtype=np.float64,
    )
    rw, rx, ry, rz = np.asarray(
        rhs,
        dtype=np.float64,
    )

    result = np.array(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ],
        dtype=np.float64,
    )

    norm = float(
        np.linalg.norm(result)
    )

    if norm < 1e-12:
        raise ValueError(
            "Quaternion norm is zero"
        )

    return result / norm


def rotate_orientation_about_z(
    orientation: Sequence[float],
    angle_deg: float,
) -> np.ndarray:
    """기존 orientation에 월드 Z축 회전을 추가한다."""
    half_angle = np.deg2rad(
        float(angle_deg)
    ) * 0.5

    z_rotation = np.array(
        [
            np.cos(half_angle),
            0.0,
            0.0,
            np.sin(half_angle),
        ],
        dtype=np.float64,
    )

    return multiply_quaternions(
        z_rotation,
        orientation,
    )


def downward_tool_orientation_for_tray(
    tray_orientation: Sequence[float],
) -> np.ndarray:
    """트레이 yaw에 맞춘 하향 그리퍼 quaternion을 만든다."""
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
    current_reference_position: np.ndarray
    current_orientation: np.ndarray
    pick_position: np.ndarray
    pick_orientation: np.ndarray
    spawn_reference_position: np.ndarray
    spawn_orientation: np.ndarray


@dataclass
class TrayRecord:
    tray_id: int
    rigid_object: object
    spawn_reference_position: np.ndarray
    spawn_orientation: np.ndarray
    top_offset_z: float


@dataclass
class ToolRecord:
    tool_index: int
    rigid_object: object
    spawn_orientation: np.ndarray
    prim_path: str = ""


class DynamicTrayRegistry:
    """트레이의 현재 pose와 생성 당시 pose를 ID별로 관리한다."""

    def __init__(self) -> None:
        self._records: Dict[int, TrayRecord] = {}
        self._tool_records: Dict[int, ToolRecord] = {}

    def register_tray(
        self,
        *,
        tray_id: int,
        rigid_object,
        spawn_reference_position: Sequence[float],
        spawn_orientation: Sequence[float],
        top_offset_z: float,
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
            top_offset_z=float(top_offset_z),
        )

    def register_tool(
        self,
        *,
        tool_index: int,
        rigid_object,
        spawn_orientation: Sequence[float],
        prim_path: str = "",
    ) -> None:
        tool_index = int(tool_index)

        self._tool_records[tool_index] = ToolRecord(
            tool_index=tool_index,
            rigid_object=rigid_object,
            spawn_orientation=np.asarray(
                spawn_orientation,
                dtype=np.float64,
            ).copy(),
            prim_path=str(prim_path),
        )

    def reset_tools_to_layout(
        self,
        *,
        tray_tool_indices: Mapping[int, int],
        tray_positions: Mapping[
            int,
            Sequence[float],
        ],
        tool_drop_height: float,
    ) -> None:
        """
        새 랜덤 배치에 맞춰 실제 도구 Prim을 각 트레이 위로 옮긴다.
        """
        assigned_tools = set()

        for tray_id, tool_index in sorted(
            tray_tool_indices.items()
        ):
            tray_id = int(tray_id)
            tool_index = int(tool_index)

            if tool_index in assigned_tools:
                raise ValueError(
                    f"tool_index={tool_index}가 중복 배정됐습니다"
                )

            if tool_index not in self._tool_records:
                raise KeyError(
                    f"등록되지 않은 tool_index={tool_index}"
                )

            if tray_id not in tray_positions:
                raise KeyError(
                    f"등록되지 않은 tray_id={tray_id}"
                )

            assigned_tools.add(tool_index)
            record = self._tool_records[tool_index]

            tray_position = np.asarray(
                tray_positions[tray_id],
                dtype=np.float64,
            )

            tool_position = (
                tray_position
                + np.array(
                    [
                        0.0,
                        0.0,
                        float(tool_drop_height),
                    ],
                    dtype=np.float64,
                )
            )

            record.rigid_object.set_world_pose(
                position=tool_position,
                orientation=record.spawn_orientation,
            )

            _clear_velocity(
                record.rigid_object
            )

        print(
            "[DynamicScene] tool layout reset: "
            f"{dict(sorted(tray_tool_indices.items()))}",
            flush=True,
        )

    def _set_tool_visible(
        self,
        tool_index: int,
        visible: bool,
    ) -> None:
        record = self._tool_records.get(int(tool_index))

        if record is None or not record.prim_path:
            return

        stage = (
            omni.usd
            .get_context()
            .get_stage()
        )
        prim = stage.GetPrimAtPath(record.prim_path)

        if not prim.IsValid():
            return

        imageable = UsdGeom.Imageable(prim)

        if visible:
            imageable.MakeVisible()
        else:
            imageable.MakeInvisible()

    def apply_random_tool_layout(
        self,
        *,
        tray_tool_indices: Mapping[int, int],
        tray_positions: Mapping[
            int,
            Sequence[float],
        ],
        all_tool_indices: Sequence[int],
        tool_place_height: float,
    ) -> None:
        """
        부분 랜덤 배치 적용:
          - 배치된 도구: 해당 트레이 위로 이동 + 보이게
          - 배치 안 된 도구(missing): 숨김(invisible)

        kinematic 이라 set_world_pose 로 정확히 놓이며 드롭이 필요 없다.
        """
        present_indices = set()

        for tray_id, tool_index in sorted(
            tray_tool_indices.items()
        ):
            tray_id = int(tray_id)
            tool_index = int(tool_index)

            if tool_index not in self._tool_records:
                raise KeyError(
                    f"등록되지 않은 tool_index={tool_index}"
                )

            if tray_id not in tray_positions:
                raise KeyError(
                    f"등록되지 않은 tray_id={tray_id}"
                )

            present_indices.add(tool_index)
            record = self._tool_records[tool_index]

            tray_position = np.asarray(
                tray_positions[tray_id],
                dtype=np.float64,
            )

            tool_position = (
                tray_position
                + np.array(
                    [
                        0.0,
                        0.0,
                        float(tool_place_height),
                    ],
                    dtype=np.float64,
                )
            )

            record.rigid_object.set_world_pose(
                position=tool_position,
                orientation=record.spawn_orientation,
            )

            _clear_velocity(record.rigid_object)
            self._set_tool_visible(tool_index, True)

        # 배치되지 않은 도구는 숨겨서 카메라/비전에 안 보이게 한다.
        for tool_index in all_tool_indices:
            if int(tool_index) not in present_indices:
                self._set_tool_visible(int(tool_index), False)

        print(
            "[DynamicScene] random tool layout: "
            f"present={sorted(present_indices)}, "
            f"hidden={[int(i) for i in all_tool_indices if int(i) not in present_indices]}",
            flush=True,
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

        current_position, current_orientation = (
            record.rigid_object.get_world_pose()
        )

        current_position = np.asarray(
            current_position,
            dtype=np.float64,
        )

        current_orientation = np.asarray(
            current_orientation,
            dtype=np.float64,
        )

        pick_position = (
            current_position
            + np.array(
                [
                    0.0,
                    0.0,
                    record.top_offset_z,
                ],
                dtype=np.float64,
            )
        )

        return TrayPoseSnapshot(
            tray_id=tray_id,
            current_reference_position=(
                current_position.copy()
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
        for record in self._records.values():
            record.rigid_object.set_world_pose(
                position=record.spawn_reference_position,
                orientation=record.spawn_orientation,
            )

            _clear_velocity(
                record.rigid_object
            )


def clamp_to_table(
    position: Sequence[float],
    *,
    x_range,
    y_range,
    margin: float,
):
    """
    트레이/도구 중심이 work_table 풋프린트 안쪽(가장자리 margin 여유)에
    있도록 X/Y 를 클램프한다. 밖으로 나갔으면 보정값과 함께 알린다.
    Z 는 그대로 둔다(생성 시 위에서 떨어뜨려 안착시키는 의도 유지).
    """
    position = np.asarray(position, dtype=np.float64).copy()

    x_min = float(x_range[0]) + float(margin)
    x_max = float(x_range[1]) - float(margin)
    y_min = float(y_range[0]) + float(margin)
    y_max = float(y_range[1]) - float(margin)

    clamped_x = float(np.clip(position[0], x_min, x_max))
    clamped_y = float(np.clip(position[1], y_min, y_max))

    if clamped_x != position[0] or clamped_y != position[1]:
        print(
            "[DynamicScene] 경고: 트레이 위치가 테이블 밖이라 클램프함 "
            f"({position[0]:.3f}, {position[1]:.3f}) -> "
            f"({clamped_x:.3f}, {clamped_y:.3f})",
            flush=True,
        )

    position[0] = clamped_x
    position[1] = clamped_y

    return position


def _clear_velocity(
    rigid_object,
) -> None:
    for method_name in (
        "set_linear_velocity",
        "set_angular_velocity",
    ):
        method = getattr(
            rigid_object,
            method_name,
            None,
        )

        if callable(method):
            method(
                np.zeros(
                    3,
                    dtype=np.float64,
                )
            )


def _find_source_rigid_prim_path(
    tool_usd_path: str,
) -> Sdf.Path:
    source_stage = Usd.Stage.Open(
        tool_usd_path,
        load=Usd.Stage.LoadNone,
    )

    if source_stage is None:
        raise RuntimeError(
            "도구 USD를 열 수 없습니다: "
            f"{tool_usd_path}"
        )

    default_prim = source_stage.GetDefaultPrim()

    if default_prim.IsValid():
        search_root = default_prim
    else:
        search_root = source_stage.GetPseudoRoot()

    rigid_candidates = []

    for prim in Usd.PrimRange(
        search_root
    ):
        if prim.HasAPI(
            UsdPhysics.RigidBodyAPI
        ):
            rigid_candidates.append(
                prim.GetPath()
            )

    if not rigid_candidates:
        expected_name = Path(
            tool_usd_path
        ).stem

        for prim in Usd.PrimRange(
            search_root
        ):
            if prim.GetName() == expected_name:
                return prim.GetPath()

        raise RuntimeError(
            "도구 USD에서 RigidBody Prim을 "
            f"찾을 수 없습니다: {tool_usd_path}"
        )

    rigid_candidates.sort(
        key=lambda path: len(
            path.pathString.split("/")
        )
    )

    return rigid_candidates[0]


def _configure_existing_tool_physics(
    tool_prim,
    *,
    mass: float,
) -> None:
    if not tool_prim.HasAPI(
        UsdPhysics.RigidBodyAPI
    ):
        UsdPhysics.RigidBodyAPI.Apply(
            tool_prim
        )

    if not tool_prim.HasAPI(
        UsdPhysics.MassAPI
    ):
        UsdPhysics.MassAPI.Apply(
            tool_prim
        )

    UsdPhysics.MassAPI(
        tool_prim
    ).GetMassAttr().Set(
        float(mass)
    )

    for child in Usd.PrimRange(
        tool_prim
    ):
        if child.GetTypeName() != "Mesh":
            continue

        if not child.HasAPI(
            UsdPhysics.CollisionAPI
        ):
            UsdPhysics.CollisionAPI.Apply(
                child
            )

        collision = (
            UsdPhysics.MeshCollisionAPI.Apply(
                child
            )
        )

        collision.GetApproximationAttr().Set(
            "convexHull"
        )


def _create_tool_reference(
    *,
    stage,
    tool_id: int,
    tool_usd_path: str,
    tool_position: Sequence[float],
    tool_orientation: Sequence[float],
    tool_scale: Sequence[float],
    tool_mass: float,
) -> None:
    source_prim_path = (
        _find_source_rigid_prim_path(
            tool_usd_path
        )
    )

    target_path = (
        f"/World/tool_{int(tool_id)}"
    )

    with Usd.EditContext(
        stage,
        stage.GetRootLayer(),
    ):
        target_prim = stage.DefinePrim(
            target_path,
            "Xform",
        )

        target_prim.GetReferences().AddReference(
            Sdf.Reference(
                assetPath=tool_usd_path,
                primPath=source_prim_path,
            )
        )

    tool_prim = stage.GetPrimAtPath(
        target_path
    )

    if not tool_prim.IsValid():
        raise RuntimeError(
            f"도구 Prim 생성 실패: {target_path}"
        )

    orientation = np.asarray(
        tool_orientation,
        dtype=np.float64,
    )

    scale = np.asarray(
        tool_scale,
        dtype=np.float64,
    )

    if orientation.shape != (4,):
        raise ValueError(
            "tool_orientation은 길이 4여야 합니다."
        )

    if scale.shape != (3,):
        raise ValueError(
            "tool_scale은 길이 3이어야 합니다."
        )

    with Usd.EditContext(
        stage,
        stage.GetRootLayer(),
    ):
        xformable = UsdGeom.Xformable(
            tool_prim
        )

        xformable.ClearXformOpOrder()
        xformable.SetResetXformStack(
            True
        )

        xformable.AddTranslateOp().Set(
            Gf.Vec3d(
                float(tool_position[0]),
                float(tool_position[1]),
                float(tool_position[2]),
            )
        )

        xformable.AddOrientOp(
            precision=UsdGeom.XformOp.PrecisionDouble
        ).Set(
            Gf.Quatd(
                float(orientation[0]),
                Gf.Vec3d(
                    float(orientation[1]),
                    float(orientation[2]),
                    float(orientation[3]),
                ),
            )
        )

        xformable.AddScaleOp().Set(
            Gf.Vec3f(
                float(scale[0]),
                float(scale[1]),
                float(scale[2]),
            )
        )

    _configure_existing_tool_physics(
        tool_prim,
        mass=tool_mass,
    )

    return target_path


def create_surgical_trays_and_tools(
    *,
    world: World,
    tray_usd_path: str,
    tool_usds: Sequence[str],
    tray_tool_indices: Mapping[int, int],
    tray_positions: Mapping[
        int,
        Sequence[float],
    ],
    tray_orientation: Sequence[float],
    tray_top_z: float,
    tool_drop_height: float,
    tool_mass: float,
    tool_scales: Sequence[
        Sequence[float]
    ],
    simulation_app,
    table_x_range=None,
    table_y_range=None,
    table_margin: float = 0.0,
) -> DynamicTrayRegistry:
    """
    2열×3행 구조로 트레이 6개와 도구 6개를 생성한다.

    배치 번호:
        4 5
        2 3
      A 0 1 B

    트레이가 90도 추가 회전되므로 도구도 동일한 yaw로
    생성해서 트레이 방향과 일치시킨다.
    """
    tray_count = len(
        tray_positions
    )

    if len(tool_usds) < tray_count:
        raise ValueError(
            "트레이 수보다 TOOL_USDS 수가 적습니다."
        )

    if len(tool_scales) < tray_count:
        raise ValueError(
            "트레이 수보다 TOOL_SCALES 수가 적습니다."
        )

    stage = (
        omni.usd
        .get_context()
        .get_stage()
    )

    registry = DynamicTrayRegistry()

    tray_orientation = np.asarray(
        tray_orientation,
        dtype=np.float64,
    )

    if tray_orientation.shape != (4,):
        raise ValueError(
            "tray_orientation은 길이 4여야 합니다."
        )

    for tray_id, position in sorted(
        tray_positions.items()
    ):
        tray_id = int(tray_id)

        position = np.asarray(
            position,
            dtype=np.float64,
        )

        # 테이블 경계가 주어지면 트레이가 테이블 밖으로 나가 떨어지지
        # 않도록 X/Y 를 풋프린트 안쪽으로 클램프한다.
        if table_x_range is not None and table_y_range is not None:
            position = clamp_to_table(
                position,
                x_range=table_x_range,
                y_range=table_y_range,
                margin=table_margin,
            )

        tray_ref = (
            f"/World/tray_{tray_id}"
        )

        add_reference_to_stage(
            usd_path=tray_usd_path,
            prim_path=tray_ref,
        )

        for _ in range(5):
            simulation_app.update()

        tray_rigid_path = (
            f"{tray_ref}/E_redtray_28"
        )

        if not stage.GetPrimAtPath(
            tray_rigid_path
        ).IsValid():
            raise RuntimeError(
                "트레이 rigid Prim을 찾지 못했습니다: "
                f"{tray_rigid_path}"
            )

        tray = world.scene.add(
            SingleRigidPrim(
                prim_path=tray_rigid_path,
                name=f"tray_{tray_id}",
                position=position,
                orientation=tray_orientation,
            )
        )

        registry.register_tray(
            tray_id=tray_id,
            rigid_object=tray,
            spawn_reference_position=position,
            spawn_orientation=tray_orientation,
            top_offset_z=tray_top_z,
        )

        tool_position = (
            position
            + np.array(
                [
                    0.0,
                    0.0,
                    float(tool_drop_height),
                ],
                dtype=np.float64,
            )
        )

        # 도구 USD의 기본 축이 트레이와 90도 다르므로,
        # 트레이 orientation에 Z축 +90도를 추가한다.
        tool_orientation = (
            rotate_orientation_about_z(
                tray_orientation,
                90.0,
            )
        )

        if tray_id not in tray_tool_indices:
            raise KeyError(
                f"tray_tool_indices에 tray={tray_id}가 없습니다"
            )

        tool_index = int(
            tray_tool_indices[tray_id]
        )

        if not 0 <= tool_index < len(tool_usds):
            raise IndexError(
                f"잘못된 tool_index={tool_index}, tray={tray_id}"
            )

        tool_prim_path = _create_tool_reference(
            stage=stage,
            tool_id=tool_index,
            tool_usd_path=tool_usds[tool_index],
            tool_position=tool_position,
            tool_orientation=tool_orientation,
            tool_scale=tool_scales[tool_index],
            tool_mass=tool_mass,
        )

        tool_rigid = world.scene.add(
            SingleRigidPrim(
                prim_path=tool_prim_path,
                name=f"tool_{tool_index}",
                position=tool_position,
                orientation=tool_orientation,
            )
        )

        registry.register_tool(
            tool_index=tool_index,
            rigid_object=tool_rigid,
            spawn_orientation=tool_orientation,
            prim_path=tool_prim_path,
        )

        print(
            f"[DynamicScene] tray={tray_id}, "
            f"tool_index={tool_index}, "
            f"position={position.tolist()}, "
            f"tray_orientation="
            f"{tray_orientation.round(5).tolist()}, "
            f"tool_orientation="
            f"{tool_orientation.round(5).tolist()}, "
            f"tool={Path(tool_usds[tool_index]).name}, "
            f"scale={tuple(tool_scales[tool_index])}",
            flush=True,
        )

    print(
        "[DynamicScene] 트레이 6개와 "
        "도구 6개 생성 완료",
        flush=True,
    )

    return registry
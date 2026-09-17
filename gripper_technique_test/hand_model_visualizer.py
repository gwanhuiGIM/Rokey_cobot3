# hand_model_visualizer.py

from __future__ import annotations

from typing import Callable, Optional, Sequence, Tuple

import numpy as np
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom

from isaacsim.core.api import World


# (좌표/시퀀스) 또는 (None, 시퀀스) 형태를 돌려주는 캐시 게터.
PositionGetter = Callable[
    [],
    Tuple[
        Optional[Tuple[float, float, float]],
        int,
    ],
]

OrientationGetter = Callable[
    [],
    Tuple[
        Optional[Tuple[float, float, float, float]],
        int,
    ],
]


class HandModelVisualizer:
    """
    손 raw 좌표 + 회전 quaternion을 Isaac Sim의 실제 손 USDA 모델로 표시한다.

    기존 HandMarkerVisualizer(구)와 동일하게 update()/reset() 인터페이스를
    제공하므로 main.py에서 교체만 하면 된다.

    핵심: USDA의 RightHandRoot / LeftHandRoot 에는 사용자가 지정한
    회전 중심(xformOp:translate:pivot)이 들어있다. 이 루트를 통째로
    reference 해서 pivot op 스택을 그대로 살리고, translate / orient 값만
    매 프레임 덮어쓴다.

    루트의 op 순서:
        [translate, translate:pivot, orient, scale, !invert!translate:pivot]

    이 구성에서 로컬 점 x 의 월드 변환은
        world(x) = translate + pivot + R(orient) * S(scale) * (x - pivot)
    이다. pivot 점(x = pivot)을 추적 위치 P 에 정확히 두려면
        translate = P - pivot
    으로 잡으면 되고, 그러면 회전은 pivot 을 중심으로 일어난다.
    """

    def __init__(
        self,
        world: World,
        *,
        simulation_app,
        coordinate_getter: PositionGetter,
        orientation_getter: OrientationGetter,
        usda_path: str,
        ref_prim_path: str,
        model_prim_path: str,
        initial_position: Optional[Sequence[float]] = None,
        initial_orientation: Optional[Sequence[float]] = None,
        orient_offset_deg: Optional[Sequence[float]] = None,
        pivot_offset: Optional[Sequence[float]] = None,
        stabilize_roll: bool = False,
        roll_axis: Optional[Sequence[float]] = None,
        roll_sign: float = 1.0,
        label: str = "HAND",
        warmup_frames: int = 8,
    ) -> None:
        self._coordinate_getter = coordinate_getter
        self._orientation_getter = orientation_getter
        self._label = str(label)
        self._model_prim_path = str(model_prim_path)

        self._last_position_sequence = -1
        self._last_orientation_sequence = -1

        if initial_position is None:
            initial_position = (0.0, 0.25, 1.0)

        if initial_orientation is None:
            # Isaac Sim 순서 (w, x, y, z) 단위 quaternion
            initial_orientation = (1.0, 0.0, 0.0, 0.0)

        if orient_offset_deg is None:
            orient_offset_deg = (0.0, 0.0, 0.0)

        if pivot_offset is None:
            pivot_offset = (0.0, 0.0, 0.0)

        # 메시 rest 프레임 정렬용 고정 보정 quaternion (w, x, y, z).
        # 추적 회전에 로컬 기준으로 곱해진다: q_applied = q_tracked * q_offset
        self._orient_offset = self._euler_xyz_deg_to_quat(
            orient_offset_deg
        )

        # 롤 축 고정(swing-twist) 설정.
        self._stabilize_roll = bool(stabilize_roll)
        self._roll_sign = float(roll_sign)

        if roll_axis is None:
            roll_axis = (0.0, 1.0, 0.0)

        roll_axis_arr = np.asarray(roll_axis, dtype=np.float64)
        roll_axis_norm = np.linalg.norm(roll_axis_arr)
        self._roll_axis = (
            roll_axis_arr / roll_axis_norm
            if roll_axis_norm > 1e-12
            else np.array([0.0, 1.0, 0.0], dtype=np.float64)
        )

        self._pivot_offset = np.asarray(
            pivot_offset,
            dtype=np.float64,
        )

        self._translate_op = None
        self._orient_op = None
        self._pivot_op = None
        # 회전 중심. asset pivot + pivot_offset. translate = P - pivot 보정에 쓴다.
        self._pivot = np.zeros(3, dtype=np.float64)

        self._stage = (
            omni.usd
            .get_context()
            .get_stage()
        )

        self._build_prim(
            usda_path=usda_path,
            ref_prim_path=ref_prim_path,
            initial_position=initial_position,
            initial_orientation=initial_orientation,
        )

        # payload(rigged_hand.usd) 로드를 위해 몇 프레임 펌프한다.
        for _ in range(max(0, int(warmup_frames))):
            simulation_app.update()

    def _build_prim(
        self,
        *,
        usda_path: str,
        ref_prim_path: str,
        initial_position: Sequence[float],
        initial_orientation: Sequence[float],
    ) -> None:
        stage = self._stage

        with Usd.EditContext(
            stage,
            stage.GetRootLayer(),
        ):
            model_prim = stage.DefinePrim(
                self._model_prim_path,
                "Xform",
            )

            if not model_prim.IsValid():
                raise RuntimeError(
                    f"[{self._label}] 손 모델 프림 생성 실패: "
                    f"{self._model_prim_path}"
                )

            # pivot op 스택을 가진 루트를 통째로 가져온다.
            model_prim.GetReferences().AddReference(
                Sdf.Reference(
                    assetPath=str(usda_path),
                    primPath=Sdf.Path(str(ref_prim_path)),
                )
            )

            xformable = UsdGeom.Xformable(model_prim)

            self._bind_existing_ops(xformable)

            self._translate_op.Set(
                self._compose_translate(initial_position)
            )
            self._orient_op.Set(
                self._make_quat(
                    self._apply_offset(initial_orientation)
                )
            )

        print(
            f"[{self._label} Model] loaded: "
            f"{self._model_prim_path} <- {ref_prim_path} "
            f"(pivot={self._pivot.round(4).tolist()})",
            flush=True,
        )

    def _bind_existing_ops(
        self,
        xformable: UsdGeom.Xformable,
    ) -> None:
        """
        reference 로 들어온 루트의 기존 translate / orient op 핸들을 잡고,
        pivot 값을 읽어둔다. op 스택(xformOpOrder)은 건드리지 않는다.
        """
        translate_op = None
        orient_op = None
        asset_pivot = np.zeros(3, dtype=np.float64)

        for op in xformable.GetOrderedXformOps():
            op_name = op.GetOpName()

            if op_name == "xformOp:translate:pivot":
                # forward/inverse 가 같은 attribute 를 공유하므로
                # forward op 핸들 하나만 잡으면 양쪽에 반영된다.
                if not op.IsInverseOp():
                    self._pivot_op = op
                    pivot_value = op.Get()

                    if pivot_value is not None:
                        asset_pivot = np.array(
                            [
                                float(pivot_value[0]),
                                float(pivot_value[1]),
                                float(pivot_value[2]),
                            ],
                            dtype=np.float64,
                        )
                continue

            if op.IsInverseOp():
                continue

            if op_name == "xformOp:translate":
                translate_op = op
            elif op_name == "xformOp:orient":
                orient_op = op

        if translate_op is None or orient_op is None:
            raise RuntimeError(
                f"[{self._label}] 루트에서 translate/orient op 를 "
                "찾지 못했습니다. asset의 xformOpOrder 를 확인하세요."
            )

        self._translate_op = translate_op
        self._orient_op = orient_op

        # 회전 중심 = asset pivot + offset. pivot op 값을 갱신하면
        # forward/!invert! 양쪽이 새 값을 쓰고, translate = P - pivot 과
        # 합쳐져 회전 중심이 정확히 추적 위치 P 에 오면서 그만큼 이동한다.
        self._pivot = asset_pivot + self._pivot_offset

        if self._pivot_op is not None:
            self._pivot_op.Set(
                Gf.Vec3d(
                    float(self._pivot[0]),
                    float(self._pivot[1]),
                    float(self._pivot[2]),
                )
            )

    def _compose_translate(
        self,
        position: Sequence[float],
    ) -> Gf.Vec3d:
        """pivot 점이 추적 위치에 오도록 translate = P - pivot 으로 보정."""
        pos = np.asarray(position, dtype=np.float64)
        shifted = pos - self._pivot

        return Gf.Vec3d(
            float(shifted[0]),
            float(shifted[1]),
            float(shifted[2]),
        )

    @staticmethod
    def _make_quat(
        orientation: Sequence[float],
    ) -> Gf.Quatd:
        """(w, x, y, z) -> Gf.Quatd."""
        return Gf.Quatd(
            float(orientation[0]),
            Gf.Vec3d(
                float(orientation[1]),
                float(orientation[2]),
                float(orientation[3]),
            ),
        )

    @staticmethod
    def _quat_mul(
        a: np.ndarray,
        b: np.ndarray,
    ) -> np.ndarray:
        """(w, x, y, z) quaternion 곱 a ⊗ b."""
        aw, ax, ay, az = a
        bw, bx, by, bz = b

        return np.array(
            [
                aw * bw - ax * bx - ay * by - az * bz,
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _euler_xyz_deg_to_quat(
        euler_deg: Sequence[float],
    ) -> np.ndarray:
        """
        Euler 각도(deg, X->Y->Z 순서) -> (w, x, y, z) quaternion.
        q = qz ⊗ qy ⊗ qx (X 회전이 먼저 적용된다).
        """
        rx, ry, rz = (
            np.deg2rad(float(euler_deg[0])),
            np.deg2rad(float(euler_deg[1])),
            np.deg2rad(float(euler_deg[2])),
        )

        qx = np.array(
            [np.cos(rx / 2.0), np.sin(rx / 2.0), 0.0, 0.0],
            dtype=np.float64,
        )
        qy = np.array(
            [np.cos(ry / 2.0), 0.0, np.sin(ry / 2.0), 0.0],
            dtype=np.float64,
        )
        qz = np.array(
            [np.cos(rz / 2.0), 0.0, 0.0, np.sin(rz / 2.0)],
            dtype=np.float64,
        )

        return HandModelVisualizer._quat_mul(
            qz,
            HandModelVisualizer._quat_mul(qy, qx),
        )

    @staticmethod
    def _quat_conj(q: np.ndarray) -> np.ndarray:
        return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)

    @staticmethod
    def _quat_rotate_vector(
        q: np.ndarray,
        v: Sequence[float],
    ) -> np.ndarray:
        """quaternion q 로 벡터 v 를 회전: v' = q v q*."""
        qv = np.array([0.0, v[0], v[1], v[2]], dtype=np.float64)
        rotated = HandModelVisualizer._quat_mul(
            HandModelVisualizer._quat_mul(q, qv),
            HandModelVisualizer._quat_conj(q),
        )
        return rotated[1:4]

    @staticmethod
    def _axis_angle_quat(
        axis: np.ndarray,
        angle: float,
    ) -> np.ndarray:
        a = np.asarray(axis, dtype=np.float64)
        norm = np.linalg.norm(a)

        if norm < 1e-12:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

        a = a / norm
        half = 0.5 * float(angle)

        return np.array(
            [
                np.cos(half),
                np.sin(half) * a[0],
                np.sin(half) * a[1],
                np.sin(half) * a[2],
            ],
            dtype=np.float64,
        )

    def _stabilize_roll_quat(
        self,
        q: np.ndarray,
    ) -> np.ndarray:
        """
        손가락이 가리키는 방향(swing, 피치/요)은 그대로 두고,
        롤(손가락축 회전)만 고정 월드 축(self._roll_axis)으로 다시 적용한다.

        q = swing ⊗ twist(finger) 로 분해 후,
        finalized = roll(roll_axis, θ) ⊗ swing 로 재조립한다.
        (roll 을 월드 프레임 바깥쪽에 두어 항상 roll_axis 로 돈다.)
        """
        # 현재 손가락축(월드) = canonical Y 를 q 로 회전.
        finger_axis = self._quat_rotate_vector(
            q,
            (0.0, 1.0, 0.0),
        )
        finger_norm = np.linalg.norm(finger_axis)

        if finger_norm < 1e-9:
            return q

        finger_axis = finger_axis / finger_norm

        # finger 축 둘레 twist 성분 추출.
        proj = np.dot(q[1:4], finger_axis) * finger_axis
        twist = np.array(
            [q[0], proj[0], proj[1], proj[2]],
            dtype=np.float64,
        )
        twist_norm = np.linalg.norm(twist)

        if twist_norm < 1e-9:
            return q

        twist = twist / twist_norm

        roll_angle = 2.0 * np.arctan2(
            float(np.dot(twist[1:4], finger_axis)),
            float(twist[0]),
        )

        # swing = q ⊗ twist^-1 (손가락 포인팅만 남김)
        swing = self._quat_mul(q, self._quat_conj(twist))

        # 고정 월드 축으로 롤을 다시 적용 (바깥쪽 = 월드 프레임)
        roll = self._axis_angle_quat(
            self._roll_axis,
            self._roll_sign * roll_angle,
        )

        return self._quat_mul(roll, swing)

    def _apply_offset(
        self,
        quaternion: Sequence[float],
    ) -> np.ndarray:
        """
        롤 안정화(옵션) 후 메시 정렬 보정을 곱해 최종 orient 를 만든다.
        q_applied = stabilized(q) ⊗ q_offset
        """
        q = np.asarray(quaternion, dtype=np.float64)
        q_norm = np.linalg.norm(q)

        if q_norm < 1e-12:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

        q = q / q_norm

        if self._stabilize_roll:
            q = self._stabilize_roll_quat(q)

        applied = self._quat_mul(q, self._orient_offset)

        norm = np.linalg.norm(applied)

        if norm < 1e-12:
            return q

        return applied / norm

    def _apply_position(self) -> None:
        hand_raw, sequence = self._coordinate_getter()

        if hand_raw is None:
            return

        if sequence == self._last_position_sequence:
            return

        position = np.asarray(
            hand_raw,
            dtype=np.float64,
        )

        if position.shape != (3,):
            print(
                f"[{self._label} Model] "
                f"잘못된 좌표 형태: {position}",
                flush=True,
            )
            return

        if not np.all(np.isfinite(position)):
            print(
                f"[{self._label} Model] "
                f"유효하지 않은 좌표: {position}",
                flush=True,
            )
            return

        self._translate_op.Set(
            self._compose_translate(position)
        )
        self._last_position_sequence = sequence

    def _apply_orientation(self) -> None:
        orientation, sequence = self._orientation_getter()

        if orientation is None:
            return

        if sequence == self._last_orientation_sequence:
            return

        quaternion = np.asarray(
            orientation,
            dtype=np.float64,
        )

        if quaternion.shape != (4,):
            print(
                f"[{self._label} Model] "
                f"잘못된 회전 형태: {quaternion}",
                flush=True,
            )
            return

        if not np.all(np.isfinite(quaternion)):
            print(
                f"[{self._label} Model] "
                f"유효하지 않은 회전: {quaternion}",
                flush=True,
            )
            return

        norm = float(np.linalg.norm(quaternion))

        if norm < 1e-9:
            return

        quaternion = quaternion / norm

        # 캐시는 (w, x, y, z) 순서. 메시 정렬 보정을 곱해서 적용한다.
        self._orient_op.Set(
            self._make_quat(
                self._apply_offset(quaternion)
            )
        )
        self._last_orientation_sequence = sequence

    def update(self) -> None:
        self._apply_position()
        self._apply_orientation()

    def reset(self) -> None:
        self._last_position_sequence = -1
        self._last_orientation_sequence = -1

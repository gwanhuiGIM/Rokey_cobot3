from __future__ import annotations

import numpy as np
import omni.usd

from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.surface_gripper import _surface_gripper


class SurfaceGripperAdapter:
    """Isaac Sim Surface Gripper를 PickPlace gripper 인터페이스에 맞춘 어댑터."""

    def __init__(
        self,
        end_effector_prim_path: str,
        surface_gripper_prim_path: str,
        *,
        write_status_to_usd: bool = True,
    ) -> None:
        self.end_effector_prim_path = end_effector_prim_path
        self.surface_gripper_prim_path = surface_gripper_prim_path
        self._write_status_to_usd = write_status_to_usd
        self._interface = None
        self._initialized = False

        # ParallelGripper 호환용 빈 배열
        self.joint_opened_positions = np.empty(
            0,
            dtype=np.float64,
        )
        self.joint_closed_positions = np.empty(
            0,
            dtype=np.float64,
        )
        self.joint_dof_indicies = np.empty(
            0,
            dtype=np.int64,
        )

    def initialize(
        self,
        physics_sim_view=None,
        articulation_apply_action_func=None,
        get_joint_positions_func=None,
        set_joint_positions_func=None,
        dof_names=None,
        **kwargs,
    ) -> None:
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(
            self.surface_gripper_prim_path
        )

        if not prim.IsValid():
            raise RuntimeError(
                "SurfaceGripper prim을 찾지 못했습니다: "
                f"{self.surface_gripper_prim_path}"
            )

        self._interface = (
            _surface_gripper
            .acquire_surface_gripper_interface()
        )

        if self._interface is None:
            raise RuntimeError(
                "SurfaceGripperInterface 획득 실패"
            )

        self._interface.set_write_to_usd(
            self._write_status_to_usd
        )
        self._initialized = True

        print(
            "[SurfaceGripper] initialized:",
            self.surface_gripper_prim_path,
            f"(type={prim.GetTypeName()})",
            flush=True,
        )

    def _require_initialized(self) -> None:
        if (
            not self._initialized
            or self._interface is None
        ):
            raise RuntimeError(
                "SurfaceGripperAdapter.initialize()가 "
                "먼저 호출되어야 합니다."
            )

    def forward(
        self,
        action: str,
    ) -> ArticulationAction:
        action_name = str(
            action
        ).strip().lower()

        if action_name == "close":
            self.close()
        elif action_name == "open":
            self.open()
        else:
            raise ValueError(
                "지원하지 않는 gripper action: "
                f"{action!r}. "
                "'open' 또는 'close'만 사용할 수 있습니다."
            )

        return ArticulationAction()

    def close(self) -> bool:
        self._require_initialized()

        result = bool(
            self._interface.close_gripper(
                self.surface_gripper_prim_path
            )
        )

        print(
            f"[SurfaceGripper] CLOSE -> {result}, "
            f"path={self.surface_gripper_prim_path}, "
            f"status={self.get_status()}, "
            f"objects={self.get_gripped_objects()}",
            flush=True,
        )

        return result

    def open(self) -> bool:
        """
        해제 명령을 전달한다.

        반환값은 명령 호출 결과이며, 실제 attachment가 물리적으로
        제거되었는지는 is_released()로 별도 확인해야 한다.
        """
        self._require_initialized()

        result = bool(
            self._interface.open_gripper(
                self.surface_gripper_prim_path
            )
        )

        print(
            f"[SurfaceGripper] OPEN -> {result}, "
            f"path={self.surface_gripper_prim_path}, "
            f"status={self.get_status()}, "
            f"objects={self.get_gripped_objects()}",
            flush=True,
        )

        return result

    def get_status(self) -> str:
        self._require_initialized()

        status = (
            self._interface.get_gripper_status(
                self.surface_gripper_prim_path
            )
        )

        return str(status)

    def get_gripped_objects(
        self,
    ) -> list[str]:
        self._require_initialized()

        return list(
            self._interface.get_gripped_objects(
                self.surface_gripper_prim_path
            )
        )

    def is_released(self) -> bool:
        """
        현재 이 Surface Gripper에 연결된 물체가 없는지 확인한다.

        status 문자열보다 실제 gripped object 목록을 기준으로
        판정한다.
        """
        return (
            len(self.get_gripped_objects())
            == 0
        )

    def get_release_state(
        self,
    ) -> dict:
        objects = self.get_gripped_objects()

        return {
            "path": self.surface_gripper_prim_path,
            "status": self.get_status(),
            "objects": objects,
            "released": len(objects) == 0,
        }

    def set_joint_positions(
        self,
        positions,
    ) -> None:
        return None

    def get_joint_positions(
        self,
    ) -> np.ndarray:
        return np.empty(
            0,
            dtype=np.float64,
        )

    def post_reset(self) -> None:
        if self._initialized:
            self.open()

    def reset(self) -> None:
        self.post_reset()
from __future__ import annotations

from typing import Iterable

import numpy as np
from isaacsim.core.utils.types import ArticulationAction

from surface_gripper_adapter import SurfaceGripperAdapter


class DualSurfaceGripperAdapter:
    """두 개의 Isaac SurfaceGripper를 하나의 PickPlace gripper처럼 제어한다."""

    def __init__(
        self,
        end_effector_prim_path: str,
        surface_gripper_prim_paths: Iterable[str],
        *,
        write_status_to_usd: bool = True,
    ) -> None:
        paths = list(
            surface_gripper_prim_paths
        )

        if len(paths) != 2:
            raise ValueError(
                "SurfaceGripper 경로는 정확히 "
                f"2개여야 합니다: {paths}"
            )

        self.end_effector_prim_path = (
            end_effector_prim_path
        )
        self.surface_gripper_prim_paths = paths

        self._grippers = [
            SurfaceGripperAdapter(
                end_effector_prim_path=(
                    end_effector_prim_path
                ),
                surface_gripper_prim_path=path,
                write_status_to_usd=(
                    write_status_to_usd
                ),
            )
            for path in paths
        ]

        self._initialized = False

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
        **kwargs,
    ) -> None:
        for gripper in self._grippers:
            gripper.initialize(
                **kwargs
            )

        self._initialized = True

        print(
            "[DualSurfaceGripper] initialized:",
            self.surface_gripper_prim_paths,
            flush=True,
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
                f"{action!r}"
            )

        return ArticulationAction()

    def _run_all(
        self,
        action_name: str,
    ) -> list[bool]:
        """
        한쪽에서 예외가 발생해도 다른 쪽 명령을 계속 실행한다.
        """
        results: list[bool] = []

        for index, gripper in enumerate(
            self._grippers
        ):
            try:
                if action_name == "close":
                    result = gripper.close()
                elif action_name == "open":
                    result = gripper.open()
                else:
                    raise ValueError(
                        action_name
                    )

            except Exception as error:
                print(
                    "[DualSurfaceGripper] "
                    f"{action_name.upper()}[{index}] "
                    f"error: {error}",
                    flush=True,
                )
                result = False

            results.append(
                bool(result)
            )

        return results

    def close(self) -> bool:
        results = self._run_all(
            "close"
        )

        print(
            "[DualSurfaceGripper] CLOSE:",
            results,
            "objects=",
            self.get_gripped_objects(),
            flush=True,
        )

        return all(results)

    def open(self) -> bool:
        results = self._run_all(
            "open"
        )

        print(
            "[DualSurfaceGripper] OPEN:",
            results,
            "states=",
            self.get_release_states(),
            flush=True,
        )

        return all(results)

    def get_status(
        self,
    ) -> list[str]:
        return [
            gripper.get_status()
            for gripper in self._grippers
        ]

    def get_gripped_objects(
        self,
    ) -> list[list[str]]:
        return [
            gripper.get_gripped_objects()
            for gripper in self._grippers
        ]

    def get_release_states(
        self,
    ) -> list[dict]:
        return [
            gripper.get_release_state()
            for gripper in self._grippers
        ]

    def is_fully_released(
        self,
    ) -> bool:
        """
        두 Surface Gripper 모두 attachment 목록이 비어 있는지 확인한다.
        """
        return all(
            gripper.is_released()
            for gripper in self._grippers
        )

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
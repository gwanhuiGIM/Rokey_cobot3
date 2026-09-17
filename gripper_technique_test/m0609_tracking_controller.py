# m0609_tracking_controller.py

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

from isaacsim.core.utils.types import ArticulationAction

from m0609_curobo_controller import M0609CuroboController


class M0609TrackingController:
    """
    /hand_xyz 목표를 cuRobo MPC로 처리하고
    M0609 관절 명령으로 적용한다.
    """

    def __init__(
        self,
        *,
        robot,
        robot_config_path: str,
        base_position: Sequence[float],
        base_yaw_deg: float,
        tool_orientation: Sequence[float],
        z_min: float,
        z_max: float,
        max_joint_step: float,
        use_mpc: bool = True,
    ) -> None:
        self._robot = robot

        self._base_position = np.asarray(
            base_position,
            dtype=np.float64,
        )

        self._tool_orientation = np.asarray(
            tool_orientation,
            dtype=np.float64,
        )

        self._z_min = float(z_min)
        self._z_max = float(z_max)
        self._max_joint_step = float(max_joint_step)

        if self._base_position.shape != (3,):
            raise ValueError(
                "base_position must contain 3 values"
            )

        if self._tool_orientation.shape != (4,):
            raise ValueError(
                "tool_orientation must contain 4 values"
            )

        print("[TrackingController] cuRobo 초기화 시작")

        self._curobo = M0609CuroboController(
            robot_config_path=robot_config_path,
            base_position=tuple(
                self._base_position
            ),
            base_yaw_deg=float(base_yaw_deg),
            use_mpc=bool(use_mpc),
        )

        print("[TrackingController] cuRobo 초기화 완료")

        self._joint_indices: Optional[np.ndarray] = None
        self._last_target: Optional[np.ndarray] = None

    @property
    def last_target(self) -> Optional[np.ndarray]:
        if self._last_target is None:
            return None

        return self._last_target.copy()

    def _create_joint_mapping(self) -> None:
        robot_dof_names = list(
            self._robot.dof_names
        )

        missing = [
            joint_name
            for joint_name in self._curobo.joint_names
            if joint_name not in robot_dof_names
        ]

        if missing:
            raise RuntimeError(
                "Isaac Sim 로봇에서 cuRobo 관절을 "
                f"찾지 못했습니다: {missing}\n"
                f"Isaac DOF: {robot_dof_names}\n"
                f"cuRobo joints: {self._curobo.joint_names}"
            )

        self._joint_indices = np.asarray(
            [
                robot_dof_names.index(joint_name)
                for joint_name
                in self._curobo.joint_names
            ],
            dtype=np.int64,
        )

        print(
            "[TrackingController] 관절 매핑 완료:"
            f" {self._curobo.joint_names}"
            f" -> {self._joint_indices.tolist()}",
            flush=True,
        )

    def _get_curobo_joints(self) -> np.ndarray:
        if self._joint_indices is None:
            self._create_joint_mapping()

        full_joint_positions = np.asarray(
            self._robot.get_joint_positions(),
            dtype=np.float64,
        )

        return full_joint_positions[
            self._joint_indices
        ]

    def _apply_curobo_joints(
        self,
        joint_positions: np.ndarray,
    ) -> None:
        if self._joint_indices is None:
            self._create_joint_mapping()

        action = ArticulationAction(
            joint_positions=np.asarray(
                joint_positions,
                dtype=np.float64,
            ),
            joint_indices=self._joint_indices,
        )

        self._robot.apply_action(
            action
        )

    def _reach_max_at_z(
        self,
        z_position: float,
    ) -> float:
        """
        팀원 코드와 동일한 높이별 수평 도달 반경 계산.
        """
        radius = (
            0.78
            - 0.467
            * (float(z_position) - 1.10)
        )

        return max(
            radius * 0.9,
            0.1,
        )

    def _clamp_to_reachable(
        self,
        target: Sequence[float],
    ) -> np.ndarray:
        clamped = np.asarray(
            target,
            dtype=np.float64,
        ).copy()

        if clamped.shape != (3,):
            raise ValueError(
                "target must contain 3 values"
            )

        clamped[2] = np.clip(
            clamped[2],
            self._z_min,
            self._z_max,
        )

        max_radius = self._reach_max_at_z(
            clamped[2]
        )

        horizontal_delta = (
            clamped[:2]
            - self._base_position[:2]
        )

        horizontal_distance = float(
            np.linalg.norm(horizontal_delta)
        )

        if (
            horizontal_distance > max_radius
            and horizontal_distance > 0.0
        ):
            clamped[:2] = (
                self._base_position[:2]
                + (
                    horizontal_delta
                    / horizontal_distance
                    * max_radius
                )
            )

        return clamped

    def _limit_joint_step(
        self,
        current_joints: np.ndarray,
        next_joints: np.ndarray,
    ) -> np.ndarray:
        current = np.asarray(
            current_joints,
            dtype=np.float64,
        )

        target = np.asarray(
            next_joints,
            dtype=np.float64,
        )

        if current.shape != target.shape:
            raise RuntimeError(
                "cuRobo 관절 결과 크기가 현재 관절과 다릅니다: "
                f"current={current.shape}, next={target.shape}"
            )

        delta = target - current

        limited_delta = np.clip(
            delta,
            -self._max_joint_step,
            self._max_joint_step,
        )

        return current + limited_delta

    def update(
        self,
        target_position: Sequence[float],
    ) -> np.ndarray:
        """
        목표 위치를 한 프레임 처리한다.

        반환값은 실제 cuRobo에 전달된 clamp 적용 목표 좌표다.
        """
        target = np.asarray(
            target_position,
            dtype=np.float64,
        )

        if target.shape != (3,):
            raise ValueError(
                "target_position must contain 3 values"
            )

        if not np.all(
            np.isfinite(target)
        ):
            raise ValueError(
                f"Invalid tracking target: {target}"
            )

        target = self._clamp_to_reachable(
            target
        )

        current_joints = (
            self._get_curobo_joints()
        )

        self._curobo.mpc_set_goal(
            target,
            self._tool_orientation,
            current_joints,
        )

        next_joints = (
            self._curobo.mpc_step(
                current_joints
            )
        )

        if next_joints is None:
            return target

        next_joints = self._limit_joint_step(
            current_joints,
            next_joints,
        )

        self._apply_curobo_joints(
            next_joints
        )

        self._last_target = target

        return target

    def reset(self) -> None:
        """
        Play 재시작 시 관절 매핑과 마지막 목표를 초기화한다.

        팀원 M0609CuroboController에 reset()이 구현돼 있으면
        함께 호출한다.
        """
        self._joint_indices = None
        self._last_target = None

        reset_method = getattr(
            self._curobo,
            "reset",
            None,
        )

        if callable(reset_method):
            reset_method()
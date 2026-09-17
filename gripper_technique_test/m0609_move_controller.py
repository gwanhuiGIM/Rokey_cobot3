from __future__ import annotations

from typing import Optional

import numpy as np

from isaacsim.core.utils.rotations import quat_to_rot_matrix

from m0609_rmpflow_controller import RMPFlowController


class M0609MoveController:
    """M0609의 한 목표 자세 이동만 담당하는 가장 기본적인 RMPFlow 컨트롤러.

    목표 위치는 실제 TCP(예: 두 흡착점 중앙)의 월드 좌표로 받는다.
    RMPFlow가 제어하는 프레임이 link_6이므로, tcp_offset_local을 이용해
    TCP 목표를 link_6 목표로 변환한다.

    Quaternion 순서는 Isaac Sim 기본 형식인 [w, x, y, z]를 사용한다.
    """

    def __init__(
        self,
        name: str,
        robot_articulation,
        urdf_path: str,
        robot_description_path: str,
        rmpflow_config_path: str,
        end_effector_frame_name: str = "link_6",
        tcp_offset_local: Optional[np.ndarray] = None,
        position_tolerance: float = 0.01,
    ) -> None:
        self._robot = robot_articulation
        self._tcp_offset_local = np.asarray(
            tcp_offset_local
            if tcp_offset_local is not None
            else np.zeros(3),
            dtype=np.float64,
        )

        if self._tcp_offset_local.shape != (3,):
            raise ValueError("tcp_offset_local은 [x, y, z] 형태여야 합니다.")

        self._position_tolerance = float(position_tolerance)
        if self._position_tolerance <= 0.0:
            raise ValueError("position_tolerance은 0보다 커야 합니다.")

        self._rmpflow = RMPFlowController(
            name=name + "_rmpflow",
            robot_articulation=robot_articulation,
            urdf_path=urdf_path,
            robot_description_path=robot_description_path,
            rmpflow_config_path=rmpflow_config_path,
            end_effector_frame_name=end_effector_frame_name,
        )

        self._target_tcp_position: Optional[np.ndarray] = None
        self._target_orientation: Optional[np.ndarray] = None

    def set_target(
        self,
        position: np.ndarray,
        orientation: np.ndarray,
    ) -> None:
        """이동할 TCP 목표 자세를 저장한다.

        position:
            TCP의 월드 위치 [x, y, z], 단위 m
        orientation:
            link_6의 목표 quaternion [w, x, y, z]
        """
        position = np.asarray(position, dtype=np.float64)
        orientation = np.asarray(orientation, dtype=np.float64)

        if position.shape != (3,):
            raise ValueError("position은 [x, y, z] 형태여야 합니다.")
        if orientation.shape != (4,):
            raise ValueError("orientation은 [w, x, y, z] 형태여야 합니다.")

        norm = np.linalg.norm(orientation)
        if norm < 1e-8:
            raise ValueError("orientation quaternion의 크기가 0입니다.")

        self._target_tcp_position = position.copy()
        self._target_orientation = orientation / norm

    def forward(self):
        """현재 목표로 이동하기 위한 다음 ArticulationAction을 계산한다."""
        if self._target_tcp_position is None or self._target_orientation is None:
            raise RuntimeError("set_target()을 먼저 호출해야 합니다.")

        target_link_position = self._tcp_target_to_link_target(
            self._target_tcp_position,
            self._target_orientation,
        )

        return self._rmpflow.forward(
            target_end_effector_position=target_link_position,
            target_end_effector_orientation=self._target_orientation,
        )

    def is_done(self) -> bool:
        """현재 TCP 위치가 목표 위치 허용 오차 안에 들어왔는지 확인한다."""
        if self._target_tcp_position is None:
            return False

        current_tcp_position = self.get_current_tcp_position()
        error = np.linalg.norm(
            current_tcp_position - self._target_tcp_position
        )
        return bool(error <= self._position_tolerance)

    def get_position_error(self) -> float:
        """현재 TCP와 목표 TCP 사이 거리 오차를 m 단위로 반환한다."""
        if self._target_tcp_position is None:
            raise RuntimeError("set_target()을 먼저 호출해야 합니다.")

        return float(
            np.linalg.norm(
                self.get_current_tcp_position()
                - self._target_tcp_position
            )
        )

    def get_current_tcp_position(self) -> np.ndarray:
        """현재 link_6 자세와 로컬 TCP offset으로 TCP 월드 위치를 계산한다."""
        link_position, link_orientation = (
            self._robot.end_effector.get_world_pose()
        )

        rotation = quat_to_rot_matrix(
            np.asarray(link_orientation, dtype=np.float64)
        )
        tcp_world_offset = rotation @ self._tcp_offset_local

        return (
            np.asarray(link_position, dtype=np.float64)
            + tcp_world_offset
        )

    def reset(self) -> None:
        """저장된 목표와 RMPFlow 내부 상태를 초기화한다."""
        self._target_tcp_position = None
        self._target_orientation = None
        self._rmpflow.reset()

    def _tcp_target_to_link_target(
        self,
        tcp_position: np.ndarray,
        link_orientation: np.ndarray,
    ) -> np.ndarray:
        """TCP 목표를 RMPFlow가 제어하는 link_6 목표 위치로 변환한다."""
        rotation = quat_to_rot_matrix(link_orientation)
        tcp_world_offset = rotation @ self._tcp_offset_local
        return tcp_position - tcp_world_offset

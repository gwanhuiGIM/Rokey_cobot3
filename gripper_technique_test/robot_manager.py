# robot_manager.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Mapping, Optional, Sequence, Tuple

import numpy as np

from operation_registry import OperationRegistry
from robot_runtime import RobotProfile, RobotTask
from tool_state_manager import ToolStateManager


@dataclass
class ManagedRobot:
    profile: RobotProfile
    state_machine: object
    active_task: Optional[RobotTask] = None
    last_state: str = "IDLE"
    last_place_release_sequence: int = 0

    # TRACKING 중 교체 요청이 들어오면,
    # 기존 도구를 원복한 뒤 같은 로봇이 수행할 작업.
    pending_replacement_tray_id: Optional[int] = None

    # 현재 수행 중인 작업과 교체 대기 작업의 레지스트리 ID.
    active_operation_id: Optional[str] = None
    pending_replacement_operation_id: Optional[str] = None


class RobotManager:
    """
    여러 로봇의 작업 배정과 트레이 PICK/PLACE 구역 락을 관리한다.

    삭제된 기능:
    - TRANSIT_2 공용 경로
    - 경로 우선순위
    - fallback 경로
    - 경유지 락
    - route owner
    - 경유지 대기 큐

    유지되는 기능:
    - 두 로봇 모두 Tray 0~5 접근
    - 담당 트레이 우선 로봇 선택
      · Tray 0/2/4: Robot A 우선
      · Tray 1/3/5: Robot B 우선
    - 우선 로봇이 바쁘면 다른 IDLE 로봇이 대신 수행
    - 신규 트레이 PICK/PLACE 구역 락
    - 신규 락은 큐 없이 단순 차단
    """

    def __init__(
        self,
        *,
        tray_positions: Mapping[
            int,
            Sequence[float],
        ],
        shared_trays: Sequence[int] = (),
        tool_state_manager: Optional[
            ToolStateManager
        ] = None,
    ) -> None:
        self._robots: Dict[str, ManagedRobot] = {}
        self._tool_state_manager = (
            tool_state_manager
        )
        self._operation_registry = OperationRegistry()

        self._tray_positions = {
            int(tray_id): np.asarray(
                position,
                dtype=np.float64,
            ).copy()
            for tray_id, position
            in tray_positions.items()
        }

        for tray_id, position in self._tray_positions.items():
            if position.shape != (3,):
                raise ValueError(
                    f"Tray {tray_id} position "
                    "must have shape (3,)"
                )

            if not np.all(np.isfinite(position)):
                raise ValueError(
                    f"Tray {tray_id} position "
                    "must be finite"
                )

        self._shared_trays = frozenset(
            int(value)
            for value in shared_trays
        )

        unknown_shared_trays = (
            self._shared_trays
            - self._tray_positions.keys()
        )

        if unknown_shared_trays:
            raise ValueError(
                "공용 구역 대상 트레이가 "
                "tray_positions에 없습니다: "
                f"{sorted(unknown_shared_trays)}"
            )

        # 신규 PICK/PLACE 구역 락.
        # 큐를 두지 않고 현재 소유자만 저장한다.
        self._tray_zone_owner: Optional[str] = None

    def register_robot(
        self,
        *,
        profile: RobotProfile,
        state_machine,
    ) -> None:
        robot_id = profile.robot_id

        if robot_id in self._robots:
            raise ValueError(
                f"Robot {robot_id} is already registered"
            )

        unknown_trays = (
            profile.reachable_trays
            - self._tray_positions.keys()
        )

        if unknown_trays:
            raise KeyError(
                f"Robot {robot_id} reachable trays "
                "are not registered: "
                f"{sorted(unknown_trays)}"
            )

        self._robots[robot_id] = ManagedRobot(
            profile=profile,
            state_machine=state_machine,
            last_state=state_machine.state_name,
            last_place_release_sequence=int(
                getattr(
                    state_machine,
                    "place_release_sequence",
                    0,
                )
            ),
        )

        print(
            f"[Manager] registered robot={robot_id}, "
            f"reachable={sorted(profile.reachable_trays)}, "
            f"route={profile.route_id}, "
            f"transit={profile.transit_position.round(4)}, "
            f"selection_position="
            f"{profile.selection_position.round(4)}",
            flush=True,
        )

    def _can_accept_command(
        self,
        managed: ManagedRobot,
    ) -> Tuple[bool, str]:
        current_state = (
            managed.state_machine.state_name
        )

        if current_state != "IDLE":
            return (
                False,
                f"Robot {managed.profile.robot_id} "
                f"is busy: state={current_state}",
            )

        if managed.active_task is not None:
            return (
                False,
                f"Robot {managed.profile.robot_id} "
                "is busy: active_task exists",
            )

        return True, "accepted"

    def _distance_to_tray(
        self,
        *,
        managed: ManagedRobot,
        tray_id: int,
    ) -> float:
        tray_position = self._tray_positions[tray_id]
        robot_position = (
            managed.profile.selection_position
        )

        return float(
            np.linalg.norm(
                tray_position - robot_position
            )
        )

    @staticmethod
    def _preferred_robot_priority(
        *,
        robot_id: str,
        tray_id: int,
    ) -> int:
        """
        두 로봇이 모두 명령을 받을 수 있을 때 적용되는
        트레이별 담당 우선순위.

        Tray 0, 2, 4 -> Robot A 우선
        Tray 1, 3, 5 -> Robot B 우선

        우선 로봇이 바쁘면 후보 목록에서 제외되므로,
        다른 로봇이 자동으로 대신 수행한다.
        """
        normalized = str(robot_id).strip().upper()
        preferred_robot = (
            "A"
            if int(tray_id) % 2 == 0
            else "B"
        )

        if normalized == preferred_robot:
            return 0

        if normalized in {"A", "B"}:
            return 1

        return 2

    def _find_candidate_robots(
        self,
        tray_id: int,
    ) -> list[ManagedRobot]:
        candidates: list[ManagedRobot] = []

        for managed in self._robots.values():
            if (
                tray_id
                not in managed.profile.reachable_trays
            ):
                continue

            can_accept, _ = self._can_accept_command(
                managed
            )

            if not can_accept:
                continue

            candidates.append(managed)

        candidates.sort(
            key=lambda managed: (
                self._preferred_robot_priority(
                    robot_id=(
                        managed.profile.robot_id
                    ),
                    tray_id=tray_id,
                ),
                self._distance_to_tray(
                    managed=managed,
                    tray_id=tray_id,
                ),
                managed.profile.robot_id,
            )
        )

        return candidates

    def _get_tool_id_for_tray(
        self,
        tray_id: int,
    ) -> Optional[str]:
        if self._tool_state_manager is None:
            return None

        return self._tool_state_manager.get_tool_for_tray(
            int(tray_id)
        )

    def _create_operation(
        self,
        *,
        tray_id: int,
        robot_id: Optional[str],
        status: str,
        is_replacement: bool,
    ) -> str:
        return self._operation_registry.create_operation(
            tray_id=tray_id,
            tool_id=self._get_tool_id_for_tray(
                tray_id
            ),
            robot_id=robot_id,
            status=status,
            is_replacement=is_replacement,
        )

    def _update_active_operation_state(
        self,
        *,
        managed: ManagedRobot,
        state_name: str,
    ) -> None:
        operation_id = managed.active_operation_id

        if operation_id is None:
            return

        self._operation_registry.update_operation(
            operation_id,
            status=state_name,
            robot_id=managed.profile.robot_id,
        )

    def _find_tracking_replacement_candidates(
        self,
        tray_id: int,
    ) -> list[ManagedRobot]:
        """
        IDLE 로봇이 없고 두 로봇이 TRACKING 중일 때
        교체 작업을 맡을 후보를 찾는다.

        교체 요청에서는 담당 홀짝 우선순위를 사용하지 않고,
        새 도구 트레이까지의 거리만 비교한다.
        """
        candidates: list[ManagedRobot] = []

        for managed in self._robots.values():
            if (
                tray_id
                not in managed.profile.reachable_trays
            ):
                continue

            if (
                managed.state_machine.state_name
                != "TRACKING"
            ):
                continue

            if managed.active_task is None:
                continue

            if (
                managed.pending_replacement_tray_id
                is not None
            ):
                continue

            candidates.append(managed)

        candidates.sort(
            key=lambda managed: (
                self._distance_to_tray(
                    managed=managed,
                    tray_id=tray_id,
                ),
                managed.profile.robot_id,
            )
        )

        return candidates

    def _queue_tracking_replacement(
        self,
        *,
        managed: ManagedRobot,
        tray_id: int,
    ) -> Tuple[bool, str]:
        robot_id = managed.profile.robot_id

        accepted, message = (
            managed.state_machine
            .request_place_from_manager(
                skip_return_home=True
            )
        )

        if not accepted:
            return False, message

        managed.pending_replacement_tray_id = (
            int(tray_id)
        )
        managed.pending_replacement_operation_id = (
            self._create_operation(
                tray_id=tray_id,
                robot_id=robot_id,
                status="PENDING_REPLACEMENT",
                is_replacement=True,
            )
        )

        distance = self._distance_to_tray(
            managed=managed,
            tray_id=tray_id,
        )

        result = (
            f"Robot {robot_id} selected for replacement: "
            f"current tool PLACE, skip RETURN_HOME, "
            f"then tray {tray_id}, "
            f"distance={distance:.4f} m"
        )

        print(
            f"[Manager REPLACE] {result}",
            flush=True,
        )

        return True, result

    def _try_assign_pending_replacement(
        self,
        managed: ManagedRobot,
    ) -> None:
        tray_id = (
            managed.pending_replacement_tray_id
        )

        if tray_id is None:
            return

        if (
            managed.state_machine.state_name
            != "IDLE"
        ):
            return

        if managed.active_task is not None:
            return

        if (
            self._tool_state_manager is not None
            and self._tool_state_manager.get_tool_for_tray(
                tray_id
            ) is None
        ):
            print(
                "[Manager REPLACE] pending request cancelled: "
                f"tray {tray_id} is empty",
                flush=True,
            )
            if (
                managed.pending_replacement_operation_id
                is not None
            ):
                self._operation_registry.cancel_operation(
                    managed.pending_replacement_operation_id
                )

            managed.pending_replacement_tray_id = None
            managed.pending_replacement_operation_id = None
            return

        task = self._create_task(
            managed=managed,
            tray_id=tray_id,
        )

        if (
            task.uses_shared_zone
            and not self._try_acquire_tray_zone(
                robot_id=managed.profile.robot_id
            )
        ):
            # 다른 로봇이 트레이 구역을 사용하는 동안은
            # 요청을 유지하고 다음 step에서 다시 시도한다.
            return

        accepted, message = (
            managed.state_machine.assign_task(
                task
            )
        )

        if not accepted:
            if task.uses_shared_zone:
                self._release_tray_zone(
                    robot_id=managed.profile.robot_id
                )
            return

        managed.active_task = task
        managed.active_operation_id = (
            managed.pending_replacement_operation_id
        )

        if managed.active_operation_id is None:
            managed.active_operation_id = (
                self._create_operation(
                    tray_id=tray_id,
                    robot_id=managed.profile.robot_id,
                    status="PICK",
                    is_replacement=True,
                )
            )
        else:
            self._operation_registry.update_operation(
                managed.active_operation_id,
                status="PICK",
                robot_id=managed.profile.robot_id,
            )

        managed.pending_replacement_tray_id = None
        managed.pending_replacement_operation_id = None

        print(
            "[Manager REPLACE] pending task assigned: "
            f"robot={managed.profile.robot_id}, "
            f"tray={tray_id}",
            flush=True,
        )

    # ========================================================
    # 신규 트레이 PICK/PLACE 구역 락
    # ========================================================

    def _try_acquire_tray_zone(
        self,
        *,
        robot_id: str,
    ) -> bool:
        robot_id = str(robot_id).strip().upper()
        owner = self._tray_zone_owner

        if owner is None:
            self._tray_zone_owner = robot_id

            print(
                "[Manager TRAY-ZONE] acquired: "
                f"robot={robot_id}",
                flush=True,
            )
            return True

        if owner == robot_id:
            return True

        return False

    def _release_tray_zone(
        self,
        *,
        robot_id: str,
    ) -> None:
        robot_id = str(robot_id).strip().upper()
        owner = self._tray_zone_owner

        if owner is None:
            return

        if owner != robot_id:
            print(
                "[Manager TRAY-ZONE] "
                "release ignored: "
                f"requester={robot_id}, "
                f"owner={owner}",
                flush=True,
            )
            return

        self._tray_zone_owner = None

        print(
            "[Manager TRAY-ZONE] released: "
            f"robot={robot_id}",
            flush=True,
        )

    def _task_uses_tray_zone(
        self,
        managed: ManagedRobot,
    ) -> bool:
        task = managed.active_task

        return bool(
            task is not None
            and task.uses_shared_zone
        )

    def _must_hold_tray_zone_before_step(
        self,
        managed: ManagedRobot,
    ) -> bool:
        if not self._task_uses_tray_zone(managed):
            return False

        state_name = (
            managed.state_machine.state_name
        )

        if (
            state_name == "IDLE"
            and managed.active_task is not None
        ):
            return True

        return state_name in {
            "PICK_APPROACH",
            "PICK_TRANSPORT",
            "PLACE",
        }

    def _release_tray_zone_after_transition(
        self,
        *,
        managed: ManagedRobot,
        previous_state: str,
        current_state: str,
    ) -> None:
        robot_id = managed.profile.robot_id

        if (
            previous_state == "PICK_TRANSPORT"
            and current_state == "TRACKING"
        ):
            self._release_tray_zone(
                robot_id=robot_id
            )
            return

        if (
            previous_state == "PICK_APPROACH"
            and current_state == "RETURN_HOME"
        ):
            self._release_tray_zone(
                robot_id=robot_id
            )
            return

        if (
            previous_state == "PLACE"
            and current_state == "RETURN_HOME"
        ):
            self._release_tray_zone(
                robot_id=robot_id
            )

    # ========================================================
    # 작업 배정
    # ========================================================

    def _create_task(
        self,
        *,
        managed: ManagedRobot,
        tray_id: int,
    ) -> RobotTask:
        profile = managed.profile

        return RobotTask.create(
            tray_id=tray_id,
            route_id=profile.route_id,
            transit_position=(
                profile.transit_position
            ),
            transit_orientation=(
                profile.transit_orientation
            ),
            joint1_delta_rad=(
                profile.tracking_joint1_delta_rad
            ),
            uses_shared_zone=(
                tray_id in self._shared_trays
            ),
        )

    def request_pick_command(
        self,
        tray_index: int,
    ) -> Tuple[bool, str]:
        tray_index = int(tray_index)

        if tray_index not in self._tray_positions:
            message = (
                f"Unknown tray: {tray_index}"
            )

            print(
                f"[Manager BLOCK] {message}",
                flush=True,
            )
            return False, message

        if (
            self._tool_state_manager is not None
            and self._tool_state_manager.get_tool_for_tray(
                tray_index
            ) is None
        ):
            message = (
                f"Tray {tray_index} is empty"
            )

            print(
                f"[Manager BLOCK] {message}",
                flush=True,
            )
            return False, message

        candidates = self._find_candidate_robots(
            tray_index
        )

        if not candidates:
            tracking_candidates = (
                self._find_tracking_replacement_candidates(
                    tray_index
                )
            )

            if tracking_candidates:
                return self._queue_tracking_replacement(
                    managed=tracking_candidates[0],
                    tray_id=tray_index,
                )

            message = (
                "No IDLE or TRACKING robot can accept tray "
                f"{tray_index}"
            )

            print(
                f"[Manager BLOCK] {message}",
                flush=True,
            )
            return False, message

        # 큐를 사용하지 않는다.
        # 구역 점유 중이면 신규 PICK 요청을 즉시 차단한다.
        if (
            tray_index in self._shared_trays
            and self._tray_zone_owner is not None
        ):
            message = (
                "Tray PICK/PLACE zone is occupied: "
                f"owner={self._tray_zone_owner}"
            )

            print(
                "[Manager BLOCK] "
                f"tray={tray_index}, "
                f"{message}",
                flush=True,
            )
            return False, message

        for managed in candidates:
            robot_id = managed.profile.robot_id

            task = self._create_task(
                managed=managed,
                tray_id=tray_index,
            )

            tray_zone_acquired = False

            if task.uses_shared_zone:
                tray_zone_acquired = (
                    self._try_acquire_tray_zone(
                        robot_id=robot_id
                    )
                )

                if not tray_zone_acquired:
                    return (
                        False,
                        "Tray PICK/PLACE zone "
                        "is occupied: "
                        f"owner="
                        f"{self._tray_zone_owner}",
                    )

            accepted, message = (
                managed.state_machine.assign_task(
                    task
                )
            )

            if not accepted:
                if tray_zone_acquired:
                    self._release_tray_zone(
                        robot_id=robot_id
                    )
                continue

            managed.active_task = task
            managed.active_operation_id = (
                self._create_operation(
                    tray_id=tray_index,
                    robot_id=robot_id,
                    status="PICK",
                    is_replacement=False,
                )
            )

            distance = self._distance_to_tray(
                managed=managed,
                tray_id=tray_index,
            )

            return (
                True,
                f"Robot {robot_id} assigned "
                f"tray {tray_index}, "
                f"route={task.route_id}, "
                f"distance={distance:.4f} m, "
                f"joint1_delta="
                f"{task.joint1_delta_rad:.4f} rad",
            )

        message = (
            f"No robot accepted tray "
            f"{tray_index}"
        )

        print(
            f"[Manager BLOCK] {message}",
            flush=True,
        )
        return False, message

    def get_tool_command_status(
        self,
        tool_id: str,
    ) -> dict:
        normalized_tool_id = str(tool_id).strip()

        result = {
            "tool_id": normalized_tool_id,
            "status": "UNKNOWN",
            "robot_id": None,
            "operation_id": None,
        }

        if not normalized_tool_id:
            return result

        operation = (
            self._operation_registry
            .get_operation_for_tool(
                normalized_tool_id
            )
        )

        if operation is not None:
            result["operation_id"] = operation.get(
                "operation_id"
            )
            result["robot_id"] = operation.get(
                "robot_id"
            )

            operation_status = str(
                operation.get("status", "")
            ).strip().upper()

            if operation_status == "PENDING_REPLACEMENT":
                result["status"] = "PENDING_REPLACEMENT"
                return result

            managed = self._robots.get(
                str(operation.get("robot_id", "")).upper()
            )

            if managed is not None:
                state_name = (
                    managed.state_machine.state_name
                )

                status_map = {
                    "PICK_APPROACH": "PICKING",
                    "PICK_TRANSPORT": "TRANSPORTING",
                    "TRACKING": "TRACKING",
                    "PLACE": "RETURNING",
                    "RETURN_HOME": "RETURNING",
                }

                if state_name in status_map:
                    result["status"] = status_map[state_name]
                    return result

            if operation_status:
                result["status"] = operation_status
                return result

        if self._tool_state_manager is None:
            return result

        tray_id = (
            self._tool_state_manager
            .get_tray_for_tool(
                normalized_tool_id
            )
        )

        if tray_id is not None:
            result["status"] = "AVAILABLE"
            result["tray_id"] = tray_id

        return result

    def _duplicate_request_message(
        self,
        *,
        tool_id: str,
        status: str,
    ) -> str:
        messages = {
            "PICKING": f"{tool_id} is already being picked",
            "TRANSPORTING": f"{tool_id} is already being transported",
            "TRACKING": f"{tool_id} is already in tracking",
            "RETURNING": f"{tool_id} is already being returned",
            "PENDING_REPLACEMENT": (
                f"{tool_id} is already pending as a replacement"
            ),
        }
        return messages.get(
            status,
            f"{tool_id} already has an active operation: {status}",
        )

    def request_tool_command(
        self,
        tool_id: str,
    ) -> Tuple[bool, str]:
        normalized_tool_id = str(tool_id).strip()

        if not normalized_tool_id:
            return False, "tool_id is empty"

        command_status = self.get_tool_command_status(
            normalized_tool_id
        )
        current_status = command_status["status"]

        if current_status in {
            "PICKING",
            "TRANSPORTING",
            "TRACKING",
            "RETURNING",
            "PENDING_REPLACEMENT",
        }:
            return (
                False,
                self._duplicate_request_message(
                    tool_id=normalized_tool_id,
                    status=current_status,
                ),
            )

        if self._tool_state_manager is None:
            return (
                False,
                "ToolStateManager is not configured",
            )

        tray_id = (
            self._tool_state_manager
            .get_tray_for_tool(normalized_tool_id)
        )

        if tray_id is None:
            return (
                False,
                f"Tool is not available on a tray: {normalized_tool_id}",
            )

        return self.request_pick_command(
            tray_id
        )

    def _find_managed_robot_for_operation(
        self,
        operation: dict,
    ) -> Optional[ManagedRobot]:
        robot_id = operation.get("robot_id")
        operation_id = operation.get("operation_id")

        if robot_id is None:
            return None

        managed = self._robots.get(
            str(robot_id).strip().upper()
        )

        if managed is None:
            return None

        if managed.state_machine.state_name not in {
            "PICK_APPROACH",
            "PICK_TRANSPORT",
            "TRACKING",
        }:
            return None

        if managed.active_task is None:
            return None

        if (
            operation_id is not None
            and managed.active_operation_id is not None
            and managed.active_operation_id != operation_id
        ):
            return None

        return managed

    def _find_robot_for_tool_return(
        self,
        tool_id: str,
    ) -> Optional[ManagedRobot]:
        normalized_tool_id = str(tool_id).strip()

        # 도구를 이미 집은 상태는 ToolStateManager의
        # HELD_BY_ROBOT 정보를 우선 사용한다.
        if self._tool_state_manager is not None:
            for robot_id, managed in self._robots.items():
                if managed.state_machine.state_name not in {
                    "PICK_TRANSPORT",
                    "TRACKING",
                }:
                    continue

                held_tool = (
                    self._tool_state_manager
                    .get_held_tool(robot_id)
                )

                if held_tool == normalized_tool_id:
                    return managed

        # PICK_APPROACH는 아직 트레이에 있으므로
        # 현재 active operation의 tool_id로 찾는다.
        operation = (
            self._operation_registry
            .get_operation_for_tool(
                normalized_tool_id
            )
        )

        if operation is None:
            return None

        return self._find_managed_robot_for_operation(
            operation
        )

    def _request_managed_robot_return(
        self,
        *,
        managed: ManagedRobot,
        tool_id: Optional[str],
        operation_id: Optional[str],
    ) -> Tuple[bool, str]:
        state_name = managed.state_machine.state_name

        if state_name not in {
            "PICK_APPROACH",
            "PICK_TRANSPORT",
            "TRACKING",
        }:
            return (
                False,
                f"Robot {managed.profile.robot_id} "
                f"cannot cancel/return in {state_name}",
            )

        if managed.active_task is None:
            return (
                False,
                f"Robot {managed.profile.robot_id} has no active task",
            )

        if managed.pending_replacement_tray_id is not None:
            return (
                False,
                f"Robot {managed.profile.robot_id} "
                "already has a replacement request",
            )

        accepted, message = (
            managed.state_machine
            .request_cancel_and_return_from_manager()
        )

        if not accepted:
            return False, message

        result = (
            "Cancel/return requested: "
            f"operation={operation_id}, "
            f"robot={managed.profile.robot_id}, "
            f"tool={tool_id}, "
            f"state={state_name}"
        )

        print(
            f"[Manager RETURN] {result}",
            flush=True,
        )

        return True, result

    def _request_operation_return(
        self,
        operation: dict,
    ) -> Tuple[bool, str]:
        managed = (
            self._find_managed_robot_for_operation(
                operation
            )
        )

        if managed is None:
            return (
                False,
                "Operation robot is not in a cancellable state",
            )

        return self._request_managed_robot_return(
            managed=managed,
            tool_id=operation.get("tool_id"),
            operation_id=operation.get("operation_id"),
        )

    def request_tool_return(
        self,
        tool_id: str,
    ) -> Tuple[bool, str]:
        """
        같은 기존 반납 토픽으로 PICK/TRACKING 작업을 취소한다.

        PICK_APPROACH:
            도구 미흡착 -> RETURN_HOME

        PICK_TRANSPORT / TRACKING:
            도구 보유 -> PLACE -> RETURN_HOME
        """
        normalized_tool_id = str(
            tool_id
        ).strip()

        if not normalized_tool_id:
            return False, "tool_id is empty"

        command_status = self.get_tool_command_status(
            normalized_tool_id
        )

        if command_status["status"] == "RETURNING":
            return (
                False,
                f"{normalized_tool_id} is already being returned",
            )

        if command_status["status"] == "AVAILABLE":
            return (
                False,
                f"{normalized_tool_id} is already on a tray",
            )

        managed = self._find_robot_for_tool_return(
            normalized_tool_id
        )

        if managed is None:
            return (
                False,
                f"No cancellable active operation for tool: "
                f"{normalized_tool_id}",
            )

        return self._request_managed_robot_return(
            managed=managed,
            tool_id=normalized_tool_id,
            operation_id=managed.active_operation_id,
        )

    def request_recent_tool_return(
        self,
    ) -> Tuple[bool, str]:
        """
        가장 최근 PICK_APPROACH/PICK_TRANSPORT/TRACKING 작업을
        기존 최근 반납 토픽으로 취소 또는 반환한다.
        """
        cancellable_states = {
            "PICK_APPROACH",
            "PICK_TRANSPORT",
            "TRACKING",
        }

        candidates = [
            managed
            for managed in self._robots.values()
            if (
                managed.state_machine.state_name
                in cancellable_states
                and managed.active_task is not None
            )
        ]

        if not candidates:
            return (
                False,
                "No cancellable active operation",
            )

        def recent_key(managed: ManagedRobot):
            operation_id = managed.active_operation_id or ""
            try:
                return int(
                    operation_id.rsplit("_", 1)[-1]
                )
            except (TypeError, ValueError):
                return -1

        managed = max(
            candidates,
            key=recent_key,
        )

        tool_id = None

        if self._tool_state_manager is not None:
            tool_id = (
                self._tool_state_manager
                .get_held_tool(
                    managed.profile.robot_id
                )
            )

        if tool_id is None and managed.active_operation_id is not None:
            try:
                operation = (
                    self._operation_registry
                    .get_operation(
                        managed.active_operation_id
                    )
                )
                tool_id = operation.get("tool_id")
            except KeyError:
                pass

        return self._request_managed_robot_return(
            managed=managed,
            tool_id=tool_id,
            operation_id=managed.active_operation_id,
        )

    def update_external_tool_detection(
        self,
        *,
        tool_id: str,
        position: Sequence[float],
        tray_id: Optional[int] = None,
        timestamp: Optional[float] = None,
    ) -> None:
        if self._tool_state_manager is None:
            raise RuntimeError(
                "ToolStateManager is not configured"
            )

        self._tool_state_manager.update_external_detection(
            tool_id=tool_id,
            position=position,
            tray_id=tray_id,
            timestamp=timestamp,
        )

    def is_safe_to_randomize(self) -> bool:
        """
        도구 재랜덤이 안전한지 검사한다.
        모든 로봇이 작업/도구운반 중이 아니어야 한다(이동 중 차단).
        """
        for managed in self._robots.values():
            if managed.active_task is not None:
                return False
            if managed.active_operation_id is not None:
                return False

        # 들고 있는 도구가 하나라도 있으면 안전하지 않다.
        if self._tool_state_manager is not None:
            for robot_id in self._robots:
                if (
                    self._tool_state_manager.get_held_tool(robot_id)
                    is not None
                ):
                    return False

        return True

    def randomize_partial_layout(
        self,
        *,
        min_count: int = 4,
        max_count: int = 6,
    ) -> Optional[Dict[str, object]]:
        """
        4~6개 도구만 무작위로 트레이에 배치한다(나머지는 missing).
        실제 Prim 이동/숨김은 호출 측(main)에서 tray_registry 로 적용한다.
        """
        if self._tool_state_manager is None:
            return None

        return (
            self._tool_state_manager
            .reset_random_partial_layout(
                min_count=min_count,
                max_count=max_count,
            )
        )

    def get_dashboard_status(self) -> dict:
        """
        웹 대시보드용 통합 상태 스냅샷.
        로봇 A/B 상태·들고있는 도구·작업, 트레이별 도구, missing 을 합친다.
        """
        robots = {}

        for robot_id, managed in self._robots.items():
            held_tool = None
            if self._tool_state_manager is not None:
                held_tool = self._tool_state_manager.get_held_tool(
                    robot_id
                )

            robots[robot_id] = {
                "state": managed.state_machine.state_name,
                "held_tool": held_tool,
                "operation_id": managed.active_operation_id,
                "tray": (
                    managed.active_task.tray_id
                    if managed.active_task is not None
                    else None
                ),
            }

        trays = {}
        missing = []

        if self._tool_state_manager is not None:
            snapshot = (
                self._tool_state_manager.get_tray_tool_snapshot()
            )
            trays = {
                str(tray_id): tool
                for tray_id, tool in snapshot.items()
            }
            missing = self._tool_state_manager.get_missing_tools()

        return {
            "robots": robots,
            "trays": trays,
            "missing": missing,
        }

    def get_tool_state_snapshot(self) -> dict:
        if self._tool_state_manager is None:
            return {}

        return (
            self._tool_state_manager
            .get_debug_snapshot()
        )

    def get_operation_registry_snapshot(
        self,
    ) -> dict:
        return self._operation_registry.get_snapshot()

    def get_active_operations(
        self,
    ) -> dict:
        return (
            self._operation_registry
            .get_active_operations()
        )

    def get_operation_for_tool(
        self,
        tool_id: str,
    ) -> Optional[dict]:
        return (
            self._operation_registry
            .get_operation_for_tool(tool_id)
        )

    def get_recent_operation_ids(
        self,
        *,
        limit: Optional[int] = None,
    ) -> list[str]:
        return (
            self._operation_registry
            .get_recent_operation_ids(
                limit=limit
            )
        )

    def request_move(
        self,
        x: float,
        y: float,
        z: float,
    ) -> Tuple[bool, str]:
        return (
            False,
            "Direct move is disabled "
            "in workflow mode",
        )

    def step(self) -> None:
        if self._tool_state_manager is not None:
            self._tool_state_manager.synchronize_external_state()

        for managed in self._robots.values():
            self._try_assign_pending_replacement(
                managed
            )

            robot_id = managed.profile.robot_id
            previous_state = (
                managed.state_machine.state_name
            )

            if self._must_hold_tray_zone_before_step(
                managed
            ):
                acquired = (
                    self._try_acquire_tray_zone(
                        robot_id=robot_id
                    )
                )

                if not acquired:
                    continue

            managed.state_machine.step()

            current_release_sequence = int(
                getattr(
                    managed.state_machine,
                    "place_release_sequence",
                    0,
                )
            )

            if (
                current_release_sequence
                != managed.last_place_release_sequence
            ):
                managed.last_place_release_sequence = (
                    current_release_sequence
                )

                if self._tool_state_manager is not None:
                    if managed.active_task is None:
                        raise RuntimeError(
                            "PLACE release confirmed without active task: "
                            f"robot={robot_id}"
                        )

                    # 실제 해제가 확인된 이 로봇의 도구만
                    # HELD_BY_ROBOT -> 해당 트레이로 복귀시킨다.
                    self._tool_state_manager.on_place_enter(
                        robot_id=robot_id,
                        tray_id=managed.active_task.tray_id,
                    )

                    self._tool_state_manager.print_tool_locations(
                        title=(
                            "physical PLACE release confirmed "
                            f"by Robot {robot_id}"
                        )
                    )

            current_state = (
                managed.state_machine.state_name
            )

            if current_state != managed.last_state:
                print(
                    f"[Manager {robot_id}] "
                    f"state: {managed.last_state} "
                    f"-> {current_state}",
                    flush=True,
                )

                managed.last_state = current_state
                self._update_active_operation_state(
                    managed=managed,
                    state_name=current_state,
                )

            if (
                self._tool_state_manager is not None
                and managed.active_task is not None
            ):
                if (
                    previous_state == "PICK_APPROACH"
                    and current_state == "PICK_TRANSPORT"
                ):
                    # 흡착 완료 시점부터 해당 도구를
                    # HELD_BY_ROBOT 상태로 관리한다.
                    self._tool_state_manager.on_tracking_enter(
                        robot_id=robot_id,
                        tray_id=managed.active_task.tray_id,
                    )

                elif (
                    previous_state == "TRACKING"
                    and current_state == "PLACE"
                ):
                    # 실제 그리퍼 해제가 확인될 때까지
                    # 해당 로봇의 도구는 HELD_BY_ROBOT으로 유지한다.
                    pass

            self._release_tray_zone_after_transition(
                managed=managed,
                previous_state=previous_state,
                current_state=current_state,
            )

            if (
                managed.active_task is not None
                and (
                    managed.state_machine.current_task
                    is None
                )
            ):
                completed_task = managed.active_task

                self._release_tray_zone(
                    robot_id=robot_id
                )

                print(
                    f"[Manager {robot_id}] "
                    "task complete: "
                    f"tray={completed_task.tray_id}, "
                    f"route={completed_task.route_id}",
                    flush=True,
                )

                if managed.active_operation_id is not None:
                    self._operation_registry.complete_operation(
                        managed.active_operation_id
                    )

                managed.active_task = None
                managed.active_operation_id = None

    def reset(
        self,
        *,
        randomize_tools: bool = False,
    ) -> Optional[Dict[int, str]]:
        self._tray_zone_owner = None

        for managed in self._robots.values():
            managed.active_task = None
            managed.active_operation_id = None
            managed.pending_replacement_tray_id = None
            managed.pending_replacement_operation_id = None
            managed.state_machine.reset()
            managed.last_state = (
                managed.state_machine.state_name
            )
            managed.last_place_release_sequence = int(
                getattr(
                    managed.state_machine,
                    "place_release_sequence",
                    0,
                )
            )

        self._operation_registry.reset()

        tool_layout = None

        if (
            randomize_tools
            and self._tool_state_manager is not None
        ):
            tool_layout = (
                self._tool_state_manager
                .reset_random_layout()
            )

        print(
            "[Manager] reset: "
            "tray-zone lock cleared"
            + (
                ", tools randomized"
                if tool_layout is not None
                else ""
            ),
            flush=True,
        )

        return tool_layout

    def get_tray_zone_owner(
        self,
    ) -> Optional[str]:
        return self._tray_zone_owner
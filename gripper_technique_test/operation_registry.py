# operation_registry.py
from __future__ import annotations

from dataclasses import asdict, dataclass
import time
from typing import Dict, List, Optional


@dataclass
class OperationRecord:
    operation_id: str
    robot_id: Optional[str]
    tool_id: Optional[str]
    tray_id: int
    status: str
    is_replacement: bool
    created_at: float
    updated_at: float
    completed_at: Optional[float] = None


class OperationRegistry:
    """
    RobotManager가 수행하는 작업을 작업 단위로 기록한다.

    관리 항목:
    - active_operations: 현재 완료되지 않은 작업
    - operation_by_tool: 도구별 현재 작업
    - recent_operation_stack: 최근 생성된 작업 ID 순서
    """

    TERMINAL_STATUSES = {
        "COMPLETED",
        "CANCELLED",
        "FAILED",
    }

    def __init__(self) -> None:
        self._next_sequence = 1
        self._operations: Dict[str, OperationRecord] = {}
        self._active_operations: Dict[
            str,
            OperationRecord,
        ] = {}
        self._operation_by_tool: Dict[str, str] = {}
        self._recent_operation_stack: List[str] = []

    def create_operation(
        self,
        *,
        tray_id: int,
        tool_id: Optional[str],
        robot_id: Optional[str],
        status: str,
        is_replacement: bool,
    ) -> str:
        operation_id = (
            f"operation_{self._next_sequence:04d}"
        )
        self._next_sequence += 1

        now = time.monotonic()

        record = OperationRecord(
            operation_id=operation_id,
            robot_id=(
                None
                if robot_id is None
                else str(robot_id).strip().upper()
            ),
            tool_id=(
                None
                if tool_id is None
                else str(tool_id).strip()
            ),
            tray_id=int(tray_id),
            status=str(status).strip().upper(),
            is_replacement=bool(is_replacement),
            created_at=now,
            updated_at=now,
        )

        self._operations[operation_id] = record
        self._active_operations[operation_id] = record
        self._recent_operation_stack.append(operation_id)

        if record.tool_id is not None:
            self._operation_by_tool[
                record.tool_id
            ] = operation_id

        self._print_record(
            "created",
            record,
        )

        return operation_id

    def update_operation(
        self,
        operation_id: str,
        *,
        status: Optional[str] = None,
        robot_id: Optional[str] = None,
        tray_id: Optional[int] = None,
        tool_id: Optional[str] = None,
    ) -> None:
        record = self._require_operation(
            operation_id
        )

        old_tool_id = record.tool_id

        if status is not None:
            record.status = (
                str(status).strip().upper()
            )

        if robot_id is not None:
            record.robot_id = (
                str(robot_id).strip().upper()
            )

        if tray_id is not None:
            record.tray_id = int(tray_id)

        if tool_id is not None:
            record.tool_id = str(tool_id).strip()

        record.updated_at = time.monotonic()

        if (
            old_tool_id is not None
            and old_tool_id != record.tool_id
            and self._operation_by_tool.get(
                old_tool_id
            ) == operation_id
        ):
            self._operation_by_tool.pop(
                old_tool_id,
                None,
            )

        if record.tool_id is not None:
            self._operation_by_tool[
                record.tool_id
            ] = operation_id

        if record.status in self.TERMINAL_STATUSES:
            record.completed_at = record.updated_at
            self._active_operations.pop(
                operation_id,
                None,
            )

            if (
                record.tool_id is not None
                and self._operation_by_tool.get(
                    record.tool_id
                ) == operation_id
            ):
                self._operation_by_tool.pop(
                    record.tool_id,
                    None,
                )
        else:
            self._active_operations[
                operation_id
            ] = record

        self._print_record(
            "updated",
            record,
        )

    def complete_operation(
        self,
        operation_id: str,
    ) -> None:
        self.update_operation(
            operation_id,
            status="COMPLETED",
        )

    def cancel_operation(
        self,
        operation_id: str,
    ) -> None:
        self.update_operation(
            operation_id,
            status="CANCELLED",
        )

    def get_operation(
        self,
        operation_id: str,
    ) -> dict:
        return asdict(
            self._require_operation(operation_id)
        )

    def get_active_operations(self) -> dict:
        return {
            operation_id: asdict(record)
            for operation_id, record
            in self._active_operations.items()
        }

    def get_operation_for_tool(
        self,
        tool_id: str,
    ) -> Optional[dict]:
        operation_id = self._operation_by_tool.get(
            str(tool_id).strip()
        )

        if operation_id is None:
            return None

        return self.get_operation(
            operation_id
        )

    def find_recent_active_operation(
        self,
        *,
        statuses: Optional[set[str]] = None,
    ) -> Optional[dict]:
        """
        최근 생성된 작업부터 확인하여 현재 활성 상태이고,
        요청한 상태와 일치하는 첫 작업을 반환한다.
        """
        normalized_statuses = None

        if statuses is not None:
            normalized_statuses = {
                str(status).strip().upper()
                for status in statuses
            }

        for operation_id in reversed(
            self._recent_operation_stack
        ):
            record = self._active_operations.get(
                operation_id
            )

            if record is None:
                continue

            if (
                normalized_statuses is not None
                and record.status
                not in normalized_statuses
            ):
                continue

            return asdict(record)

        return None

    def get_recent_operation_ids(
        self,
        *,
        limit: Optional[int] = None,
    ) -> list[str]:
        values = list(
            reversed(self._recent_operation_stack)
        )

        if limit is None:
            return values

        return values[: max(0, int(limit))]

    def get_snapshot(self) -> dict:
        return {
            "active_operations": (
                self.get_active_operations()
            ),
            "operation_by_tool": dict(
                self._operation_by_tool
            ),
            "recent_operation_stack": (
                self.get_recent_operation_ids()
            ),
        }

    def reset(self) -> None:
        self._operations.clear()
        self._active_operations.clear()
        self._operation_by_tool.clear()
        self._recent_operation_stack.clear()
        self._next_sequence = 1

        print(
            "[OperationRegistry] reset",
            flush=True,
        )

    def _require_operation(
        self,
        operation_id: str,
    ) -> OperationRecord:
        operation_id = str(operation_id)

        if operation_id not in self._operations:
            raise KeyError(
                f"Unknown operation_id: {operation_id}"
            )

        return self._operations[operation_id]

    @staticmethod
    def _print_record(
        action: str,
        record: OperationRecord,
    ) -> None:
        print(
            f"[OperationRegistry] {action}: "
            f"id={record.operation_id}, "
            f"robot={record.robot_id}, "
            f"tool={record.tool_id}, "
            f"tray={record.tray_id}, "
            f"status={record.status}, "
            f"replacement={record.is_replacement}",
            flush=True,
        )
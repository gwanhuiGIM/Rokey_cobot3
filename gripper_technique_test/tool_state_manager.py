# tool_state_manager.py
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
import random
import time
from typing import Dict, Mapping, Optional, Sequence

import numpy as np


class ToolLocation(Enum):
    ON_TRAY = auto()
    HELD_BY_ROBOT = auto()
    UNKNOWN = auto()


@dataclass
class ExternalToolDetection:
    position: np.ndarray
    tray_id: Optional[int]
    updated_at: float


@dataclass
class ToolState:
    tool_id: str
    location: ToolLocation
    tray_id: Optional[int]
    held_by_robot: Optional[str]
    internal_position: Optional[np.ndarray]
    external_detection: Optional[ExternalToolDetection]


class ToolStateManager:
    """
    수술 도구의 논리적 위치를 관리한다.

    우선순위:
    1. 로봇이 들고 있는 상태
    2. 유효시간 내 외부 인식 결과
    3. 내부 상태 추적 결과

    외부 ROS2 구독은 나중에 update_external_detection()만
    호출하도록 연결하면 된다.
    """

    def __init__(
        self,
        *,
        tray_positions: Mapping[int, Sequence[float]],
        tool_ids: Sequence[str],
        random_seed: Optional[int] = None,
        external_timeout_sec: float = 1.0,
        tray_match_radius: float = 0.18,
    ) -> None:
        self._tray_positions = {
            int(tray_id): np.asarray(
                position,
                dtype=np.float64,
            ).copy()
            for tray_id, position in tray_positions.items()
        }

        self._tool_ids = tuple(
            str(tool_id).strip()
            for tool_id in tool_ids
        )

        if len(self._tool_ids) != len(set(self._tool_ids)):
            raise ValueError("tool_ids must be unique")

        if len(self._tool_ids) != len(self._tray_positions):
            raise ValueError(
                "현재 단계에서는 도구 수와 트레이 수가 같아야 합니다"
            )

        self.external_timeout_sec = float(
            external_timeout_sec
        )
        self.tray_match_radius = float(
            tray_match_radius
        )

        if self.external_timeout_sec <= 0.0:
            raise ValueError(
                "external_timeout_sec must be > 0"
            )

        if self.tray_match_radius <= 0.0:
            raise ValueError(
                "tray_match_radius must be > 0"
            )

        self._random = random.Random(
            random_seed
        )

        self._states: Dict[str, ToolState] = {}
        self._tray_tools: Dict[int, Optional[str]] = {
            tray_id: None
            for tray_id in self._tray_positions
        }
        self._held_tools: Dict[str, Optional[str]] = {}

        self._initialize_random_layout()

    def _initialize_random_layout(self) -> None:
        self.reset_random_layout(
            log_prefix="random initial layout"
        )

    def reset_random_layout(
        self,
        *,
        log_prefix: str = "random reset layout",
    ) -> Dict[int, str]:
        """
        Stop -> Play 재시작 시 호출한다.

        논리 상태를 완전히 초기화하고 도구를 다시 셔플한다.
        가능한 경우 직전 배치와 동일한 순열은 피한다.
        """
        previous_layout = tuple(
            self._tray_tools.get(tray_id)
            for tray_id in sorted(self._tray_positions)
        )

        shuffled_tool_ids = list(
            self._tool_ids
        )

        for _ in range(16):
            self._random.shuffle(
                shuffled_tool_ids
            )

            if (
                len(shuffled_tool_ids) <= 1
                or tuple(shuffled_tool_ids)
                != previous_layout
            ):
                break

        # 극히 드물게 동일 순열이 계속 나오면 한 칸 회전시킨다.
        if (
            len(shuffled_tool_ids) > 1
            and tuple(shuffled_tool_ids)
            == previous_layout
        ):
            shuffled_tool_ids = (
                shuffled_tool_ids[1:]
                + shuffled_tool_ids[:1]
            )

        self._states.clear()
        self._held_tools.clear()

        for tray_id in self._tray_tools:
            self._tray_tools[tray_id] = None

        for tray_id, tool_id in zip(
            sorted(self._tray_positions),
            shuffled_tool_ids,
        ):
            position = self._tray_positions[
                tray_id
            ].copy()

            self._tray_tools[tray_id] = tool_id
            self._states[tool_id] = ToolState(
                tool_id=tool_id,
                location=ToolLocation.ON_TRAY,
                tray_id=tray_id,
                held_by_robot=None,
                internal_position=position,
                external_detection=None,
            )

        snapshot = {
            tray_id: tool_id
            for tray_id, tool_id
            in self._tray_tools.items()
            if tool_id is not None
        }

        print(
            f"[ToolState] {log_prefix}: "
            f"{snapshot}",
            flush=True,
        )
        self.print_tool_locations(
            title=log_prefix
        )

        return snapshot

    def reset_random_partial_layout(
        self,
        *,
        min_count: int = 4,
        max_count: int = 6,
        log_prefix: str = "random partial layout",
    ) -> Dict[str, object]:
        """
        도구 중 4~6개만 무작위로 등장시키고, 등장한 도구를 무작위 트레이에
        배정한다. 등장하지 않은 도구는 missing(논리적으로 부재) 처리한다.

        반환:
            {
              "layout": {tray_id: tool_id},   # 등장한 것만
              "present": [tool_id, ...],
              "missing": [tool_id, ...],
            }
        """
        tray_ids = sorted(self._tray_positions)
        n_trays = len(tray_ids)

        upper = min(int(max_count), n_trays, len(self._tool_ids))
        lower = max(0, min(int(min_count), upper))
        count = self._random.randint(lower, upper)

        chosen_tools = self._random.sample(
            list(self._tool_ids),
            count,
        )
        chosen_trays = self._random.sample(
            tray_ids,
            count,
        )

        self._states.clear()
        self._held_tools.clear()

        for tray_id in self._tray_tools:
            self._tray_tools[tray_id] = None

        layout: Dict[int, str] = {}

        for tray_id, tool_id in zip(
            chosen_trays,
            chosen_tools,
        ):
            position = self._tray_positions[tray_id].copy()

            self._tray_tools[tray_id] = tool_id
            self._states[tool_id] = ToolState(
                tool_id=tool_id,
                location=ToolLocation.ON_TRAY,
                tray_id=tray_id,
                held_by_robot=None,
                internal_position=position,
                external_detection=None,
            )
            layout[tray_id] = tool_id

        present = set(chosen_tools)

        # 등장하지 않은 도구 = missing.
        for tool_id in self._tool_ids:
            if tool_id in present:
                continue

            self._states[tool_id] = ToolState(
                tool_id=tool_id,
                location=ToolLocation.UNKNOWN,
                tray_id=None,
                held_by_robot=None,
                internal_position=None,
                external_detection=None,
            )

        missing = [
            tool_id
            for tool_id in self._tool_ids
            if tool_id not in present
        ]

        print(
            f"[ToolState] {log_prefix}: "
            f"present={sorted(present)}, missing={missing}, "
            f"layout={dict(sorted(layout.items()))}",
            flush=True,
        )
        self.print_tool_locations(title=log_prefix)

        return {
            "layout": layout,
            "present": list(chosen_tools),
            "missing": missing,
        }

    def get_missing_tools(self) -> list:
        """트레이에도 없고 로봇도 안 든 도구 이름 리스트."""
        missing = []

        for tool_id in self._tool_ids:
            state = self._states.get(tool_id)
            location = (
                state.location if state is not None else None
            )

            if location not in (
                ToolLocation.ON_TRAY,
                ToolLocation.HELD_BY_ROBOT,
            ):
                missing.append(tool_id)

        return missing

    def print_tool_locations(
        self,
        *,
        title: str = "current tool locations",
    ) -> None:
        print(
            "",
            flush=True,
        )
        print(
            "=" * 72,
            flush=True,
        )
        print(
            f"[ToolState] {title}",
            flush=True,
        )

        for tray_id in sorted(
            self._tray_positions
        ):
            tool_id = self._tray_tools.get(
                tray_id
            )
            tray_position = (
                self._tray_positions[tray_id]
            )

            if tool_id is None:
                print(
                    f"  Tray {tray_id}: EMPTY | "
                    f"position="
                    f"{tray_position.round(4).tolist()}",
                    flush=True,
                )
                continue

            state = self._states[tool_id]

            if state.location == ToolLocation.HELD_BY_ROBOT:
                location_text = (
                    f"HELD_BY_ROBOT({state.held_by_robot})"
                )
                position_text = "None"
            else:
                effective_position = (
                    state.internal_position
                )

                location_text = state.location.name
                position_text = (
                    "None"
                    if effective_position is None
                    else str(
                        effective_position
                        .round(4)
                        .tolist()
                    )
                )

            print(
                f"  Tray {tray_id}: "
                f"{tool_id} | "
                f"{location_text} | "
                f"position={position_text}",
                flush=True,
            )

        held_entries = [
            f"{robot_id}={tool_id}"
            for robot_id, tool_id
            in sorted(self._held_tools.items())
            if tool_id is not None
        ]

        print(
            "  Held tools: "
            + (
                ", ".join(held_entries)
                if held_entries
                else "none"
            ),
            flush=True,
        )
        print(
            "=" * 72,
            flush=True,
        )
        print(
            "",
            flush=True,
        )

    def get_tray_tool_snapshot(
        self,
    ) -> Dict[int, Optional[str]]:
        return dict(
            self._tray_tools
        )

    def get_tool_tray_snapshot(
        self,
    ) -> Dict[str, Optional[int]]:
        return {
            tool_id: state.tray_id
            for tool_id, state
            in self._states.items()
        }

    def get_tool_for_tray(
        self,
        tray_id: int,
    ) -> Optional[str]:
        return self._tray_tools.get(
            int(tray_id)
        )

    def get_tray_for_tool(
        self,
        tool_id: str,
        *,
        now: Optional[float] = None,
    ) -> Optional[int]:
        tool_id = str(tool_id).strip()

        if tool_id not in self._states:
            return None

        state = self._states[tool_id]

        if state.location == ToolLocation.HELD_BY_ROBOT:
            return None

        detection = state.external_detection

        if (
            detection is not None
            and self._external_is_fresh(
                detection,
                now=now,
            )
        ):
            return detection.tray_id

        return state.tray_id

    def get_held_tool(
        self,
        robot_id: str,
    ) -> Optional[str]:
        return self._held_tools.get(
            str(robot_id).strip().upper()
        )

    def on_tracking_enter(
        self,
        *,
        robot_id: str,
        tray_id: int,
    ) -> None:
        robot_id = str(robot_id).strip().upper()
        tray_id = int(tray_id)

        tool_id = self._tray_tools.get(
            tray_id
        )

        if tool_id is None:
            print(
                "[ToolState WARNING] TRACKING 진입 시 "
                f"tray={tray_id}가 이미 비어 있습니다",
                flush=True,
            )
            return

        self._tray_tools[tray_id] = None
        self._held_tools[robot_id] = tool_id

        state = self._states[tool_id]
        state.location = ToolLocation.HELD_BY_ROBOT
        state.tray_id = None
        state.held_by_robot = robot_id
        state.internal_position = None

        print(
            f"[ToolState] robot={robot_id}, "
            f"tool={tool_id}: tray {tray_id} "
            "-> HELD_BY_ROBOT",
            flush=True,
        )
        self.print_tool_locations(
            title=(
                f"TRACKING entered by Robot {robot_id}"
            )
        )

    def on_place_enter(
        self,
        *,
        robot_id: str,
        tray_id: int,
    ) -> None:
        robot_id = str(robot_id).strip().upper()
        tray_id = int(tray_id)

        tool_id = self._held_tools.get(
            robot_id
        )

        if tool_id is None:
            print(
                "[ToolState WARNING] PLACE 진입 시 "
                f"robot={robot_id}가 든 도구가 없습니다",
                flush=True,
            )
            return

        previous_tool = self._tray_tools.get(
            tray_id
        )

        if (
            previous_tool is not None
            and previous_tool != tool_id
        ):
            previous_state = self._states[
                previous_tool
            ]
            previous_state.location = (
                ToolLocation.UNKNOWN
            )
            previous_state.tray_id = None
            previous_state.held_by_robot = None
            previous_state.internal_position = None

            print(
                "[ToolState WARNING] PLACE 대상 트레이에 "
                f"기존 도구가 있어 UNKNOWN 처리: "
                f"tray={tray_id}, tool={previous_tool}",
                flush=True,
            )

        self._tray_tools[tray_id] = tool_id
        self._held_tools[robot_id] = None

        state = self._states[tool_id]
        state.location = ToolLocation.ON_TRAY
        state.tray_id = tray_id
        state.held_by_robot = None
        state.internal_position = (
            self._tray_positions[tray_id].copy()
        )

        print(
            f"[ToolState] robot={robot_id}, "
            f"tool={tool_id}: HELD_BY_ROBOT "
            f"-> tray {tray_id}",
            flush=True,
        )
        self.print_tool_locations(
            title=(
                f"PLACE entered by Robot {robot_id}"
            )
        )

    def update_external_detection(
        self,
        *,
        tool_id: str,
        position: Sequence[float],
        tray_id: Optional[int] = None,
        timestamp: Optional[float] = None,
    ) -> None:
        """
        나중에 ROS2 Subscriber 콜백에서 호출할 진입점.

        예:
            tool_state_manager.update_external_detection(
                tool_id=msg.tool_id,
                position=[msg.x, msg.y, msg.z],
                tray_id=msg.tray_id,
            )
        """
        tool_id = str(tool_id).strip()

        if tool_id not in self._states:
            raise KeyError(
                f"Unknown tool_id: {tool_id}"
            )

        position_array = np.asarray(
            position,
            dtype=np.float64,
        )

        if position_array.shape != (3,):
            raise ValueError(
                "external position must have shape (3,)"
            )

        if not np.all(np.isfinite(position_array)):
            raise ValueError(
                "external position must be finite"
            )

        resolved_tray_id = (
            self._resolve_external_tray(
                position=position_array,
                tray_id=tray_id,
            )
        )

        detection = ExternalToolDetection(
            position=position_array.copy(),
            tray_id=resolved_tray_id,
            updated_at=(
                time.monotonic()
                if timestamp is None
                else float(timestamp)
            ),
        )

        state = self._states[tool_id]
        state.external_detection = detection

        # 로봇이 들고 있는 동안에는 외부 인식을 저장만 하고
        # 현재 논리 상태에는 반영하지 않는다.
        if state.location == ToolLocation.HELD_BY_ROBOT:
            print(
                "[ToolState] external detection cached "
                f"while held: tool={tool_id}",
                flush=True,
            )
            return

        self._apply_external_detection(
            tool_id=tool_id,
            detection=detection,
        )

    def synchronize_external_state(
        self,
        *,
        now: Optional[float] = None,
    ) -> None:
        current_time = (
            time.monotonic()
            if now is None
            else float(now)
        )

        for tool_id, state in self._states.items():
            detection = state.external_detection

            if detection is None:
                continue

            if state.location == ToolLocation.HELD_BY_ROBOT:
                continue

            if self._external_is_fresh(
                detection,
                now=current_time,
            ):
                self._apply_external_detection(
                    tool_id=tool_id,
                    detection=detection,
                )

    def _apply_external_detection(
        self,
        *,
        tool_id: str,
        detection: ExternalToolDetection,
    ) -> None:
        state = self._states[tool_id]
        target_tray_id = detection.tray_id

        old_tray_id = state.tray_id

        if (
            old_tray_id is not None
            and self._tray_tools.get(old_tray_id)
            == tool_id
        ):
            self._tray_tools[old_tray_id] = None

        if target_tray_id is None:
            state.location = ToolLocation.UNKNOWN
            state.tray_id = None
            state.held_by_robot = None
            state.internal_position = (
                detection.position.copy()
            )

            print(
                "[ToolState] external -> UNKNOWN: "
                f"tool={tool_id}, "
                f"position={detection.position.round(4)}",
                flush=True,
            )
            return

        displaced_tool = self._tray_tools.get(
            target_tray_id
        )

        if (
            displaced_tool is not None
            and displaced_tool != tool_id
        ):
            displaced_state = self._states[
                displaced_tool
            ]
            displaced_state.location = (
                ToolLocation.UNKNOWN
            )
            displaced_state.tray_id = None
            displaced_state.held_by_robot = None
            displaced_state.internal_position = None

        self._tray_tools[target_tray_id] = tool_id

        state.location = ToolLocation.ON_TRAY
        state.tray_id = target_tray_id
        state.held_by_robot = None
        state.internal_position = (
            detection.position.copy()
        )

        print(
            "[ToolState] external applied: "
            f"tool={tool_id}, tray={target_tray_id}, "
            f"position={detection.position.round(4)}",
            flush=True,
        )
        self.print_tool_locations(
            title="external detection applied"
        )

    def _resolve_external_tray(
        self,
        *,
        position: np.ndarray,
        tray_id: Optional[int],
    ) -> Optional[int]:
        if tray_id is not None:
            resolved = int(tray_id)

            if resolved not in self._tray_positions:
                raise KeyError(
                    f"Unknown tray_id: {resolved}"
                )

            return resolved

        nearest_tray_id: Optional[int] = None
        nearest_distance = float("inf")

        for candidate_id, tray_position in (
            self._tray_positions.items()
        ):
            distance = float(
                np.linalg.norm(
                    position[:2]
                    - tray_position[:2]
                )
            )

            if distance < nearest_distance:
                nearest_distance = distance
                nearest_tray_id = candidate_id

        if nearest_distance > self.tray_match_radius:
            return None

        return nearest_tray_id

    def _external_is_fresh(
        self,
        detection: ExternalToolDetection,
        *,
        now: Optional[float] = None,
    ) -> bool:
        current_time = (
            time.monotonic()
            if now is None
            else float(now)
        )

        return (
            current_time - detection.updated_at
            <= self.external_timeout_sec
        )

    def get_debug_snapshot(self) -> dict:
        return {
            "tray_tools": self.get_tray_tool_snapshot(),
            "held_tools": dict(self._held_tools),
            "tools": {
                tool_id: {
                    "location": state.location.name,
                    "tray_id": state.tray_id,
                    "held_by_robot": state.held_by_robot,
                }
                for tool_id, state
                in self._states.items()
            },
        }
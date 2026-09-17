# hand_input.py

from __future__ import annotations

from typing import Callable, Optional, Protocol, Tuple


HandSample = Tuple[
    Optional[Tuple[float, float, float]],
    int,
]
HandModeSample = Tuple[str, int]


class HandInput(Protocol):
    """상태 머신이 요구하는 손 입력 인터페이스."""

    def get_target(self) -> HandSample:
        ...

    def get_mode(self) -> HandModeSample:
        ...

    def reset_mode(self, mode: str = "TRACKING") -> int:
        ...


class CachedHandInput:
    """
    ROS Bridge의 캐시 함수를 상태 머신에 주입하기 위한 어댑터.

    현재는 기존 /hand_xyz, /hand_mode를 그대로 사용한다.
    이후 왼손/오른손 캐시 함수만 다르게 넘기면 상태 머신을
    수정하지 않고 서로 다른 손 입력을 연결할 수 있다.
    """

    def __init__(
        self,
        *,
        input_id: str,
        target_getter: Callable[[], HandSample],
        mode_getter: Callable[[], HandModeSample],
        mode_resetter: Callable[[str], int],
    ) -> None:
        self.input_id = str(input_id)
        self._target_getter = target_getter
        self._mode_getter = mode_getter
        self._mode_resetter = mode_resetter

    def get_target(self) -> HandSample:
        return self._target_getter()

    def get_mode(self) -> HandModeSample:
        return self._mode_getter()

    def reset_mode(self, mode: str = "TRACKING") -> int:
        return self._mode_resetter(mode)
#!/usr/bin/env bash
#
# gripper_technique_test (Isaac Sim 메인) 실행 헬퍼.
# Isaac Sim 경로를 자동 탐색해서 main.py 를 실행한다.
# ISAAC_SIM_PATH 환경변수가 이미 있으면 그것을 우선 사용한다.
#
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"

detect_isaac() {
    if [ -n "${ISAAC_SIM_PATH:-}" ] && [ -x "$ISAAC_SIM_PATH/python.sh" ]; then
        echo "$ISAAC_SIM_PATH"; return 0
    fi
    local c
    for c in \
        "$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release" \
        "$HOME/isaacsim/_build/linux-x86_64/release" \
        "$HOME"/.local/share/ov/pkg/isaac*sim*/ \
        "$HOME"/isaac*sim*/ ; do
        [ -x "$c/python.sh" ] && { echo "$c"; return 0; }
    done
    local found
    found=$(find "$HOME" -maxdepth 6 -name python.sh -path "*isaac*release*" 2>/dev/null | head -1)
    [ -n "$found" ] && { echo "$(dirname "$found")"; return 0; }
    return 1
}

if ! ISAAC="$(detect_isaac)"; then
    echo "[ERROR] Isaac Sim 을 찾지 못했습니다." >&2
    echo "        export ISAAC_SIM_PATH=\"<...>/_build/linux-x86_64/release\" 후 다시 실행하세요." >&2
    exit 1
fi
echo "[run] Isaac Sim: $ISAAC"

# ROS2 환경 (이미 설정돼 있으면 유지)
: "${ROS_DOMAIN_ID:=137}"; export ROS_DOMAIN_ID
: "${RMW_IMPLEMENTATION:=rmw_fastrtps_cpp}"; export RMW_IMPLEMENTATION
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$ISAAC/exts/isaacsim.ros2.bridge/humble/lib"

cd "$HERE"
exec "$ISAAC/python.sh" main.py

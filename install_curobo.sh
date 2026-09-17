#!/usr/bin/env bash
# =============================================================================
#  install_curobo.sh  —  cuRobo 를 Isaac Sim 의 Python 에 "버전 맞춰서" 설치
# -----------------------------------------------------------------------------
#  cuRobo 는 gripper_technique_test(모션 플래닝) 전용이다.
#  ★ 반드시 Isaac Sim 내장 Python 에 설치해야 한다 (시스템 venv 아님).
#  ★ 이 프로젝트가 검증한 버전:  cuRobo v0.7.8
#     (Isaac Python 3.11 / torch 2.7.0+cu128 와 짝이 맞음)
#
#  하는 일:
#    1) Isaac Sim 경로 자동 탐색 (없으면 ISAAC_SIM_PATH 로 알려달라고 안내)
#    2) 이미 v0.7.8 이 깔려 있으면 → 아무것도 안 하고 종료(안전)
#    3) cuRobo 소스를 받아 v0.7.8 태그로 고정(checkout) 후 Isaac Python 에 설치
#    4) 설치 후 import 테스트로 실제 동작 확인
#
#  사용법:   ./install_curobo.sh
#  (옵션)    ISAAC_SIM_PATH=/경로 ./install_curobo.sh
#  (옵션)    CUROBO_DIR=~/curobo  CUROBO_TAG=v0.7.8 ./install_curobo.sh
# =============================================================================
set -u

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; BOLD='\033[1m'; NC='\033[0m'
ok()   { echo -e "${GREEN}[OK]${NC} $*"; }
info() { echo -e "${YELLOW}[안내]${NC} $*"; }
err()  { echo -e "${RED}[오류]${NC} $*"; }
step() { echo -e "\n${BOLD}$*${NC}"; }

CUROBO_TAG="${CUROBO_TAG:-v0.7.8}"          # ★ 이 프로젝트 검증 버전
CUROBO_DIR="${CUROBO_DIR:-$HOME/curobo}"    # 소스를 받을 위치
CUROBO_URL="https://github.com/NVlabs/curobo.git"

echo "============================================================"
echo " cuRobo 설치  (목표 버전: ${CUROBO_TAG})"
echo "============================================================"

# ── 1. Isaac Sim python.sh 찾기 ──────────────────────────────
step "[1/4] Isaac Sim 경로 확인"
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

if ISAAC_SIM_PATH="$(detect_isaac)"; then
    ok "Isaac Sim: $ISAAC_SIM_PATH"
else
    err "Isaac Sim 을 찾지 못했습니다. 경로를 알려주고 다시 실행하세요:"
    err '  ISAAC_SIM_PATH="<...>/_build/linux-x86_64/release" ./install_curobo.sh'
    exit 1
fi
PY="$ISAAC_SIM_PATH/python.sh"

ISAAC_PYVER="$("$PY" -c 'import sys;print("%d.%d.%d"%sys.version_info[:3])' 2>/dev/null | tail -1)"
ISAAC_TORCH="$("$PY" -c 'import torch;print(torch.__version__)' 2>/dev/null | tail -1)"
ok "Isaac Python ${ISAAC_PYVER:-?} / torch ${ISAAC_TORCH:-미확인}"
if [ -z "$ISAAC_TORCH" ]; then
    err "Isaac Python 에서 torch 를 찾지 못했습니다. Isaac Sim 설치를 먼저 확인하세요."
    exit 1
fi

# ── 2. 이미 설치돼 있는지 ────────────────────────────────────
step "[2/4] 기존 cuRobo 확인"
CUR_VER="$("$PY" -c 'import curobo;print(curobo.__version__)' 2>/dev/null | tail -1)"
WANT="${CUROBO_TAG#v}"   # v0.7.8 → 0.7.8
if [ "$CUR_VER" = "$WANT" ]; then
    ok "cuRobo ${CUR_VER} 가 이미 설치돼 있습니다. (목표와 일치 → 설치 생략)"
    "$PY" -c "from curobo.wrap.reacher.motion_gen import MotionGen; print('import OK')" 2>/dev/null \
        | tail -1 | grep -q "import OK" && ok "import 테스트 통과. 끝." && exit 0
    info "버전은 맞지만 import 가 불완전 → 재설치를 진행합니다."
elif [ -n "$CUR_VER" ]; then
    info "현재 cuRobo ${CUR_VER} (목표 ${WANT}). ${WANT} 로 맞춰 재설치합니다."
else
    info "cuRobo 미설치 → ${WANT} 신규 설치합니다."
fi

# ── 3. 소스 받고 태그 고정 후 설치 ───────────────────────────
step "[3/4] 소스 준비 + 설치 (빌드라 수 분 걸릴 수 있음)"
if [ ! -d "$CUROBO_DIR/.git" ]; then
    info "clone: $CUROBO_URL → $CUROBO_DIR"
    git clone "$CUROBO_URL" "$CUROBO_DIR" || { err "git clone 실패"; exit 1; }
else
    ok "기존 소스 재사용: $CUROBO_DIR"
fi

( cd "$CUROBO_DIR" \
  && git fetch --tags --quiet \
  && git checkout --quiet "$CUROBO_TAG" ) \
  || { err "태그 $CUROBO_TAG 체크아웃 실패"; exit 1; }
ok "소스를 $CUROBO_TAG 로 고정"

info "Isaac Python 으로 설치 중...  (torch 는 Isaac 번들 사용 → --no-build-isolation)"
"$PY" -m pip install -e "$CUROBO_DIR" --no-build-isolation || {
    err "설치 실패. 흔한 원인:"
    err "  - 디스크 용량 부족 / 빌드 중 메모리 부족"
    err "  - Isaac Python 의 torch 손상 → 'import torch' 먼저 확인"
    exit 1
}

# ── 4. 동작 테스트 ───────────────────────────────────────────
step "[4/4] 설치 검증"
FINAL_VER="$("$PY" -c 'import curobo;print(curobo.__version__)' 2>/dev/null | tail -1)"
if "$PY" -c "from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig; from curobo.types.math import Pose; print('OK')" 2>/dev/null | tail -1 | grep -q OK; then
    ok "cuRobo ${FINAL_VER} 설치/임포트 성공!"
    echo ""
    echo "============================================================"
    ok "완료. 이제 gripper 모듈을 실행할 수 있습니다:"
    echo "    cd gripper_technique_test && ./run.sh"
    echo "============================================================"
else
    err "설치는 됐지만 import 테스트 실패. 위 로그를 확인하세요."
    exit 1
fi

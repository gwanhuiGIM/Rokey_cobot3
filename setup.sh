#!/usr/bin/env bash
#
# surgical_robot_main 통합 셋업 스크립트
#
# 자동화 대상 (이 스크립트가 처리):
#   - handtracking_final / vision_detection_model / voicellm 의 venv + pip 의존성
#   - 사전조건 점검(ROS2 / Python / GPU / Isaac 경로)
#
# 전제 (이미 설치됨 가정 — 이 스크립트가 설치하지 않음):
#   - ROS2 Humble/Jazzy
#   - Isaac Sim 5.1 (ISAAC_SIM_PATH 로 경로 지정)
#   - NVIDIA GPU 드라이버 / CUDA
#
# 추가 설치 안내:
#   - cuRobo 는 Isaac python 에 설치 필요(빌드 무거움) → 절차만 안내
#   - GEMINI_API_KEY (voicellm) 는 실행 시 환경변수로 주입
#
# 사용법:  ./setup.sh
#
set -u

ROOT="$(cd "$(dirname "$0")" && pwd)"
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
ok()   { echo -e "${GREEN}[OK]${NC} $*"; }
warn() { echo -e "${YELLOW}[안내]${NC} $*"; }
err()  { echo -e "${RED}[주의]${NC} $*"; }

echo "============================================================"
echo " surgical_robot_main 셋업"
echo "============================================================"

# ── 1. 사전조건 점검 ─────────────────────────────────
echo ""
echo "[1/4] 사전조건 점검"

if [ -f /opt/ros/humble/setup.bash ]; then
    ROS_SETUP=/opt/ros/humble/setup.bash; ok "ROS2 Humble 발견"
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    ROS_SETUP=/opt/ros/jazzy/setup.bash; ok "ROS2 Jazzy 발견"
else
    err "ROS2(humble/jazzy) 를 찾지 못했습니다. ROS2 설치 후 다시 실행하세요."
    ROS_SETUP=""
fi

command -v python3 >/dev/null && ok "python3: $(python3 --version)" || err "python3 없음"
command -v nvidia-smi >/dev/null && ok "NVIDIA GPU 드라이버 감지" || warn "nvidia-smi 없음 — GPU/CUDA 필요(YOLO/Whisper/Isaac)"

# ── 1-1. 시스템(apt) 패키지 설치 ─────────────────────
# pip 으로 안 되는 것들. sudo 권한 필요(없으면 건너뛰고 안내).
echo ""
echo "  -> 시스템 패키지(apt) 설치"
APT_PKGS="python3-venv python3-pip ros-humble-cv-bridge v4l-utils ffmpeg libportaudio2 unzip"
if command -v sudo >/dev/null; then
    if sudo apt-get update -qq && sudo apt-get install -y $APT_PKGS; then
        ok "시스템 패키지 설치 완료"
    else
        warn "apt 설치 일부 실패 — 수동 설치 필요:  sudo apt install -y $APT_PKGS"
    fi
else
    warn "sudo 없음 — 아래를 수동 설치하세요:  sudo apt install -y $APT_PKGS"
fi

# Isaac Sim 경로 자동 탐색 (이미 ISAAC_SIM_PATH 가 있으면 그대로 사용)
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
    export ISAAC_SIM_PATH
    ok "Isaac Sim 자동 탐색: $ISAAC_SIM_PATH"
else
    warn "Isaac Sim 을 자동으로 찾지 못했습니다. 직접 지정 후 다시 실행:"
    warn '  export ISAAC_SIM_PATH="<...>/_build/linux-x86_64/release"'
fi

# ── 1-2. 대용량 씬 에셋 복원 (분할 압축 → 폴더) ──────
# operating 씬은 GitHub 100MB 제한 때문에 scene_operating.zip.part-* 로 분할 커밋했다.
# 폴더가 아직 없고 조각이 있으면 자동으로 합쳐서 풀어준다. (이미 있으면 생략)
SCENE_DIR="$ROOT/gripper_technique_test/Collected_full_scene_operating"
SCENE_PARTS="$ROOT/gripper_technique_test/scene_operating.zip.part-"
echo ""
echo "[1-2] 씬 에셋 복원 확인"
if [ -f "$SCENE_DIR/full_scene_backup.usda" ]; then
    ok "씬 에셋 이미 복원돼 있음 — 건너뜀"
elif ls "${SCENE_PARTS}"* >/dev/null 2>&1; then
    if command -v unzip >/dev/null; then
        ( cd "$ROOT/gripper_technique_test" \
          && cat scene_operating.zip.part-* > scene_operating.zip \
          && unzip -q -o scene_operating.zip \
          && rm -f scene_operating.zip ) \
          && ok "씬 에셋 복원 완료 (Collected_full_scene_operating/)" \
          || err "씬 에셋 복원 실패 — 수동: cd gripper_technique_test && cat scene_operating.zip.part-* > s.zip && unzip s.zip"
    else
        warn "unzip 이 없어 씬 복원 생략 — 'sudo apt install unzip' 후: cat scene_operating.zip.part-* > s.zip && unzip s.zip"
    fi
else
    warn "씬 조각(scene_operating.zip.part-*)도 폴더도 없음 — 에셋 누락 상태"
fi

# ── 2. 루트 공용 venv 생성 + 의존성 설치 ─────────────
# 비-Isaac 모듈 4개(handtracking/vision/voicellm/dashboard)는 루트 .venv 하나를
# 공용으로 쓴다. 설치 목록은 master requirements.txt 단일 소스.
# RTX 5080(Blackwell) 등 최신 GPU 는 cu128 torch 가 필요(cu121 이하 안 됨).
TORCH_INDEX="https://download.pytorch.org/whl/cu128"

echo ""
echo "[2/4] 루트 공용 venv (.venv) + Python 의존성"
if [ -n "$ROS_SETUP" ]; then
    # ROS setup.bash 는 미설정 변수(AMENT_TRACE_SETUP_FILES 등)를 참조하므로
    # set -u 상태에서 source 하면 "unbound variable" 로 죽는다 → 잠시 -u 해제.
    set +u
    # shellcheck disable=SC1090
    source "$ROS_SETUP"   # rclpy/메시지가 venv 에서 보이도록(system-site-packages)
    set -u
fi
( cd "$ROOT" || exit 1
  python3 -m venv --system-site-packages .venv
  # shellcheck disable=SC1091
  source .venv/bin/activate
  pip install --upgrade pip setuptools wheel >/dev/null
  # torch+torchvision 을 cu128 로 "먼저" → 이후 ultralytics/whisper 가 CPU판으로
  # 덮지 않게(특히 ultralytics 가 torchvision 을 끌어와 cu128 을 덮는 것 방지)
  echo "  -> torch+torchvision (cu128) 먼저 설치"
  pip install torch torchvision --index-url "$TORCH_INDEX" || err "torch 설치 실패"
  echo "  -> requirements.txt 설치"
  pip install -r "$ROOT/requirements.txt"
  deactivate ) && ok "루트 venv 설치 완료" || err "루트 venv 설치 실패"

echo ""
echo "[3/4] torch 동작 확인"
( cd "$ROOT" && source .venv/bin/activate \
  && python -c "import torch; print('torch', torch.__version__, 'cuda', torch.cuda.is_available())" \
  && deactivate ) || warn "torch import/CUDA 확인 실패 — GPU 드라이버/네트워크 확인"

# ── cuRobo (Isaac python 에 설치) ───────────────────
# 버전(v0.7.8) 고정 + 설치 + 검증을 install_curobo.sh 가 전담한다.
echo ""
echo "[4/4] cuRobo (Isaac Sim python) — install_curobo.sh 호출"
if [ -x "$ROOT/install_curobo.sh" ]; then
    ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-}" "$ROOT/install_curobo.sh" || \
        warn "cuRobo 설치 미완료 — 나중에 ./install_curobo.sh 단독 실행으로 재시도 가능"
else
    warn "install_curobo.sh 가 없습니다. 수동 설치는 README 4장 참고."
fi

# ── 4. 마무리 안내 ───────────────────────────────────
echo ""
echo "============================================================"
echo " 셋업 완료. 남은 수동 단계:"
echo "============================================================"
echo "  1) export GEMINI_API_KEY=...   (voicellm 음성 사용 시)"
echo "  2) 모든 터미널 공통:"
echo "       source $ROS_SETUP"
echo "       export ROS_DOMAIN_ID=137 RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
echo "       source $ROOT/.venv/bin/activate     # 루트 공용 venv (gripper 제외)"
echo ""
echo "  실행 순서/명령은 README.md 5장 참고."
echo "  (gripper 만 Isaac python.sh 로 실행 — venv 불필요)"
echo "============================================================"

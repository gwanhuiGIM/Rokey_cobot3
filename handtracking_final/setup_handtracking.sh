#!/usr/bin/env bash
set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$PROJECT_DIR/.venv"

echo "[1/5] Checking ROS2..."

if [ -f "/opt/ros/humble/setup.bash" ]; then
    ROS_SETUP="/opt/ros/humble/setup.bash"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    ROS_SETUP="/opt/ros/jazzy/setup.bash"
else
    echo "[ERROR] ROS2 Humble 또는 Jazzy를 찾을 수 없습니다."
    echo "ROS2를 먼저 설치해주세요."
    exit 1
fi

echo "ROS2 found: $ROS_SETUP"

echo "[2/5] Installing system packages..."
sudo apt update
sudo apt install -y \
    python3-venv \
    python3-pip \
    v4l-utils

echo "[3/5] Creating virtual environment..."

if [ ! -d "$VENV_DIR" ]; then
    source "$ROS_SETUP"
    python3 -m venv --system-site-packages "$VENV_DIR"
else
    echo "Virtual environment already exists."
fi

echo "[4/5] Installing Python packages..."
source "$ROS_SETUP"
source "$VENV_DIR/bin/activate"

python -m pip install --upgrade pip setuptools wheel
# (모듈별 requirements.txt 는 폐지 — 루트 master requirements.txt 로 일원화.
#  이 독립 스크립트는 handtracking 에 필요한 3개만 직접 설치한다.)
python -m pip install "numpy<2" opencv-python mediapipe

echo "[5/5] Checking required files..."

if [ ! -f "$PROJECT_DIR/hand_landmarker.task" ]; then
    echo "[ERROR] hand_landmarker.task 파일이 없습니다."
    exit 1
fi

if [ ! -f "$PROJECT_DIR/hand_trackerorigin.py" ]; then
    echo "[ERROR] hand_trackerorigin.py 파일이 없습니다."
    exit 1
fi

cat > "$PROJECT_DIR/run_handtracking.sh" <<'EOF'
#!/usr/bin/env bash
set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "[ERROR] ROS2 setup.bash를 찾을 수 없습니다."
    exit 1
fi

source "$PROJECT_DIR/.venv/bin/activate"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-136}"
export CAMERA_SOURCE="${CAMERA_SOURCE:-auto}"
export MIRROR_VIEW="${MIRROR_VIEW:-0}"
export SWAP_HAND_LABELS="${SWAP_HAND_LABELS:-0}"

cd "$PROJECT_DIR"

python3 hand_trackerorigin.py
EOF

chmod +x "$PROJECT_DIR/run_handtracking.sh"

echo
echo "=========================================="
echo "Setup complete."
echo
echo "Run:"
echo "  ./run_handtracking.sh"
echo
echo "Logitech camera example:"
echo "  CAMERA_SOURCE=4 ./run_handtracking.sh"
echo
echo "Mirror mode:"
echo "  CAMERA_SOURCE=4 MIRROR_VIEW=1 ./run_handtracking.sh"
echo "=========================================="
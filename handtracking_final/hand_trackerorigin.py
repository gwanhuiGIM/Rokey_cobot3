import cv2
import math
import os

# 기본 ROS2 Domain ID. 실행 환경에서 ROS_DOMAIN_ID를 따로 지정하면 그 값을 우선한다.
os.environ.setdefault("ROS_DOMAIN_ID", "137")

import sys
import threading
import time
from dataclasses import dataclass

import mediapipe as mp
import numpy as np
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

import rclpy
from geometry_msgs.msg import Point, Quaternion
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


# =====================================================
# Config
# =====================================================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.getenv(
    "HAND_MODEL_PATH",
    os.path.join(BASE_DIR, "hand_landmarker.task"),
)

# 로컬 USB/내장 카메라 사용.
# CAMERA_SOURCE=auto이면 0,1,2... 순서로 실제 프레임이 나오는 장치를 자동 탐색한다.
# 특정 장치를 고정하려면 예: CAMERA_SOURCE=0 python3 hand_tracker.py
CAMERA_SOURCE = os.getenv("CAMERA_SOURCE", "auto")
CAMERA_SCAN_MAX = int(os.getenv("CAMERA_SCAN_MAX", "6"))
DISPLAY_WIDTH = int(os.getenv("DISPLAY_WIDTH", "960"))
DISPLAY_HEIGHT = int(os.getenv("DISPLAY_HEIGHT", "540"))

# MediaPipe에는 더 작은 프레임을 넣고, 결과 좌표는 정규화 좌표이므로
# 화면 표시/좌표 계산은 DISPLAY 크기로 그대로 복원된다.
INFERENCE_WIDTH = int(os.getenv("INFERENCE_WIDTH", "640"))

# 화면을 거울처럼 좌우 반전할지 여부
MIRROR_VIEW = os.getenv("MIRROR_VIEW", "1").strip().lower() in {"1", "true", "yes", "on"}

# 좌우 손 라벨 교환. MediaPipe handedness가 카메라/미러 설정에 따라 반대로
# 나올 때(오른손인데 Left로 표시) 1로 켜서 Left<->Right 를 바꾼다.
SWAP_HAND_LABELS = os.getenv("SWAP_HAND_LABELS", "0").strip().lower() in {"1", "true", "yes", "on"}

# 거리 보정
REAL_PALM_WIDTH = 0.09
NEAR_CM = 30.0
FAR_CM = 100.0

# 좌표 발행 주기
PUB_INTERVAL = 1.0 / 30.0

# 손 위치 -> 로봇 EE 목표 위치 오프셋
# 손↔로봇(EE) 위치 차이 ≈ 20cm (수직 위). Y는 0, Z만 +0.20.
EE_Y_OFFSET = 0.0
EE_Z_OFFSET = 0.20
TABLE_HEIGHT = 1.0

# 손 추종 기본 위치 보정. hand_pos(raw)에 적용되어 EE 까지 함께 이동한다.
# HAND_Y_BACK_SHIFT: 양수면 Y로 더 뒤로. HAND_Z_DOWN_SHIFT: 양수면 더 아래로.
HAND_Y_BACK_SHIFT = 0.10
HAND_Z_DOWN_SHIFT = 0.0

# 카메라와 가까울수록 높은 Z, 멀수록 낮은 Z
HAND_Z_NEAR = 0.45
HAND_Z_FAR = 0.05

# 제스처
GESTURE_HOLD_SEC = 1.5
MODE_FOLLOW = "FOLLOW"
MODE_PLACE = "PLACE"
# 초기/대기 중립 모드. 로봇은 이 값을 구독/처리하지 않으므로 아무 동작 안 함
# (PLACE 도 FOLLOW 도 아니라 놓지도, 따라가지도 않음).
MODE_WAITING = "WAITING"

# 손바닥 앞/뒤 판정
# 반대로 잡히면 실행할 때 PALM_Z_SIGN=-1 사용
PALM_Z_SIGN = float(os.getenv("PALM_Z_SIGN", "1.0"))

# 정면/후면으로 확실히 인정하는 각도 기준.
# 1.0에 가까울수록 카메라 축과 거의 평행해야 한다.
PALM_ENTER_THRESHOLD = 0.55
PALM_EXIT_THRESHOLD = 0.30
PALM_FILTER_ALPHA = 0.25
PALM_STABLE_SEC = 0.20

# 카메라 중심에서 손 중심까지 이어주는 선
ORIGIN_LINE_COLOR = (175, 184, 196)

HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (0, 17), (17, 18), (18, 19), (19, 20),
]

# OpenCV는 BGR 순서
HAND_STYLE = {
    "Right": {
        "accent": (168, 236, 92),     # mint
        "bone": (140, 214, 94),
        "joint": (190, 255, 122),
        "label": (205, 255, 155),
    },
    "Left": {
        "accent": (255, 168, 96),     # electric blue
        "bone": (236, 146, 76),
        "joint": (255, 194, 126),
        "label": (255, 210, 160),
    },
}

PALM_LINE_COLOR = (235, 190, 92)

# Modern overlay palette
UI_CARD = (25, 28, 34)
UI_CARD_2 = (32, 36, 44)
UI_BORDER = (65, 71, 82)
UI_TEXT = (240, 243, 247)
UI_MUTED = (164, 172, 184)
UI_GREEN = (122, 226, 126)
UI_AMBER = (74, 187, 255)
UI_RED = (102, 102, 245)
UI_BLUE = (255, 164, 82)
UI_DARK = (13, 15, 19)

ROS_DOMAIN_DISPLAY = os.getenv("ROS_DOMAIN_ID", "137")


# =====================================================
# Camera
# =====================================================
def parse_camera_source(value: str):
    value = value.strip()

    if value.lower() == "auto":
        return None

    try:
        return int(value)
    except ValueError:
        # 동영상 파일, RTSP 주소 등도 필요하면 사용할 수 있다.
        return value


def _video_backend():
    # Linux에서는 V4L2를 명시하면 로컬 USB 카메라 지연이 더 안정적인 경우가 많다.
    if sys.platform.startswith("linux"):
        return cv2.CAP_V4L2
    return cv2.CAP_ANY


def _configure_camera(camera):
    # 카메라 해상도, FPS, 영상 포맷은 드라이버 기본값을 그대로 사용한다.
    # 실시간 지연을 줄이기 위해 버퍼 크기만 최소화한다.
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)


def _try_open_camera(source):
    backend = _video_backend() if isinstance(source, int) else cv2.CAP_ANY
    camera = cv2.VideoCapture(source, backend)
    _configure_camera(camera)

    if not camera.isOpened():
        camera.release()
        return None, None

    # isOpened()만 성공하고 실제 프레임은 나오지 않는 장치를 제외한다.
    for _ in range(8):
        ok, frame = camera.read()
        if ok and frame is not None and frame.size > 0:
            return camera, frame
        time.sleep(0.03)

    camera.release()
    return None, None


def open_local_camera(requested_source):
    """
    반환:
      camera, first_frame, active_source
    """
    if requested_source is not None:
        camera, first_frame = _try_open_camera(requested_source)
        if camera is None:
            raise RuntimeError(
                f"Camera open failed: CAMERA_SOURCE={requested_source}. "
                "Use `v4l2-ctl --list-devices` or try CAMERA_SOURCE=auto."
            )
        return camera, first_frame, requested_source

    for source in range(CAMERA_SCAN_MAX):
        camera, first_frame = _try_open_camera(source)
        if camera is not None:
            return camera, first_frame, source

    raise RuntimeError(
        "No working local camera found. "
        "Run `v4l2-ctl --list-devices`, then execute with CAMERA_SOURCE=<index>."
    )


class LatestFrameCamera:
    """
    카메라 읽기를 별도 스레드에서 계속 수행하고 가장 최신 프레임 하나만 보관한다.
    오래된 프레임 큐가 쌓이지 않아서 실시간 조작 지연이 줄어든다.
    """
    def __init__(self, camera, first_frame):
        self.camera = camera
        self.lock = threading.Lock()
        self.latest_frame = first_frame
        self.frame_id = 0
        self.running = True
        self.consecutive_failures = 0

        self.thread = threading.Thread(
            target=self._capture_loop,
            name="camera-capture",
            daemon=True,
        )
        self.thread.start()

    def _capture_loop(self):
        while self.running:
            ok, frame = self.camera.read()

            if not ok or frame is None:
                self.consecutive_failures += 1
                if self.consecutive_failures >= 30:
                    self.running = False
                    break
                time.sleep(0.01)
                continue

            self.consecutive_failures = 0

            with self.lock:
                self.latest_frame = frame
                self.frame_id += 1

    def read_latest(self, previous_frame_id):
        with self.lock:
            if self.latest_frame is None or self.frame_id == previous_frame_id:
                return None, previous_frame_id
            return self.latest_frame, self.frame_id

    def is_running(self):
        return self.running

    def release(self):
        self.running = False

        if self.thread.is_alive():
            self.thread.join(timeout=1.0)

        self.camera.release()


# =====================================================
# MediaPipe
# =====================================================
if not os.path.isfile(MODEL_PATH):
    raise FileNotFoundError(f"MediaPipe model not found: {MODEL_PATH}")

options = vision.HandLandmarkerOptions(
    base_options=python.BaseOptions(model_asset_path=MODEL_PATH),
    running_mode=vision.RunningMode.VIDEO,
    num_hands=2,
    min_hand_detection_confidence=0.55,
    min_hand_presence_confidence=0.55,
    min_tracking_confidence=0.55,
)
detector = vision.HandLandmarker.create_from_options(options)

requested_camera_source = parse_camera_source(CAMERA_SOURCE)
capture, first_frame, active_camera_source = open_local_camera(
    requested_camera_source
)
camera = LatestFrameCamera(capture, first_frame)
ACTIVE_CAMERA_DISPLAY = str(active_camera_source)


# =====================================================
# ROS2
# =====================================================
rclpy.init()
ros_node = Node("dual_hand_publisher")

stream_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# 상태/방향 토픽은 나중에 구독한 노드도 마지막 값을 받을 수 있게 유지한다.
state_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


@dataclass
class HandPublishers:
    raw: object
    xyz: object
    mode: object
    palm: object
    orient: object


publishers = {
    "Left": HandPublishers(
        raw=ros_node.create_publisher(Point, "/left_hand_raw", stream_qos),
        xyz=ros_node.create_publisher(Point, "/left_hand_xyz", stream_qos),
        mode=ros_node.create_publisher(String, "/left_hand_mode", state_qos),
        palm=ros_node.create_publisher(String, "/left_palm_direction", state_qos),
        orient=ros_node.create_publisher(
            Quaternion, "/left_hand_orient", stream_qos
        ),
    ),
    "Right": HandPublishers(
        raw=ros_node.create_publisher(Point, "/right_hand_raw", stream_qos),
        xyz=ros_node.create_publisher(Point, "/right_hand_xyz", stream_qos),
        mode=ros_node.create_publisher(String, "/right_hand_mode", state_qos),
        palm=ros_node.create_publisher(String, "/right_palm_direction", state_qos),
        orient=ros_node.create_publisher(
            Quaternion, "/right_hand_orient", stream_qos
        ),
    ),
}

# 기존 단일 오른손 코드와의 호환용
legacy_raw_pub = ros_node.create_publisher(Point, "/hand_raw", stream_qos)
legacy_xyz_pub = ros_node.create_publisher(Point, "/hand_xyz", stream_qos)
legacy_mode_pub = ros_node.create_publisher(String, "/hand_mode", state_qos)

# 대시보드용: 손 스켈레톤이 그려진 화면을 JPEG 압축 이미지로 발행.
# cv_bridge 없이 sensor_msgs/CompressedImage 로 바로 전송한다.
from sensor_msgs.msg import CompressedImage

HAND_IMAGE_TOPIC = "/hand_tracking/image/compressed"
PUBLISH_IMAGE = os.getenv("PUBLISH_IMAGE", "1").strip().lower() in {"1", "true", "yes", "on"}
IMAGE_JPEG_QUALITY = int(os.getenv("IMAGE_JPEG_QUALITY", "70"))
IMAGE_PUB_INTERVAL = 1.0 / float(os.getenv("IMAGE_PUB_FPS", "15"))

image_pub = ros_node.create_publisher(
    CompressedImage, HAND_IMAGE_TOPIC, stream_qos
)
_last_image_pub_time = 0.0


def publish_hand_image(frame):
    """손 추적 화면(BGR)을 JPEG 로 압축해 발행. FPS 제한 적용."""
    global _last_image_pub_time

    if not PUBLISH_IMAGE:
        return

    now = time.monotonic()
    if now - _last_image_pub_time < IMAGE_PUB_INTERVAL:
        return

    ok, buffer = cv2.imencode(
        ".jpg",
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), IMAGE_JPEG_QUALITY],
    )
    if not ok:
        return

    message = CompressedImage()
    message.format = "jpeg"
    message.data = buffer.tobytes()
    image_pub.publish(message)
    _last_image_pub_time = now


# =====================================================
# Coordinate processing
# =====================================================
class CoordProcessor:
    def __init__(self, alpha=0.25):
        self.alpha = alpha
        self.filtered_ee = None
        self.z_floor = TABLE_HEIGHT - 0.05

    def reset(self):
        self.filtered_ee = None

    def update(self, camera_x, camera_y, camera_distance):
        """
        반환값:
          hand_pos: 카메라 결과를 프로젝트 좌표로 바꾼 손 위치
          ee_pos:   오프셋 + 스무딩이 적용된 로봇 EE 목표 위치
        """
        near_m = NEAR_CM / 100.0
        far_m = FAR_CM / 100.0

        distance = float(np.clip(camera_distance, near_m, far_m))
        ratio = (distance - near_m) / (far_m - near_m)

        # 가까울수록 높고, 멀수록 낮다.
        relative_z = HAND_Z_NEAR + ratio * (HAND_Z_FAR - HAND_Z_NEAR)

        hand_x = float(camera_x)
        hand_y = float(camera_y) - 0.30 - HAND_Y_BACK_SHIFT
        hand_z = TABLE_HEIGHT + relative_z - HAND_Z_DOWN_SHIFT

        hand_pos = np.array([hand_x, hand_y, hand_z], dtype=float)

        unfiltered_ee = np.array(
            [
                hand_x,
                hand_y + EE_Y_OFFSET,
                max(hand_z + EE_Z_OFFSET, self.z_floor),
            ],
            dtype=float,
        )

        if self.filtered_ee is None:
            self.filtered_ee = unfiltered_ee.copy()
        else:
            self.filtered_ee = (
                self.alpha * unfiltered_ee
                + (1.0 - self.alpha) * self.filtered_ee
            )

        return hand_pos, self.filtered_ee.copy()


processors = {
    "Left": CoordProcessor(alpha=0.25),
    "Right": CoordProcessor(alpha=0.25),
}


# =====================================================
# Hand pose detection
# =====================================================
def classify_hand_pose(hand) -> str:
    """
    손 회전에 덜 민감하도록 손바닥 중심으로부터 손가락 끝과 PIP까지의
    3차원 거리를 비교한다.

    반환:
      FIST  : 네 손가락 중 3개 이상 확실히 접힘
      OPEN  : 네 손가락 중 3개 이상 확실히 펴짐
      OTHER : 중간 자세, 반쯤 편 손 등
    """
    palm_center = np.mean(
        np.array(
            [[hand[i].x, hand[i].y, hand[i].z] for i in [0, 5, 9, 13, 17]],
            dtype=float,
        ),
        axis=0,
    )

    tip_ids = [8, 12, 16, 20]
    pip_ids = [6, 10, 14, 18]

    curled = 0
    extended = 0

    for tip_id, pip_id in zip(tip_ids, pip_ids):
        tip = np.array(
            [hand[tip_id].x, hand[tip_id].y, hand[tip_id].z],
            dtype=float,
        )
        pip = np.array(
            [hand[pip_id].x, hand[pip_id].y, hand[pip_id].z],
            dtype=float,
        )

        tip_distance = np.linalg.norm(tip - palm_center)
        pip_distance = np.linalg.norm(pip - palm_center)

        if tip_distance < pip_distance * 0.98:
            curled += 1
        elif tip_distance > pip_distance * 1.10:
            extended += 1

    if curled >= 3:
        return "FIST"
    if extended >= 3:
        return "OPEN"
    return "OTHER"


# =====================================================
# Palm-facing detection
# =====================================================
def _point3(landmarks, index):
    point = landmarks[index]
    return np.array([point.x, point.y, point.z], dtype=float)


def compute_palm_camera_score(landmarks, label: str) -> float:
    """
    카메라 축에 대한 손바닥 법선 점수:
      +1에 가까움: 손바닥이 카메라를 향함
      -1에 가까움: 손등이 카메라를 향함
       0에 가까움: 손이 옆으로 기울어짐

    가능하면 MediaPipe world landmarks를 사용한다.
    """
    wrist = _point3(landmarks, 0)
    index_mcp = _point3(landmarks, 5)
    middle_mcp = _point3(landmarks, 9)
    ring_mcp = _point3(landmarks, 13)
    pinky_mcp = _point3(landmarks, 17)

    # 손바닥 평면을 여러 삼각형으로 계산해 z 노이즈를 완화한다.
    pairs = [
        (index_mcp - wrist, middle_mcp - wrist),
        (middle_mcp - wrist, ring_mcp - wrist),
        (ring_mcp - wrist, pinky_mcp - wrist),
        (index_mcp - wrist, pinky_mcp - wrist),
    ]

    normals = []

    for vector_a, vector_b in pairs:
        normal = np.cross(vector_a, vector_b)
        norm = np.linalg.norm(normal)

        if norm > 1e-8:
            normals.append(normal / norm)

    if not normals:
        return 0.0

    normal = np.mean(normals, axis=0)
    normal_norm = np.linalg.norm(normal)

    if normal_norm < 1e-8:
        return 0.0

    normal /= normal_norm

    # 좌우 손의 관절 순서에 따른 법선 방향을 같은 기준으로 맞춘다.
    if label == "Left":
        normal *= -1.0

    # MediaPipe 카메라 좌표에서 음의 z 방향을 카메라 쪽으로 사용.
    # 실제 환경에서 반대면 PALM_Z_SIGN=-1로 실행한다.
    camera_score = -float(normal[2]) * PALM_Z_SIGN
    return float(np.clip(camera_score, -1.0, 1.0))


class PalmFacingFilter:
    """
    순간적인 landmark 떨림 때문에 TOWARD/AWAY가 바뀌지 않도록
    EMA + 진입/이탈 임계값 + 짧은 안정화 시간을 사용한다.
    """
    def __init__(self):
        self.filtered_score = None
        self.state = "SIDEWAYS"
        self.candidate = None
        self.candidate_started = None

    def reset(self):
        self.filtered_score = None
        self.state = "SIDEWAYS"
        self.candidate = None
        self.candidate_started = None

    def update(self, raw_score: float):
        now = time.monotonic()

        if self.filtered_score is None:
            self.filtered_score = raw_score
        else:
            self.filtered_score = (
                PALM_FILTER_ALPHA * raw_score
                + (1.0 - PALM_FILTER_ALPHA) * self.filtered_score
            )

        score = self.filtered_score

        # 현재 상태 유지 조건. 점수가 약해지면 SIDEWAYS로 바로 빠져
        # 제스처 유지 타이머가 잘못 누적되지 않게 한다.
        if self.state == "TOWARD_CAMERA" and score >= PALM_EXIT_THRESHOLD:
            self.candidate = None
            self.candidate_started = None
            return self.state, score

        if self.state == "AWAY_CAMERA" and score <= -PALM_EXIT_THRESHOLD:
            self.candidate = None
            self.candidate_started = None
            return self.state, score

        if self.state != "SIDEWAYS":
            self.state = "SIDEWAYS"
            self.candidate = None
            self.candidate_started = None

        if score >= PALM_ENTER_THRESHOLD:
            desired = "TOWARD_CAMERA"
        elif score <= -PALM_ENTER_THRESHOLD:
            desired = "AWAY_CAMERA"
        else:
            self.candidate = None
            self.candidate_started = None
            return self.state, score

        if self.candidate != desired:
            self.candidate = desired
            self.candidate_started = now
            return self.state, score

        if now - self.candidate_started >= PALM_STABLE_SEC:
            self.state = desired
            self.candidate = None
            self.candidate_started = None

        return self.state, score


palm_filters = {
    "Left": PalmFacingFilter(),
    "Right": PalmFacingFilter(),
}


# =====================================================
# Hand orientation (3D rotation) estimation
# =====================================================
# MediaPipe 카메라 좌표 -> Isaac Sim 월드 좌표(Z-up) 매핑.
# 위치 변환(hand_x=camera_x, hand_y=camera_up, hand_z=가까울수록 +)과
# 동일한 규약을 회전에도 적용한다.
#   world_x =  mp_x   (오른쪽)
#   world_y = -mp_y   (위, MediaPipe는 y가 아래 방향)
#   world_z = -mp_z   (카메라 쪽이 +Z)
# det(M) = +1 이므로 손방향이 뒤집히지 않는 정상 회전이다.
_MP_TO_WORLD = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
    ],
    dtype=float,
)


def rotation_matrix_to_quaternion(matrix):
    """3x3 회전행렬 -> Isaac Sim 순서 quaternion (w, x, y, z)."""
    trace = matrix[0, 0] + matrix[1, 1] + matrix[2, 2]

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (matrix[2, 1] - matrix[1, 2]) / s
        y = (matrix[0, 2] - matrix[2, 0]) / s
        z = (matrix[1, 0] - matrix[0, 1]) / s
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / s
        x = 0.25 * s
        y = (matrix[0, 1] + matrix[1, 0]) / s
        z = (matrix[0, 2] + matrix[2, 0]) / s
    elif matrix[1, 1] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / s
        x = (matrix[0, 1] + matrix[1, 0]) / s
        y = 0.25 * s
        z = (matrix[1, 2] + matrix[2, 1]) / s
    else:
        s = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / s
        x = (matrix[0, 2] + matrix[2, 0]) / s
        y = (matrix[1, 2] + matrix[2, 1]) / s
        z = 0.25 * s

    quaternion = np.array([w, x, y, z], dtype=float)
    norm = np.linalg.norm(quaternion)

    if norm < 1e-12:
        return None

    return quaternion / norm


def compute_hand_orientation(landmarks, label: str):
    """
    손 landmark로부터 손바닥 좌표계를 만들고 Isaac Sim 월드 기준
    quaternion(w, x, y, z)을 계산한다.

    손 로컬 축 정의:
      Y축: wrist(0) -> middle MCP(9)  손가락 방향
      가로: index MCP(5) -> pinky MCP(17)
      Z축: 손바닥 법선 = 가로 x Y
      X축: Y x Z 로 재직교화

    왼손은 가로축을 반전해 좌우 손이 같은 로컬 규약을 갖게 한다.
    가능하면 world landmarks를 사용한다.
    """
    wrist = _point3(landmarks, 0)
    index_mcp = _point3(landmarks, 5)
    pinky_mcp = _point3(landmarks, 17)
    middle_mcp = _point3(landmarks, 9)

    y_axis = middle_mcp - wrist
    y_norm = np.linalg.norm(y_axis)

    if y_norm < 1e-8:
        return None

    y_axis = y_axis / y_norm

    lateral = pinky_mcp - index_mcp

    if label == "Left":
        lateral = -lateral

    z_axis = np.cross(lateral, y_axis)
    z_norm = np.linalg.norm(z_axis)

    if z_norm < 1e-8:
        return None

    z_axis = z_axis / z_norm

    x_axis = np.cross(y_axis, z_axis)
    x_norm = np.linalg.norm(x_axis)

    if x_norm < 1e-8:
        return None

    x_axis = x_axis / x_norm

    rotation_mp = np.column_stack((x_axis, y_axis, z_axis))
    rotation_world = _MP_TO_WORLD @ rotation_mp

    return rotation_matrix_to_quaternion(rotation_world)


class OrientationFilter:
    """
    quaternion 떨림 완화용 EMA.
    부호(반구)를 직전 결과에 맞춰 정렬한 뒤 보간하고 정규화한다.
    """

    def __init__(self, alpha: float = 0.35):
        self.alpha = float(alpha)
        self.filtered = None

    def reset(self):
        self.filtered = None

    def update(self, quaternion):
        if quaternion is None:
            return self.filtered

        quaternion = np.asarray(quaternion, dtype=float)

        if self.filtered is None:
            self.filtered = quaternion
            return self.filtered

        # 부호 정렬: 같은 회전이라도 q와 -q가 섞이면 보간이 튄다.
        if float(np.dot(self.filtered, quaternion)) < 0.0:
            quaternion = -quaternion

        blended = (
            self.alpha * quaternion
            + (1.0 - self.alpha) * self.filtered
        )

        norm = np.linalg.norm(blended)

        if norm < 1e-12:
            return self.filtered

        self.filtered = blended / norm
        return self.filtered


orientation_filters = {
    "Left": OrientationFilter(alpha=0.35),
    "Right": OrientationFilter(alpha=0.35),
}


# =====================================================
# Gesture mode
# =====================================================
class GestureModeTracker:
    """
    FOLLOW:
      TOWARD_CAMERA + FIST 상태를 1.5초 연속 유지

    PLACE:
      AWAY_CAMERA + OPEN 상태를 1.5초 연속 유지

    손바닥이 옆을 향하거나 손 자세가 조건과 달라지거나,
    손 검출이 끊기면 유지 타이머를 즉시 초기화한다.
    """
    def __init__(self):
        # 기본 = WAITING(중립). PLACE/FOLLOW 는 제스처로만 발생한다.
        # (PLACE 로 시작하면 로봇이 TRACKING 진입 즉시 놓아버리는 문제 방지)
        self.mode = MODE_WAITING
        self.candidate_mode = None
        self.candidate_started = None

    def reset(self):
        self.mode = MODE_WAITING
        self.candidate_mode = None
        self.candidate_started = None

    def cancel_candidate(self):
        self.candidate_mode = None
        self.candidate_started = None

    def update(self, hand_pose: str, palm_direction: str):
        now = time.monotonic()

        if hand_pose == "FIST" and palm_direction == "TOWARD_CAMERA":
            desired_mode = MODE_FOLLOW
        elif hand_pose == "OPEN" and palm_direction == "AWAY_CAMERA":
            desired_mode = MODE_PLACE
        else:
            self.cancel_candidate()
            return False, 0.0, None

        if desired_mode == self.mode:
            self.cancel_candidate()
            return False, 0.0, desired_mode

        if self.candidate_mode != desired_mode:
            self.candidate_mode = desired_mode
            self.candidate_started = now
            return False, 0.0, desired_mode

        elapsed = now - self.candidate_started

        if elapsed >= GESTURE_HOLD_SEC:
            self.mode = desired_mode
            self.cancel_candidate()
            return True, GESTURE_HOLD_SEC, desired_mode

        return False, elapsed, desired_mode


gesture_trackers = {
    "Left": GestureModeTracker(),
    "Right": GestureModeTracker(),
}


# =====================================================
# Drawing helpers — modern UI
# =====================================================
def draw_rounded_rect(image, pt1, pt2, color, radius=16, thickness=-1):
    x1, y1 = pt1
    x2, y2 = pt2
    radius = max(1, min(radius, (x2 - x1) // 2, (y2 - y1) // 2))

    if thickness < 0:
        cv2.rectangle(image, (x1 + radius, y1), (x2 - radius, y2), color, -1)
        cv2.rectangle(image, (x1, y1 + radius), (x2, y2 - radius), color, -1)
        cv2.circle(image, (x1 + radius, y1 + radius), radius, color, -1)
        cv2.circle(image, (x2 - radius, y1 + radius), radius, color, -1)
        cv2.circle(image, (x1 + radius, y2 - radius), radius, color, -1)
        cv2.circle(image, (x2 - radius, y2 - radius), radius, color, -1)
        return

    cv2.line(image, (x1 + radius, y1), (x2 - radius, y1), color, thickness, cv2.LINE_AA)
    cv2.line(image, (x1 + radius, y2), (x2 - radius, y2), color, thickness, cv2.LINE_AA)
    cv2.line(image, (x1, y1 + radius), (x1, y2 - radius), color, thickness, cv2.LINE_AA)
    cv2.line(image, (x2, y1 + radius), (x2, y2 - radius), color, thickness, cv2.LINE_AA)
    cv2.ellipse(image, (x1 + radius, y1 + radius), (radius, radius), 180, 0, 90, color, thickness, cv2.LINE_AA)
    cv2.ellipse(image, (x2 - radius, y1 + radius), (radius, radius), 270, 0, 90, color, thickness, cv2.LINE_AA)
    cv2.ellipse(image, (x2 - radius, y2 - radius), (radius, radius), 0, 0, 90, color, thickness, cv2.LINE_AA)
    cv2.ellipse(image, (x1 + radius, y2 - radius), (radius, radius), 90, 0, 90, color, thickness, cv2.LINE_AA)


def draw_text(frame, text, position, color=UI_TEXT, scale=0.46, thickness=1):
    cv2.putText(
        frame,
        text,
        position,
        cv2.FONT_HERSHEY_DUPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def draw_pill(frame, text, x, y, bg_color, text_color=UI_TEXT, scale=0.40):
    (text_w, text_h), _ = cv2.getTextSize(
        text,
        cv2.FONT_HERSHEY_DUPLEX,
        scale,
        1,
    )
    width = text_w + 20
    height = 24
    draw_rounded_rect(frame, (x, y), (x + width, y + height), bg_color, 12, -1)
    draw_text(
        frame,
        text,
        (x + 10, y + 16 + max(0, (text_h - 10) // 2)),
        text_color,
        scale,
        1,
    )
    return width


def draw_skeleton(frame, hand, width, height, style):
    points = [(int(lm.x * width), int(lm.y * height)) for lm in hand]

    for start, end in HAND_CONNECTIONS:
        cv2.line(frame, points[start], points[end], UI_DARK, 5, cv2.LINE_AA)
        cv2.line(
            frame,
            points[start],
            points[end],
            style["bone"],
            2,
            cv2.LINE_AA,
        )

    for index, (x, y) in enumerate(points):
        radius = 6 if index in (0, 9) else 4
        cv2.circle(frame, (x, y), radius + 3, UI_DARK, -1, cv2.LINE_AA)
        cv2.circle(frame, (x, y), radius, style["joint"], -1, cv2.LINE_AA)


def draw_palm_width_line(frame, hand, width, height):
    x1, y1 = int(hand[5].x * width), int(hand[5].y * height)
    x2, y2 = int(hand[17].x * width), int(hand[17].y * height)

    cv2.line(frame, (x1, y1), (x2, y2), UI_DARK, 7, cv2.LINE_AA)
    cv2.line(frame, (x1, y1), (x2, y2), PALM_LINE_COLOR, 3, cv2.LINE_AA)

    return math.hypot(x2 - x1, y2 - y1)


def draw_origin_to_hand_line(frame, origin, hand_center, label, dx_px, dy_px):
    accent = HAND_STYLE[label]["accent"]

    cv2.line(frame, origin, hand_center, UI_DARK, 5, cv2.LINE_AA)
    cv2.line(frame, origin, hand_center, ORIGIN_LINE_COLOR, 2, cv2.LINE_AA)
    cv2.circle(frame, hand_center, 7, UI_DARK, -1, cv2.LINE_AA)
    cv2.circle(frame, hand_center, 4, accent, -1, cv2.LINE_AA)

    midpoint = (
        (origin[0] + hand_center[0]) // 2,
        (origin[1] + hand_center[1]) // 2,
    )

    tag = f"{dx_px:+d}, {dy_px:+d}px"
    (tw, _), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_DUPLEX, 0.38, 1)
    tag_x = midpoint[0] - tw // 2
    tag_y = midpoint[1] - 22

    draw_rounded_rect(
        frame,
        (tag_x - 8, tag_y - 15),
        (tag_x + tw + 8, tag_y + 6),
        UI_CARD,
        8,
        -1,
    )
    draw_text(frame, tag, (tag_x, tag_y), accent, 0.38, 1)


def draw_center_origin(frame, center):
    x, y = center
    cv2.circle(frame, (x, y), 11, UI_DARK, -1, cv2.LINE_AA)
    cv2.circle(frame, (x, y), 8, (235, 239, 244), 1, cv2.LINE_AA)
    cv2.circle(frame, (x, y), 2, (235, 239, 244), -1, cv2.LINE_AA)


def draw_coordinate_box(frame, x, y, width, title, values, accent):
    draw_rounded_rect(
        frame,
        (x, y),
        (x + width, y + 42),
        UI_CARD_2,
        10,
        -1,
    )
    draw_rounded_rect(
        frame,
        (x, y),
        (x + width, y + 42),
        UI_BORDER,
        10,
        1,
    )
    draw_text(frame, title, (x + 10, y + 16), UI_MUTED, 0.34, 1)
    draw_text(frame, values, (x + 10, y + 34), accent, 0.42, 1)


def draw_hand_panel(
    frame,
    label,
    detected,
    mode,
    candidate_mode,
    elapsed,
    hand_pos,
    ee_pos,
    publishing,
):
    _, width = frame.shape[:2]
    style = HAND_STYLE[label]

    card_width = width // 2 - 18
    card_height = 178
    card_x = 8 if label == "Left" else width // 2 + 10
    card_y = 8

    # Translucent card
    overlay = frame.copy()
    draw_rounded_rect(
        overlay,
        (card_x, card_y),
        (card_x + card_width, card_y + card_height),
        UI_CARD,
        18,
        -1,
    )
    cv2.addWeighted(overlay, 0.90, frame, 0.10, 0, frame)
    draw_rounded_rect(
        frame,
        (card_x, card_y),
        (card_x + card_width, card_y + card_height),
        UI_BORDER,
        18,
        1,
    )

    # Accent rail
    draw_rounded_rect(
        frame,
        (card_x + 7, card_y + 12),
        (card_x + 11, card_y + card_height - 12),
        style["accent"],
        2,
        -1,
    )

    draw_text(
        frame,
        f"{label.upper()} HAND",
        (card_x + 22, card_y + 25),
        UI_TEXT,
        0.55,
        2,
    )

    mode_color = UI_BLUE if mode == MODE_FOLLOW else UI_AMBER
    pub_color = UI_GREEN if publishing else (58, 63, 72)

    mode_w = draw_pill(
        frame,
        mode,
        card_x + card_width - 168,
        card_y + 10,
        mode_color,
        UI_DARK,
        0.38,
    )
    draw_pill(
        frame,
        "LIVE" if publishing else "IDLE",
        card_x + card_width - 82,
        card_y + 10,
        pub_color,
        UI_DARK if publishing else UI_MUTED,
        0.36,
    )

    pose = detected["pose"]
    direction = detected["palm_direction"]
    score = detected["palm_score"]

    pose_color = UI_GREEN if pose == "OPEN" else UI_BLUE if pose == "FIST" else UI_MUTED
    draw_pill(frame, pose, card_x + 22, card_y + 38, pose_color, UI_DARK, 0.35)

    palm_text = f"{direction}  {score:+.2f}"
    draw_text(
        frame,
        palm_text,
        (card_x + 98, card_y + 55),
        UI_MUTED,
        0.40,
        1,
    )

    box_width = (card_width - 54) // 2
    if hand_pos is None:
        raw_values = "calibration required"
        ee_values = "calibration required"
    else:
        raw_values = f"{hand_pos[0]:+.3f}  {hand_pos[1]:+.3f}  {hand_pos[2]:.3f}"
        ee_values = f"{ee_pos[0]:+.3f}  {ee_pos[1]:+.3f}  {ee_pos[2]:.3f}"

    draw_coordinate_box(
        frame,
        card_x + 22,
        card_y + 70,
        box_width,
        "HAND / RAW   X   Y   Z",
        raw_values,
        (126, 222, 255),
    )
    draw_coordinate_box(
        frame,
        card_x + 32 + box_width,
        card_y + 70,
        box_width,
        "EE TARGET   X   Y   Z",
        ee_values,
        UI_GREEN,
    )

    camera_distance = detected["camera_position"][2]
    if camera_distance is None:
        meta = "DIST --   |   PIXEL --"
    else:
        meta = (
            f"DIST {camera_distance:.3f} m   |   "
            f"PIXEL {detected['pixel'][0]}, {detected['pixel'][1]}"
        )
    draw_text(frame, meta, (card_x + 22, card_y + 128), UI_MUTED, 0.37, 1)

    if candidate_mode is None or candidate_mode == mode:
        condition = "FOLLOW  front palm + fist   |   PLACE  back palm + open"
        progress = 0.0
    else:
        required = (
            "FRONT + FIST"
            if candidate_mode == MODE_FOLLOW
            else "BACK + OPEN"
        )
        condition = (
            f"{candidate_mode}  {required}   "
            f"{elapsed:.1f}/{GESTURE_HOLD_SEC:.1f}s"
        )
        progress = min(elapsed / GESTURE_HOLD_SEC, 1.0)

    draw_text(frame, condition, (card_x + 22, card_y + 151), UI_TEXT, 0.36, 1)

    bar_x1 = card_x + 22
    bar_x2 = card_x + card_width - 18
    bar_y1 = card_y + 161
    bar_y2 = card_y + 168

    draw_rounded_rect(
        frame,
        (bar_x1, bar_y1),
        (bar_x2, bar_y2),
        (49, 54, 63),
        4,
        -1,
    )

    if progress > 0.0:
        fill_x = bar_x1 + max(8, int((bar_x2 - bar_x1) * progress))
        draw_rounded_rect(
            frame,
            (bar_x1, bar_y1),
            (fill_x, bar_y2),
            style["accent"],
            4,
            -1,
        )


def draw_missing_hand_panel(frame, label, mode):
    _, width = frame.shape[:2]
    style = HAND_STYLE[label]
    card_width = width // 2 - 18
    card_height = 178
    card_x = 8 if label == "Left" else width // 2 + 10
    card_y = 8

    overlay = frame.copy()
    draw_rounded_rect(
        overlay,
        (card_x, card_y),
        (card_x + card_width, card_y + card_height),
        UI_CARD,
        18,
        -1,
    )
    cv2.addWeighted(overlay, 0.86, frame, 0.14, 0, frame)
    draw_rounded_rect(
        frame,
        (card_x, card_y),
        (card_x + card_width, card_y + card_height),
        UI_BORDER,
        18,
        1,
    )

    draw_text(
        frame,
        f"{label.upper()} HAND",
        (card_x + 22, card_y + 28),
        UI_TEXT,
        0.55,
        2,
    )
    draw_pill(
        frame,
        mode,
        card_x + card_width - 92,
        card_y + 12,
        UI_BLUE if mode == MODE_FOLLOW else UI_AMBER,
        UI_DARK,
        0.38,
    )

    draw_text(
        frame,
        "NO HAND DETECTED",
        (card_x + 22, card_y + 85),
        style["accent"],
        0.70,
        2,
    )
    draw_text(
        frame,
        "Gesture timer paused and reset",
        (card_x + 22, card_y + 112),
        UI_MUTED,
        0.42,
        1,
    )


def draw_calibration_banner(frame, text):
    height, width = frame.shape[:2]
    card_width = min(610, width - 60)
    card_height = 76
    x1 = (width - card_width) // 2
    y1 = height // 2 - card_height // 2
    x2 = x1 + card_width
    y2 = y1 + card_height

    overlay = frame.copy()
    draw_rounded_rect(overlay, (x1, y1), (x2, y2), UI_CARD, 20, -1)
    cv2.addWeighted(overlay, 0.93, frame, 0.07, 0, frame)
    draw_rounded_rect(frame, (x1, y1), (x2, y2), UI_BLUE, 20, 1)

    draw_text(frame, "CALIBRATION", (x1 + 24, y1 + 28), UI_BLUE, 0.48, 2)
    draw_text(frame, text, (x1 + 24, y1 + 55), UI_TEXT, 0.58, 1)


def draw_footer(frame, fps, recording, calibration_step):
    height, width = frame.shape[:2]
    bar_h = 30
    y1 = height - bar_h

    overlay = frame.copy()
    cv2.rectangle(overlay, (0, y1), (width, height), UI_DARK, -1)
    cv2.addWeighted(overlay, 0.88, frame, 0.12, 0, frame)

    state = "RUNNING" if recording else f"CALIBRATION {calibration_step}/2"
    state_color = UI_GREEN if recording else UI_AMBER

    draw_text(frame, "DUAL HAND TELEOP", (14, height - 10), UI_TEXT, 0.42, 2)
    draw_text(
        frame,
        f"ROS DOMAIN {ROS_DOMAIN_DISPLAY}",
        (width // 2 - 105, height - 10),
        UI_MUTED,
        0.38,
        1,
    )
    draw_text(
        frame,
        f"CAM {ACTIVE_CAMERA_DISPLAY}   {fps:4.1f} FPS",
        (width - 180, height - 10),
        UI_MUTED,
        0.38,
        1,
    )
    cv2.circle(frame, (width // 2 - 128, height - 14), 4, state_color, -1, cv2.LINE_AA)


# =====================================================
# Math / publishing helpers
# =====================================================
def compute_camera_position(palm_px, dx_px, dy_px, focal_length):
    """
    카메라 중심 기준 손 위치:
      x/y: 손바닥 폭으로 환산한 화면 중심 대비 미터 거리
      z:   핀홀 모델로 계산한 카메라-손 거리
    """
    if focal_length is None or palm_px <= 0:
        return None, None, None

    distance_m = focal_length * REAL_PALM_WIDTH / palm_px
    meter_per_pixel = REAL_PALM_WIDTH / palm_px

    return (
        dx_px * meter_per_pixel,
        -dy_px * meter_per_pixel,
        distance_m,
    )


def compute_focal(near_px, far_px):
    f_near = near_px * (NEAR_CM / 100.0) / REAL_PALM_WIDTH
    f_far = far_px * (FAR_CM / 100.0) / REAL_PALM_WIDTH
    return (f_near + f_far) / 2.0


def publish_mode(label, mode):
    publishers[label].mode.publish(String(data=mode))

    if label == "Right":
        legacy_mode_pub.publish(String(data=mode))


def publish_palm_direction(label, direction):
    publishers[label].palm.publish(String(data=direction))


def publish_orientation(label, quaternion):
    """손 회전 quaternion을 발행한다. 입력 순서는 (w, x, y, z)."""
    if quaternion is None:
        return

    publishers[label].orient.publish(
        Quaternion(
            w=float(quaternion[0]),
            x=float(quaternion[1]),
            y=float(quaternion[2]),
            z=float(quaternion[3]),
        )
    )


def publish_coordinates(label, hand_pos, ee_pos):
    """
    ROS Point에는 float64 전체 정밀도로 발행한다.
    화면 오버레이에서만 소수점 셋째 자리로 보기 좋게 표시한다.
    """
    raw_message = Point(
        x=float(hand_pos[0]),
        y=float(hand_pos[1]),
        z=float(hand_pos[2]),
    )
    xyz_message = Point(
        x=float(ee_pos[0]),
        y=float(ee_pos[1]),
        z=float(ee_pos[2]),
    )

    publishers[label].raw.publish(raw_message)
    publishers[label].xyz.publish(xyz_message)

    if label == "Right":
        legacy_raw_pub.publish(raw_message)
        legacy_xyz_pub.publish(xyz_message)


# =====================================================
# Runtime state
# =====================================================
near_px = None
far_px = None
focal_length = None
calibration_step = 0
recording = False

last_publish_time = {
    "Left": 0.0,
    "Right": 0.0,
}
last_palm_direction = {
    "Left": None,
    "Right": None,
}
last_mode_heartbeat = {
    "Left": 0.0,
    "Right": 0.0,
}

fps_value = 0.0
fps_frames = 0
fps_window_started = time.monotonic()

last_camera_frame_id = -1
last_mp_timestamp_ms = -1


print("=" * 78)
print("Dual Hand Tracker - local camera / optimized pipeline")
print(f"Requested camera: {CAMERA_SOURCE}")
print(f"Active camera: {ACTIVE_CAMERA_DISPLAY}")
print(f"Inference width: {INFERENCE_WIDTH}px")
print(f"Mirror view: {'ON' if MIRROR_VIEW else 'OFF'}")
print(f"Swap hand labels: {'ON' if SWAP_HAND_LABELS else 'OFF'}")
print("SPACE: 30cm calibration -> 100cm calibration -> start")
print("R: reset | Q/ESC: quit")
print("FOLLOW: palm toward camera + fist for 1.5 sec")
print("PLACE : palm away from camera + open hand for 1.5 sec")
print("Near hand -> high Z | Far hand -> low Z")
print("=" * 78)


try:
    while True:
        frame, new_frame_id = camera.read_latest(last_camera_frame_id)

        if frame is None:
            if not camera.is_running():
                print("[ERROR] Camera frame read failed repeatedly")
                break

            # 같은 프레임을 여러 번 추론하지 않는다.
            time.sleep(0.001)
            continue

        last_camera_frame_id = new_frame_id
        frame = cv2.resize(
            frame,
            (DISPLAY_WIDTH, DISPLAY_HEIGHT),
            interpolation=cv2.INTER_AREA,
        )

        fps_frames += 1
        fps_now = time.monotonic()
        fps_elapsed = fps_now - fps_window_started
        if fps_elapsed >= 0.5:
            fps_value = fps_frames / fps_elapsed
            fps_frames = 0
            fps_window_started = fps_now

        if MIRROR_VIEW:
            frame = cv2.flip(frame, 1)

        height, width = frame.shape[:2]
        origin = (width // 2, height // 2)

        inference_height = max(
            1,
            int(height * INFERENCE_WIDTH / width),
        )
        inference_frame = cv2.resize(
            frame,
            (INFERENCE_WIDTH, inference_height),
            interpolation=cv2.INTER_AREA,
        )
        rgb = cv2.cvtColor(inference_frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=rgb,
        )

        mp_timestamp_ms = time.monotonic_ns() // 1_000_000
        if mp_timestamp_ms <= last_mp_timestamp_ms:
            mp_timestamp_ms = last_mp_timestamp_ms + 1
        last_mp_timestamp_ms = mp_timestamp_ms

        result = detector.detect_for_video(
            mp_image,
            mp_timestamp_ms,
        )

        draw_center_origin(frame, origin)

        detected_hands = {}
        calibration_candidates = []

        if result.hand_landmarks:
            for index, image_hand in enumerate(result.hand_landmarks):
                raw_label = result.handedness[index][0].category_name

                # MediaPipe handedness는 미러링된 입력을 기준으로 한다.
                # 카메라/미러 설정에 따라 좌우가 반대로 잡히면
                # SWAP_HAND_LABELS=1 로 Left<->Right 를 교환한다.
                label = raw_label
                if SWAP_HAND_LABELS:
                    label = "Left" if raw_label == "Right" else "Right"

                style = HAND_STYLE[label]

                # 손바닥 앞/뒤 판정에는 world landmark가 더 안정적이다.
                if (
                    hasattr(result, "hand_world_landmarks")
                    and result.hand_world_landmarks
                    and index < len(result.hand_world_landmarks)
                ):
                    orientation_hand = result.hand_world_landmarks[index]
                else:
                    orientation_hand = image_hand

                draw_skeleton(frame, image_hand, width, height, style)
                palm_px = draw_palm_width_line(
                    frame,
                    image_hand,
                    width,
                    height,
                )
                calibration_candidates.append(palm_px)

                center_landmark = image_hand[9]
                pixel_x = int(center_landmark.x * width)
                pixel_y = int(center_landmark.y * height)

                dx_px = pixel_x - origin[0]
                dy_px = pixel_y - origin[1]

                draw_origin_to_hand_line(
                    frame,
                    origin,
                    (pixel_x, pixel_y),
                    label,
                    dx_px,
                    dy_px,
                )

                hand_pose = classify_hand_pose(image_hand)

                raw_palm_score = compute_palm_camera_score(
                    orientation_hand,
                    label,
                )
                palm_direction, filtered_palm_score = palm_filters[label].update(
                    raw_palm_score
                )

                raw_orientation = compute_hand_orientation(
                    orientation_hand,
                    label,
                )
                hand_orientation = orientation_filters[label].update(
                    raw_orientation
                )

                camera_position = compute_camera_position(
                    palm_px,
                    dx_px,
                    dy_px,
                    focal_length,
                )

                detected_hands[label] = {
                    "pose": hand_pose,
                    "palm_direction": palm_direction,
                    "palm_score": filtered_palm_score,
                    "camera_position": camera_position,
                    "pixel": (pixel_x, pixel_y),
                    "palm_px": palm_px,
                    "orientation": hand_orientation,
                }

                draw_text(
                    frame,
                    f"{label.upper()}  {hand_pose}  "
                    f"{palm_direction} {filtered_palm_score:+.2f}",
                    (pixel_x + 12, max(182, pixel_y - 12)),
                    style["label"],
                    0.50,
                    2,
                )

        calibration_palm_px = (
            max(calibration_candidates)
            if calibration_candidates
            else 0.0
        )

        now = time.monotonic()

        for label in ("Left", "Right"):
            info = detected_hands.get(label)

            if info is None:
                gesture_trackers[label].cancel_candidate()
                draw_missing_hand_panel(
                    frame,
                    label,
                    gesture_trackers[label].mode,
                )
                continue

            hand_pos = None
            ee_pos = None

            camera_x, camera_y, camera_distance = info["camera_position"]

            # 캘리브레이션 이후에는 모드와 무관하게 좌표를 계산하여
            # RAW/EE 오버레이를 항상 보여준다.
            if camera_x is not None:
                hand_pos, ee_pos = processors[label].update(
                    camera_x,
                    camera_y,
                    camera_distance,
                )

            mode = gesture_trackers[label].mode
            candidate_mode = None
            elapsed = 0.0

            if recording:
                changed, elapsed, candidate_mode = gesture_trackers[label].update(
                    info["pose"],
                    info["palm_direction"],
                )
                mode = gesture_trackers[label].mode

                if changed:
                    publish_mode(label, mode)
                    print(
                        f"[{label}] MODE -> {mode} "
                        f"(palm={info['palm_direction']}, pose={info['pose']})"
                    )

                    # PLACE는 일회성: 한 번 보내고 즉시 WAITING으로 복귀한다.
                    # (다음 사이클에서 로봇이 또 PLACE를 받지 않도록)
                    if mode == MODE_PLACE:
                        gesture_trackers[label].mode = MODE_WAITING
                        mode = MODE_WAITING
                        publish_mode(label, MODE_WAITING)
                        print(f"[{label}] MODE -> WAITING (place 1회 후 복귀)")

                # 1초마다 현재 모드를 다시 보내 후발 구독자/일시적 패킷 문제를 방지한다.
                if now - last_mode_heartbeat[label] >= 1.0:
                    publish_mode(label, mode)
                    last_mode_heartbeat[label] = now

                if info["palm_direction"] != last_palm_direction[label]:
                    publish_palm_direction(label, info["palm_direction"])
                    last_palm_direction[label] = info["palm_direction"]

                # 로봇으로 보내는 좌표는 FOLLOW일 때만 발행한다.
                # 손 모델 위치/회전도 좌표와 같은 주기로 함께 발행해
                # 시뮬레이터의 손이 마커와 동일하게 움직이고 회전한다.
                if (
                    mode == MODE_FOLLOW
                    and hand_pos is not None
                    and now - last_publish_time[label] >= PUB_INTERVAL
                ):
                    publish_coordinates(label, hand_pos, ee_pos)
                    publish_orientation(label, info["orientation"])
                    last_publish_time[label] = now

            publishing = (
                recording
                and mode == MODE_FOLLOW
                and hand_pos is not None
            )

            draw_hand_panel(
                frame=frame,
                label=label,
                detected=info,
                mode=mode,
                candidate_mode=candidate_mode,
                elapsed=elapsed,
                hand_pos=hand_pos,
                ee_pos=ee_pos,
                publishing=publishing,
            )

        if calibration_step == 0:
            draw_calibration_banner(
                frame,
                "ANY HAND at 30cm, press SPACE",
            )
        elif calibration_step == 1:
            draw_calibration_banner(
                frame,
                "ANY HAND at 100cm, press SPACE",
            )
        elif calibration_step == 2 and not recording:
            draw_calibration_banner(
                frame,
                "press SPACE to START",
            )

        draw_footer(
            frame,
            fps=fps_value,
            recording=recording,
            calibration_step=calibration_step,
        )

        publish_hand_image(frame)

        cv2.imshow("Dual Hand Tracker", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == 32:
            if calibration_step == 0 and calibration_palm_px > 0:
                near_px = calibration_palm_px
                calibration_step = 1
                print(f"[CALIB] 30cm = {near_px:.1f}px")

            elif calibration_step == 1 and calibration_palm_px > 0:
                far_px = calibration_palm_px

                if near_px <= far_px:
                    print(
                        "[CALIB ERROR] 30cm palm width must be larger "
                        "than the 100cm palm width. Press R and retry."
                    )
                    continue

                focal_length = compute_focal(near_px, far_px)
                calibration_step = 2
                print(f"[CALIB] 100cm = {far_px:.1f}px")
                print(f"[CALIB] focal_length = {focal_length:.1f}")

            elif calibration_step == 2 and not recording:
                recording = True

                for hand_label in ("Left", "Right"):
                    gesture_trackers[hand_label].reset()
                    processors[hand_label].reset()
                    palm_filters[hand_label].reset()
                    orientation_filters[hand_label].reset()
                    publish_mode(hand_label, MODE_WAITING)
                    publish_palm_direction(hand_label, "SIDEWAYS")

                print("[START] Both hands begin in WAITING (중립)")

        elif key in (ord("r"), ord("R")):
            near_px = None
            far_px = None
            focal_length = None
            calibration_step = 0
            recording = False

            for hand_label in ("Left", "Right"):
                gesture_trackers[hand_label].reset()
                processors[hand_label].reset()
                palm_filters[hand_label].reset()
                orientation_filters[hand_label].reset()
                last_palm_direction[hand_label] = None
                last_publish_time[hand_label] = 0.0
                last_mode_heartbeat[hand_label] = 0.0

            fps_value = 0.0
            fps_frames = 0
            fps_window_started = time.monotonic()

            print("[RESET]")

        elif key in (27, ord("q"), ord("Q")):
            break

finally:
    camera.release()
    cv2.destroyAllWindows()
    detector.close()
    ros_node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()
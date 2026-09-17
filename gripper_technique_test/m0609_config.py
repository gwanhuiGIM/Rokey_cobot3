# m0609_config.py
from pathlib import Path


_THIS_DIR = Path(__file__).resolve().parent


# ============================================================
# 전체 Scene USD
# ============================================================

ROBOT_USD_PATH = str(
    _THIS_DIR
    / "Collected_full_scene_operating"
    / "full_scene_backup.usda"
)


# ============================================================
# Robot A / Robot B Prim 및 Scene 이름
# ============================================================

ROBOT_A_PRIM_PATH = "/World/m0609_01"
ROBOT_A_SCENE_NAME = "m0609_robot_a"

ROBOT_B_PRIM_PATH = "/World/m0609"
ROBOT_B_SCENE_NAME = "m0609_robot_b"

EE_LINK_NAME = "link_6"


# ============================================================
# Surface Gripper 설정
# ============================================================

def _surface_gripper_paths(
    robot_prim_path: str,
):
    base_path = (
        f"{robot_prim_path}"
        "/onrobot_rg2ft/gripper_body/"
        "dual_suction_tool"
    )

    return [
        (
            f"{base_path}"
            "/suction_contact_left/"
            "SurfaceGripper_left"
        ),
        (
            f"{base_path}"
            "/suction_contact_right/"
            "SurfaceGripper_right"
        ),
    ]


ROBOT_A_SURFACE_GRIPPER_PATHS = (
    _surface_gripper_paths(
        ROBOT_A_PRIM_PATH
    )
)

ROBOT_B_SURFACE_GRIPPER_PATHS = (
    _surface_gripper_paths(
        ROBOT_B_PRIM_PATH
    )
)

SURFACE_GRIPPER_WRITE_STATUS_TO_USD = True


# ============================================================
# 로봇 Drive 설정
# ============================================================

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8


# ============================================================
# RMPflow 설정
# ============================================================

RMPFLOW_DIR = str(
    _THIS_DIR
    / "rmpflow"
)

M0609_URDF_PATH = str(
    _THIS_DIR
    / "doosan-robot2"
    / "urdf"
    / "m0609_isaac_sim.urdf"
)

M0609_DESCRIPTION_PATH = str(
    _THIS_DIR
    / "rmpflow"
    / "m0609_description.yaml"
)

M0609_RMPFLOW_CONFIG_PATH = str(
    _THIS_DIR
    / "rmpflow"
    / "m0609_rmpflow_common.yaml"
)


# ============================================================
# cuRobo 손 추종 설정
#
# 실제 Robot A/B base pose는 main.py에서 각 Prim의
# world pose를 읽어 자동으로 계산한다.
# ============================================================

CUROBO_ROBOT_CONFIG_PATH = str(
    _THIS_DIR
    / "m0609_v1.yml"
)

TRACKING_TOOL_ORIENTATION = (
    0.0,
    0.0,
    1.0,
    0.0,
)

TRACKING_Z_MIN = 1.10
TRACKING_Z_MAX = 1.55
TRACKING_MAX_JOINT_STEP = 0.02
TRACKING_USE_MPC = True


# ============================================================
# 실제 트레이 / 수술 도구 동적 생성 설정
# ============================================================

TABLE_HEIGHT = 1.0

TRAY_USD_PATH = str(
    _THIS_DIR
    / "Collected_model_redtray_scaled_for_180mm_pads"
    / "model_redtray_scaled_for_180mm_pads.usda"
)


# ============================================================
# 손 모델(좌/우) 시각화 USDA
# ============================================================
# payload(@./rigged_hand.usd/Rigged Hand.usd@)가 상대경로라서
# hand_assets 폴더 안의 USDA를 직접 로드해야 메시가 풀린다.
HAND_USDA_PATH = str(
    _THIS_DIR.parent
    / "handtracking_final"
    / "hand_assets"
    / "leftandrignt_hand_assets.usda"
)

# USDA 안에서 reference 할 프림 경로.
# 루트(RightHandRoot/LeftHandRoot)에 사용자가 Add Pivot으로 지정한
# xformOp:translate:pivot(회전 중심축)이 들어있으므로, 깊은 메시 프림이
# 아니라 이 루트를 통째로 reference 해서 pivot op 스택을 그대로 살린다.
# translate / orient 값만 매 프레임 덮어쓰고 pivot은 유지한다.
HAND_RIGHT_REF_PRIM = "/World/RightHandRoot"
HAND_LEFT_REF_PRIM = "/World/LeftHandRoot"

# 시뮬레이터 stage에 생성할, 루트를 reference 하는 프림 경로
HAND_RIGHT_MODEL_PRIM = "/World/RightHandModel"
HAND_LEFT_MODEL_PRIM = "/World/LeftHandModel"

# 손 메시 rest 프레임 정렬 보정 (Euler 각도, deg, X->Y->Z 순서 적용).
# 추적 회전은 "가로=X, 손가락=Y, 손바닥법선=Z" 규약으로 들어오는데,
# asset 메시의 rest 프레임이 이 규약과 다르면 회전축이 엉뚱하게 보인다.
# 메시 로컬 기준 재정렬이라 q_applied = q_tracked * q_offset 로 적용된다.
# 좌/우 손은 미러라서 서로 다른 보정이 필요할 수 있다.
# (실측 튜닝: 한 축씩 90단위로 바꿔가며 손가락/손바닥 방향을 맞춘다.)
# 제거된 루트 orient(우손 +90° about Z, 좌손 -90° about Z)를 복원한다.
# "옆으로 회전 -> 앞뒤로 회전"은 Z축 90° 차이로 수평 두 축이 뒤바뀐 증상.
# 그래도 어긋나면 부호(+90/-90)나 축(Z->X 또는 Y)을 바꿔 튜닝한다.
HAND_RIGHT_ORIENT_OFFSET_DEG = (0.0, 0.0, 90.0)
HAND_LEFT_ORIENT_OFFSET_DEG = (0.0, 0.0, -90.0)

# 롤 축 고정 모드 (swing-twist 하이브리드).
# 손가락이 가리키는 방향(피치/요)은 풀 3D 로 그대로 추적하고,
# 손 뒤집기(롤) 성분만 손가락축이 아니라 아래의 고정 월드 축으로만 돌게 만든다.
# False 로 두면 이전처럼 완전한 풀 3D 회전을 적용한다.
HAND_STABILIZE_ROLL = True

# 롤을 고정할 월드 축. gizmo에서 확인한 대로 월드 Y.
# 다른 축으로 돌리고 싶으면 (1,0,0) 또는 (0,0,1) 로 바꾼다.
HAND_ROLL_AXIS = (0.0, 1.0, 0.0)

# 롤 방향이 반대면 부호를 뒤집는다.
HAND_RIGHT_ROLL_SIGN = 1.0
HAND_LEFT_ROLL_SIGN = 1.0

# 회전 중심(pivot) 미세 보정 (미터, 루트 로컬 좌표계).
# asset에 지정된 pivot 값에 이 값을 더해 회전축을 손 시각 중심으로 맞춘다.
# 회전축이 손보다 위에 있으면 해당 축 성분을 음수로 조금씩(예: -0.05) 내린다.
# 루트 로컬 축이라 화면상의 상하와 1:1로 일치하지 않을 수 있으니
# 한 축씩 ±0.05 단위로 바꿔보며 맞춘다.
HAND_RIGHT_PIVOT_OFFSET = (0.0, 0.0, 0.0)
HAND_LEFT_PIVOT_OFFSET = (0.0, 0.0, 0.0)

TOOL_DIR = (
    _THIS_DIR
    / "SurgicalInstruments_A"
    / "Model"
)

TOOL_USDS = (
    str(
        TOOL_DIR
        / "sm_ligatureneedle_a01_01.usd"
    ),
    str(
        TOOL_DIR
        / "sm_caliper_a01_01.usd"
    ),
    str(
        TOOL_DIR
        / "sm_clamps_a01_01.usd"
    ),
    str(
        TOOL_DIR
        / "sm_mallet_a01_01.usd"
    ),
    str(
        TOOL_DIR
        / "sm_handsaws_a01_01.usd"
    ),
    str(
        TOOL_DIR
        / "sm_knife_a01_01.usd"
    ),
)

TOOL_NAMES = (
    "봉합바늘",
    "캘리퍼",
    "클램프",
    "망치",
    "톱",
    "메스",
)

# 도구 초기 랜덤 배치 설정.
# None이면 실행할 때마다 다른 배치가 생성된다.
# 정수값을 넣으면 같은 랜덤 배치를 재현할 수 있다.
TOOL_RANDOM_SEED = None

# 외부 도구 인식값이 최신으로 인정되는 시간.
EXTERNAL_TOOL_DETECTION_TIMEOUT_SEC = 1.0

# 외부 좌표만 들어왔을 때 가장 가까운 트레이로 판단할 최대 거리.
EXTERNAL_TOOL_TRAY_MATCH_RADIUS = 0.18

TOOL_DROP_HEIGHT = 0.05
TOOL_MASS = 0.001

TOOL_SCALES = (
    (0.0050, 0.0050, 0.0050),  # 봉합바늘 (NEW) - 띄워보고 조정
    (0.0060, 0.0060, 0.0060),  # 캘리퍼
    (0.0075, 0.0075, 0.0075),  # 클램프
    (0.0050, 0.0050, 0.0050),  # 망치 (NEW) - 띄워보고 조정
    (0.0035, 0.0035, 0.0035),  # 톱
    (0.0040, 0.0040, 0.0040),  # 메스
)

TRAY_TOP_Z = 0.0186
TRAY_Z = 1.05

TRAY_ORIENTATION = (
    0.0,
    0.0,
    0.0,
    1.0,
)

# 새 2열×3행 배치.
#
# 화면 기준:
#
#   4  5
#   2  3
# A 0  1 B
#
# 업로드된 full_scene.usda 기준:
# Robot A = (-0.55, 0.50, 1.00)
# Robot B = ( 0.55, 0.50, 1.00)
#
# Robot A/B의 y=0.50이 0·1행과 2·3행 사이에 오도록
# 첫 행은 y=0.35, 둘째 행은 y=0.65로 배치한다.
# 행 간격은 0.25 m, 열 중심 간격은 0.25 m이다.
#
# 새 scene(full_scene_backup.usda)은 work_table 이 기존보다 Y -0.366m
# (원점 쪽)으로 당겨졌다. 테이블 월드 범위: X[-0.25,0.25] Y[-0.116,0.784].
# 트레이 Y 를 동일하게 당겨 6개 모두 테이블 위에 안착시킨다.
TRAY_SPAWN_POSITIONS = {
    0: (-0.125, 0.034, TRAY_Z),
    1: ( 0.125, 0.034, TRAY_Z),
    2: (-0.125, 0.284, TRAY_Z),
    3: ( 0.125, 0.284, TRAY_Z),
    4: (-0.125, 0.534, TRAY_Z),
    5: ( 0.125, 0.534, TRAY_Z),
}

# work_table 월드 풋프린트 (full_scene_backup.usda 기준).
# 동적 생성된 트레이가 테이블 밖으로 나가 떨어지지 않도록 클램프 가드에 쓴다.
WORK_TABLE_X_RANGE = (-0.25, 0.25)
WORK_TABLE_Y_RANGE = (-0.116, 0.784)
WORK_TABLE_TOP_Z = 1.005

# 트레이 중심이 테이블 가장자리에서 최소 이만큼 안쪽에 있도록 한다(트레이 반폭 여유).
TABLE_EDGE_MARGIN = 0.08

# 두 로봇 모두 모든 트레이에 접근할 수 있다.
ROBOT_A_SUPPORTED_TRAY_COMMANDS = (
    0,
    1,
    2,
    3,
    4,
    5,
)

ROBOT_B_SUPPORTED_TRAY_COMMANDS = (
    0,
    1,
    2,
    3,
    4,
    5,
)


# ============================================================
# 로봇 위쪽 중간 경유지 설정
#
# 현재 배치:
#
#     4 5
#   a 2 3 b
#   A 0 1 B
#
# a, b는 각 로봇 베이스를 기준으로 Y축 +0.30m 지점이다.
# X 좌표는 각 로봇 베이스와 동일하게 유지한다.
#
# 중간 경유지 도착 후 각 로봇은 바깥쪽 방향으로 180도 회전한다.
# Robot A: +180도
# Robot B: -180도
# ============================================================

TRANSIT_HEIGHT = 1.35

ROBOT_TRANSIT_Y_OFFSET = 0.30

ROBOT_A_TRACKING_JOINT1_DELTA_DEG = 180.0
ROBOT_B_TRACKING_JOINT1_DELTA_DEG = -180.0

JOINT1_TURN_TOLERANCE_DEG = 1.0
JOINT1_TURN_MAX_STEP_DEG = 1.00

SAFE_JOINT_RETURN_MAX_STEP_DEG = 0.35
SAFE_JOINT_RETURN_TOLERANCE_DEG = 1.0

RETURN_HOME_MAX_STEP_DEG = 0.20
RETURN_HOME_WRIST_MAX_STEP_DEG = 1.00
RETURN_HOME_TOLERANCE_DEG = 1.0


# ============================================================
# PICK / PLACE 설정
# ============================================================

PICK_EVENTS_DT = (
    0.008,
    0.005,
    0.02,
    0.15,
    0.0025,
    0.01,
    0.0025,
    1.0,
    0.008,
    0.08,
)

PICK_DEFAULT_EE_OFFSET = (
    0.0,
    0.0,
    0.20,
)

PICK_APPROACH_Z_CORRECTION = 0.032

TRANSPORT_Z_OFFSET = 0.10

PLACE_LINK6_ABOVE_TRAY = 0.136
PLACE_HIGH_OFFSET = 0.15
PLACE_APPROACH_GAP = 0.015
PLACE_MOVE_TOLERANCE = 0.04

PLACE_RELEASE_MIN_WAIT_FRAMES = 30
PLACE_RELEASE_STABLE_FRAMES = 10
PLACE_RELEASE_RETRY_INTERVAL = 15
PLACE_RELEASE_TIMEOUT_FRAMES = 120


# ============================================================
# 일반 제어 설정
# ============================================================

INITIAL_SETTLING_FRAMES = 30


# ============================================================
# 설정 파일 검증
# ============================================================

_REQUIRED_FILES = [
    (
        "ROBOT_USD_PATH",
        ROBOT_USD_PATH,
    ),
    (
        "M0609_URDF_PATH",
        M0609_URDF_PATH,
    ),
    (
        "M0609_DESCRIPTION_PATH",
        M0609_DESCRIPTION_PATH,
    ),
    (
        "M0609_RMPFLOW_CONFIG_PATH",
        M0609_RMPFLOW_CONFIG_PATH,
    ),
    (
        "CUROBO_ROBOT_CONFIG_PATH",
        CUROBO_ROBOT_CONFIG_PATH,
    ),
    (
        "TRAY_USD_PATH",
        TRAY_USD_PATH,
    ),
]

for setting_name, setting_path in _REQUIRED_FILES:
    if not Path(setting_path).is_file():
        raise FileNotFoundError(
            f"{setting_name} 파일을 찾을 수 없습니다: "
            f"{setting_path}"
        )

for tool_index, tool_path in enumerate(
    TOOL_USDS
):
    if not Path(tool_path).is_file():
        raise FileNotFoundError(
            f"TOOL_USDS[{tool_index}] "
            "파일을 찾을 수 없습니다: "
            f"{tool_path}"
        )
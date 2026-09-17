# Dual Hand Tracking ROS2

MediaPipe Hand Landmarker와 ROS2를 사용한 양손 추적 및 로봇 End-Effector 목표 좌표 발행 프로젝트입니다.

## 주요 기능

* 왼손과 오른손 동시 추적
* 손 중심 좌표 계산
* 손바닥 폭 기반 거리 추정
* 30cm / 100cm 거리 캘리브레이션
* 손바닥 방향 판정
* 손 제스처 기반 FOLLOW / PLACE 모드 전환
* ROS2 좌표 및 상태 토픽 발행
* 카메라 자동 탐색
* 실시간 최신 프레임 처리
* ROS Domain ID 기본값 137

---

## 요구 환경

* Ubuntu Linux
* Python 3.10 이상
* ROS2 Humble 또는 ROS2 Jazzy
* USB 카메라 또는 내장 카메라
* Git

ROS2는 사전에 설치되어 있어야 합니다.

---

## 저장소 Clone

```bash
git clone https://github.com/ROKEY-Project-F2/handtracking_final.git
cd handtracking_final
```

Private 저장소인 경우 GitHub 로그인 권한이 필요합니다.

---

## 최초 설치

setup 파일에 실행 권한을 부여합니다.

```bash
chmod +x setup_handtracking.sh
```

설치를 실행합니다.

```bash
./setup_handtracking.sh
```

setup 파일은 다음 작업을 자동으로 수행합니다.

1. ROS2 Humble 또는 Jazzy 탐색
2. 필요한 Ubuntu 패키지 설치
3. `.venv` 가상환경 생성
4. ROS2 Python 패키지를 사용할 수 있도록 설정
5. Python 패키지 설치
6. 필수 모델과 Python 파일 확인
7. 실행용 `run_handtracking.sh` 생성

---

## 기본 실행

setup 완료 후 다음 명령으로 실행합니다.

```bash
./run_handtracking.sh
```

기본 설정은 다음과 같습니다.

```text
ROS_DOMAIN_ID=137
CAMERA_SOURCE=auto
MIRROR_VIEW=0
SWAP_HAND_LABELS=0
```

---


## 로지텍 웹캠 사용 시

```bash
CAMERA_SOURCE=4 MIRROR_VIEW=0 SWAP_HAND_LABELS=1 ./run_handtracking.sh
```

---

## 특정 카메라 사용

카메라 장치를 확인합니다.

```bash
v4l2-ctl --list-devices
```

예를 들어 로지텍 카메라가 `/dev/video4`라면:

```bash
CAMERA_SOURCE=4 ./run_handtracking.sh
```

장치 경로를 직접 지정할 수도 있습니다.

```bash
CAMERA_SOURCE=/dev/video4 ./run_handtracking.sh
```

---

## 미러 화면 설정

미러 화면 사용:

```bash
MIRROR_VIEW=1 ./run_handtracking.sh
```

미러 화면 사용 안 함:

```bash
MIRROR_VIEW=0 ./run_handtracking.sh
```

특정 카메라와 함께 실행:

```bash
CAMERA_SOURCE=4 MIRROR_VIEW=0 ./run_handtracking.sh
```

---

## 왼손 / 오른손 라벨 교환

카메라에 따라 MediaPipe의 왼손과 오른손 판정이 반대로 나타날 수 있습니다.

라벨을 그대로 사용:

```bash
SWAP_HAND_LABELS=0 ./run_handtracking.sh
```

왼손과 오른손 라벨 교환:

```bash
SWAP_HAND_LABELS=1 ./run_handtracking.sh
```

예시:

```bash
CAMERA_SOURCE=4 MIRROR_VIEW=0 SWAP_HAND_LABELS=1 ./run_handtracking.sh
```

---

## ROS Domain 변경

기본 ROS Domain ID는 137입니다.

다른 Domain을 사용하려면:

```bash
ROS_DOMAIN_ID=42 ./run_handtracking.sh
```

---

## 전체 옵션 지정 실행

```bash
ROS_DOMAIN_ID=137 \
CAMERA_SOURCE=4 \
MIRROR_VIEW=0 \
SWAP_HAND_LABELS=0 \
./run_handtracking.sh
```

한 줄 실행:

```bash
ROS_DOMAIN_ID=137 CAMERA_SOURCE=4 MIRROR_VIEW=0 SWAP_HAND_LABELS=0 ./run_handtracking.sh
```

---

## 프로그램 조작법

프로그램 실행 후 캘리브레이션을 진행합니다.

### 1단계

손을 카메라에서 약 30cm 거리에 두고 `SPACE`를 누릅니다.

### 2단계

손을 카메라에서 약 100cm 거리에 두고 `SPACE`를 누릅니다.

### 3단계

다시 `SPACE`를 눌러 추적을 시작합니다.

키보드 조작:

```text
SPACE  캘리브레이션 및 시작
R      캘리브레이션 초기화
Q      프로그램 종료
ESC    프로그램 종료
```

---

## 제스처 모드

### FOLLOW

다음 상태를 1.5초 동안 유지합니다.

```text
손바닥이 카메라 방향
+
주먹 자세
```

FOLLOW 상태에서는 로봇 End-Effector 목표 좌표를 ROS2로 발행합니다.

### PLACE

다음 상태를 1.5초 동안 유지합니다.

```text
손등이 카메라 방향
+
손을 편 자세
```

PLACE 상태에서는 좌표 발행을 중지합니다.

---

## ROS2 Topic

왼손:

```text
/left_hand_raw
/left_hand_xyz
/left_hand_mode
/left_palm_direction
```

오른손:

```text
/right_hand_raw
/right_hand_xyz
/right_hand_mode
/right_palm_direction
```

기존 단일 오른손 코드 호환용:

```text
/hand_raw
/hand_xyz
/hand_mode
```

토픽 확인:

```bash
ros2 topic list
```

오른손 목표 좌표 확인:

```bash
ros2 topic echo /right_hand_xyz
```

왼손 목표 좌표 확인:

```bash
ros2 topic echo /left_hand_xyz
```

모드 확인:

```bash
ros2 topic echo /right_hand_mode
```

---

## 가상환경 직접 활성화

필요한 경우 수동으로 활성화할 수 있습니다.

ROS2 Humble:

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
```

ROS2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
```

직접 실행:

```bash
ROS_DOMAIN_ID=137 CAMERA_SOURCE=auto MIRROR_VIEW=0 python3 hand_trackerorigin.py
```

---

## 문제 해결

### 카메라 장치 확인

```bash
v4l2-ctl --list-devices
```

### 카메라 접근 권한 확인

```bash
ls -l /dev/video*
```

사용자를 video 그룹에 추가:

```bash
sudo usermod -aG video $USER
```

적용하려면 로그아웃 후 다시 로그인해야 합니다.

### 카메라가 다른 프로그램에서 사용 중인지 확인

```bash
fuser /dev/video4
```

### Python 패키지 확인

```bash
source .venv/bin/activate
python -c "import cv2, mediapipe, numpy, rclpy; print('IMPORT OK')"
```

### ROS Domain 확인

```bash
echo $ROS_DOMAIN_ID
```

### ROS2 토픽이 보이지 않을 때

두 터미널 모두 동일한 Domain을 사용해야 합니다.

```bash
export ROS_DOMAIN_ID=137
```

---

## 프로젝트 구조

```text
handtracking_final/
├── hand_trackerorigin.py
├── hand_landmarker.task
├── requirements.txt
├── setup_handtracking.sh
├── README.md
└── .gitignore
```

setup 실행 후 다음 파일과 폴더가 생성됩니다.

```text
.venv/
run_handtracking.sh
```

이 파일들은 로컬 실행용이며 GitHub에는 업로드하지 않습니다.

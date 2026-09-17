# Dual M0609 Surgical Tool Robot

Isaac Sim과 ROS2를 이용해 두 대의 M0609 로봇이 수술 도구를 집고, 손 위치를 추적하고, 도구 요청·교체·반납 명령을 처리하는 프로젝트입니다.

현재 구조는 다음 기능을 포함합니다.

- 두 대의 M0609 로봇 제어
- 6개 트레이와 도구 상태 관리
- 도구 이름 기반 요청
- 로봇별 작업 할당
- 손 위치 추적
- 도구 교체
- 특정 도구 반납
- 최근 작업 반납
- PICK 도중 취소
- 중복 요청 방지
- 작업 상태 및 결과 응답
- 작업 레지스트리 관리

---

## 1. 전체 동작 구조

```text
ROS2 도구 요청
-> ToolStateManager에서 도구가 있는 트레이 검색
-> RobotManager가 사용할 로봇 선택
-> PICK_APPROACH
-> PICK_TRANSPORT
-> TRACKING
-> 반납 또는 교체 요청
-> PLACE
-> RETURN_HOME
-> IDLE
```

두 로봇 모두 0~5번 트레이에 접근할 수 있습니다.

기본 우선순위는 다음과 같습니다.

```text
트레이 0, 2, 4
-> Robot A 우선

트레이 1, 3, 5
-> Robot B 우선
```

우선 로봇이 작업 중이면 다른 로봇이 해당 트레이를 처리할 수 있습니다.

두 로봇이 모두 `TRACKING` 상태에서 새 도구가 요청되면, 요청한 도구의 트레이와 더 가까운 로봇이 기존 도구를 반납한 뒤 새 도구를 집습니다.

---

## 2. 로봇 상태

상위 상태는 다음과 같습니다.

```text
IDLE
PICK_APPROACH
PICK_TRANSPORT
TRACKING
PLACE
RETURN_HOME
```

### IDLE

새 도구 요청을 기다리는 상태입니다.

### PICK_APPROACH

도구를 실제로 흡착하기 전 단계입니다.

포함 동작:

```text
집기 전 경유지 이동
-> 집기 방향 회전
-> 트레이 접근
-> 도구 흡착
```

이 상태에서 취소하면 도구를 집지 않았으므로 PLACE를 수행하지 않습니다.

```text
PICK_APPROACH 취소
-> RETURN_HOME
-> IDLE
```

### PICK_TRANSPORT

도구 흡착이 끝난 뒤 TRACKING 위치로 이동하는 상태입니다.

```text
흡착 완료
-> 운반 경유지 이동
-> joint1 바깥 방향 회전
-> TRACKING
```

이 상태에서 취소하면 도구를 원래 트레이에 내려놓습니다.

경유지 도착 전:

```text
현재 이동 중단
-> 트레이 PLACE 상단으로 복귀
-> PLACE
-> RETURN_HOME
-> IDLE
```

경유지 도착 후:

```text
운반 중 움직인 joint1 역회전
-> 경유지 자세 복원
-> 트레이 PLACE
-> RETURN_HOME
-> IDLE
```

### TRACKING

도구를 들고 손 위치 또는 지정된 추적 목표를 따라가는 상태입니다.

이 상태에서 다음 명령을 받을 수 있습니다.

```text
특정 도구 반납
최근 도구 반납
새 도구 요청에 의한 교체
```

### PLACE

도구를 원래 트레이에 내려놓는 상태입니다.

```text
안전 자세 복귀
-> joint1 원복
-> 경유지 이동
-> 트레이 상단 이동
-> 하강
-> 그리퍼 해제
-> 상승
```

### RETURN_HOME

작업 종료 후 로봇이 초기 IDLE 관절 자세로 복귀하는 상태입니다.

교체 작업에서는 `RETURN_HOME`을 생략하고 바로 다음 도구를 집을 수 있습니다.

---

## 3. 주요 파일

```text
main.py
robot_manager.py
m0609_state_machine.py
m0609_ros_bridge.py
tool_state_manager.py
operation_registry.py
```

### main.py

Isaac Sim 실행, 로봇 및 씬 생성, RobotManager 실행을 담당합니다.

### robot_manager.py

두 로봇의 작업 할당, 도구 요청, 반납, 교체, 중복 요청 방지, 작업 상태 관리를 담당합니다.

### m0609_state_machine.py

각 로봇의 상태와 실제 이동 동작을 처리합니다.

### m0609_ros_bridge.py

Isaac Sim OmniGraph 기반 ROS2 Subscriber와 Publisher를 생성합니다.

### tool_state_manager.py

도구 위치와 보유 상태를 관리합니다.

도구 상태 예:

```text
ON_TRAY
HELD_BY_ROBOT
UNKNOWN
```

### operation_registry.py

작업 ID, 로봇, 도구, 트레이, 작업 상태를 기록합니다.

---

## 4. 실행 환경

기준 환경:

```text
Ubuntu
Isaac Sim 5.1
ROS2 Humble
Python 3
```

ROS2 환경 변수 예시:

```bash
export ROS_DOMAIN_ID=137
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Isaac Sim 내부 ROS2 Bridge 라이브러리를 사용하는 환경에서는 다음 경로가 필요할 수 있습니다.

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/exts/isaacsim.ros2.bridge/humble/lib
```

Isaac Sim 설치 위치가 다르면 자신의 설치 경로에 맞게 수정해야 합니다.

환경 변수를 매번 입력하지 않으려면 `~/.bashrc`에 추가할 수 있습니다.

```bash
nano ~/.bashrc
```

파일 마지막에 환경 변수를 추가한 뒤 적용합니다.

```bash
source ~/.bashrc
```

---

## 5. 프로젝트 실행

프로젝트 디렉터리로 이동합니다.

```bash
cd ~/cobot3_ws/isaacpjt/gripper_technique_test
```

Isaac Sim Python 환경으로 `main.py`를 실행합니다.

설치 구조에 따라 실행 방법은 다를 수 있습니다.

예시:

```bash
~/dev_ws/isaac_sim/isaacsim/python.sh main.py
```

프로젝트에서 별도의 실행 스크립트를 사용한다면 해당 스크립트로 실행합니다.

```bash
./run.sh
```

Isaac Sim 창이 열린 뒤 반드시 Play를 눌러야 ROS2 OmniGraph Subscriber가 동작합니다.

정상적으로 ROS2 Bridge가 생성되면 터미널에 다음과 비슷한 로그가 출력됩니다.

```text
[ROS2 Bridge] ready:
 tool request: /m0609/pick_command
 tool return: /m0609/return_tool
 recent return: /m0609/return_recent
 command result: /m0609/tool_command_result
```

---

## 6. ROS2 토픽 목록

### 명령 토픽

| 기능 | 토픽 | 메시지 타입 |
|---|---|---|
| 도구 요청 | `/m0609/pick_command` | `std_msgs/msg/String` |
| 특정 도구 반납 | `/m0609/return_tool` | `std_msgs/msg/String` |
| 최근 작업 반납 | `/m0609/return_recent` | `std_msgs/msg/Empty` |

### 결과 토픽

| 기능 | 토픽 | 메시지 타입 |
|---|---|---|
| 도구 명령 결과 | `/m0609/tool_command_result` | `std_msgs/msg/String` |
| 이동 명령 결과 | `/m0609/move_result` | `std_msgs/msg/String` |

### 손 추적 토픽

| 기능 | 토픽 |
|---|---|
| 왼손 원본 위치 | `/left_hand_raw` |
| 왼손 목표 위치 | `/left_hand_xyz` |
| 왼손 모드 | `/left_hand_mode` |
| 왼손 손바닥 방향 | `/left_palm_direction` |
| 오른손 원본 위치 | `/right_hand_raw` |
| 오른손 목표 위치 | `/right_hand_xyz` |
| 오른손 모드 | `/right_hand_mode` |
| 오른손 손바닥 방향 | `/right_palm_direction` |

---

## 7. 외부 모듈 연동 규격

이 프로젝트는 음성인식, 손 추적, 도구 비전 인식 모듈과 연동할 수 있습니다.

현재 실제 ROS2 Bridge에 구현된 항목과, Python API만 구현된 항목을 구분해야 합니다.

| 외부 모듈 | 전달 데이터 | 현재 연동 방식 |
|---|---|---|
| 음성인식·명령 분류 | 도구 이름, 반납 명령 | ROS2 토픽 구현 완료 |
| 손 추적 | 좌우 손 위치, 모드, 손바닥 방향 | ROS2 토픽 구현 완료 |
| 도구 비전 상태 반영 | 도구 ID, 위치, 트레이 ID, 검출 시각 | Python API 구현 완료 |
| 비전 freshness 처리 | 외부 검출 우선, timeout 후 내부 상태 복귀 | 구현 완료 |
| 도구 비전 ROS2 토픽 | 비전 검출 결과 | 아직 ROS2 Subscriber 미구현 |

### 7.1 음성인식 모듈이 보내야 하는 명령

음성인식 모듈은 STT 결과를 그대로 보내는 것이 아니라, 명령 분류 결과를 도구 이름 또는 반납 명령으로 변환해서 보내야 합니다.

예를 들어 다음 음성을 인식했다고 가정합니다.

```text
"메스 줘"
"메스 가져와"
"메스 반납해"
"방금 도구 반납해"
```

음성 분류 모듈의 출력은 다음 ROS2 토픽으로 전달합니다.

#### 도구 요청

토픽:

```text
/m0609/pick_command
```

타입:

```text
std_msgs/msg/String
```

전송 예시:

```bash
ros2 topic pub --once \
  /m0609/pick_command \
  std_msgs/msg/String \
  "{data: '메스'}"
```

음성인식 코드 예시:

```python
from std_msgs.msg import String

message = String()
message.data = classified_tool_name
tool_request_publisher.publish(message)
```

`classified_tool_name`은 `ToolStateManager`에 등록된 도구 이름과 일치해야 합니다.

#### 특정 도구 반납

토픽:

```text
/m0609/return_tool
```

타입:

```text
std_msgs/msg/String
```

전송 예시:

```bash
ros2 topic pub --once \
  /m0609/return_tool \
  std_msgs/msg/String \
  "{data: '메스'}"
```

#### 최근 작업 반납

토픽:

```text
/m0609/return_recent
```

타입:

```text
std_msgs/msg/Empty
```

전송 예시:

```bash
ros2 topic pub --once \
  /m0609/return_recent \
  std_msgs/msg/Empty \
  "{}"
```

음성 명령 분류 예:

```text
"메스 줘"
-> REQUEST_TOOL
-> /m0609/pick_command
-> data: "메스"

"메스 반납해"
-> RETURN_TOOL
-> /m0609/return_tool
-> data: "메스"

"방금 도구 반납해"
-> RETURN_RECENT
-> /m0609/return_recent
```

### 7.2 실시간 손 위치 모듈이 보내야 하는 데이터

Robot A는 왼손 입력, Robot B는 오른손 입력을 사용합니다.

#### 왼손 위치

원본 위치:

```text
/left_hand_raw
geometry_msgs/msg/Point
```

로봇 추적 목표 위치:

```text
/left_hand_xyz
geometry_msgs/msg/Point
```

실시간 발행 예시:

```bash
ros2 topic pub --rate 30 \
  /left_hand_xyz \
  geometry_msgs/msg/Point \
  "{x: 0.30, y: 0.20, z: 0.80}"
```

#### 오른손 위치

원본 위치:

```text
/right_hand_raw
geometry_msgs/msg/Point
```

로봇 추적 목표 위치:

```text
/right_hand_xyz
geometry_msgs/msg/Point
```

실시간 발행 예시:

```bash
ros2 topic pub --rate 30 \
  /right_hand_xyz \
  geometry_msgs/msg/Point \
  "{x: -0.30, y: 0.20, z: 0.80}"
```

#### 손 추적 모드

왼손:

```text
/left_hand_mode
std_msgs/msg/String
```

오른손:

```text
/right_hand_mode
std_msgs/msg/String
```

예시:

```bash
ros2 topic pub --once \
  /left_hand_mode \
  std_msgs/msg/String \
  "{data: 'TRACKING'}"
```

#### 손바닥 방향

왼손:

```text
/left_palm_direction
geometry_msgs/msg/Vector3
```

오른손:

```text
/right_palm_direction
geometry_msgs/msg/Vector3
```

전송 예시:

```bash
ros2 topic pub --rate 30 \
  /left_palm_direction \
  geometry_msgs/msg/Vector3 \
  "{x: 0.0, y: 0.0, z: 1.0}"
```

손 위치와 손바닥 방향은 실시간 입력이므로 일반적으로 `20~30 Hz` 정도로 발행합니다.

현재 브릿지의 위치 데이터 QoS는 다음 기준입니다.

```text
history: keepLast
depth: 1
reliability: bestEffort
durability: volatile
```

따라서 오래된 손 위치를 쌓지 않고 가장 최근 값 위주로 처리합니다.

### 7.3 비전 모듈이 보내야 하는 도구 위치 정보

현재 `RobotManager`에는 외부 도구 검출 결과를 받는 Python API가 구현되어 있습니다.

```python
robot_manager.update_external_tool_detection(
    tool_id="메스",
    position=(0.12, 0.41, 0.73),
    tray_id=1,
    timestamp=None,
)
```

외부 비전 결과가 들어오면 해당 정보를 내부 추정값보다 우선해서 사용합니다.

```text
새로운 비전 검출 수신
-> 외부 검출 위치와 트레이 정보를 우선 사용
-> 일정 시간 동안 새로운 검출이 없으면 freshness 만료
-> 외부 검출값 사용 중단
-> 내부 ToolStateManager 상태로 자동 복귀
```

따라서 비전 노드가 일시적으로 끊기더라도 마지막 외부 좌표를 계속 영구적으로 사용하는 구조가 아닙니다.

다만 로봇이 이미 들고 있는 도구는 비전의 테이블 검출 결과로 덮어쓰지 않습니다.

```text
HELD_BY_ROBOT
-> 외부 테이블 검출보다 우선

ON_TRAY 또는 외부 위치 정보
-> freshness가 유효한 동안 외부 검출 우선
-> freshness 만료 후 내부 상태 사용
```

각 필드 의미:

| 필드 | 의미 |
|---|---|
| `tool_id` | 인식된 도구 이름 또는 도구 ID |
| `position` | 월드 좌표계 기준 `(x, y, z)` |
| `tray_id` | 도구가 놓인 트레이 번호. 모르면 `None` |
| `timestamp` | 검출 시각. 생략하면 내부 시각 사용 |

비전 모듈은 최소한 다음 정보를 만들어야 합니다.

```text
도구 ID
월드 좌표계 위치 x, y, z
트레이 ID
검출 시각
```

예시 검출 결과:

```json
{
  "tool_id": "메스",
  "position": {
    "x": -0.125,
    "y": 0.400,
    "z": 0.730
  },
  "tray_id": 0,
  "timestamp": 1782432000.0
}
```

주의할 점:

```text
카메라 좌표를 그대로 보내면 안 됨
-> Isaac Sim 월드 좌표계 또는 프로젝트 기준 좌표계로 변환 필요

로봇이 들고 있는 도구는 테이블 검출 결과로 덮어쓰지 않음
-> HELD_BY_ROBOT 상태가 우선

외부 검출 결과는 freshness가 유효한 동안만 사용
-> 일정 시간 동안 새 검출이 없으면 만료
-> 내부 ToolStateManager 상태로 자동 복귀
```

#### 현재 ROS2 구현 상태

비전 검출용 ROS2 Subscriber는 아직 `m0609_ros_bridge.py`에 추가되지 않았습니다.

따라서 현재는 다음 두 방식 중 하나를 사용해야 합니다.

```text
1. 비전 코드와 RobotManager가 같은 프로세스
   -> update_external_tool_detection() 직접 호출

2. 비전 코드가 별도 ROS2 노드
   -> 비전 검출용 ROS2 Subscriber를 추가 구현해야 함
```

비전용 ROS2 토픽을 추가할 경우 권장 규격은 다음과 같습니다.

```text
/m0609/tool_detection
std_msgs/msg/String
```

JSON 데이터 예시:

```json
{
  "tool_id": "메스",
  "x": -0.125,
  "y": 0.400,
  "z": 0.730,
  "tray_id": 0,
  "timestamp": 1782432000.0
}
```

ROS2 발행 예시:

```bash
ros2 topic pub --once \
  /m0609/tool_detection \
  std_msgs/msg/String \
  '{data: "{\"tool_id\":\"메스\",\"x\":-0.125,\"y\":0.4,\"z\":0.73,\"tray_id\":0,\"timestamp\":1782432000.0}"}'
```

위 `/m0609/tool_detection` 토픽은 README에서 권장하는 인터페이스이며, 현재 브릿지 코드에는 아직 Subscriber가 구현되어 있지 않습니다.

---

## 8. 도구 요청

도구 요청은 트레이 번호가 아니라 도구 이름 또는 등록된 도구 ID로 전송합니다.

예시:

```bash
ros2 topic pub --once \
  /m0609/pick_command \
  std_msgs/msg/String \
  "{data: '메스'}"
```

다른 도구 요청 예시:

```bash
ros2 topic pub --once \
  /m0609/pick_command \
  std_msgs/msg/String \
  "{data: '클램프'}"
```

```bash
ros2 topic pub --once \
  /m0609/pick_command \
  std_msgs/msg/String \
  "{data: '봉합바늘'}"
```

도구 이름은 `ToolStateManager`에 등록된 이름과 정확히 일치해야 합니다.

도구 요청 처리 과정:

```text
도구 이름 수신
-> 현재 도구 위치 확인
-> 도구가 있는 트레이 확인
-> 사용 가능한 로봇 선택
-> PICK_APPROACH
-> PICK_TRANSPORT
-> TRACKING
```

---

## 9. 특정 도구 반납

현재 작업 중인 특정 도구를 반납합니다.

```bash
ros2 topic pub --once \
  /m0609/return_tool \
  std_msgs/msg/String \
  "{data: '메스'}"
```

상태별 동작:

```text
PICK_APPROACH
-> 집기 취소
-> RETURN_HOME
-> IDLE
```

```text
PICK_TRANSPORT
-> 현재 단계에 맞는 복귀 동작
-> PLACE
-> RETURN_HOME
-> IDLE
```

```text
TRACKING
-> PLACE
-> RETURN_HOME
-> IDLE
```

이미 트레이에 있는 도구를 반납 요청하면 거절됩니다.

---

## 10. 최근 작업 반납

가장 최근에 생성된 활성 작업을 취소하거나 반납합니다.

```bash
ros2 topic pub --once \
  /m0609/return_recent \
  std_msgs/msg/Empty \
  "{}"
```

두 로봇이 동시에 작업 중이면 가장 최근 operation ID를 가진 작업이 대상이 됩니다.

---

## 11. 도구 교체

두 로봇이 모두 `TRACKING` 상태이고 새 도구를 요청하면 교체 작업으로 처리됩니다.

예시:

```text
Robot A: 메스 TRACKING
Robot B: 클램프 TRACKING
새 요청: 봉합바늘
```

처리 과정:

```text
봉합바늘 트레이와 가까운 로봇 선택
-> 현재 도구 PLACE
-> RETURN_HOME 생략
-> 봉합바늘 PICK
-> TRACKING
```

새 도구 요청은 기존 도구 요청 토픽을 그대로 사용합니다.

```bash
ros2 topic pub --once \
  /m0609/pick_command \
  std_msgs/msg/String \
  "{data: '봉합바늘'}"
```

---

## 12. 명령 결과 확인

결과 토픽을 먼저 구독합니다.

```bash
ros2 topic echo /m0609/tool_command_result
```

그다음 별도 터미널에서 명령을 보냅니다.

```bash
ros2 topic pub --once \
  /m0609/pick_command \
  std_msgs/msg/String \
  "{data: '메스'}"
```

결과 예시:

```json
{
  "command": "REQUEST_TOOL",
  "tool_id": "메스",
  "accepted": true,
  "status": "PICKING",
  "robot_id": "A",
  "operation_id": "operation_1",
  "message": "..."
}
```

중복 요청 예시:

```json
{
  "command": "REQUEST_TOOL",
  "tool_id": "메스",
  "accepted": false,
  "status": "TRANSPORTING",
  "robot_id": "A",
  "operation_id": "operation_1",
  "message": "메스 is already being transported"
}
```

---

## 13. 명령 상태

도구 명령 상태는 다음과 같습니다.

```text
AVAILABLE
PICKING
TRANSPORTING
TRACKING
RETURNING
PENDING_REPLACEMENT
UNKNOWN
```

### AVAILABLE

도구가 트레이에 있고 새 요청이 가능한 상태입니다.

### PICKING

로봇이 도구를 집기 위해 접근 중인 상태입니다.

### TRANSPORTING

도구를 집은 뒤 TRACKING 위치로 이동 중인 상태입니다.

### TRACKING

도구를 들고 손 추적 중인 상태입니다.

### RETURNING

PLACE 또는 RETURN_HOME이 진행 중인 상태입니다.

### PENDING_REPLACEMENT

기존 도구 반납 후 새 도구를 집기 위해 대기 중인 상태입니다.

### UNKNOWN

현재 도구 위치 또는 작업 상태를 확정할 수 없는 상태입니다.

---

## 14. 중복 요청 처리

같은 도구가 이미 작업 중이면 새 작업을 생성하지 않습니다.

```text
이미 PICKING
-> 중복 요청 거절

이미 TRANSPORTING
-> 중복 요청 거절

이미 TRACKING
-> 중복 요청 거절

이미 RETURNING
-> 중복 요청 거절

이미 PENDING_REPLACEMENT
-> 중복 요청 거절
```

같은 반납 명령이 반복되는 경우에도 현재 반납 상태를 확인해 중복 처리를 막습니다.

---

## 15. 손 추적 입력

왼손 목표 위치 예시:

```bash
ros2 topic pub --rate 30 \
  /left_hand_xyz \
  geometry_msgs/msg/Point \
  "{x: 0.3, y: 0.2, z: 0.8}"
```

오른손 목표 위치 예시:

```bash
ros2 topic pub --rate 30 \
  /right_hand_xyz \
  geometry_msgs/msg/Point \
  "{x: -0.3, y: 0.2, z: 0.8}"
```

왼손 모드 변경:

```bash
ros2 topic pub --once \
  /left_hand_mode \
  std_msgs/msg/String \
  "{data: 'TRACKING'}"
```

오른손 모드 변경:

```bash
ros2 topic pub --once \
  /right_hand_mode \
  std_msgs/msg/String \
  "{data: 'TRACKING'}"
```

실제 좌표 범위와 좌표계 변환은 손 추적 모듈의 설정을 따라야 합니다.

---

## 16. 권장 테스트 순서

### 테스트 1: 일반 도구 요청

```bash
ros2 topic echo /m0609/tool_command_result
```

```bash
ros2 topic pub --once \
  /m0609/pick_command \
  std_msgs/msg/String \
  "{data: '메스'}"
```

확인 항목:

```text
IDLE
-> PICK_APPROACH
-> PICK_TRANSPORT
-> TRACKING
```

### 테스트 2: PICK_APPROACH 취소

도구를 집기 전에 실행합니다.

```bash
ros2 topic pub --once \
  /m0609/return_tool \
  std_msgs/msg/String \
  "{data: '메스'}"
```

확인 항목:

```text
PICK_APPROACH
-> RETURN_HOME
-> IDLE
```

### 테스트 3: PICK_TRANSPORT 취소

도구 흡착 후 경유지 이동 또는 joint1 회전 중 실행합니다.

```bash
ros2 topic pub --once \
  /m0609/return_tool \
  std_msgs/msg/String \
  "{data: '메스'}"
```

확인 항목:

```text
PICK_TRANSPORT
-> PLACE
-> RETURN_HOME
-> IDLE
```

### 테스트 4: TRACKING 반납

TRACKING 진입 후 실행합니다.

```bash
ros2 topic pub --once \
  /m0609/return_tool \
  std_msgs/msg/String \
  "{data: '메스'}"
```

확인 항목:

```text
TRACKING
-> PLACE
-> RETURN_HOME
-> IDLE
```

### 테스트 5: 최근 작업 반납

```bash
ros2 topic pub --once \
  /m0609/return_recent \
  std_msgs/msg/Empty \
  "{}"
```

### 테스트 6: 중복 요청

동일 도구를 연속 요청합니다.

```bash
ros2 topic pub --once \
  /m0609/pick_command \
  std_msgs/msg/String \
  "{data: '메스'}"
```

```bash
ros2 topic pub --once \
  /m0609/pick_command \
  std_msgs/msg/String \
  "{data: '메스'}"
```

두 번째 요청이 `accepted: false`로 반환되는지 확인합니다.

### 테스트 7: 도구 교체

두 로봇을 각각 TRACKING 상태로 만든 뒤 세 번째 도구를 요청합니다.

```bash
ros2 topic pub --once \
  /m0609/pick_command \
  std_msgs/msg/String \
  "{data: '메스'}"
```

```bash
ros2 topic pub --once \
  /m0609/pick_command \
  std_msgs/msg/String \
  "{data: '클램프'}"
```

두 로봇이 TRACKING에 들어간 뒤:

```bash
ros2 topic pub --once \
  /m0609/pick_command \
  std_msgs/msg/String \
  "{data: '봉합바늘'}"
```

확인 항목:

```text
가까운 로봇 선택
-> 기존 도구 PLACE
-> 바로 새 도구 PICK
-> TRACKING
```

---

## 17. 로그 확인

도구 요청 수신:

```text
[ROS2 Bridge] tool request
```

특정 도구 반납 수신:

```text
[ROS2 Bridge] return_tool received
```

최근 작업 반납 수신:

```text
[ROS2 Bridge] return_recent received
```

반납 요청 승인:

```text
[Manager RETURN] Cancel/return requested
```

상태 변경:

```text
[StateMachine A] IDLE -> PICK_APPROACH
[StateMachine A] PICK_APPROACH -> PICK_TRANSPORT
[StateMachine A] PICK_TRANSPORT -> TRACKING
```

---

## 18. 문제 확인

### ROS2 토픽이 보이지 않는 경우

Isaac Sim이 Play 상태인지 확인합니다.

```bash
ros2 topic list
```

예상 토픽:

```text
/m0609/pick_command
/m0609/return_tool
/m0609/return_recent
/m0609/tool_command_result
```

토픽이 없다면 다음을 확인합니다.

```text
Isaac Sim Play 상태
ROS_DOMAIN_ID 일치 여부
RMW_IMPLEMENTATION 일치 여부
Isaac Sim ROS2 Bridge extension 활성화 여부
LD_LIBRARY_PATH 설정
```

### 명령은 수신되지만 동작하지 않는 경우

터미널 로그에서 다음 순서로 확인합니다.

```text
ROS2 Bridge 수신 로그
-> RobotManager 승인 또는 거절 로그
-> StateMachine 상태 전환 로그
```

### 도구 이름 오류

등록되지 않은 이름을 보내면 도구를 찾을 수 없습니다.

```text
Tool is not available on a tray
```

`ToolStateManager`에 등록된 실제 도구 이름을 사용해야 합니다.

### 중복 요청 거절

이미 작업 중이면 정상적으로 거절됩니다.

```text
accepted=false
status=PICKING / TRANSPORTING / TRACKING / RETURNING
```

---

## 19. 현재 구현 범위

현재 구현된 기능:

```text
두 로봇 작업 할당
도구 랜덤 배치
도구 이름 기반 요청
PICK 상태 분리
손 추적
특정 도구 반납
최근 작업 반납
PICK 도중 취소
도구 교체
직접 다음 도구 PICK
작업 레지스트리
중복 요청 방지
ROS2 결과 응답
```

현재 별도로 자동 복구하지 않는 항목:

```text
Isaac Sim 내부 예외 후 자동 재시작
그리퍼 물리 실패 자동 판정
외부 센서와 내부 ToolStateManager 불일치 자동 보정
프로세스 강제 종료 후 작업 복원
```

이 항목은 정상 취소·반납 기능과는 별도의 오류 복구 기능입니다.


씬 복원 명령어
```bash
cat Collected_full_scene.tar.gz.part-* > Collected_full_scene.tar.gz
tar -xzf Collected_full_scene.tar.gz
```

이미지 복원 명령어
```bash
cat docs.tar.gz.part-* > docs.tar.gz
tar -xzf docs.tar.gz
```
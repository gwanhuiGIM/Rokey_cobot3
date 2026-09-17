# dashboard

수술로봇 통합 상태 웹 대시보드.
ROS2 토픽을 구독해 로봇 A/B 상태, 트레이별 도구(비전), missing, 명령 피드를
브라우저에 실시간(WebSocket) 표시합니다.

## 구독 토픽
- `/m0609/status` (String JSON) — 로봇 상태/내부 트레이/missing (Isaac 발행)
- `/m0609/tool_detection` (String JSON) — 비전 트레이별 도구/빈칸
- `/m0609/tool_command_result` (String JSON) — 명령 결과 이벤트

## 설치
```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```
(rclpy 는 시스템 ROS2에서 옴 → `--system-site-packages` venv 또는 ROS source 후 실행)

## 실행
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=137
python3 dashboard_server.py
```
브라우저: http://localhost:8137

## 환경변수
- `DASHBOARD_HOST` (기본 0.0.0.0) / `DASHBOARD_PORT` (기본 8137)

## 비고
- 카메라 영상 패널은 부하/렉 우려로 기본 미포함. 상태 데이터 기반 커스텀 시각화.
- 로봇 A/B 패널은 Isaac 의 `/m0609/status` 발행이 있어야 채워집니다.

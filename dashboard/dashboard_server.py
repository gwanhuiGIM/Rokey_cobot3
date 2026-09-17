#!/usr/bin/env python3
# dashboard_server.py
#
# 수술로봇 통합 상태 웹 대시보드 백엔드.
#   - rclpy 노드로 ROS2 토픽 구독 (로봇 상태 / 트레이(비전) / 명령 이벤트)
#   - FastAPI + WebSocket 으로 브라우저에 상태를 실시간 push
#   - 정적 프론트(static/index.html) 서빙
#
# 실행:
#   source /opt/ros/humble/setup.bash
#   export ROS_DOMAIN_ID=137
#   python3 dashboard_server.py      # 기본 http://0.0.0.0:8137
#
import json
import os
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Empty
from sensor_msgs.msg import CompressedImage

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "static"

HOST = os.getenv("DASHBOARD_HOST", "0.0.0.0")
PORT = int(os.getenv("DASHBOARD_PORT", "8137"))

# ── 토픽 ──────────────────────────────────────────
STATUS_TOPIC = "/m0609/status"                 # 로봇/트레이/missing 통합 (Isaac)
TOOL_DETECTION_TOPIC = "/m0609/tool_detection"  # 비전: 트레이별 도구/빈칸
RESULT_TOPIC = "/m0609/tool_command_result"     # 명령 결과 이벤트
VOICE_LOG_TOPIC = "/m0609/voice_log"            # 음성 STT/분류/녹음상태
HAND_IMAGE_TOPIC = "/hand_tracking/image/compressed"  # 손 추적 화면(JPEG)

# 대시보드 → 로봇 명령 토픽
PICK_COMMAND_TOPIC = "/m0609/pick_command"
RETURN_TOOL_TOPIC = "/m0609/return_tool"
RETURN_RECENT_TOPIC = "/m0609/return_recent"

ALL_TOOLS = ["메스", "캘리퍼", "클램프", "망치", "톱", "봉합바늘"]

# 최신 손 추적 JPEG 프레임 (MJPEG 중계용)
_IMG_LOCK = threading.Lock()
_LATEST_JPEG = {"data": None, "t": 0.0}

# ── 공유 상태 (ROS 스레드 ↔ 웹) ───────────────────
_LOCK = threading.Lock()
STATE = {
    "robots": {},          # {A:{state,held_tool,tray,operation_id}, B:{...}}
    "trays_internal": {},  # 내부 상태 기준 {tray_id: tool|null}
    "trays_vision": {},    # 비전 기준 {tray_id: tool|"EMPTY"}
    "missing": [],
    "events": [],          # 최근 명령 결과 (최신순)
    "ops": [],             # 로봇 상태전이 이력 (최신순)
    "voice": [],           # 음성 로그 (최신순)
    "listening": False,    # 녹음 중 여부
    "robot_prev_state": {},  # op 전이 감지용
    "sources": {"status": 0.0, "vision": 0.0, "result": 0.0, "voice": 0.0, "camera": 0.0},
}
MAX_EVENTS = 30

NODE = None  # DashboardNode 전역 참조 (HTTP 명령 발행용)


def _touch(source: str) -> None:
    STATE["sources"][source] = time.time()


class DashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("surgical_dashboard")
        self.create_subscription(String, STATUS_TOPIC, self._on_status, 10)
        self.create_subscription(
            String, TOOL_DETECTION_TOPIC, self._on_detection, 10
        )
        self.create_subscription(String, RESULT_TOPIC, self._on_result, 10)
        self.create_subscription(String, VOICE_LOG_TOPIC, self._on_voice, 10)
        # 손 추적 이미지는 BEST_EFFORT 로 발행되므로 구독도 맞춰준다.
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            CompressedImage, HAND_IMAGE_TOPIC, self._on_image, best_effort
        )

        # 대시보드 → 로봇 명령 publisher
        self.pub_pick = self.create_publisher(String, PICK_COMMAND_TOPIC, 10)
        self.pub_return = self.create_publisher(String, RETURN_TOOL_TOPIC, 10)
        self.pub_return_recent = self.create_publisher(
            Empty, RETURN_RECENT_TOPIC, 10
        )

        self.get_logger().info("dashboard 구독/발행 준비 완료")

    def _on_voice(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        with _LOCK:
            if data.get("event") == "listening":
                STATE["listening"] = True
            else:
                STATE["listening"] = False
                STATE["voice"].insert(0, data)
                del STATE["voice"][MAX_EVENTS:]
            _touch("voice")

    def _on_image(self, msg: CompressedImage) -> None:
        with _IMG_LOCK:
            _LATEST_JPEG["data"] = bytes(msg.data)
            _LATEST_JPEG["t"] = time.time()
        with _LOCK:
            _touch("camera")

    def _on_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        with _LOCK:
            robots = data.get("robots", {})
            # 로봇 상태 전이 감지 → op 로그
            for rid, info in robots.items():
                new_state = info.get("state")
                prev = STATE["robot_prev_state"].get(rid)
                if new_state and new_state != prev:
                    STATE["robot_prev_state"][rid] = new_state
                    if prev is not None:
                        STATE["ops"].insert(0, {
                            "t": time.time(), "robot": rid,
                            "from": prev, "to": new_state,
                            "tool": info.get("held_tool"),
                        })
                        del STATE["ops"][MAX_EVENTS:]
            STATE["robots"] = robots or STATE["robots"]
            STATE["trays_internal"] = data.get("trays", STATE["trays_internal"])
            STATE["missing"] = data.get("missing", STATE["missing"])
            _touch("status")

    def _on_detection(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        tray_id = data.get("tray_id")
        tool_id = data.get("tool_id")
        if tray_id is None:
            return
        with _LOCK:
            STATE["trays_vision"][str(tray_id)] = tool_id
            _touch("vision")

    def _on_result(self, msg: String) -> None:
        with _LOCK:
            entry = {"t": time.time(), "raw": msg.data}
            try:
                entry["data"] = json.loads(msg.data)
            except Exception:
                entry["data"] = None
            STATE["events"].insert(0, entry)
            del STATE["events"][MAX_EVENTS:]
            _touch("result")


def _ros_spin() -> None:
    global NODE
    rclpy.init()
    NODE = DashboardNode()
    try:
        rclpy.spin(NODE)
    finally:
        NODE.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ── 웹 ────────────────────────────────────────────
app = FastAPI(title="Surgical Robot Dashboard")


@app.get("/")
def index() -> HTMLResponse:
    # 브라우저가 옛 프론트(JS/HTML)를 캐시하지 않도록 막는다.
    return HTMLResponse(
        (STATIC_DIR / "index.html").read_text(encoding="utf-8"),
        headers={"Cache-Control": "no-store, must-revalidate"},
    )


@app.post("/cmd/pick/{tool}")
def cmd_pick(tool: str) -> dict:
    if NODE is None:
        return {"ok": False, "msg": "ROS node not ready"}
    NODE.pub_pick.publish(String(data=tool))
    return {"ok": True, "sent": f"pick {tool}"}


@app.post("/cmd/return/{tool}")
def cmd_return(tool: str) -> dict:
    if NODE is None:
        return {"ok": False, "msg": "ROS node not ready"}
    NODE.pub_return.publish(String(data=tool))
    return {"ok": True, "sent": f"return {tool}"}


@app.post("/cmd/return_recent")
def cmd_return_recent() -> dict:
    if NODE is None:
        return {"ok": False, "msg": "ROS node not ready"}
    NODE.pub_return_recent.publish(Empty())
    return {"ok": True, "sent": "return_recent"}


@app.get("/tools")
def tools() -> dict:
    return {"tools": ALL_TOOLS}


@app.get("/camera")
async def camera() -> StreamingResponse:
    """손 추적 화면을 MJPEG 로 중계한다 (<img src='/camera'>)."""
    import asyncio

    async def gen():
        while True:
            with _IMG_LOCK:
                data = _LATEST_JPEG["data"]
            if data:
                yield (
                    b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                    + data
                    + b"\r\n"
                )
            await asyncio.sleep(1.0 / 15.0)

    return StreamingResponse(
        gen(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.websocket("/ws")
async def ws(websocket: WebSocket) -> None:
    await websocket.accept()
    try:
        while True:
            with _LOCK:
                now = time.time()
                payload = {
                    "robots": STATE["robots"],
                    "trays_internal": STATE["trays_internal"],
                    "trays_vision": STATE["trays_vision"],
                    "missing": STATE["missing"],
                    "events": STATE["events"][:10],
                    "ops": STATE["ops"][:10],
                    "voice": STATE["voice"][:8],
                    "listening": STATE["listening"],
                    "tools": ALL_TOOLS,
                    "online": {
                        k: (now - v) < 2.0
                        for k, v in STATE["sources"].items()
                    },
                    "server_time": now,
                }
            await websocket.send_text(json.dumps(payload, ensure_ascii=False))
            await _async_sleep(0.1)  # 10Hz
    except WebSocketDisconnect:
        return
    except Exception:
        return


async def _async_sleep(sec: float) -> None:
    import asyncio

    await asyncio.sleep(sec)


if STATIC_DIR.exists():
    app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


def main() -> None:
    threading.Thread(target=_ros_spin, name="ros-spin", daemon=True).start()
    print(f"[dashboard] http://{HOST}:{PORT}  (ROS_DOMAIN_ID={os.getenv('ROS_DOMAIN_ID')})")
    uvicorn.run(app, host=HOST, port=PORT, log_level="warning")


if __name__ == "__main__":
    main()

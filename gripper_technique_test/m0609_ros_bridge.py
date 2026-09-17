# m0609_ros_bridge.py

from __future__ import annotations

from typing import Optional, Tuple

import omni.graph.core as og
import omni.usd
from isaacsim.core.utils.extensions import enable_extension


GRAPH_PATH = "/World/ROS_M0609_Graph"

COMMAND_TOPIC = "/m0609/move_command"
RESULT_TOPIC = "/m0609/move_result"
# 도구 이름/ID 기반 명령 토픽
PICK_COMMAND_TOPIC = "/m0609/pick_command"
RETURN_TOOL_TOPIC = "/m0609/return_tool"
RETURN_RECENT_TOPIC = "/m0609/return_recent"
TOOL_COMMAND_RESULT_TOPIC = "/m0609/tool_command_result"
STATUS_TOPIC = "/m0609/status"

LEFT_HAND_RAW_TOPIC = "/left_hand_raw"
LEFT_HAND_TARGET_TOPIC = "/left_hand_xyz"
LEFT_HAND_MODE_TOPIC = "/left_hand_mode"
LEFT_PALM_DIRECTION_TOPIC = "/left_palm_direction"
LEFT_HAND_ORIENT_TOPIC = "/left_hand_orient"

RIGHT_HAND_RAW_TOPIC = "/right_hand_raw"
RIGHT_HAND_TARGET_TOPIC = "/right_hand_xyz"
RIGHT_HAND_MODE_TOPIC = "/right_hand_mode"
RIGHT_PALM_DIRECTION_TOPIC = "/right_palm_direction"
RIGHT_HAND_ORIENT_TOPIC = "/right_hand_orient"

STRING_TYPE = "token"

HAND_POSITION_QOS = r"""
{
    "history": "keepLast",
    "depth": 1,
    "reliability": "bestEffort",
    "durability": "volatile",
    "deadline": 0.0,
    "lifespan": 0.0,
    "liveliness": "systemDefault",
    "leaseDuration": 0.0
}
"""

HAND_MODE_QOS = r"""
{
    "history": "keepLast",
    "depth": 10,
    "reliability": "reliable",
    "durability": "volatile",
    "deadline": 0.0,
    "lifespan": 0.0,
    "liveliness": "systemDefault",
    "leaseDuration": 0.0
}
"""


_COMMAND_MANAGER = None


# ============================================================
# Left hand cache
# ============================================================
_LATEST_LEFT_HAND_RAW: Optional[Tuple[float, float, float]] = None
_LEFT_HAND_RAW_SEQUENCE = 0

_LATEST_LEFT_HAND_TARGET: Optional[Tuple[float, float, float]] = None
_LEFT_HAND_TARGET_SEQUENCE = 0

_LATEST_LEFT_HAND_MODE = "TRACKING"
_LEFT_HAND_MODE_SEQUENCE = 0

_LATEST_LEFT_PALM_DIRECTION: Optional[Tuple[float, float, float]] = None
_LEFT_PALM_DIRECTION_SEQUENCE = 0

_LATEST_LEFT_HAND_ORIENT: Optional[Tuple[float, float, float, float]] = None
_LEFT_HAND_ORIENT_SEQUENCE = 0


# ============================================================
# Right hand cache
# ============================================================
_LATEST_RIGHT_HAND_RAW: Optional[Tuple[float, float, float]] = None
_RIGHT_HAND_RAW_SEQUENCE = 0

_LATEST_RIGHT_HAND_TARGET: Optional[Tuple[float, float, float]] = None
_RIGHT_HAND_TARGET_SEQUENCE = 0

_LATEST_RIGHT_HAND_MODE = "TRACKING"
_RIGHT_HAND_MODE_SEQUENCE = 0

_LATEST_RIGHT_PALM_DIRECTION: Optional[Tuple[float, float, float]] = None
_RIGHT_PALM_DIRECTION_SEQUENCE = 0

_LATEST_RIGHT_HAND_ORIENT: Optional[Tuple[float, float, float, float]] = None
_RIGHT_HAND_ORIENT_SEQUENCE = 0


def get_command_manager():
    return _COMMAND_MANAGER


# 대시보드용 통합 상태를 매 틱 /m0609/status 로 발행하는 스크립트.
_STATUS_SCRIPT = r"""
import json

from m0609_ros_bridge import get_command_manager


def compute(db):
    try:
        manager = get_command_manager()
        if manager is not None:
            db.outputs.result = json.dumps(
                manager.get_dashboard_status(),
                ensure_ascii=False,
            )
    except Exception as error:
        print(
            "[ROS2 Bridge] status error: " + str(error),
            flush=True,
        )

    return True
"""


def _setup_status_publisher(simulation_app) -> None:
    """
    /m0609/status (String JSON) 발행기를 기존 그래프에 '추가'한다.
    기존 bridge 를 깨지 않도록 별도 edit + try/except 로 격리한다.
    실패해도 핵심 bridge 동작에는 영향이 없다.
    """
    import omni.graph.core as og

    try:
        og.Controller.edit(
            {
                "graph_path": GRAPH_PATH,
                "evaluator_name": "execution",
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    (
                        "HandleStatus",
                        "omni.graph.scriptnode.ScriptNode",
                    ),
                    (
                        "StatusPublisher",
                        "isaacsim.ros2.bridge.ROS2Publisher",
                    ),
                ],
                og.Controller.Keys.SET_VALUES: [
                    (
                        "StatusPublisher.inputs:messagePackage",
                        "std_msgs",
                    ),
                    (
                        "StatusPublisher.inputs:messageSubfolder",
                        "msg",
                    ),
                    (
                        "StatusPublisher.inputs:messageName",
                        "String",
                    ),
                    (
                        "StatusPublisher.inputs:topicName",
                        STATUS_TOPIC,
                    ),
                    (
                        "HandleStatus.inputs:script",
                        _STATUS_SCRIPT,
                    ),
                ],
                og.Controller.Keys.CONNECT: [
                    (
                        "OnPlaybackTick.outputs:tick",
                        "HandleStatus.inputs:execIn",
                    ),
                    (
                        "ROS2Context.outputs:context",
                        "StatusPublisher.inputs:context",
                    ),
                ],
            },
        )

        for _ in range(5):
            simulation_app.update()

        status_node = og.Controller.node(
            f"{GRAPH_PATH}/HandleStatus"
        )
        og.Controller.create_attribute(
            node=status_node,
            attr_name="outputs:result",
            attr_type=STRING_TYPE,
            attr_port=(
                og.AttributePortType
                .ATTRIBUTE_PORT_TYPE_OUTPUT
            ),
        )

        for _ in range(3):
            simulation_app.update()

        og.Controller.connect(
            f"{GRAPH_PATH}/HandleStatus.outputs:execOut",
            f"{GRAPH_PATH}/StatusPublisher.inputs:execIn",
        )
        og.Controller.connect(
            f"{GRAPH_PATH}/HandleStatus.outputs:result",
            f"{GRAPH_PATH}/StatusPublisher.inputs:data",
        )

        print(
            "[ROS2 Bridge] status publisher ready: " + STATUS_TOPIC,
            flush=True,
        )

    except Exception as error:
        print(
            "[ROS2 Bridge] status publisher 설정 실패(무시): "
            + str(error),
            flush=True,
        )


# ============================================================
# Left hand cache API
# ============================================================
def set_latest_left_hand_raw(
    x: float,
    y: float,
    z: float,
) -> None:
    global _LATEST_LEFT_HAND_RAW
    global _LEFT_HAND_RAW_SEQUENCE

    _LATEST_LEFT_HAND_RAW = (
        float(x),
        float(y),
        float(z),
    )
    _LEFT_HAND_RAW_SEQUENCE += 1


def get_latest_left_hand_raw():
    return (
        _LATEST_LEFT_HAND_RAW,
        _LEFT_HAND_RAW_SEQUENCE,
    )


def set_latest_left_hand_target(
    x: float,
    y: float,
    z: float,
) -> None:
    global _LATEST_LEFT_HAND_TARGET
    global _LEFT_HAND_TARGET_SEQUENCE

    _LATEST_LEFT_HAND_TARGET = (
        float(x),
        float(y),
        float(z),
    )
    _LEFT_HAND_TARGET_SEQUENCE += 1


def get_latest_left_hand_target():
    return (
        _LATEST_LEFT_HAND_TARGET,
        _LEFT_HAND_TARGET_SEQUENCE,
    )


def set_latest_left_hand_mode(
    mode: str,
) -> None:
    global _LATEST_LEFT_HAND_MODE
    global _LEFT_HAND_MODE_SEQUENCE

    _LATEST_LEFT_HAND_MODE = str(mode).strip().upper()
    _LEFT_HAND_MODE_SEQUENCE += 1


def reset_left_hand_mode_cache(
    mode: str = "TRACKING",
) -> int:
    global _LATEST_LEFT_HAND_MODE
    global _LEFT_HAND_MODE_SEQUENCE

    _LATEST_LEFT_HAND_MODE = str(mode).strip().upper()
    _LEFT_HAND_MODE_SEQUENCE += 1

    return _LEFT_HAND_MODE_SEQUENCE


def get_latest_left_hand_mode():
    return (
        _LATEST_LEFT_HAND_MODE,
        _LEFT_HAND_MODE_SEQUENCE,
    )


def set_latest_left_palm_direction(
    x: float,
    y: float,
    z: float,
) -> None:
    global _LATEST_LEFT_PALM_DIRECTION
    global _LEFT_PALM_DIRECTION_SEQUENCE

    _LATEST_LEFT_PALM_DIRECTION = (
        float(x),
        float(y),
        float(z),
    )
    _LEFT_PALM_DIRECTION_SEQUENCE += 1


def get_latest_left_palm_direction():
    return (
        _LATEST_LEFT_PALM_DIRECTION,
        _LEFT_PALM_DIRECTION_SEQUENCE,
    )


def set_latest_left_hand_orient(
    w: float,
    x: float,
    y: float,
    z: float,
) -> None:
    global _LATEST_LEFT_HAND_ORIENT
    global _LEFT_HAND_ORIENT_SEQUENCE

    _LATEST_LEFT_HAND_ORIENT = (
        float(w),
        float(x),
        float(y),
        float(z),
    )
    _LEFT_HAND_ORIENT_SEQUENCE += 1


def get_latest_left_hand_orient():
    return (
        _LATEST_LEFT_HAND_ORIENT,
        _LEFT_HAND_ORIENT_SEQUENCE,
    )


# ============================================================
# Right hand cache API
# ============================================================
def set_latest_right_hand_raw(
    x: float,
    y: float,
    z: float,
) -> None:
    global _LATEST_RIGHT_HAND_RAW
    global _RIGHT_HAND_RAW_SEQUENCE

    _LATEST_RIGHT_HAND_RAW = (
        float(x),
        float(y),
        float(z),
    )
    _RIGHT_HAND_RAW_SEQUENCE += 1


def get_latest_right_hand_raw():
    return (
        _LATEST_RIGHT_HAND_RAW,
        _RIGHT_HAND_RAW_SEQUENCE,
    )


def set_latest_right_hand_target(
    x: float,
    y: float,
    z: float,
) -> None:
    global _LATEST_RIGHT_HAND_TARGET
    global _RIGHT_HAND_TARGET_SEQUENCE

    _LATEST_RIGHT_HAND_TARGET = (
        float(x),
        float(y),
        float(z),
    )
    _RIGHT_HAND_TARGET_SEQUENCE += 1


def get_latest_right_hand_target():
    return (
        _LATEST_RIGHT_HAND_TARGET,
        _RIGHT_HAND_TARGET_SEQUENCE,
    )


def set_latest_right_hand_mode(
    mode: str,
) -> None:
    global _LATEST_RIGHT_HAND_MODE
    global _RIGHT_HAND_MODE_SEQUENCE

    _LATEST_RIGHT_HAND_MODE = str(mode).strip().upper()
    _RIGHT_HAND_MODE_SEQUENCE += 1


def reset_right_hand_mode_cache(
    mode: str = "TRACKING",
) -> int:
    global _LATEST_RIGHT_HAND_MODE
    global _RIGHT_HAND_MODE_SEQUENCE

    _LATEST_RIGHT_HAND_MODE = str(mode).strip().upper()
    _RIGHT_HAND_MODE_SEQUENCE += 1

    return _RIGHT_HAND_MODE_SEQUENCE


def get_latest_right_hand_mode():
    return (
        _LATEST_RIGHT_HAND_MODE,
        _RIGHT_HAND_MODE_SEQUENCE,
    )


def set_latest_right_palm_direction(
    x: float,
    y: float,
    z: float,
) -> None:
    global _LATEST_RIGHT_PALM_DIRECTION
    global _RIGHT_PALM_DIRECTION_SEQUENCE

    _LATEST_RIGHT_PALM_DIRECTION = (
        float(x),
        float(y),
        float(z),
    )
    _RIGHT_PALM_DIRECTION_SEQUENCE += 1


def get_latest_right_palm_direction():
    return (
        _LATEST_RIGHT_PALM_DIRECTION,
        _RIGHT_PALM_DIRECTION_SEQUENCE,
    )


def set_latest_right_hand_orient(
    w: float,
    x: float,
    y: float,
    z: float,
) -> None:
    global _LATEST_RIGHT_HAND_ORIENT
    global _RIGHT_HAND_ORIENT_SEQUENCE

    _LATEST_RIGHT_HAND_ORIENT = (
        float(w),
        float(x),
        float(y),
        float(z),
    )
    _RIGHT_HAND_ORIENT_SEQUENCE += 1


def get_latest_right_hand_orient():
    return (
        _LATEST_RIGHT_HAND_ORIENT,
        _RIGHT_HAND_ORIENT_SEQUENCE,
    )


_COMMAND_SCRIPT = r"""
import json

from m0609_ros_bridge import get_command_manager


def compute(db):
    request_id = ""

    try:
        command = json.loads(str(db.inputs.command))
        request_id = str(command.get("request_id", ""))

        x = float(command["x"])
        y = float(command["y"])
        z = float(command["z"])

        manager = get_command_manager()

        if manager is None:
            raise RuntimeError("Robot manager is not registered")

        accepted, message = manager.request_move(x, y, z)

        result = {
            "request_id": request_id,
            "accepted": bool(accepted),
            "message": str(message),
        }

    except Exception as error:
        result = {
            "request_id": request_id,
            "accepted": False,
            "message": str(error),
        }

    db.outputs.result = json.dumps(
        result,
        ensure_ascii=False,
    )

    return True
"""


_PICK_COMMAND_SCRIPT = r"""
import json
from m0609_ros_bridge import get_command_manager


def compute(db):
    tool_id = str(db.inputs.command).strip()

    try:
        manager = get_command_manager()
        if manager is None:
            raise RuntimeError("Robot manager is not registered")

        accepted, message = manager.request_tool_command(tool_id)
        status = manager.get_tool_command_status(tool_id)

        result = {
            "command": "REQUEST_TOOL",
            "tool_id": tool_id,
            "accepted": bool(accepted),
            "status": status.get("status", "UNKNOWN"),
            "robot_id": status.get("robot_id"),
            "operation_id": status.get("operation_id"),
            "message": str(message),
        }
    except Exception as error:
        result = {
            "command": "REQUEST_TOOL",
            "tool_id": tool_id,
            "accepted": False,
            "status": "ERROR",
            "message": str(error),
        }

    db.outputs.result = json.dumps(result, ensure_ascii=False)
    print(f"[ROS2 Bridge] tool result: {db.outputs.result}", flush=True)
    return True
"""


_RETURN_TOOL_SCRIPT = r"""
import json
from m0609_ros_bridge import get_command_manager


def compute(db):
    tool_id = str(db.inputs.command).strip()

    try:
        manager = get_command_manager()
        if manager is None:
            raise RuntimeError("Robot manager is not registered")

        accepted, message = manager.request_tool_return(tool_id)
        status = manager.get_tool_command_status(tool_id)

        result = {
            "command": "RETURN_TOOL",
            "tool_id": tool_id,
            "accepted": bool(accepted),
            "status": status.get("status", "UNKNOWN"),
            "robot_id": status.get("robot_id"),
            "operation_id": status.get("operation_id"),
            "message": str(message),
        }
    except Exception as error:
        result = {
            "command": "RETURN_TOOL",
            "tool_id": tool_id,
            "accepted": False,
            "status": "ERROR",
            "message": str(error),
        }

    db.outputs.result = json.dumps(result, ensure_ascii=False)
    print(f"[ROS2 Bridge] return result: {db.outputs.result}", flush=True)
    return True
"""


_RETURN_RECENT_SCRIPT = r"""
import json
from m0609_ros_bridge import get_command_manager


def compute(db):
    try:
        manager = get_command_manager()
        if manager is None:
            raise RuntimeError("Robot manager is not registered")

        accepted, message = manager.request_recent_tool_return()

        result = {
            "command": "RETURN_RECENT",
            "tool_id": None,
            "accepted": bool(accepted),
            "status": "ACCEPTED" if accepted else "REJECTED",
            "message": str(message),
        }
    except Exception as error:
        result = {
            "command": "RETURN_RECENT",
            "tool_id": None,
            "accepted": False,
            "status": "ERROR",
            "message": str(error),
        }

    db.outputs.result = json.dumps(result, ensure_ascii=False)
    print(f"[ROS2 Bridge] recent return result: {db.outputs.result}", flush=True)
    return True
"""


def _make_vector_script(
    setter_name: str,
    error_name: str,
) -> str:
    return f"""
from m0609_ros_bridge import {setter_name}


def compute(db):
    try:
        {setter_name}(
            float(db.inputs.x),
            float(db.inputs.y),
            float(db.inputs.z),
        )
    except Exception as error:
        print(
            "[ROS2 Bridge] {error_name} error: "
            f"{{error}}",
            flush=True,
        )

    return True
"""


def _make_mode_script(
    setter_name: str,
    error_name: str,
) -> str:
    return f"""
from m0609_ros_bridge import {setter_name}


def compute(db):
    try:
        {setter_name}(str(db.inputs.mode))
    except Exception as error:
        print(
            "[ROS2 Bridge] {error_name} error: "
            f"{{error}}",
            flush=True,
        )

    return True
"""


def _make_quat_script(
    setter_name: str,
    error_name: str,
) -> str:
    return f"""
from m0609_ros_bridge import {setter_name}


def compute(db):
    try:
        {setter_name}(
            float(db.inputs.w),
            float(db.inputs.x),
            float(db.inputs.y),
            float(db.inputs.z),
        )
    except Exception as error:
        print(
            "[ROS2 Bridge] {error_name} error: "
            f"{{error}}",
            flush=True,
        )

    return True
"""


_LEFT_HAND_RAW_SCRIPT = _make_vector_script(
    "set_latest_left_hand_raw",
    "left hand raw",
)
_LEFT_HAND_TARGET_SCRIPT = _make_vector_script(
    "set_latest_left_hand_target",
    "left hand target",
)
_LEFT_HAND_MODE_SCRIPT = _make_mode_script(
    "set_latest_left_hand_mode",
    "left hand mode",
)
_LEFT_PALM_DIRECTION_SCRIPT = _make_vector_script(
    "set_latest_left_palm_direction",
    "left palm direction",
)

_RIGHT_HAND_RAW_SCRIPT = _make_vector_script(
    "set_latest_right_hand_raw",
    "right hand raw",
)
_RIGHT_HAND_TARGET_SCRIPT = _make_vector_script(
    "set_latest_right_hand_target",
    "right hand target",
)
_RIGHT_HAND_MODE_SCRIPT = _make_mode_script(
    "set_latest_right_hand_mode",
    "right hand mode",
)
_RIGHT_PALM_DIRECTION_SCRIPT = _make_vector_script(
    "set_latest_right_palm_direction",
    "right palm direction",
)

_LEFT_HAND_ORIENT_SCRIPT = _make_quat_script(
    "set_latest_left_hand_orient",
    "left hand orient",
)
_RIGHT_HAND_ORIENT_SCRIPT = _make_quat_script(
    "set_latest_right_hand_orient",
    "right hand orient",
)


def _remove_existing_graph(
    simulation_app,
) -> None:
    stage = omni.usd.get_context().get_stage()
    graph_prim = stage.GetPrimAtPath(GRAPH_PATH)

    if not graph_prim.IsValid():
        return

    stage.RemovePrim(GRAPH_PATH)

    for _ in range(3):
        simulation_app.update()


def _create_vector_ports(
    node_name: str,
) -> None:
    node = og.Controller.node(
        f"{GRAPH_PATH}/{node_name}"
    )

    for axis in ("x", "y", "z"):
        og.Controller.create_attribute(
            node=node,
            attr_name=f"inputs:{axis}",
            attr_type="double",
        )


def _create_quat_ports(
    node_name: str,
) -> None:
    node = og.Controller.node(
        f"{GRAPH_PATH}/{node_name}"
    )

    for axis in ("x", "y", "z", "w"):
        og.Controller.create_attribute(
            node=node,
            attr_name=f"inputs:{axis}",
            attr_type="double",
        )


def _create_script_ports() -> None:
    command_node = og.Controller.node(
        f"{GRAPH_PATH}/HandleCommand"
    )
    og.Controller.create_attribute(
        node=command_node,
        attr_name="inputs:command",
        attr_type=STRING_TYPE,
    )
    og.Controller.create_attribute(
        node=command_node,
        attr_name="outputs:result",
        attr_type=STRING_TYPE,
        attr_port=(
            og.AttributePortType
            .ATTRIBUTE_PORT_TYPE_OUTPUT
        ),
    )

    pick_node = og.Controller.node(
        f"{GRAPH_PATH}/HandlePickCommand"
    )
    og.Controller.create_attribute(
        node=pick_node,
        attr_name="inputs:command",
        attr_type=STRING_TYPE,
    )
    og.Controller.create_attribute(
        node=pick_node,
        attr_name="outputs:result",
        attr_type=STRING_TYPE,
        attr_port=(
            og.AttributePortType
            .ATTRIBUTE_PORT_TYPE_OUTPUT
        ),
    )

    return_tool_node = og.Controller.node(
        f"{GRAPH_PATH}/HandleReturnTool"
    )
    og.Controller.create_attribute(
        node=return_tool_node,
        attr_name="inputs:command",
        attr_type=STRING_TYPE,
    )
    og.Controller.create_attribute(
        node=return_tool_node,
        attr_name="outputs:result",
        attr_type=STRING_TYPE,
        attr_port=(
            og.AttributePortType
            .ATTRIBUTE_PORT_TYPE_OUTPUT
        ),
    )

    return_recent_node = og.Controller.node(
        f"{GRAPH_PATH}/HandleReturnRecent"
    )
    og.Controller.create_attribute(
        node=return_recent_node,
        attr_name="outputs:result",
        attr_type=STRING_TYPE,
        attr_port=(
            og.AttributePortType
            .ATTRIBUTE_PORT_TYPE_OUTPUT
        ),
    )

    status_node = og.Controller.node(
        f"{GRAPH_PATH}/HandleStatus"
    )
    og.Controller.create_attribute(
        node=status_node,
        attr_name="outputs:result",
        attr_type=STRING_TYPE,
        attr_port=(
            og.AttributePortType
            .ATTRIBUTE_PORT_TYPE_OUTPUT
        ),
    )

    for node_name in (
        "HandleLeftHandRaw",
        "HandleLeftHandTarget",
        "HandleLeftPalmDirection",
        "HandleRightHandRaw",
        "HandleRightHandTarget",
        "HandleRightPalmDirection",
    ):
        _create_vector_ports(node_name)

    for node_name in (
        "HandleLeftHandOrient",
        "HandleRightHandOrient",
    ):
        _create_quat_ports(node_name)

    for node_name in (
        "HandleLeftHandMode",
        "HandleRightHandMode",
    ):
        node = og.Controller.node(
            f"{GRAPH_PATH}/{node_name}"
        )
        og.Controller.create_attribute(
            node=node,
            attr_name="inputs:mode",
            attr_type=STRING_TYPE,
        )


def _subscriber_settings(
    node_name: str,
    *,
    message_package: str,
    message_name: str,
    topic_name: str,
    qos_profile: str,
):
    return [
        (
            f"{node_name}.inputs:messagePackage",
            message_package,
        ),
        (
            f"{node_name}.inputs:messageSubfolder",
            "msg",
        ),
        (
            f"{node_name}.inputs:messageName",
            message_name,
        ),
        (
            f"{node_name}.inputs:topicName",
            topic_name,
        ),
        (
            f"{node_name}.inputs:qosProfile",
            qos_profile,
        ),
    ]


def setup_m0609_ros_bridge(
    manager,
    simulation_app,
) -> None:
    global _COMMAND_MANAGER

    _COMMAND_MANAGER = manager

    reset_left_hand_mode_cache("TRACKING")
    reset_right_hand_mode_cache("TRACKING")

    enable_extension("isaacsim.ros2.bridge")
    enable_extension("omni.graph.scriptnode")

    for _ in range(3):
        simulation_app.update()

    _remove_existing_graph(simulation_app)

    create_nodes = [
        (
            "OnPlaybackTick",
            "omni.graph.action.OnPlaybackTick",
        ),
        (
            "ROS2Context",
            "isaacsim.ros2.bridge.ROS2Context",
        ),
        (
            "CommandSubscriber",
            "isaacsim.ros2.bridge.ROS2Subscriber",
        ),
        (
            "HandleCommand",
            "omni.graph.scriptnode.ScriptNode",
        ),
        (
            "ResultPublisher",
            "isaacsim.ros2.bridge.ROS2Publisher",
        ),
        (
            "PickCommandSubscriber",
            "isaacsim.ros2.bridge.ROS2Subscriber",
        ),
        (
            "HandlePickCommand",
            "omni.graph.scriptnode.ScriptNode",
        ),
        (
            "ReturnToolSubscriber",
            "isaacsim.ros2.bridge.ROS2Subscriber",
        ),
        (
            "HandleReturnTool",
            "omni.graph.scriptnode.ScriptNode",
        ),
        (
            "ReturnRecentSubscriber",
            "isaacsim.ros2.bridge.ROS2Subscriber",
        ),
        (
            "HandleReturnRecent",
            "omni.graph.scriptnode.ScriptNode",
        ),
        (
            "PickResultPublisher",
            "isaacsim.ros2.bridge.ROS2Publisher",
        ),
        (
            "ReturnToolResultPublisher",
            "isaacsim.ros2.bridge.ROS2Publisher",
        ),
        (
            "ReturnRecentResultPublisher",
            "isaacsim.ros2.bridge.ROS2Publisher",
        ),
        (
            "HandleStatus",
            "omni.graph.scriptnode.ScriptNode",
        ),
        (
            "StatusPublisher",
            "isaacsim.ros2.bridge.ROS2Publisher",
        ),
    ]

    hand_nodes = (
        ("LeftHandRaw", "HandleLeftHandRaw"),
        ("LeftHandTarget", "HandleLeftHandTarget"),
        ("LeftHandMode", "HandleLeftHandMode"),
        ("LeftPalmDirection", "HandleLeftPalmDirection"),
        ("LeftHandOrient", "HandleLeftHandOrient"),
        ("RightHandRaw", "HandleRightHandRaw"),
        ("RightHandTarget", "HandleRightHandTarget"),
        ("RightHandMode", "HandleRightHandMode"),
        ("RightPalmDirection", "HandleRightPalmDirection"),
        ("RightHandOrient", "HandleRightHandOrient"),
    )

    for subscriber_prefix, handler_name in hand_nodes:
        create_nodes.extend(
            [
                (
                    f"{subscriber_prefix}Subscriber",
                    "isaacsim.ros2.bridge.ROS2Subscriber",
                ),
                (
                    handler_name,
                    "omni.graph.scriptnode.ScriptNode",
                ),
            ]
        )

    set_values = [
        (
            "CommandSubscriber.inputs:messagePackage",
            "std_msgs",
        ),
        (
            "CommandSubscriber.inputs:messageSubfolder",
            "msg",
        ),
        (
            "CommandSubscriber.inputs:messageName",
            "String",
        ),
        (
            "CommandSubscriber.inputs:topicName",
            COMMAND_TOPIC,
        ),
        (
            "ResultPublisher.inputs:messagePackage",
            "std_msgs",
        ),
        (
            "ResultPublisher.inputs:messageSubfolder",
            "msg",
        ),
        (
            "ResultPublisher.inputs:messageName",
            "String",
        ),
        (
            "ResultPublisher.inputs:topicName",
            RESULT_TOPIC,
        ),
        (
            "HandleCommand.inputs:script",
            _COMMAND_SCRIPT,
        ),
        (
            "PickCommandSubscriber.inputs:messagePackage",
            "std_msgs",
        ),
        (
            "PickCommandSubscriber.inputs:messageSubfolder",
            "msg",
        ),
        (
            "PickCommandSubscriber.inputs:messageName",
            "String",
        ),
        (
            "PickCommandSubscriber.inputs:topicName",
            PICK_COMMAND_TOPIC,
        ),
        (
            "HandlePickCommand.inputs:script",
            _PICK_COMMAND_SCRIPT,
        ),
        (
            "ReturnToolSubscriber.inputs:messagePackage",
            "std_msgs",
        ),
        (
            "ReturnToolSubscriber.inputs:messageSubfolder",
            "msg",
        ),
        (
            "ReturnToolSubscriber.inputs:messageName",
            "String",
        ),
        (
            "ReturnToolSubscriber.inputs:topicName",
            RETURN_TOOL_TOPIC,
        ),
        (
            "HandleReturnTool.inputs:script",
            _RETURN_TOOL_SCRIPT,
        ),
        (
            "ReturnRecentSubscriber.inputs:messagePackage",
            "std_msgs",
        ),
        (
            "ReturnRecentSubscriber.inputs:messageSubfolder",
            "msg",
        ),
        (
            "ReturnRecentSubscriber.inputs:messageName",
            "Empty",
        ),
        (
            "ReturnRecentSubscriber.inputs:topicName",
            RETURN_RECENT_TOPIC,
        ),
        (
            "HandleReturnRecent.inputs:script",
            _RETURN_RECENT_SCRIPT,
        ),
        (
            "PickResultPublisher.inputs:messagePackage",
            "std_msgs",
        ),
        (
            "PickResultPublisher.inputs:messageSubfolder",
            "msg",
        ),
        (
            "PickResultPublisher.inputs:messageName",
            "String",
        ),
        (
            "PickResultPublisher.inputs:topicName",
            TOOL_COMMAND_RESULT_TOPIC,
        ),
        (
            "ReturnToolResultPublisher.inputs:messagePackage",
            "std_msgs",
        ),
        (
            "ReturnToolResultPublisher.inputs:messageSubfolder",
            "msg",
        ),
        (
            "ReturnToolResultPublisher.inputs:messageName",
            "String",
        ),
        (
            "ReturnToolResultPublisher.inputs:topicName",
            TOOL_COMMAND_RESULT_TOPIC,
        ),
        (
            "ReturnRecentResultPublisher.inputs:messagePackage",
            "std_msgs",
        ),
        (
            "ReturnRecentResultPublisher.inputs:messageSubfolder",
            "msg",
        ),
        (
            "ReturnRecentResultPublisher.inputs:messageName",
            "String",
        ),
        (
            "ReturnRecentResultPublisher.inputs:topicName",
            TOOL_COMMAND_RESULT_TOPIC,
        ),
    ]

    set_values.extend(
        _subscriber_settings(
            "LeftHandRawSubscriber",
            message_package="geometry_msgs",
            message_name="Point",
            topic_name=LEFT_HAND_RAW_TOPIC,
            qos_profile=HAND_POSITION_QOS,
        )
    )
    set_values.extend(
        _subscriber_settings(
            "LeftHandTargetSubscriber",
            message_package="geometry_msgs",
            message_name="Point",
            topic_name=LEFT_HAND_TARGET_TOPIC,
            qos_profile=HAND_POSITION_QOS,
        )
    )
    set_values.extend(
        _subscriber_settings(
            "LeftHandModeSubscriber",
            message_package="std_msgs",
            message_name="String",
            topic_name=LEFT_HAND_MODE_TOPIC,
            qos_profile=HAND_MODE_QOS,
        )
    )
    set_values.extend(
        _subscriber_settings(
            "LeftPalmDirectionSubscriber",
            message_package="geometry_msgs",
            message_name="Vector3",
            topic_name=LEFT_PALM_DIRECTION_TOPIC,
            qos_profile=HAND_POSITION_QOS,
        )
    )

    set_values.extend(
        _subscriber_settings(
            "RightHandRawSubscriber",
            message_package="geometry_msgs",
            message_name="Point",
            topic_name=RIGHT_HAND_RAW_TOPIC,
            qos_profile=HAND_POSITION_QOS,
        )
    )
    set_values.extend(
        _subscriber_settings(
            "RightHandTargetSubscriber",
            message_package="geometry_msgs",
            message_name="Point",
            topic_name=RIGHT_HAND_TARGET_TOPIC,
            qos_profile=HAND_POSITION_QOS,
        )
    )
    set_values.extend(
        _subscriber_settings(
            "RightHandModeSubscriber",
            message_package="std_msgs",
            message_name="String",
            topic_name=RIGHT_HAND_MODE_TOPIC,
            qos_profile=HAND_MODE_QOS,
        )
    )
    set_values.extend(
        _subscriber_settings(
            "RightPalmDirectionSubscriber",
            message_package="geometry_msgs",
            message_name="Vector3",
            topic_name=RIGHT_PALM_DIRECTION_TOPIC,
            qos_profile=HAND_POSITION_QOS,
        )
    )

    set_values.extend(
        _subscriber_settings(
            "LeftHandOrientSubscriber",
            message_package="geometry_msgs",
            message_name="Quaternion",
            topic_name=LEFT_HAND_ORIENT_TOPIC,
            qos_profile=HAND_POSITION_QOS,
        )
    )
    set_values.extend(
        _subscriber_settings(
            "RightHandOrientSubscriber",
            message_package="geometry_msgs",
            message_name="Quaternion",
            topic_name=RIGHT_HAND_ORIENT_TOPIC,
            qos_profile=HAND_POSITION_QOS,
        )
    )

    set_values.extend(
        [
            (
                "HandleLeftHandRaw.inputs:script",
                _LEFT_HAND_RAW_SCRIPT,
            ),
            (
                "HandleLeftHandTarget.inputs:script",
                _LEFT_HAND_TARGET_SCRIPT,
            ),
            (
                "HandleLeftHandMode.inputs:script",
                _LEFT_HAND_MODE_SCRIPT,
            ),
            (
                "HandleLeftPalmDirection.inputs:script",
                _LEFT_PALM_DIRECTION_SCRIPT,
            ),
            (
                "HandleRightHandRaw.inputs:script",
                _RIGHT_HAND_RAW_SCRIPT,
            ),
            (
                "HandleRightHandTarget.inputs:script",
                _RIGHT_HAND_TARGET_SCRIPT,
            ),
            (
                "HandleRightHandMode.inputs:script",
                _RIGHT_HAND_MODE_SCRIPT,
            ),
            (
                "HandleRightPalmDirection.inputs:script",
                _RIGHT_PALM_DIRECTION_SCRIPT,
            ),
            (
                "HandleLeftHandOrient.inputs:script",
                _LEFT_HAND_ORIENT_SCRIPT,
            ),
            (
                "HandleRightHandOrient.inputs:script",
                _RIGHT_HAND_ORIENT_SCRIPT,
            ),
        ]
    )

    set_values.extend(
        [
            ("StatusPublisher.inputs:messagePackage", "std_msgs"),
            ("StatusPublisher.inputs:messageSubfolder", "msg"),
            ("StatusPublisher.inputs:messageName", "String"),
            ("StatusPublisher.inputs:topicName", STATUS_TOPIC),
            ("HandleStatus.inputs:script", _STATUS_SCRIPT),
        ]
    )

    tick_connections = [
        (
            "OnPlaybackTick.outputs:tick",
            "CommandSubscriber.inputs:execIn",
        ),
        (
            "OnPlaybackTick.outputs:tick",
            "PickCommandSubscriber.inputs:execIn",
        ),
        (
            "OnPlaybackTick.outputs:tick",
            "ReturnToolSubscriber.inputs:execIn",
        ),
        (
            "OnPlaybackTick.outputs:tick",
            "ReturnRecentSubscriber.inputs:execIn",
        ),
        (
            "OnPlaybackTick.outputs:tick",
            "HandleStatus.inputs:execIn",
        ),
    ]

    context_connections = [
        (
            "ROS2Context.outputs:context",
            "CommandSubscriber.inputs:context",
        ),
        (
            "ROS2Context.outputs:context",
            "ResultPublisher.inputs:context",
        ),
        (
            "ROS2Context.outputs:context",
            "PickCommandSubscriber.inputs:context",
        ),
        (
            "ROS2Context.outputs:context",
            "ReturnToolSubscriber.inputs:context",
        ),
        (
            "ROS2Context.outputs:context",
            "ReturnRecentSubscriber.inputs:context",
        ),
        (
            "ROS2Context.outputs:context",
            "PickResultPublisher.inputs:context",
        ),
        (
            "ROS2Context.outputs:context",
            "ReturnToolResultPublisher.inputs:context",
        ),
        (
            "ROS2Context.outputs:context",
            "ReturnRecentResultPublisher.inputs:context",
        ),
        (
            "ROS2Context.outputs:context",
            "StatusPublisher.inputs:context",
        ),
    ]

    for subscriber_prefix, _ in hand_nodes:
        subscriber_name = f"{subscriber_prefix}Subscriber"

        tick_connections.append(
            (
                "OnPlaybackTick.outputs:tick",
                f"{subscriber_name}.inputs:execIn",
            )
        )
        context_connections.append(
            (
                "ROS2Context.outputs:context",
                f"{subscriber_name}.inputs:context",
            )
        )

    graph, _, _, _ = og.Controller.edit(
        {
            "graph_path": GRAPH_PATH,
            "evaluator_name": "execution",
        },
        {
            og.Controller.Keys.CREATE_NODES: create_nodes,
            og.Controller.Keys.SET_VALUES: set_values,
            og.Controller.Keys.CONNECT: (
                tick_connections
                + context_connections
            ),
        },
    )

    for _ in range(10):
        simulation_app.update()

    _create_script_ports()

    for _ in range(3):
        simulation_app.update()

    def path(
        node: str,
        port: str,
    ) -> str:
        return f"{GRAPH_PATH}/{node}.{port}"

    second_connections = [
        (
            path(
                "CommandSubscriber",
                "outputs:execOut",
            ),
            path(
                "HandleCommand",
                "inputs:execIn",
            ),
        ),
        (
            path(
                "CommandSubscriber",
                "outputs:data",
            ),
            path(
                "HandleCommand",
                "inputs:command",
            ),
        ),
        (
            path(
                "HandleCommand",
                "outputs:execOut",
            ),
            path(
                "ResultPublisher",
                "inputs:execIn",
            ),
        ),
        (
            path(
                "HandleCommand",
                "outputs:result",
            ),
            path(
                "ResultPublisher",
                "inputs:data",
            ),
        ),
        (
            path(
                "HandleStatus",
                "outputs:execOut",
            ),
            path(
                "StatusPublisher",
                "inputs:execIn",
            ),
        ),
        (
            path(
                "HandleStatus",
                "outputs:result",
            ),
            path(
                "StatusPublisher",
                "inputs:data",
            ),
        ),
        (
            path(
                "PickCommandSubscriber",
                "outputs:execOut",
            ),
            path(
                "HandlePickCommand",
                "inputs:execIn",
            ),
        ),
        (
            path(
                "PickCommandSubscriber",
                "outputs:data",
            ),
            path(
                "HandlePickCommand",
                "inputs:command",
            ),
        ),
        (
            path(
                "ReturnToolSubscriber",
                "outputs:execOut",
            ),
            path(
                "HandleReturnTool",
                "inputs:execIn",
            ),
        ),
        (
            path(
                "ReturnToolSubscriber",
                "outputs:data",
            ),
            path(
                "HandleReturnTool",
                "inputs:command",
            ),
        ),
        (
            path(
                "ReturnRecentSubscriber",
                "outputs:execOut",
            ),
            path(
                "HandleReturnRecent",
                "inputs:execIn",
            ),
        ),
        (
            path(
                "HandlePickCommand",
                "outputs:execOut",
            ),
            path(
                "PickResultPublisher",
                "inputs:execIn",
            ),
        ),
        (
            path(
                "HandlePickCommand",
                "outputs:result",
            ),
            path(
                "PickResultPublisher",
                "inputs:data",
            ),
        ),
        (
            path(
                "HandleReturnTool",
                "outputs:execOut",
            ),
            path(
                "ReturnToolResultPublisher",
                "inputs:execIn",
            ),
        ),
        (
            path(
                "HandleReturnTool",
                "outputs:result",
            ),
            path(
                "ReturnToolResultPublisher",
                "inputs:data",
            ),
        ),
        (
            path(
                "HandleReturnRecent",
                "outputs:execOut",
            ),
            path(
                "ReturnRecentResultPublisher",
                "inputs:execIn",
            ),
        ),
        (
            path(
                "HandleReturnRecent",
                "outputs:result",
            ),
            path(
                "ReturnRecentResultPublisher",
                "inputs:data",
            ),
        ),
    ]

    vector_connections = (
        (
            "LeftHandRawSubscriber",
            "HandleLeftHandRaw",
        ),
        (
            "LeftHandTargetSubscriber",
            "HandleLeftHandTarget",
        ),
        (
            "LeftPalmDirectionSubscriber",
            "HandleLeftPalmDirection",
        ),
        (
            "RightHandRawSubscriber",
            "HandleRightHandRaw",
        ),
        (
            "RightHandTargetSubscriber",
            "HandleRightHandTarget",
        ),
        (
            "RightPalmDirectionSubscriber",
            "HandleRightPalmDirection",
        ),
    )

    for subscriber_name, handler_name in vector_connections:
        second_connections.append(
            (
                path(
                    subscriber_name,
                    "outputs:execOut",
                ),
                path(
                    handler_name,
                    "inputs:execIn",
                ),
            )
        )

        for axis in ("x", "y", "z"):
            second_connections.append(
                (
                    path(
                        subscriber_name,
                        f"outputs:{axis}",
                    ),
                    path(
                        handler_name,
                        f"inputs:{axis}",
                    ),
                )
            )

    quat_connections = (
        (
            "LeftHandOrientSubscriber",
            "HandleLeftHandOrient",
        ),
        (
            "RightHandOrientSubscriber",
            "HandleRightHandOrient",
        ),
    )

    for subscriber_name, handler_name in quat_connections:
        second_connections.append(
            (
                path(
                    subscriber_name,
                    "outputs:execOut",
                ),
                path(
                    handler_name,
                    "inputs:execIn",
                ),
            )
        )

        for axis in ("x", "y", "z", "w"):
            second_connections.append(
                (
                    path(
                        subscriber_name,
                        f"outputs:{axis}",
                    ),
                    path(
                        handler_name,
                        f"inputs:{axis}",
                    ),
                )
            )

    mode_connections = (
        (
            "LeftHandModeSubscriber",
            "HandleLeftHandMode",
        ),
        (
            "RightHandModeSubscriber",
            "HandleRightHandMode",
        ),
    )

    for subscriber_name, handler_name in mode_connections:
        second_connections.extend(
            [
                (
                    path(
                        subscriber_name,
                        "outputs:execOut",
                    ),
                    path(
                        handler_name,
                        "inputs:execIn",
                    ),
                ),
                (
                    path(
                        subscriber_name,
                        "outputs:data",
                    ),
                    path(
                        handler_name,
                        "inputs:mode",
                    ),
                ),
            ]
        )

    og.Controller.edit(
        graph,
        {
            og.Controller.Keys.CONNECT: second_connections,
        },
    )

    for _ in range(3):
        simulation_app.update()

    # status publisher 는 이제 초기 그래프 빌드(build_graph)에 통합돼 있다.
    # (동적 추가는 재생 중 그래프 편집이 자주 깨져서 제거함)

    print(
        "[ROS2 Bridge] ready:\n"
        f" tool request: {PICK_COMMAND_TOPIC} "
        "(std_msgs/String)\n"
        f" dashboard status: {STATUS_TOPIC} "
        "(std_msgs/String)\n"
        f" tool return: {RETURN_TOOL_TOPIC} "
        "(std_msgs/String)\n"
        f" recent return: {RETURN_RECENT_TOPIC} "
        "(std_msgs/Empty)\n"
        f" command result: {TOOL_COMMAND_RESULT_TOPIC} "
        "(std_msgs/String)\n"
        f" left raw: {LEFT_HAND_RAW_TOPIC}\n"
        f" left target: {LEFT_HAND_TARGET_TOPIC}\n"
        f" left mode: {LEFT_HAND_MODE_TOPIC}\n"
        f" left palm: {LEFT_PALM_DIRECTION_TOPIC}\n"
        f" left orient: {LEFT_HAND_ORIENT_TOPIC}\n"
        f" right raw: {RIGHT_HAND_RAW_TOPIC}\n"
        f" right target: {RIGHT_HAND_TARGET_TOPIC}\n"
        f" right mode: {RIGHT_HAND_MODE_TOPIC}\n"
        f" right palm: {RIGHT_PALM_DIRECTION_TOPIC}\n"
        f" right orient: {RIGHT_HAND_ORIENT_TOPIC}\n"
        " Robot B tracking input: RIGHT HAND",
        flush=True,
    )
# main.py
from isaacsim import SimulationApp


simulation_app = SimulationApp(
    {
        "headless": False,
    }
)


from isaacsim.core.utils.extensions import enable_extension


enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.robot.surface_gripper")
enable_extension("omni.graph.scriptnode")

for _ in range(10):
    simulation_app.update()


import sys
import threading
import time

import numpy as np
import omni.usd

from isaacsim.core.api import World
from isaacsim.core.utils.rotations import (
    quat_to_euler_angles,
)

from m0609_config import (
    CUROBO_ROBOT_CONFIG_PATH,
    EE_LINK_NAME,
    INITIAL_SETTLING_FRAMES,
    JOINT1_TURN_MAX_STEP_DEG,
    JOINT1_TURN_TOLERANCE_DEG,
    M0609_DESCRIPTION_PATH,
    M0609_RMPFLOW_CONFIG_PATH,
    M0609_URDF_PATH,
    PICK_APPROACH_Z_CORRECTION,
    PICK_DEFAULT_EE_OFFSET,
    PICK_EVENTS_DT,
    PLACE_APPROACH_GAP,
    PLACE_HIGH_OFFSET,
    PLACE_LINK6_ABOVE_TRAY,
    PLACE_MOVE_TOLERANCE,
    PLACE_RELEASE_MIN_WAIT_FRAMES,
    PLACE_RELEASE_RETRY_INTERVAL,
    PLACE_RELEASE_STABLE_FRAMES,
    PLACE_RELEASE_TIMEOUT_FRAMES,
    RMPFLOW_DIR,
    ROBOT_A_PRIM_PATH,
    ROBOT_A_TRACKING_JOINT1_DELTA_DEG,
    ROBOT_A_SCENE_NAME,
    ROBOT_A_SUPPORTED_TRAY_COMMANDS,
    ROBOT_B_PRIM_PATH,
    ROBOT_B_TRACKING_JOINT1_DELTA_DEG,
    ROBOT_TRANSIT_Y_OFFSET,
    ROBOT_B_SCENE_NAME,
    ROBOT_B_SUPPORTED_TRAY_COMMANDS,
    RETURN_HOME_MAX_STEP_DEG,
    RETURN_HOME_TOLERANCE_DEG,
    RETURN_HOME_WRIST_MAX_STEP_DEG,
    ROBOT_USD_PATH,
    SAFE_JOINT_RETURN_MAX_STEP_DEG,
    SAFE_JOINT_RETURN_TOLERANCE_DEG,
    TABLE_HEIGHT,
    EXTERNAL_TOOL_DETECTION_TIMEOUT_SEC,
    EXTERNAL_TOOL_TRAY_MATCH_RADIUS,
    HAND_USDA_PATH,
    HAND_LEFT_REF_PRIM,
    HAND_LEFT_MODEL_PRIM,
    HAND_LEFT_ORIENT_OFFSET_DEG,
    HAND_LEFT_PIVOT_OFFSET,
    HAND_LEFT_ROLL_SIGN,
    HAND_RIGHT_REF_PRIM,
    HAND_RIGHT_MODEL_PRIM,
    HAND_RIGHT_ORIENT_OFFSET_DEG,
    HAND_RIGHT_PIVOT_OFFSET,
    HAND_RIGHT_ROLL_SIGN,
    HAND_ROLL_AXIS,
    HAND_STABILIZE_ROLL,
    TOOL_DROP_HEIGHT,
    TOOL_MASS,
    TOOL_NAMES,
    TOOL_RANDOM_SEED,
    TOOL_SCALES,
    TOOL_USDS,
    TRACKING_MAX_JOINT_STEP,
    TRACKING_TOOL_ORIENTATION,
    TRACKING_USE_MPC,
    TRACKING_Z_MAX,
    TRACKING_Z_MIN,
    TRANSIT_HEIGHT,
    TRANSPORT_Z_OFFSET,
    TRAY_ORIENTATION,
    TRAY_SPAWN_POSITIONS,
    TRAY_TOP_Z,
    TRAY_USD_PATH,
    WORK_TABLE_X_RANGE,
    WORK_TABLE_Y_RANGE,
    TABLE_EDGE_MARGIN,
)


if RMPFLOW_DIR not in sys.path:
    sys.path.insert(
        0,
        RMPFLOW_DIR,
    )


from hand_input import CachedHandInput
from hand_model_visualizer import (
    HandModelVisualizer,
)
from m0609_dynamic_scene import (
    create_surgical_trays_and_tools,
)
from m0609_move_controller import (
    M0609MoveController,
)
from m0609_pick_place_controller_surface import (
    PickPlaceController,
)
from m0609_ros_bridge import (
    get_latest_left_hand_mode,
    get_latest_left_hand_orient,
    get_latest_left_hand_raw,
    get_latest_left_hand_target,
    get_latest_right_hand_mode,
    get_latest_right_hand_orient,
    get_latest_right_hand_raw,
    get_latest_right_hand_target,
    reset_left_hand_mode_cache,
    reset_right_hand_mode_cache,
    setup_m0609_ros_bridge,
)
from m0609_state_machine import (
    M0609StateMachine,
)
from m0609_task import (
    M0609BasicTask,
    initialize_robot,
)
from m0609_tracking_controller import (
    M0609TrackingController,
)
from robot_manager import RobotManager
from robot_runtime import RobotProfile
from tool_state_manager import ToolStateManager


def _open_full_scene() -> None:
    result = (
        omni.usd
        .get_context()
        .open_stage(ROBOT_USD_PATH)
    )

    if result is False:
        raise RuntimeError(
            "Stage를 열지 못했습니다: "
            f"{ROBOT_USD_PATH}"
        )

    for _ in range(80):
        simulation_app.update()

    stage = (
        omni.usd
        .get_context()
        .get_stage()
    )

    missing_robot_paths = [
        path
        for path in (
            ROBOT_A_PRIM_PATH,
            ROBOT_B_PRIM_PATH,
        )
        if not stage.GetPrimAtPath(path).IsValid()
    ]

    if missing_robot_paths:
        raise RuntimeError(
            "로봇 Prim을 찾지 못했습니다:\n- "
            + "\n- ".join(missing_robot_paths)
        )


def _get_robot_base_pose(
    robot,
):
    position, orientation = (
        robot.get_world_pose()
    )

    position = np.asarray(
        position,
        dtype=np.float64,
    )

    orientation = np.asarray(
        orientation,
        dtype=np.float64,
    )

    euler = quat_to_euler_angles(
        orientation
    )

    yaw_deg = float(
        np.degrees(euler[2])
    )

    return position, yaw_deg



def _create_tracking_controller(
    *,
    robot,
    robot_id: str,
    base_position,
    base_yaw_deg: float,
):
    print(
        f"[main] Robot {robot_id} base pose: "
        f"position={base_position.round(4)}, "
        f"yaw={base_yaw_deg:.2f} deg",
        flush=True,
    )

    return M0609TrackingController(
        robot=robot,
        robot_config_path=(
            CUROBO_ROBOT_CONFIG_PATH
        ),
        base_position=base_position,
        base_yaw_deg=base_yaw_deg,
        tool_orientation=(
            TRACKING_TOOL_ORIENTATION
        ),
        z_min=TRACKING_Z_MIN,
        z_max=TRACKING_Z_MAX,
        max_joint_step=(
            TRACKING_MAX_JOINT_STEP
        ),
        use_mpc=TRACKING_USE_MPC,
    )


def _create_pick_place_controller(
    *,
    robot,
    robot_id: str,
):
    return PickPlaceController(
        name=(
            "pick_place_controller_"
            f"{robot_id.lower()}"
        ),
        gripper=robot.gripper,
        robot_articulation=robot,
        end_effector_initial_height=(
            TABLE_HEIGHT + 0.20
        ),
        events_dt=list(PICK_EVENTS_DT),
        urdf_path=M0609_URDF_PATH,
        robot_description_path=(
            M0609_DESCRIPTION_PATH
        ),
        rmpflow_config_path=(
            M0609_RMPFLOW_CONFIG_PATH
        ),
        end_effector_frame_name=(
            EE_LINK_NAME
        ),
    )


def _create_move_controller(
    *,
    robot,
    robot_id: str,
):
    return M0609MoveController(
        name=(
            "workflow_move_controller_"
            f"{robot_id.lower()}"
        ),
        robot_articulation=robot,
        urdf_path=M0609_URDF_PATH,
        robot_description_path=(
            M0609_DESCRIPTION_PATH
        ),
        rmpflow_config_path=(
            M0609_RMPFLOW_CONFIG_PATH
        ),
        end_effector_frame_name=(
            EE_LINK_NAME
        ),
        tcp_offset_local=np.zeros(
            3,
            dtype=np.float64,
        ),
        position_tolerance=(
            PLACE_MOVE_TOLERANCE
        ),
    )


def _create_state_machine(
    *,
    robot_id: str,
    robot,
    tray_registry,
    hand_input,
    initial_joint_positions,
    tracking_controller,
    pick_place_controller,
    move_controller,
):
    return M0609StateMachine(
        robot_id=robot_id,
        robot=robot,
        tray_registry=tray_registry,
        hand_input=hand_input,
        idle_joint_positions=(
            initial_joint_positions
        ),
        tracking_controller=(
            tracking_controller
        ),
        pick_place_controller=(
            pick_place_controller
        ),
        move_controller=move_controller,
        pick_default_ee_offset=(
            PICK_DEFAULT_EE_OFFSET
        ),
        pick_approach_z_correction=(
            PICK_APPROACH_Z_CORRECTION
        ),
        place_link6_above_tray=(
            PLACE_LINK6_ABOVE_TRAY
        ),
        place_high_offset=(
            PLACE_HIGH_OFFSET
        ),
        place_approach_gap=(
            PLACE_APPROACH_GAP
        ),
        transport_z_offset=(
            TRANSPORT_Z_OFFSET
        ),
        place_release_min_wait_frames=(
            PLACE_RELEASE_MIN_WAIT_FRAMES
        ),
        place_release_stable_frames=(
            PLACE_RELEASE_STABLE_FRAMES
        ),
        place_release_retry_interval=(
            PLACE_RELEASE_RETRY_INTERVAL
        ),
        place_release_timeout_frames=(
            PLACE_RELEASE_TIMEOUT_FRAMES
        ),
        joint1_turn_tolerance_rad=np.deg2rad(
            JOINT1_TURN_TOLERANCE_DEG
        ),
        joint1_turn_max_step_rad=np.deg2rad(
            JOINT1_TURN_MAX_STEP_DEG
        ),
        safe_joint_return_max_step_rad=np.deg2rad(
            SAFE_JOINT_RETURN_MAX_STEP_DEG
        ),
        safe_joint_return_tolerance_rad=np.deg2rad(
            SAFE_JOINT_RETURN_TOLERANCE_DEG
        ),
        return_home_max_step_rad=np.deg2rad(
            RETURN_HOME_MAX_STEP_DEG
        ),
        return_home_wrist_max_step_rad=np.deg2rad(
            RETURN_HOME_WRIST_MAX_STEP_DEG
        ),
        idle_joint_tolerance=np.deg2rad(
            RETURN_HOME_TOLERANCE_DEG
        ),
    )


# 터미널에 "randomize" 입력 시 도구 위치를 재랜덤한다.
# 백그라운드 스레드는 플래그만 세우고, 실제 적용은 메인 루프에서
# 안전할 때(로봇 이동 중이 아닐 때)만 수행한다.
_RANDOMIZE_REQUEST = threading.Event()


def _start_stdin_listener() -> None:
    def _listen() -> None:
        for line in sys.stdin:
            command = line.strip().lower()

            if command == "randomize":
                _RANDOMIZE_REQUEST.set()
                print(
                    "[randomize] 요청 접수 — 안전할 때 적용됩니다.",
                    flush=True,
                )
            elif command:
                print(
                    f"[stdin] 알 수 없는 명령: {command} "
                    "(사용 가능: randomize)",
                    flush=True,
                )

    threading.Thread(
        target=_listen,
        name="stdin-command-listener",
        daemon=True,
    ).start()


def main() -> None:
    _open_full_scene()

    world = World(
        stage_units_in_meters=1.0,
    )

    task = M0609BasicTask()
    world.add_task(task)

    tool_state_manager = ToolStateManager(
        tray_positions=TRAY_SPAWN_POSITIONS,
        tool_ids=TOOL_NAMES,
        random_seed=TOOL_RANDOM_SEED,
        external_timeout_sec=(
            EXTERNAL_TOOL_DETECTION_TIMEOUT_SEC
        ),
        tray_match_radius=(
            EXTERNAL_TOOL_TRAY_MATCH_RADIUS
        ),
    )

    tool_index_by_name = {
        tool_name: index
        for index, tool_name
        in enumerate(TOOL_NAMES)
    }

    tray_tool_indices = {
        tray_id: tool_index_by_name[tool_name]
        for tray_id, tool_name
        in tool_state_manager
        .get_tray_tool_snapshot()
        .items()
    }

    tray_registry = (
        create_surgical_trays_and_tools(
            world=world,
            tray_usd_path=TRAY_USD_PATH,
            tool_usds=TOOL_USDS,
            tray_tool_indices=tray_tool_indices,
            tray_positions=(
                TRAY_SPAWN_POSITIONS
            ),
            tray_orientation=(
                TRAY_ORIENTATION
            ),
            tray_top_z=TRAY_TOP_Z,
            tool_drop_height=(
                TOOL_DROP_HEIGHT
            ),
            tool_mass=TOOL_MASS,
            tool_scales=TOOL_SCALES,
            simulation_app=(
                simulation_app
            ),
            table_x_range=WORK_TABLE_X_RANGE,
            table_y_range=WORK_TABLE_Y_RANGE,
            table_margin=TABLE_EDGE_MARGIN,
        )
    )

    world.reset()

    robot_a = world.scene.get_object(
        ROBOT_A_SCENE_NAME
    )

    robot_b = world.scene.get_object(
        ROBOT_B_SCENE_NAME
    )

    if robot_a is None:
        raise RuntimeError(
            "Scene Robot A 객체 없음: "
            f"{ROBOT_A_SCENE_NAME}"
        )

    if robot_b is None:
        raise RuntimeError(
            "Scene Robot B 객체 없음: "
            f"{ROBOT_B_SCENE_NAME}"
        )

    initialize_robot(
        robot_a,
        world,
    )

    initialize_robot(
        robot_b,
        world,
    )

    initial_joint_positions_a = (
        np.asarray(
            robot_a.get_joint_positions(),
            dtype=np.float64,
        ).copy()
    )

    initial_joint_positions_b = (
        np.asarray(
            robot_b.get_joint_positions(),
            dtype=np.float64,
        ).copy()
    )

    (
        robot_a_base_position,
        robot_a_base_yaw_deg,
    ) = _get_robot_base_pose(robot_a)

    (
        robot_b_base_position,
        robot_b_base_yaw_deg,
    ) = _get_robot_base_pose(robot_b)

    # 현재 배치:
    #
    #     4 5
    #   a 2 3 b
    #   A 0 1 B
    #
    # a, b는 각 로봇 베이스에서 Y축 +0.30m 지점이다.
    # X 좌표는 각 로봇 베이스와 동일하게 유지한다.
    robot_a_transit_position = np.array(
        [
            robot_a_base_position[0],
            robot_a_base_position[1]
            + ROBOT_TRANSIT_Y_OFFSET,
            TRANSIT_HEIGHT,
        ],
        dtype=np.float64,
    )

    robot_b_transit_position = np.array(
        [
            robot_b_base_position[0],
            robot_b_base_position[1]
            + ROBOT_TRANSIT_Y_OFFSET,
            TRANSIT_HEIGHT,
        ],
        dtype=np.float64,
    )

    print(
        "[main] upper transit positions:\n"
        f" A/TRANSIT_A="
        f"{robot_a_transit_position.round(4)}\n"
        f" B/TRANSIT_B="
        f"{robot_b_transit_position.round(4)}",
        flush=True,
    )

    tracking_controller_a = (
        _create_tracking_controller(
            robot=robot_a,
            robot_id="A",
            base_position=(
                robot_a_base_position
            ),
            base_yaw_deg=(
                robot_a_base_yaw_deg
            ),
        )
    )

    tracking_controller_b = (
        _create_tracking_controller(
            robot=robot_b,
            robot_id="B",
            base_position=(
                robot_b_base_position
            ),
            base_yaw_deg=(
                robot_b_base_yaw_deg
            ),
        )
    )

    pick_place_controller_a = (
        _create_pick_place_controller(
            robot=robot_a,
            robot_id="A",
        )
    )

    pick_place_controller_b = (
        _create_pick_place_controller(
            robot=robot_b,
            robot_id="B",
        )
    )

    move_controller_a = (
        _create_move_controller(
            robot=robot_a,
            robot_id="A",
        )
    )

    move_controller_b = (
        _create_move_controller(
            robot=robot_b,
            robot_id="B",
        )
    )

    hand_input_a = CachedHandInput(
        input_id="LEFT_HAND",
        target_getter=(
            get_latest_left_hand_target
        ),
        mode_getter=(
            get_latest_left_hand_mode
        ),
        mode_resetter=(
            reset_left_hand_mode_cache
        ),
    )

    hand_input_b = CachedHandInput(
        input_id="RIGHT_HAND",
        target_getter=(
            get_latest_right_hand_target
        ),
        mode_getter=(
            get_latest_right_hand_mode
        ),
        mode_resetter=(
            reset_right_hand_mode_cache
        ),
    )

    state_machine_a = (
        _create_state_machine(
            robot_id="A",
            robot=robot_a,
            tray_registry=tray_registry,
            hand_input=hand_input_a,
            initial_joint_positions=(
                initial_joint_positions_a
            ),
            tracking_controller=(
                tracking_controller_a
            ),
            pick_place_controller=(
                pick_place_controller_a
            ),
            move_controller=(
                move_controller_a
            ),
        )
    )

    state_machine_b = (
        _create_state_machine(
            robot_id="B",
            robot=robot_b,
            tray_registry=tray_registry,
            hand_input=hand_input_b,
            initial_joint_positions=(
                initial_joint_positions_b
            ),
            tracking_controller=(
                tracking_controller_b
            ),
            pick_place_controller=(
                pick_place_controller_b
            ),
            move_controller=(
                move_controller_b
            ),
        )
    )

    robot_manager = RobotManager(
        tray_positions=TRAY_SPAWN_POSITIONS,
        shared_trays=tuple(
            TRAY_SPAWN_POSITIONS.keys()
        ),
        tool_state_manager=tool_state_manager,
    )

    robot_manager.register_robot(
        profile=RobotProfile.create(
            robot_id="A",
            reachable_trays=(
                ROBOT_A_SUPPORTED_TRAY_COMMANDS
            ),
            route_id="TRANSIT_A",
            transit_position=(
                robot_a_transit_position
            ),
            transit_orientation=(
                TRACKING_TOOL_ORIENTATION
            ),
            tracking_joint1_delta_rad=np.deg2rad(
                ROBOT_A_TRACKING_JOINT1_DELTA_DEG
            ),
            selection_position=(
                robot_a_base_position
            ),
        ),
        state_machine=state_machine_a,
    )

    robot_manager.register_robot(
        profile=RobotProfile.create(
            robot_id="B",
            reachable_trays=(
                ROBOT_B_SUPPORTED_TRAY_COMMANDS
            ),
            route_id="TRANSIT_B",
            transit_position=(
                robot_b_transit_position
            ),
            transit_orientation=(
                TRACKING_TOOL_ORIENTATION
            ),
            tracking_joint1_delta_rad=np.deg2rad(
                ROBOT_B_TRACKING_JOINT1_DELTA_DEG
            ),
            selection_position=(
                robot_b_base_position
            ),
        ),
        state_machine=state_machine_b,
    )

    setup_m0609_ros_bridge(
        manager=robot_manager,
        simulation_app=simulation_app,
    )

    right_hand_marker = (
        HandModelVisualizer(
            world=world,
            simulation_app=simulation_app,
            coordinate_getter=(
                get_latest_right_hand_raw
            ),
            orientation_getter=(
                get_latest_right_hand_orient
            ),
            usda_path=HAND_USDA_PATH,
            ref_prim_path=HAND_RIGHT_REF_PRIM,
            model_prim_path=(
                HAND_RIGHT_MODEL_PRIM
            ),
            orient_offset_deg=(
                HAND_RIGHT_ORIENT_OFFSET_DEG
            ),
            pivot_offset=(
                HAND_RIGHT_PIVOT_OFFSET
            ),
            stabilize_roll=HAND_STABILIZE_ROLL,
            roll_axis=HAND_ROLL_AXIS,
            roll_sign=HAND_RIGHT_ROLL_SIGN,
            initial_position=[
                0.0,
                0.25,
                1.0,
            ],
            label="RIGHT HAND",
        )
    )

    left_hand_marker = (
        HandModelVisualizer(
            world=world,
            simulation_app=simulation_app,
            coordinate_getter=(
                get_latest_left_hand_raw
            ),
            orientation_getter=(
                get_latest_left_hand_orient
            ),
            usda_path=HAND_USDA_PATH,
            ref_prim_path=HAND_LEFT_REF_PRIM,
            model_prim_path=(
                HAND_LEFT_MODEL_PRIM
            ),
            orient_offset_deg=(
                HAND_LEFT_ORIENT_OFFSET_DEG
            ),
            pivot_offset=(
                HAND_LEFT_PIVOT_OFFSET
            ),
            stabilize_roll=HAND_STABILIZE_ROLL,
            roll_axis=HAND_ROLL_AXIS,
            roll_sign=HAND_LEFT_ROLL_SIGN,
            initial_position=[
                0.0,
                -0.25,
                1.0,
            ],
            label="LEFT HAND",
        )
    )

    for _ in range(
        INITIAL_SETTLING_FRAMES
    ):
        world.step(
            render=True,
        )

    print(
        "\n[M0609 Robot A/B 준비 완료]"
    )
    print(
        "- Robot A/B: trays 0~5 공용 접근"
    )
    print(
        "- Robot 선택: 트레이까지 가까운 로봇"
    )
    print(
        "- 거리 동률: Robot B 우선"
    )
    print(
        "- Tool layout: random initial placement"
    )
    print(
        "- Tool state: TRACKING=held, PLACE=on tray"
    )
    print(
        "- External tool detection API prepared"
    )
    print(
        f"- Initial tools: "
        f"{tool_state_manager.get_tray_tool_snapshot()}"
    )
    print(
        "- Tray zone: 신규 PICK/PLACE 구역 LOCK, 큐 없음"
    )
    print(
        "- Robot A: upper transit -> joint1 +180deg -> TRACKING"
    )
    print(
        "- Robot B: upper transit -> joint1 -180deg -> TRACKING"
    )
    print()

    was_playing = False

    _start_stdin_listener()
    print(
        "- 터미널에 'randomize' 입력 시 도구 위치 재랜덤 "
        "(로봇 이동 중이 아닐 때만 적용)",
        flush=True,
    )

    all_tool_indices = list(range(len(TOOL_NAMES)))

    def _try_apply_randomize() -> None:
        if not _RANDOMIZE_REQUEST.is_set():
            return

        # 로봇 이동/도구운반 중이면 보류했다가 안전해지면 적용한다.
        if not robot_manager.is_safe_to_randomize():
            return

        _RANDOMIZE_REQUEST.clear()

        result = robot_manager.randomize_partial_layout(
            min_count=4,
            max_count=6,
        )

        if result is None:
            return

        present_layout = {
            tray_id: tool_index_by_name[tool_name]
            for tray_id, tool_name in result["layout"].items()
        }

        tray_registry.apply_random_tool_layout(
            tray_tool_indices=present_layout,
            tray_positions=TRAY_SPAWN_POSITIONS,
            all_tool_indices=all_tool_indices,
            tool_place_height=TOOL_DROP_HEIGHT,
        )

        print(
            "[randomize] 적용 완료 "
            f"present={result['present']}, "
            f"missing={result['missing']}",
            flush=True,
        )

    while simulation_app.is_running():
        world.step(
            render=True,
        )

        time.sleep(0.01)

        is_playing = world.is_playing()

        if (
            is_playing
            and not was_playing
        ):
            world.reset()

            initialize_robot(
                robot_a,
                world,
            )

            initialize_robot(
                robot_b,
                world,
            )

            tray_registry.reset_to_spawn()

            tracking_controller_a.reset()
            tracking_controller_b.reset()

            # 로봇/오퍼레이션 상태만 리셋하고, 도구는 부분 랜덤(4~6개)으로
            # 배치한다. 미등장 도구는 숨겨서 missing 으로 만든다.
            robot_manager.reset(randomize_tools=False)

            partial_result = (
                robot_manager.randomize_partial_layout(
                    min_count=4,
                    max_count=6,
                )
            )

            if partial_result is None:
                raise RuntimeError(
                    "Tool random layout reset failed"
                )

            present_layout = {
                tray_id: tool_index_by_name[tool_name]
                for tray_id, tool_name
                in partial_result["layout"].items()
            }

            tray_registry.apply_random_tool_layout(
                tray_tool_indices=present_layout,
                tray_positions=(
                    TRAY_SPAWN_POSITIONS
                ),
                all_tool_indices=all_tool_indices,
                tool_place_height=(
                    TOOL_DROP_HEIGHT
                ),
            )

            print(
                "[main Play] 도구 부분 랜덤 배치 "
                f"present={partial_result['present']}, "
                f"missing={partial_result['missing']}",
                flush=True,
            )

            right_hand_marker.reset()
            left_hand_marker.reset()

            print(
                "[main] Robot A/B 및 도구 랜덤 배치 "
                "Play 재초기화 완료",
                flush=True,
            )

        if is_playing:
            right_hand_marker.update()
            left_hand_marker.update()
            robot_manager.step()
            _try_apply_randomize()

        was_playing = is_playing


if __name__ == "__main__":
    import sys
    import traceback
    from datetime import datetime
    from pathlib import Path

    try:
        main()

    except KeyboardInterrupt:
        print(
            "\n[main] 사용자 중단으로 종료합니다.",
            flush=True,
        )
        simulation_app.close()

    except BaseException:
        timestamp = datetime.now().strftime(
            "%Y%m%d_%H%M%S"
        )

        crash_log_path = Path(
            f"isaac_sim_crash_{timestamp}.log"
        ).resolve()

        traceback_text = traceback.format_exc()

        print(
            "\n"
            "========================================\n"
            "[FATAL] Isaac Sim 실행 중 예외 발생\n"
            "========================================",
            file=sys.stderr,
            flush=True,
        )

        print(
            traceback_text,
            file=sys.stderr,
            flush=True,
        )

        try:
            crash_log_path.write_text(
                traceback_text,
                encoding="utf-8",
            )

            print(
                "[FATAL] 오류 로그 저장 위치: "
                f"{crash_log_path}",
                file=sys.stderr,
                flush=True,
            )

        except Exception as log_error:
            print(
                "[FATAL] 오류 로그 저장 실패: "
                f"{log_error}",
                file=sys.stderr,
                flush=True,
            )

        print(
            "[FATAL] 오류 확인을 위해 "
            "Isaac Sim 창을 유지합니다.\n"
            "터미널에서 위 traceback을 확인하세요.\n"
            "창을 닫으면 프로세스가 종료됩니다.",
            file=sys.stderr,
            flush=True,
        )

        # 예외가 발생해도 즉시 simulation_app.close()를 호출하지 않는다.
        # 사용자가 창을 직접 닫을 때까지 GUI와 터미널을 유지한다.
        while simulation_app.is_running():
            try:
                simulation_app.update()

            except BaseException:
                # 이미 치명적 오류가 발생한 뒤이므로,
                # 유지 루프에서 추가 오류가 나도 원래 traceback을 보존한다.
                break

        simulation_app.close()

    else:
        simulation_app.close()
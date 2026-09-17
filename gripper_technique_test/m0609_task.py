# m0609_task.py

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import omni.usd
from pxr import Usd, UsdPhysics

from isaacsim.core.api.tasks import BaseTask
from isaacsim.robot.manipulators.manipulators import (
    SingleManipulator,
)

from dual_surface_gripper_adapter import (
    DualSurfaceGripperAdapter,
)
from m0609_config import (
    DRIVE_DAMPING,
    DRIVE_MAX_FORCE,
    DRIVE_STIFFNESS,
    EE_LINK_NAME,
    ROBOT_A_PRIM_PATH,
    ROBOT_A_SCENE_NAME,
    ROBOT_A_SURFACE_GRIPPER_PATHS,
    ROBOT_B_PRIM_PATH,
    ROBOT_B_SCENE_NAME,
    ROBOT_B_SURFACE_GRIPPER_PATHS,
    SURFACE_GRIPPER_WRITE_STATUS_TO_USD,
)


@dataclass(frozen=True)
class RobotRegistration:
    robot_id: str
    prim_path: str
    scene_name: str
    surface_gripper_paths: Sequence[str]


ROBOT_REGISTRATIONS = (
    RobotRegistration(
        robot_id="A",
        prim_path=ROBOT_A_PRIM_PATH,
        scene_name=ROBOT_A_SCENE_NAME,
        surface_gripper_paths=(
            ROBOT_A_SURFACE_GRIPPER_PATHS
        ),
    ),
    RobotRegistration(
        robot_id="B",
        prim_path=ROBOT_B_PRIM_PATH,
        scene_name=ROBOT_B_SCENE_NAME,
        surface_gripper_paths=(
            ROBOT_B_SURFACE_GRIPPER_PATHS
        ),
    ),
)


def find_prim_path_by_name(
    root_path: str,
    name: str,
):
    """root_path 아래에서 이름이 일치하는 Prim 경로를 찾는다."""

    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(
        root_path
    )

    if not root_prim.IsValid():
        return None

    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == name:
            return str(prim.GetPath())

    return None


def initialize_robot(
    robot,
    world,
) -> None:
    """SingleManipulator와 Surface Gripper를 초기화한다."""

    print(
        f"[task] {robot.name}.initialize 시작"
    )
    robot.initialize()
    print(
        f"[task] {robot.name}.initialize 완료"
    )

    print(
        f"[task] {robot.name}.gripper.initialize 시작"
    )
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=(
            robot.apply_action
        ),
        get_joint_positions_func=(
            robot.get_joint_positions
        ),
        set_joint_positions_func=(
            robot.set_joint_positions
        ),
        dof_names=robot.dof_names,
    )
    print(
        f"[task] {robot.name}.gripper.initialize 완료"
    )

    robot.gripper.open()


class M0609BasicTask(BaseTask):
    """
    이미 열린 full_scene.usda 안의 Robot A와 Robot B를
    검색하고 Isaac Sim Scene 객체로 등록한다.

    이 Task에서는 full_scene.usda를 다시 Reference하지 않는다.
    """

    def __init__(
        self,
        name: str = "m0609_basic_task",
    ) -> None:
        super().__init__(
            name=name,
            offset=None,
        )

        self._robots = {}

    def set_up_scene(self, scene) -> None:
        super().set_up_scene(scene)

        stage = omni.usd.get_context().get_stage()

        for registration in ROBOT_REGISTRATIONS:
            self._register_one_robot(
                scene=scene,
                stage=stage,
                registration=registration,
            )

        print(
            "\n[완료] full_scene의 Robot A/B 등록 완료\n",
            flush=True,
        )

    def _register_one_robot(
        self,
        *,
        scene,
        stage,
        registration: RobotRegistration,
    ) -> None:
        robot_root = stage.GetPrimAtPath(
            registration.prim_path
        )

        if not robot_root.IsValid():
            top_level_paths = []

            world_prim = stage.GetPrimAtPath(
                "/World"
            )

            if world_prim.IsValid():
                top_level_paths = [
                    str(child.GetPath())
                    for child
                    in world_prim.GetChildren()
                ]

            raise RuntimeError(
                "full_scene.usda에서 로봇 Prim을 "
                "찾지 못했습니다.\n"
                f"Robot: {registration.robot_id}\n"
                f"기대 경로: {registration.prim_path}\n"
                f"/World 하위 Prim: {top_level_paths}"
            )

        ee_path = find_prim_path_by_name(
            registration.prim_path,
            EE_LINK_NAME,
        )

        if ee_path is None:
            raise RuntimeError(
                f"{registration.prim_path} 아래에서 "
                f"'{EE_LINK_NAME}'을 찾지 못했습니다."
            )

        missing_grippers = [
            path
            for path
            in registration.surface_gripper_paths
            if not stage.GetPrimAtPath(path).IsValid()
        ]

        if missing_grippers:
            raise RuntimeError(
                f"Robot {registration.robot_id} "
                "SurfaceGripper Prim을 찾지 못했습니다:\n- "
                + "\n- ".join(missing_grippers)
            )

        drive_count = 0

        for prim in Usd.PrimRange(robot_root):
            for drive_type in (
                "angular",
                "linear",
            ):
                drive = UsdPhysics.DriveAPI.Get(
                    prim,
                    drive_type,
                )

                if not drive:
                    continue

                drive.GetStiffnessAttr().Set(
                    DRIVE_STIFFNESS
                )
                drive.GetDampingAttr().Set(
                    DRIVE_DAMPING
                )
                drive.GetMaxForceAttr().Set(
                    DRIVE_MAX_FORCE
                )
                drive_count += 1

        gripper = DualSurfaceGripperAdapter(
            end_effector_prim_path=ee_path,
            surface_gripper_prim_paths=(
                registration.surface_gripper_paths
            ),
            write_status_to_usd=(
                SURFACE_GRIPPER_WRITE_STATUS_TO_USD
            ),
        )

        robot = scene.add(
            SingleManipulator(
                prim_path=registration.prim_path,
                name=registration.scene_name,
                end_effector_prim_path=ee_path,
                gripper=gripper,
            )
        )

        self._robots[
            registration.robot_id
        ] = robot

        print(
            f"[task] Robot {registration.robot_id} 등록 완료: "
            f"name={registration.scene_name}, "
            f"prim={registration.prim_path}, "
            f"ee={ee_path}, "
            f"drives={drive_count}",
            flush=True,
        )

    def post_reset(self) -> None:
        for robot in self._robots.values():
            robot.gripper.post_reset()
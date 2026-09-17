"""
M0609 cuRobo 컨트롤러 (cuRobo v0.7.8 = v1 API, warp 1.8.x)
RMPFlowController 대체. self-collision 완전.
좌표: 로봇 base가 Z축 base_yaw_deg 회전. 위치만 역회전 보정.
방향(quaternion)은 보정 안 함 (보정하면 self-collision 깨짐).
"""

from pathlib import Path

import numpy as np
import torch

from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState, RobotConfig
from curobo.util_file import load_yaml
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig
from curobo.rollout.rollout_base import Goal


JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


def _empty_world():
    return {
        "cuboid": {
            "dummy_floor": {
                "dims": [0.01, 0.01, 0.01],
                "pose": [0.0, 0.0, -10.0, 1, 0, 0, 0.0],
            },
        },
    }


class M0609CuroboController:
    def __init__(
        self,
        robot_config_path=None,
        base_position=(0.0, 0.0, 0.0),
        use_mpc=True,
        base_yaw_deg=90.0,
    ):
        self.base_pos = np.array(base_position, dtype=float)
        self.base_yaw_deg = base_yaw_deg
        self.tensor_args = TensorDeviceType()

        # 이식성: 설정 yml 과 URDF/asset 을 프로젝트 내부 상대경로로 해석.
        base_dir = Path(__file__).resolve().parent

        if robot_config_path is None:
            robot_config_path = str(base_dir / "m0609_v1.yml")

        cfg_dict = load_yaml(robot_config_path)["robot_cfg"]

        # yml 안의 절대경로 대신, 번들된 doosan-robot2 를 런타임에 지정.
        doosan_dir = base_dir / "doosan-robot2"
        cfg_dict["kinematics"]["urdf_path"] = str(
            doosan_dir / "urdf" / "m0609_isaac_sim.urdf"
        )
        cfg_dict["kinematics"]["asset_root_path"] = str(doosan_dir)

        world = _empty_world()

        print("[cuRobo] MotionGen 초기화 중...")
        mg_cfg = MotionGenConfig.load_from_robot_config(cfg_dict, world, interpolation_dt=0.01)
        self.motion_gen = MotionGen(mg_cfg)
        self.motion_gen.warmup()
        self.joint_names = JOINT_NAMES
        print("[cuRobo] MotionGen 준비")

        self.mpc = None
        if use_mpc:
            print("[cuRobo] MPC 초기화 중...")
            robot_cfg = RobotConfig.from_dict(cfg_dict, self.tensor_args)
            mpc_cfg = MpcSolverConfig.load_from_robot_config(
                robot_cfg, world, store_rollouts=False, step_dt=0.03
            )
            self.mpc = MpcSolver(mpc_cfg)
            self._mpc_goal_buffer = None
            self._mpc_state = None
            print("[cuRobo] MPC 준비")

    def _world_to_base(self, world_pos):
        rel = np.asarray(world_pos, dtype=float) - self.base_pos
        yaw = np.radians(self.base_yaw_deg)
        c, s = np.cos(-yaw), np.sin(-yaw)
        return np.array([c * rel[0] - s * rel[1], s * rel[0] + c * rel[1], rel[2]])

    def _make_goal_pose(self, target_pos_world, target_quat):
        tb = self._world_to_base(target_pos_world)
        q = np.asarray(target_quat, dtype=float)
        return Pose.from_list([
            float(tb[0]), float(tb[1]), float(tb[2]),
            float(q[0]), float(q[1]), float(q[2]), float(q[3]),
        ])

    def fk(self, joint_positions):
        q = JointState.from_position(
            torch.tensor(joint_positions, device="cuda", dtype=torch.float32).view(1, -1),
            joint_names=self.joint_names,
        )
        state = self.motion_gen.rollout_fn.compute_kinematics(q)
        pos = state.ee_pos_seq.flatten().cpu().numpy()
        quat = state.ee_quat_seq.flatten().cpu().numpy()
        return pos, quat

    def plan_to(self, target_pos_world, target_quat, current_joints):
        goal_pose = self._make_goal_pose(target_pos_world, target_quat)
        start = JointState.from_position(
            torch.tensor(current_joints, device="cuda", dtype=torch.float32).view(1, -1),
            joint_names=self.joint_names,
        )
        result = self.motion_gen.plan_single(start, goal_pose, MotionGenPlanConfig(max_attempts=3))
        if result.success.item():
            traj = result.get_interpolated_plan()
            return traj.position.cpu().numpy()
        return None

    def mpc_set_goal(self, target_pos_world, target_quat, current_joints):
        if self.mpc is None:
            raise RuntimeError("MPC 비활성")
        goal_pose = self._make_goal_pose(target_pos_world, target_quat)
        cur = JointState.from_position(
            torch.tensor(current_joints, device="cuda", dtype=torch.float32).view(1, -1),
            joint_names=self.mpc.joint_names,
        )
        if self._mpc_goal_buffer is None:
            goal = Goal(current_state=cur, goal_state=cur, goal_pose=goal_pose)
            self._mpc_goal_buffer = self.mpc.setup_solve_single(goal, 1)
            self.mpc.update_goal(self._mpc_goal_buffer)
            self._mpc_state = cur.clone()
        else:
            self._mpc_goal_buffer.goal_pose.copy_(goal_pose)
            self.mpc.update_goal(self._mpc_goal_buffer)

    def mpc_step(self, current_joints):
        if self.mpc is None:
            raise RuntimeError("MPC 비활성")
        cur = JointState.from_position(
            torch.tensor(current_joints, device="cuda", dtype=torch.float32).view(1, -1),
            joint_names=self.mpc.joint_names,
        )
        result = self.mpc.step(cur, 1)
        return result.action.position.flatten().cpu().numpy()
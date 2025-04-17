from typing import Any, Dict

import numpy as np
import sapien
import torch

from snake import Snake
from mani_skill.envs.sapien_env import BaseEnv
from mani_skill.sensors.camera import CameraConfig
from mani_skill.utils import sapien_utils
from mani_skill.utils.building.ground import build_ground
from mani_skill.utils.registration import register_env
from mani_skill.utils.structs.types import GPUMemoryConfig, SimConfig
from mani_skill.utils.building import actors
from mani_skill.utils.structs.pose import Pose


@register_env("SnakeReach-v1", max_episode_steps=50)
class SnakeReachEnv(BaseEnv):
    SUPPORTED_REWARD_MODES = ["dense"]
    agent: Snake
    goal_thresh = 0.025

    def __init__(self, *args, robot_uids="snake", **kwargs):
        super().__init__(*args, robot_uids=robot_uids, **kwargs)

    @property
    def _default_sensor_configs(self):
        pose = sapien_utils.look_at([1.25, -1.25, 1.5], [0.0, 0.0, 0.2])
        return [CameraConfig("base_camera", pose, 128, 128, np.pi / 2, 0.01, 100)]

    @property
    def _default_human_render_camera_configs(self):
        pose = sapien_utils.look_at([2.5, -2.5, 3], [0.0, 0.0, 0.2])
        return CameraConfig("render_camera", pose, 2048, 2048, 1, 0.01, 100)

    def _load_agent(self, options: dict):
        super()._load_agent(options, sapien.Pose())

    def _load_scene(self, options: dict):
        self.ground = build_ground(self.scene)
        self.ground.set_collision_group_bit(group=2, bit_idx=30, bit=1)

        self.goal_site = actors.build_sphere(
            self.scene,
            radius=0.05,
            color=[0, 1, 0, 1],
            name="goal_site",
            body_type="kinematic",
            add_collision=False,
            initial_pose=sapien.Pose(),
        )
        self._hidden_objects.append(self.goal_site)

    def _initialize_episode(self, env_idx: torch.Tensor, options: dict):
        with torch.device(self.device):

            b = len(env_idx)
            goal_xyz = torch.rand((b,3)) * 1.3

            
            self.goal_site.set_pose(Pose.create_from_pq(goal_xyz))


    def evaluate(self):
        is_ee_at_goal = (
            torch.linalg.norm(self.goal_site.pose.p - self.agent.ee.pose.p, axis=1)
            <= self.goal_thresh
        )
        return {"success":is_ee_at_goal}

    def _get_obs_extra(self, info: Dict):
        obs = dict(
            ee_pose = self.agent.ee.pose.raw_pose,
            goal_pos = self.goal_site.pose.p
        )
        if "state" in self.obs_mode:
            obs.update(
                ee_to_goal_pos = self.goal_site.pose.p - self.agent.ee.pose.p
            )
        return obs


    def compute_dense_reward(self, obs: Any, action: torch.Tensor, info: Dict):
        tcp_to_obj_dist = torch.linalg.norm(
            self.goal_site.pose.p - self.agent.ee.pose.p, axis=1
        )
        reaching_reward = 4 * (1 - torch.tanh(5 * tcp_to_obj_dist))
        reward = reaching_reward

        reward[info["success"]] = 5
        return reward

    def compute_normalized_dense_reward(
        self, obs: Any, action: torch.Tensor, info: Dict
    ):
        return self.compute_dense_reward(obs=obs, action=action, info=info) / 5
from mani_skill.utils import common, sapien_utils
import numpy as np
from mani_skill.agents.base_agent import BaseAgent, Keyframe
from mani_skill.agents.controllers import *
from mani_skill.agents.registration import register_agent

@register_agent()
class Snake(BaseAgent):

    joints = 0
    uid = "snake"
    urdf_path = f"./robot_arm.urdf"

    arm_joint_names = [
        f"joint{i+1}" for i in range(10+1)
    ]

    arm_stiffness = 1e3
    arm_damping = 1e2
    arm_force_limit = 100

    gripper_stiffness = 1e3
    gripper_damping = 1e2
    gripper_force_limit = 100

    @property
    def _controller_configs(self):
        arm_pd_joint_pos = PDJointPosControllerConfig(
            self.arm_joint_names,
            lower=None,
            upper=None,
            stiffness=self.arm_stiffness,
            damping=self.arm_damping,
            force_limit=self.arm_force_limit,
            normalize_action=False,
        )

        arm_pd_joint_delta_pos = PDJointPosControllerConfig(
            self.arm_joint_names,
            lower=-0.1,
            upper=0.1,
            stiffness=self.arm_stiffness,
            damping=self.arm_damping,
            force_limit=self.arm_force_limit,
            use_delta=True,
        )


        controller_configs = {
            "pd_joint_pos" :{
                "arm": arm_pd_joint_pos
            },
            "pd_joint_delta_pos" :{
                "arm": arm_pd_joint_delta_pos
            }
        }


        return deepcopy_dict(controller_configs)

    def _after_init(self):
        self.ee = sapien_utils.get_obj_by_name(self.robot.get_links(), "end_effector")
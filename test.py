import torch
import snake
import mani_skill.envs
import gymnasium as gym
from PIL import Image
import torch

env = gym.make("Empty-v1", robot_uids="snake", render_mode="rgb_array", control_mode ="pd_joint_pos")
env.reset()
env.step(env.action_space.sample())

Image.fromarray(env.render()[0].detach().cpu().numpy()).save("test.jpg")
import torch
import snake
import mani_skill.envs
from SnakeReachEnv import SnakeReachEnv
import gymnasium as gym
from PIL import Image
import torch

env = gym.make("SnakeReach-v1",render_mode="rgb_array")
obs,_  = env.reset()
out = env.step(env.action_space.sample())

Image.fromarray(env.render()[0].detach().cpu().numpy()).save("test.jpg")
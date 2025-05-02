# TDMPC High dim

To edit DOF go set the joints variable in the Snake/snake.py file. Joints need to be set to n+1 where n comes from the corresponding `robot_arm_{n}.urdf`.

## Deps

Requirements.txt is all of the tdmpc2/ppo deps.

```
pip install -r requirements.txt
```

Run in the same same directory as setup.py. This allows the Snake robot to be accesible for ppo/tdmpc.

```
pip install -e .
```

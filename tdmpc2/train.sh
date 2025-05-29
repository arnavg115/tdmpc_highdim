python train.py model_size=5 steps=1_000_000 buffer_size=500_000 exp_name="tdmpc2" \
        env_id="PlanarReach-v1" num_envs=32 control_mode="pd_joint_delta_pos" env_type=gpu obs=state
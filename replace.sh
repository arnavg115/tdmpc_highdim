


# Check for argument
if [ -z "$1" ]; then
  echo "Usage: $0 <new_integer_value>"
  exit 1
fi

new_value="$1"
use_wandb=true

# Replace the integer value of the variable `joints`
sed -i "s/joints = .*/joints = $new_value/" Snake/planar.py

pip install -r requirements.txt
pip install -e .

cd tdmpc2

python train.py model_size=5 steps=1_500_000 buffer_size=500_000 exp_name="tdmpc2" \
        env_id="PlanarReach-v1" num_envs=32 control_mode="pd_joint_delta_pos" env_type=gpu obs=state \
        wandb=$use_wandb wandb_name=tdmpc2-PlanarReach-v1-state-joint-$new_value

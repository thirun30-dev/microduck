# source this: ROS 2 Jazzy + this workspace + the venv's onnxruntime / huggingface_hub
source /opt/ros/jazzy/setup.bash
source "$HOME/microduck_ws/install/setup.bash"
export PYTHONPATH="$HOME/microduck_ws/.venv/lib/python3.12/site-packages:$PYTHONPATH"

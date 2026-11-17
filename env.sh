# source this (bash): ROS 2 Jazzy + this workspace + the venv's onnxruntime / huggingface_hub
# Works from any clone location, not only ~/microduck_ws.
MICRODUCK_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export MICRODUCK_WS

source /opt/ros/jazzy/setup.bash
if [ -f "$MICRODUCK_WS/install/setup.bash" ]; then
  source "$MICRODUCK_WS/install/setup.bash"
else
  echo "env.sh: $MICRODUCK_WS/install not found; run colcon build (or bootstrap.sh) first" >&2
fi
for _sp in "$MICRODUCK_WS"/.venv/lib/python3*/site-packages; do
  [ -d "$_sp" ] && export PYTHONPATH="$_sp${PYTHONPATH:+:$PYTHONPATH}"
done
unset _sp

#!/usr/bin/env bash
# One-time setup after cloning: upstream sources, Python venv, build.
#   git clone --recurse-submodules <this repo> ~/microduck_ws && ~/microduck_ws/bootstrap.sh
set -e
cd "$(dirname "$0")"
git submodule update --init
# Upstream Rust / mjlab sources are reference only; keep colcon away from them.
touch src/microduck/COLCON_IGNORE src/microduck_rl/COLCON_IGNORE
[ -d .venv ] || python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements.txt
source /opt/ros/jazzy/setup.bash
colcon build
echo "done: source $(pwd)/env.sh"

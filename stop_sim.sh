#!/usr/bin/env bash
# Stop everything sim.launch.py started. Only matches the sim's own executables (never a
# shell whose command line merely mentions them, which used to kill the caller).
pids() {
  ps -eo pid=,comm=,args= | awk -v me=$$ '
    $1 != me && $2 !~ /^(bash|sh|zsh|timeout|awk|ps)$/ &&
    /policy_node|parameter_bridge|rviz2|robot_state_publisher|gz sim|ruby .*gz|ros2 launch microduck_gazebo/ {print $1}'
}
P=$(pids); [ -n "$P" ] && kill -INT $P 2>/dev/null
for _ in 1 2 3 4; do [ -z "$(pids)" ] && exit 0; sleep 0.5; done
P=$(pids); [ -n "$P" ] && kill -9 $P 2>/dev/null
exit 0

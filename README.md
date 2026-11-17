# microduck_ws

A ROS 2 Jazzy + Gazebo Harmonic simulation of Pollen Robotics'
[Microduck](https://github.com/pollen-robotics/microduck). It is driven by the RL policies
published on the Hugging Face Hub (`pollen-robotics/microduck-policies`, revision `v10`),
and shown in RViz.

## Packages

| package | what it does |
|---|---|
| `microduck_description` | URDF and meshes, converted from `microduck_rl`'s `robot_walk.xml` by `scripts/mjcf_to_urdf.py`. `display.launch.py` shows the model in RViz with joint sliders. |
| `microduck_gazebo` | `flat.sdf` world, ROS↔Gazebo bridge, `sim.launch.py`, and the **MicroduckServo** system plugin (see below). |
| `microduck_policy` | `policy_node` runs an ONNX policy fetched from the Hub at 50 Hz (same obs/action layout as `robotd`). `teleop` is a keyboard controller. `fetch_policies` lists and caches the policy set. |
| `microduck`, `microduck_rl` | Upstream sources, used for reference only (`COLCON_IGNORE`). |

## Run

```bash
source ~/microduck_ws/env.sh
ros2 launch microduck_gazebo sim.launch.py            # Gazebo + RViz + velstand policy
ros2 run microduck_policy teleop                      # in a second terminal: drive it
~/microduck_ws/stop_sim.sh                            # stop everything
```

Launch arguments: `rviz:=false`, `policy:=false` (only hold the home pose),
`policy_file:=alpha_walking.onnx`, `hf_repo:=…`, `hf_revision:=…`.

At startup the duck hangs from an invisible anchor while its joints move from 0 to the home
pose. Then the policy node publishes `/microduck/detach` and the duck drops onto its feet.

Topics: `/cmd_vel` (Twist: `linear.x`, `linear.y`, `angular.z`), `/microduck/head`
(Float64MultiArray: neck_pitch, head_pitch, head_yaw, head_roll), `/joint_states`,
`/microduck/imu`, `/microduck/odom`, `/tf`.

Teleop keys: `w`/`x` forward/back, `a`/`d` turn, `q`/`e` sidestep, `space` stop,
`i`/`k`/`j`/`l`/`u`/`o` head, `h` centre head.

Rebuild: `cd ~/microduck_ws && colcon build && source env.sh`

## Why a custom servo plugin

The policies were trained in MuJoCo with XL330 servos that have **joint armature
0.0018 kg·m²**, the gearbox-reflected rotor inertia. That is about 100× the link-side
inertia of the head and ankle joints. SDF and Gazebo have no armature. With Gazebo's stock
`JointPositionController` the duck stood but fell about 2 s into walking. Removing armature
in MuJoCo reproduces the same fall. Adding the inertia to the links instead does not fix it,
because that also changes how the whole body rotates.

`MicroduckServo` gives each joint a virtual rotor with that inertia, driven by the trained
servo law (`kp=0.55`, ±0.96 N·m, damping 0.053). The rotor is coupled to the joint through a
stiff spring-damper (`ks=10`, `ds=0.002`), which stays stable at the 1 ms physics step. The
gains are written into the URDF by the converter.

Measured in Gazebo (velstand, v10): 0.3 m/s → 0.91 m in 9 s, and 0.5 m/s → 1.83 m in 9 s.
Tilt stays at or below 6°, with no falls.

## Known limits

- **Turning in place barely works.** At `angular.z = 1.0` the duck turns about 10° in
  8 s. MuJoCo gives the same result, so this comes from the policy, not from Gazebo.
  Turning while walking works, but in Gazebo it reaches about half the commanded yaw rate.
- Below about 0.2 m/s the policy stands still instead of stepping (teleop starts at 0.3).
- The duck drifts sideways a little on long straight walks. Nothing closes the heading loop.
- Only the walking and standing policies (`velstand`, `alpha_walking`, `alpha_stand`) fit
  this node's 61-value observation. The roller, kick and pick policies expect other inputs
  or hardware.

## Setup from a fresh clone

```bash
git clone --recurse-submodules https://github.com/thirun30-dev/microduck.git ~/microduck_ws
~/microduck_ws/bootstrap.sh        # submodules, venv (onnxruntime, huggingface_hub), colcon build
source ~/microduck_ws/env.sh
```

Requires ROS 2 Jazzy with `ros_gz` (Gazebo Harmonic). `src/microduck` and `src/microduck_rl`
are git submodules of the upstream Pollen Robotics repos.

## License

The code in this workspace is Apache-2.0. The meshes in `microduck_description/meshes` come
from Pollen Robotics' `microduck_rl` and are licensed **CC BY-NC-SA 4.0** (attribution, no
commercial use, share-alike). The upstream submodules keep their own licenses.

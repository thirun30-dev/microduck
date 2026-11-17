"""Gazebo Harmonic + RViz + (optionally) the Hugging Face policy.

    ros2 launch microduck_gazebo sim.launch.py               # sim + rviz + velstand policy
    ros2 launch microduck_gazebo sim.launch.py policy:=false # just hold the home pose
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc = get_package_share_directory("microduck_description")
    gzpkg = get_package_share_directory("microduck_gazebo")
    urdf = open(os.path.join(desc, "urdf", "microduck.urdf")).read()
    use_sim_time = {"use_sim_time": True}

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r {os.path.join(gzpkg, 'worlds', 'flat.sdf')}",
                          "on_exit_shutdown": "true"}.items())

    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-name", "microduck", "-topic", "/robot_description",
                            "-x", "0", "-y", "0", "-z", "0.131"])

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("policy", default_value="true"),
        DeclareLaunchArgument("policy_file", default_value="velstand.onnx"),
        DeclareLaunchArgument("hf_repo", default_value="pollen-robotics/microduck-policies"),
        DeclareLaunchArgument("hf_revision", default_value="v10"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH",
                               os.path.dirname(desc) + os.pathsep +
                               os.environ.get("GZ_SIM_RESOURCE_PATH", "")),
        # MicroduckServo plugin (also set by this package's env hook).
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH",
                               os.path.join(os.path.dirname(os.path.dirname(gzpkg)), "lib") +
                               os.pathsep + os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")),
        gz,
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}, use_sim_time]),
        spawn,
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": os.path.join(gzpkg, "config", "bridge.yaml")},
                         use_sim_time]),
        Node(package="rviz2", executable="rviz2", condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", os.path.join(desc, "rviz", "microduck.rviz")],
             parameters=[use_sim_time]),
        TimerAction(period=4.0, actions=[Node(
            package="microduck_policy", executable="policy_node", output="screen",
            condition=IfCondition(LaunchConfiguration("policy")),
            parameters=[use_sim_time, {
                "hf_repo": LaunchConfiguration("hf_repo"),
                "hf_revision": LaunchConfiguration("hf_revision"),
                "policy_file": LaunchConfiguration("policy_file")}])]),
    ])

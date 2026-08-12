"""
Launch file for the voice-controlled quadrotor assignment.

Starts:
  1. Gazebo Harmonic with the X3 quadrotor world.
  2. ros_gz_bridge, translating:
       - /X3/gazebo/command/twist  (ROS2 -> Gazebo)  geometry_msgs/Twist
       - /model/x3/odometry        (Gazebo -> ROS2)  nav_msgs/Odometry

Note: this launch file does NOT set the NVIDIA PRIME render-offload
environment variables. If your system needs them (hybrid graphics /
Optimus laptops), export them in your shell before running this launch
file, e.g.:
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    export __VK_LAYER_NV_optimus=NVIDIA_only
    ros2 launch quadrotor_control quadrotor_sim.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('quadrotor_control')
    world_path = os.path.join(pkg_share, 'worlds', 'quadrotor_world.sdf')

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', world_path],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='quadrotor_bridge',
        output='screen',
        arguments=[
            '/X3/gazebo/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/x3/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
    )

    return LaunchDescription([
        gz_sim,
        bridge,
    ])

import os
import shutil
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    tb4_gz_pkg = get_package_share_directory('turtlebot4_gz_bringup')
    questions_pkg = get_package_share_directory('questions')
    
    # --- THE FIX: DYNAMIC FILE COPY ---
    # Because Gazebo blindly looks for 'level3.sdf' in your current working directory,
    # we dynamically copy the installed file to your terminal's location right before launch.
    world_src = os.path.join(questions_pkg, 'worlds', 'level3.sdf')
    world_dest = os.path.join(os.getcwd(), 'level3.sdf')
    
    if os.path.exists(world_src):
        try:
            shutil.copy(world_src, world_dest)
        except Exception:
            pass # Ignore if there are permission issues or it already exists
    # ----------------------------------
    
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(questions_pkg, 'worlds')
    )
    
    turtlebot4_gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb4_gz_pkg, 'launch', 'turtlebot4_gz.launch.py')),
        launch_arguments={
            'world': 'level3',  
            'model': 'lite',
            'x': '0.0',
            'y': '0.0',
            'z': '0.0',
            'yaw': '0.0',
            'spawn_dock': 'false'
        }.items()
    )

    websocket_broadcaster_node = Node(
        package='questions',
        executable='websocket_broadcaster',
        name='websocket_broadcaster',
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        turtlebot4_gz_launch,
        websocket_broadcaster_node
    ])
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Get package paths
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    depth_follower_pkg = get_package_share_directory('depth_follower')

    # World and model paths
    warehouse_world = os.path.join(depth_follower_pkg, 'worlds', 'warehouse_turtlebot.world')
    cam_sdf = os.path.join(depth_follower_pkg, 'models', 'depth_cam', 'model.sdf')

    # Debug print (shows in terminal to verify path)
    print(f"\n>>> Loading world file: {warehouse_world}\n")

    # Path to TurtleBot3 Gazebo launcher
    tb3_world_launch = os.path.join(tb3_gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')

    return LaunchDescription([
        # Set model type
        SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger'),

        # Launch TurtleBot3 world, overriding the world path
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_world_launch),
            launch_arguments={
                'use_sim_time': 'true',
                'world': warehouse_world
            }.items(),
        ),

        # Spawn the depth camera model
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', cam_sdf,
                '-entity', 'depth_camera',
                '-x', '0',
                '-y', '0',
                '-z', '0.5'
            ],
            output='screen'
        ),

        LogInfo(msg='✅ Launching TurtleBot3 Burger with depth camera in custom warehouse world!')
    ])

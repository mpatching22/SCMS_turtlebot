import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('depth_follower')
    gazebo_ros_share = FindPackageShare('gazebo_ros')

    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'warehouse.world.sdf'])
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'burger_with_rgbd.urdf.xacro'])
    params_file = PathJoinSubstitution([pkg_share, 'config', 'follower_params.yaml'])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([gazebo_ros_share, 'launch', 'gzserver.launch.py'])
            ),
            launch_arguments={'world': world_file, 'verbose': 'true'}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([gazebo_ros_share, 'launch', 'gzclient.launch.py'])
            ),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_file])}],
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'turtlebot3', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0'],
            output='screen',
        ),

        Node(
            package='depth_follower',
            executable='depth_follower_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
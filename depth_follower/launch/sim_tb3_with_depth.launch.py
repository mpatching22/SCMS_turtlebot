from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Get paths to necessary packages
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    tb3_world_launch = os.path.join(tb3_gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')

    # Path to our custom depth camera model
    pkg_share = get_package_share_directory('depth_follower')
    cam_sdf = os.path.join(pkg_share, 'models', 'depth_cam', 'model.sdf')

    return LaunchDescription([
        # Ensure we use the Burger model
        SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger'),

        # Launch the default TurtleBot3 world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_world_launch),
        ),

        # Spawn the custom depth camera into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'depth_camera', '-file', cam_sdf, '-x', '0', '-y', '0', '-z', '0.3'],
            output='screen'
        ),
    ])

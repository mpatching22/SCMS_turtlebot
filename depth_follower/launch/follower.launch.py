from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('depth_follower')
    params = os.path.join(pkg, 'config', 'follower.params.yaml')

    node = Node(
        package='depth_follower',
        executable='depth_follower_node',
        name='depth_follower',
        parameters=[params],
        output='screen'
    )

    return LaunchDescription([node])

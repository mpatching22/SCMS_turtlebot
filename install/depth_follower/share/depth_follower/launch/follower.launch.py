from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depth_follower',
            executable='depth_follower',
            name='color_follower',
            output='screen',
            parameters=[{
                'color_topic': '/image_raw',
                'cmd_vel_topic': '/cmd_vel',
                'max_linear_mps': 0.2,
                'max_angular_rps': 0.8,
                'k_turn': 0.003,
                'min_bbox_area': 500
            }]
        )
    ])

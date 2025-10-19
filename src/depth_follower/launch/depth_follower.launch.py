import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('depth_follower')

    # --- Gazebo setup ---
    gazebo_cmd = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:{pkg_share}/models && '
            'gazebo --verbose /usr/share/gazebo-11/worlds/empty.world '
            '-s libgazebo_ros_init.so -s libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    # --- Spawn TurtleBot3 ---
    spawn_turtlebot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger',
            '-file', '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    # --- Spawn Person ---
    spawn_person = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'person_simple',
            '-file', os.path.join(pkg_share, 'models', 'person_simple', 'model.sdf'),
            '-x', '1.5', '-y', '0', '-z', '0'
        ],
        output='screen'
    )

    # --- Spawn Depth Camera ---
    spawn_depth_cam = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'depth_camera',
            '-file', os.path.join(pkg_share, 'models', 'depth_cam', 'model.sdf'),
            '-x', '0', '-y', '0', '-z', '0.15'
        ],
        output='screen'
    )

    # --- Left Shelf ---
    spawn_shelf_left = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'shelf_left',
            '-file', os.path.join(pkg_share, 'models', 'shelf_simple', 'model.sdf'),
            '-x', '0', '-y', '1.0', '-z', '0'
        ],
        output='screen'
    )

    # --- Right Shelf ---
    spawn_shelf_right = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'shelf_right',
            '-file', os.path.join(pkg_share, 'models', 'shelf_simple', 'model.sdf'),
            '-x', '0', '-y', '-1.0', '-z', '0'
        ],
        output='screen'
    )

    # --- Launch RViz2 ---
    rviz_config = os.path.join(pkg_share, 'launch', 'depth_follower.rviz')
    launch_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        gazebo_cmd,
        spawn_turtlebot,
        spawn_person,
        spawn_depth_cam,
        spawn_shelf_left,
        spawn_shelf_right,
        launch_rviz
    ])


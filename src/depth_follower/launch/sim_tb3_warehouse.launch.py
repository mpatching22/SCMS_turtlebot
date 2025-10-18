from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    world_path = PathJoinSubstitution([
        FindPackageShare('depth_follower'),
        'worlds',
        'warehouse_turtlebot.world'
    ])

    # Make sure Gazebo can see your models (source path; install is fine too)
    model_path = PathJoinSubstitution([
        FindPackageShare('depth_follower'), 'models'
    ])

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=model_path
    )

    # Start Gazebo server + client and load the ROS plugins on the server
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            world_path,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Spawn TurtleBot3 from the Gazebo model database (requires factory up)
    spawn_tb3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_tb3',
        output='screen',
        arguments=[
            '-entity', 'burger',
            '-database', 'turtlebot3_burger',
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ]
    )

    # Spawn your depth camera from file
    depth_cam_sdf = PathJoinSubstitution([
        FindPackageShare('depth_follower'), 'models', 'depth_cam', 'model.sdf'
    ])
    spawn_depth_cam = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_depth_cam',
        output='screen',
        arguments=[
            '-entity', 'depth_camera',
            '-file', depth_cam_sdf,
            '-x', '0.0', '-y', '0.0', '-z', '0.6'
        ]
    )

    # Make sure we only try spawning once gzserver is running
    spawn_after_gzserver = RegisterEventHandler(
        OnProcessStart(
            target_action=gzserver,
            on_start=[
                # small delay to let the /spawn_entity service come up
                TimerAction(period=2.0, actions=[spawn_tb3, spawn_depth_cam])
            ],
        )
    )

    return LaunchDescription([
        set_model_path,
        gzserver,
        gzclient,
        spawn_after_gzserver,
    ])

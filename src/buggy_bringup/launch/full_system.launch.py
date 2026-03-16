import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    bringup_dir     = get_package_share_directory('buggy_bringup')
    description_dir = get_package_share_directory('buggy_description')

    world_file = os.path.join(bringup_dir, 'worlds', 'srm_campus.world')
    urdf_file  = os.path.join(description_dir, 'urdf', 'buggy.urdf.xacro')

    # Process xacro → URDF
    robot_description = Command(['xacro ', urdf_file])

    return LaunchDescription([

        # ── Arguments ──
        DeclareLaunchArgument('world',    default_value='srm_campus'),
        DeclareLaunchArgument('headless', default_value='false'),

        # ── Gazebo ──
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file,
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # ── Robot state publisher ──
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # ── Spawn buggy in Gazebo ──
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_buggy',
            output='screen',
            arguments=[
                '-entity', 'srm_buggy',
                '-topic',  'robot_description',
                '-x', '-38', '-y', '0', '-z', '0.2'
            ]
        ),

        # ── Core nodes (delayed to let Gazebo start first) ──
        TimerAction(period=5.0, actions=[
            Node(
                package='buggy_brain',
                executable='path_planner',
                name='path_planner_node',
                output='screen'
            ),
        ]),

        TimerAction(period=6.0, actions=[
            Node(
                package='buggy_brain',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen'
            ),
        ]),

        TimerAction(period=7.0, actions=[
            Node(
                package='buggy_brain',
                executable='obstacle_detector',
                name='obstacle_detector',
                output='screen'
            ),
        ]),

        TimerAction(period=8.0, actions=[
            Node(
                package='buggy_brain',
                executable='state_machine',
                name='state_machine',
                output='screen'
            ),
        ]),

    ])

import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    bringup_dir     = get_package_share_directory('buggy_bringup')
    description_dir = get_package_share_directory('buggy_description')

    world_file = os.path.join(bringup_dir, 'worlds', 'srm_campus.world')
    urdf_file  = os.path.join(description_dir, 'urdf', 'buggy_urdf.xacro')

    robot_description = ParameterValue(
        xacro.process_file(urdf_file).toxml(),
        value_type=str
    )

    sim_time = {'use_sim_time': True}

    return LaunchDescription([

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
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': True}]
        ),

        # ── Spawn buggy ──
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_buggy',
            output='screen',
            arguments=[
                '-entity', 'srm_buggy',
                '-topic',  'robot_description',
                '-x', '-30',
                '-y', '0',
                '-z', '0.425',
                '-R', '0',
                '-P', '0',
                '-Y', '3.14159'
            ]
        ),

        # ── Path planner ──
        TimerAction(period=5.0, actions=[
            Node(
                package='buggy_brain',
                executable='path_planner',
                name='path_planner_node',
                output='screen',
                parameters=[sim_time]
            ),
        ]),

        # ── Waypoint follower ──
        TimerAction(period=6.0, actions=[
            Node(
                package='buggy_brain',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[sim_time]
            ),
        ]),

        # ── Obstacle detector ──
        TimerAction(period=7.0, actions=[
            Node(
                package='buggy_brain',
                executable='obstacle_detector',
                name='obstacle_detector',
                output='screen',
                parameters=[sim_time]
            ),
        ]),

        # ── State machine ──
        TimerAction(period=8.0, actions=[
            Node(
                package='buggy_brain',
                executable='state_machine',
                name='state_machine',
                output='screen',
                parameters=[sim_time]
            ),
        ]),

        # ── Crowd detector ──
        TimerAction(period=9.0, actions=[
            Node(
                package='buggy_brain',
                executable='crowd_detector',
                name='crowd_detector',
                output='screen',
                parameters=[sim_time]
            ),
        ]),

        # ── Ultrasonic monitor ──
        TimerAction(period=9.0, actions=[
            Node(
                package='buggy_brain',
                executable='ultrasonic_monitor',
                name='ultrasonic_monitor',
                output='screen',
                parameters=[sim_time]
            ),
        ]),

        # ── Demo visualizer ──
        TimerAction(period=10.0, actions=[
            Node(
                package='buggy_brain',
                executable='demo_visualizer',
                name='demo_visualizer',
                output='screen',
                parameters=[sim_time]
            ),
        ]),

    ])

#!/usr/bin/env python3
"""
full_system.launch.py  v2.0 — SRM Autonomous Buggy
────────────────────────────────────────────────────
Launches Gazebo + robot + state publishers.
Buggy spawns at BUGGY_HUB (0, 0) facing North (+Y toward SRM_IST).
(Brain nodes are disabled by default; see lines 84-99 to enable them.)
Team Alpha owns this file. §Day 5 integration point.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    bringup_pkg     = get_package_share_directory('buggy_bringup')
    description_pkg = get_package_share_directory('buggy_description')
    gazebo_ros_pkg  = get_package_share_directory('gazebo_ros')

    world_file        = os.path.join(bringup_pkg,     'worlds', 'srm_campus.world')
    xacro_file        = os.path.join(description_pkg, 'urdf',   'buggy_urdf.xacro')
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str
    )

    # ── Launch arguments ──────────────────────────────────────
    declare_gui         = DeclareLaunchArgument('gui',         default_value='true')
    declare_verbose     = DeclareLaunchArgument('verbose',     default_value='false')
    declare_entity_name = DeclareLaunchArgument('entity_name', default_value='srm_aquila_buggy')
    gui         = LaunchConfiguration('gui')
    verbose     = LaunchConfiguration('verbose')
    entity_name = LaunchConfiguration('entity_name')

    # ── 1. Gazebo world ───────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world':   world_file,
            'gui':     gui,
            'verbose': verbose,
        }.items(),
    )

    # ── 2. Robot description ──────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},          # §Known Pitfalls — clock sync
        ],
        output='screen',
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
    )

    # ── 3. Spawn at BUGGY_HUB (0,0) facing North
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', entity_name,
            '-x', '0.0',     # BUGGY_HUB X  §2.2
            '-y', '0.0',     # BUGGY_HUB Y  §2.2
            '-z', '0.425',   # wheel_radius + chassis clearance
            '-Y', '1.5708',  # 90° yaw → facing North toward SRM_IST
        ],
        output='screen',
    )

    # ── 4. Brain nodes — uncomment when Team Bravo adds them ──
    # brain_nodes = [
    #     Node(package='buggy_brain', executable='path_planner',
    #          parameters=[{'use_sim_time': True}], output='screen'),
    #     Node(package='buggy_brain', executable='waypoint_follower',
    #          parameters=[{'use_sim_time': True}], output='screen'),
    #     Node(package='buggy_brain', executable='obstacle_detector',
    #          parameters=[{'use_sim_time': True}], output='screen'),
    #     Node(package='buggy_brain', executable='state_machine',
    #          parameters=[{'use_sim_time': True}], output='screen'),
    #     Node(package='buggy_brain', executable='speed_controller',
    #          parameters=[{'use_sim_time': True}], output='screen'),
    #     # TODO(Team Bravo): add console_scripts before enabling:
    #     # ultrasonic_monitor, crowd_detector, demo_visualizer
    # ]

    return LaunchDescription([
        declare_gui,
        declare_verbose,
        declare_entity_name,
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        # *brain_nodes,   # ← Team Bravo: uncomment on Day 5
    ])
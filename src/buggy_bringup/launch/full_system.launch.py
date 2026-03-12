#!/usr/bin/env python3
"""
full_system.launch.py — SRM Autonomous Buggy
Team Alpha owns this file. Brain nodes are added by Team Bravo.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    bringup_pkg     = get_package_share_directory('buggy_bringup')
    description_pkg = get_package_share_directory('buggy_description')
    gazebo_ros_pkg  = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(bringup_pkg, 'worlds', 'srm_campus.world')
    xacro_file = os.path.join(description_pkg, 'urdf', 'buggy_urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # ── SECTION 1: Gazebo world ────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'gui': 'true', 'verbose': 'false'}.items(),
    )

    # ── SECTION 2: Robot description ──────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}, {'use_sim_time': True}],
        output='screen',
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
    )

    # ── SECTION 3: Spawn buggy (delayed 3s) ───────────────────────────
    spawn_entity = TimerAction(
        period=3.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'srm_aquila_buggy',
                '-x', '-20.0', '-y', '0.0', '-z', '0.6', '-Y', '0.0',
            ],
            output='screen',
        )]
    )

    # ── SECTION 4: BRAIN NODES — uncomment as Team Bravo adds them ────
    # brain_nodes = [
    #     Node(package='buggy_brain', executable='path_planner_node',   parameters=[{'use_sim_time': True}], output='screen'),
    #     Node(package='buggy_brain', executable='waypoint_follower',   parameters=[{'use_sim_time': True}], output='screen'),
    #     Node(package='buggy_brain', executable='obstacle_detector',   parameters=[{'use_sim_time': True}], output='screen'),
    #     Node(package='buggy_brain', executable='ultrasonic_monitor',  parameters=[{'use_sim_time': True}], output='screen'),
    #     Node(package='buggy_brain', executable='state_machine',       parameters=[{'use_sim_time': True}], output='screen'),
    #     Node(package='buggy_brain', executable='speed_controller',    parameters=[{'use_sim_time': True}], output='screen'),
    #     Node(package='buggy_brain', executable='crowd_detector',      parameters=[{'use_sim_time': True}], output='screen'),
    #     Node(package='buggy_brain', executable='demo_visualizer',     parameters=[{'use_sim_time': True}], output='screen'),
    # ]

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        # *brain_nodes,   # ← Team Bravo: uncomment this line on Day 5
    ])

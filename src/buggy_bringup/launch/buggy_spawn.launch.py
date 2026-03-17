#!/usr/bin/env python3
"""
buggy_spawn.launch.py  v3.0 — SRM Autonomous Buggy
────────────────────────────────────────────────────
Spawns buggy inside buggy shelter bay
Facing East (+X), looking out of the bay
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    declare_entity_name = DeclareLaunchArgument(
        'entity_name', default_value='srm_aquila_buggy',
        description='Name of the robot entity spawned in Gazebo'
    )
    entity_name = LaunchConfiguration('entity_name')

    description_pkg   = get_package_share_directory('buggy_description')
    xacro_file        = os.path.join(description_pkg, 'urdf', 'buggy_urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ],
        output='screen',
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_srm_buggy',
        arguments=[
            '-topic', 'robot_description',
            '-entity', entity_name,
            '-x', '-13.0',   # inside buggy shelter bay, just off roundabout
            '-y', '0.0',
            '-z', '0.15',
            '-Y', '0.0',     # 0.0 yaw → facing East, looking out of the bay
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_entity_name,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
    ])
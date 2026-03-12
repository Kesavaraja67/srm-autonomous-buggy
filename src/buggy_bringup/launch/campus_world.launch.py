#!/usr/bin/env python3
"""
campus_world.launch.py
----------------------
Launches Gazebo Classic with the SRM campus world only.
Does NOT spawn the buggy — use buggy_spawn.launch.py for that,
or use full_system.launch.py which includes both.

Usage:
    ros2 launch buggy_bringup campus_world.launch.py
    ros2 launch buggy_bringup campus_world.launch.py gui:=false   ← headless
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Package paths ──────────────────────────────────────────────────
    bringup_pkg = get_package_share_directory('buggy_bringup')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(bringup_pkg, 'worlds', 'srm_campus.world')

    # ── Launch arguments ───────────────────────────────────────────────
    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to false to run Gazebo headless (no render window)'
    )

    declare_verbose = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Set to true for verbose Gazebo output'
    )

    gui     = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')

    # ── Gazebo server + client ─────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world':   world_file,
            'gui':     gui,
            'verbose': verbose,
            'pause':   'false',
        }.items(),
    )

    return LaunchDescription([
        declare_gui,
        declare_verbose,
        gazebo_launch,
    ])

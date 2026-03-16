#!/usr/bin/env python3
"""
campus_world.launch.py  v2.0 — SRM Autonomous Buggy
─────────────────────────────────────────────────────
Launches ONLY Gazebo with the SRM campus world.
No robot spawned here — use buggy_spawn.launch.py after.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    bringup_pkg    = get_package_share_directory('buggy_bringup')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    world_file     = os.path.join(bringup_pkg, 'worlds', 'srm_campus.world')

    declare_gui     = DeclareLaunchArgument('gui',     default_value='true')
    declare_verbose = DeclareLaunchArgument('verbose', default_value='false')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world':   world_file,
            'gui':     LaunchConfiguration('gui'),
            'verbose': LaunchConfiguration('verbose'),
        }.items(),
    )

    return LaunchDescription([
        declare_gui,
        declare_verbose,
        gazebo,
    ])
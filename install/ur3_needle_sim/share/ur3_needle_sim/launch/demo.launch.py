#!/usr/bin/env python3
import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'ur3_needle_sim'


def generate_launch_description():
    pkg = get_package_share_directory(PACKAGE)
    use_sim_time = LaunchConfiguration('use_sim_time')
    depth_m = LaunchConfiguration('depth_m')
    params = os.path.join(pkg, 'config', 'needle_params.yaml')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    insert_node = Node(
        package=PACKAGE,
        executable='needle_insert_node',
        output='screen',
        parameters=[params, {
            'use_sim_time': use_sim_time,
            'insertion.depth_m': depth_m,
        }],
    )

    depth_node = Node(
        package=PACKAGE,
        executable='depth_monitor',
        output='screen',
        parameters=[params, {
            'use_sim_time': use_sim_time,
            'insertion.depth_m': depth_m,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('depth_m', default_value='0.025'),
        sim,
        insert_node,
        depth_node,
    ])

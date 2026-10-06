#!/usr/bin/env python3
import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

PACKAGE = 'ur3_needle_sim'


def generate_launch_description():
    pkg = get_package_share_directory(PACKAGE)
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf = os.path.join(pkg, 'resource', 'ur3e_needle.urdf')
    controllers = os.path.join(pkg, 'resource', 'ros2_control_config.yaml')
    # Absolute path (str), not PathJoinSubstitution: WebotsLauncher only redirects
    # the command to its temp world (with injected Ros2Supervisor) for plain strings.
    world_path = os.path.join(pkg, 'worlds', 'ur3_needle.wbt')

    webots = WebotsLauncher(
        world=world_path,
        mode='realtime',
        ros2_supervisor=True,
    )

    driver = WebotsController(
        robot_name='UR3e',
        namespace='ur3',
        parameters=[
            {'robot_description': urdf},
            {'use_sim_time': use_sim_time},
            {'set_robot_state_publisher': True},
            controllers,
        ],
        respawn=True,
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='ur3',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>',
            'use_sim_time': use_sim_time,
        }],
    )

    # One spawner for both controllers — parallel spawners race the CM lock
    # switch-timeout: Webots may still be warming up at first activate
    spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'ur_joint_state_broadcaster',
            'ur_joint_trajectory_controller',
            '-c', 'ur3/controller_manager',
            '--activate-as-group',
            '--controller-manager-timeout', '100',
            '--switch-timeout', '90',
            '--service-call-timeout', '60',
        ],
    )

    waiting = WaitForControllerConnection(
        target_driver=driver,
        nodes_to_start=[spawner],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        webots,
        webots._supervisor,
        rsp,
        driver,
        waiting,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])

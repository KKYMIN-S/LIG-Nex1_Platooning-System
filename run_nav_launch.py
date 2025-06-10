#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Nav2 bringup launch 파일 위치
    bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(bringup_dir, 'launch', 'nav2_bringup_launch.py')

    # Launch Arguments
    map_yaml = LaunchConfiguration('map', default='/home/soda/ros2_ws/map.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=str(map_yaml),
            description='Full path to map yaml file to load'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),

        Node(
            package='teleop_drive',
            node_executable='odom_and_scan_publisher',
            name='odom_and_scan_publisher',
            parameters=[{'use_sim_time': False}],
            output='screen'
        ),

        Node(
            package='teleop_drive',
            node_executable='teleop_listener',
            name='teleop_listener',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # ðŸŸ  Nav2 bringup launch í¬í•¨
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'map': '/home/soda/ros2_ws/map.yaml',
                'use_sim_time': 'false',
                'params_file' : '/home/soda/ros2_ws/src/navigation2/nav2_bringup/bringup/params/nav2_params.yaml'
            }.items()
        ),
        
        Node(
            package='rviz2',
            node_executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/soda/ros2_ws/rviz/slam_config.rviz'],
            parameters=[{'use_sim_time': False}],
            output='screen'
        )

    ])

    

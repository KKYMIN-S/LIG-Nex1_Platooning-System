from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # odom 퍼블리셔 노드
        Node(
            package='teleop_drive',
            node_executable='odom_publisher',
            name='odom_publisher',
            output='screen',
        ),
        Node(
            package='teleop_drive',
            node_executable='teleop_listener',
            name='teleop_listener',
            output='screen',
            env={'PYTHONPATH': '/media/soda/da379f50-c4e9-42d3-8f4a-5f6e4a6550ea/home/soda/rootOnNVMe'}
        ),
        Node(
            package='teleop_twist_keyboard',
            node_executable='teleop_twist_keyboard',
            name='keyboard_control',
            output='screen',
            prefix='xterm -e',  # 선택 사항
        )

    ])


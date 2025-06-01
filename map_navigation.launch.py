from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([

        # ✅ 맵 서버 실행 (저장된 맵 불러오기)
        Node(
            package='nav2_map_server',
            node_executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': '/home/soda/ros2_ws/src/3F_lab.yaml'}]
        ),

        # ✅ AMCL (위치 추정)
        Node(
            package='nav2_amcl',
            node_executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{'use_map_topic': True}]
        ),

        # ✅ 로컬 컨트롤러
        Node(
            package='nav2_controller',
            node_executable='controller_server',
            name='controller_server',
            output='screen'
        ),

        # ✅ 글로벌 플래너
        Node(
            package='nav2_planner',
            node_executable='planner_server',
            name='planner_server',
            output='screen'
        ),

        # ✅ 복구 동작 서버
        Node(
            package='nav2_recoveries',
            node_executable='recoveries_server',
            name='recoveries_server',
            output='screen'
        ),

        # ✅ 행동 트리 기반 내비게이터
        Node(
            package='nav2_bt_navigator',
            node_executable='bt_navigator',
            name='bt_navigator',
            output='screen'
        ),

        # ✅ 정적 TF: base_link -> laser
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            name='tf_baselink_to_laser',
            arguments=['0.2', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),

        # ✅ teleop listener (조이스틱 또는 키보드 수동 조작용)
        Node(
            package='teleop_drive',
            node_executable='teleop_listener',
            name='teleop_listener',
            output='screen'
        ),

        # ✅ RViz 실행
        ExecuteProcess(
            cmd=['rviz2', '-d', '/home/soda/ros2_ws/src/3F_lab.rviz'],
            output='screen'
        )
    ])

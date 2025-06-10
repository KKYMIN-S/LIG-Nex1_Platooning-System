from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',  # node_executable → executable (최신 문법)
            name='slam_toolbox',
            output='screen',
            parameters=[
                get_package_share_directory("slam_toolbox") + '/config/mapper_params_online_async.yaml',
                {
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_link'
                }
            ]
        )
    ])
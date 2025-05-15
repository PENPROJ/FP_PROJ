from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # YAML config 파일의 절대 경로 설정
    config_file = os.path.join(
        get_package_share_directory('test_pkg'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([

        Node(
            package='test_pkg',
            executable='su_fkik',
            name='su_fkik',
            parameters=[config_file],
            output='screen'
        ),

        # su_rviz 노드 실행
        Node(
            package='test_pkg',
            executable='su_rviz',
            name='su_rviz',
            parameters=[config_file],
            output='screen'
        ),

        # trajgen 노드 실행
        Node(
            package='test_pkg',
            executable='trajectory_generator',
            name='trajectory_generator',
            parameters=[config_file],
            output='screen',
        ),

        # force topic bridge 노드 실행
        Node(
            package='test_pkg',
            executable='wrench_bridge',
            name='wrench_bridge',
            parameters=[config_file],
            output='screen',
        ),

    ])

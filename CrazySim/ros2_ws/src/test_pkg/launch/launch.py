from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([
        # su_fkik 노드 실행
        Node(
            package='test_pkg',
            executable='su_fkik',
            output='screen'
        ),

        # su_rviz 노드 실행
        Node(
            package='test_pkg',
            executable='su_rviz',
            output='screen'
        ),

        # trajgen 노드 실행
        Node(
            package='test_pkg',
            executable='trajectory_generator',
            output='screen',
        ),     
        
        # force topic bridge 노드 실행
        Node(
            package='test_pkg',
            executable='wrench_bridge',
            output='screen',
        ),                   

    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('test_pkg')
    urdf_path = os.path.join(pkg_path, 'models', 'model.urdf')
    rviz_config_file = os.path.join(
        get_package_share_directory('test_pkg'),
        'config',
        'config.rviz'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_path).read()}],
            output='screen'
        ),


        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
            output='screen',
            parameters=[{'use_sim_time': True}]            
        ),
    ])


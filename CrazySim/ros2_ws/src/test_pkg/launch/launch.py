from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package paths
    test_pkg_path = get_package_share_directory('test_pkg')
    crazyflie_pkg_path = get_package_share_directory('crazyflie')

    # Configs
    config_file = os.path.join(test_pkg_path, 'config', 'config.yaml')
    urdf_path = os.path.join(test_pkg_path, 'models', 'model.urdf')
    rviz_config_file = os.path.join(test_pkg_path, 'config', 'config.rviz')

    # Launch argument
    backend_arg = LaunchConfiguration('backend', default='cflib')

    # Crazyflie launch (즉시 실행)
    crazyflie_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(crazyflie_pkg_path, 'launch', 'launch.py')
        ),
        launch_arguments={'backend': backend_arg}.items()
    )

    # 나머지 노드 (2초 지연 실행)
    delayed_nodes = TimerAction(
        period=2.0,
        actions=[

            Node(
                package='test_pkg',
                executable='su_fkik',
                name='su_fkik',
                parameters=[config_file],
                output='screen'
            ),

            Node(
                package='test_pkg',
                executable='su_rviz',
                name='su_rviz',
                parameters=[config_file],
                output='screen'
            ),

            Node(
                package='test_pkg',
                executable='trajectory_generator',
                name='trajectory_generator',
                parameters=[config_file],
                output='screen'
            ),

            Node(
                package='test_pkg',
                executable='wrench_bridge',
                name='wrench_bridge',
                parameters=[config_file],
                output='screen'
            ),

            Node(
                package='test_pkg',
                executable='data_logging',   # ✅ 추가된 부분
                name='data_logging',
                parameters=[config_file],
                output='screen'
            ),

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
                parameters=[{'use_sim_time': True}],
                output='screen'
            ),
        ]
    )

    return LaunchDescription([
        crazyflie_launch,
        delayed_nodes
    ])


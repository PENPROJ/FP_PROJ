from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
from datetime import datetime

def generate_launch_description():
    # 토픽 리스트 정의
    topics_to_record = [
        '/cf2/pose',
        '/pen/EE_cmd_xyzYaw',
        '/turtle1/pose',
        '/turtle1/cmd_vel'
    ]

    # MMDDHHMM 형식 시간 문자열
    time_str = datetime.now().strftime("%m%d%H%M")

    # 절대 경로로 bag 저장 디렉토리 설정
    output_dir = f'/home/mrl-seuk/sitl_crazy/CrazySim/ros2_ws/src/test_pkg/bag/{time_str}'

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                *topics_to_record,
                '--output', output_dir
            ],
            output='screen'
        )
    ])


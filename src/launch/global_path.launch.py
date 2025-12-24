import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # === 1. 현재 Launch 파일의 위치를 기준으로 경로 설정 ===
    launch_dir = os.path.dirname(os.path.abspath(__file__))

    # === 2. 상대 경로로 파일 위치 지정 ===
    script_path = os.path.join(launch_dir, '../script/path_following.py')
    rviz_config_path = os.path.join(launch_dir, '../rviz/default.rviz')

    # === 3. 환경 변수 설정 (Domain ID) ===
    env_vars = os.environ.copy()
    env_vars['ROS_DOMAIN_ID'] = '1'

    return LaunchDescription([
        # python 스크립트 실행
        ExecuteProcess(
            cmd=['python3', script_path],
            output='screen',
            env=env_vars
        ),

        # RViz2 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
            env=env_vars
        )
    ])
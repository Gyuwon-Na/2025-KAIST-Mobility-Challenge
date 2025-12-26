import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    launch_dir = os.path.dirname(os.path.abspath(__file__))

    script_path = os.path.join(launch_dir, "../script/path_following.py")
    logger_script_path = os.path.join(launch_dir, "../script/trajectory_logger.py")
    tuner_script_path = os.path.join(launch_dir, "../script/MPC_tuner.py")  # [추가]
    rviz_config_path = os.path.join(launch_dir, "../rviz/default.rviz")

    env_vars = os.environ.copy()
    env_vars["ROS_DOMAIN_ID"] = "1"

    return LaunchDescription(
        [
            ExecuteProcess(cmd=["python3", script_path], output="screen", env=env_vars),
            ExecuteProcess(
                cmd=["python3", logger_script_path], output="screen", env=env_vars
            ),
            ExecuteProcess(
                cmd=["python3", tuner_script_path], output="screen", env=env_vars
            ),  # [추가]
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_path],
                output="screen",
                env=env_vars,
            ),
        ]
    )

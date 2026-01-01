import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("bisa")

    # 파일 경로 설정
    rviz_config = os.path.join(pkg_dir, "rviz", "bisa.rviz")
    mpc_config = os.path.join(pkg_dir, "config", "mpc_params.yaml")
    path_config = os.path.join(pkg_dir, "config", "path.yaml")
    bridge_config_path = os.path.join(pkg_dir, "config", "bridge_config.yaml")

    env_vars = os.environ.copy()
    env_vars["ROS_DOMAIN_ID"] = "1"

    problem_arg = DeclareLaunchArgument(
        "problem",
        default_value="problem2",
        description="Target problem key in path.yaml",
    )

    return LaunchDescription(
        [
            problem_arg,  # Argument 등록
            # 1. HD Map Visualizer
            Node(
                package="domain_bridge",
                executable="domain_bridge",
                name="domain_bridge",
                output="screen",
                arguments=[bridge_config_path],  # 아까 만든 yaml 파일 로드
            ),
            Node(
                package="bisa",
                executable="hdmap_visualizer.py",
                name="hdmap_visualizer",
                output="screen",
            ),
            # 2. Global Path Publisher
            Node(
                package="bisa",
                executable="global_path_pub.py",
                name="global_path_publisher",
                output="screen",
                parameters=[
                    path_config,
                    {"target_problem": LaunchConfiguration("problem")},
                ],
            ),
            # 3. Local Path Publisher
            Node(
                package="bisa",
                executable="local_path_pub",
                name="local_path_pub",
                output="screen",
            ),
            # 4. MPC Path Tracker
            Node(
                package="bisa",
                executable="mpc_path_tracker",
                name="mpc_path_tracker",
                output="screen",
                parameters=[mpc_config],
            ),
            # 5. MPC Tuner GUI
            Node(
                package="bisa",
                executable="mpc_tuner_gui.py",
                name="mpc_tuner_gui",
                output="screen",
            ),
            # 5. Obstacle Relay
            Node(
                package="bisa",
                executable="obstacle_relay.py",
                name="obstacle_relay",
                output="screen",
            ),
            # 6. RViz2
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )

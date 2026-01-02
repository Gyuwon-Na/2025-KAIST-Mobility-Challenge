import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("bisa")

    # 파일 경로 설정
    rviz_config = os.path.join(pkg_dir, "rviz", "problem2.rviz")
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
            Node(
                package="domain_bridge",
                executable="domain_bridge",
                name="domain_bridge",
                output="screen",
                arguments=[bridge_config_path],
            ),
            # 1. HD Map Visualizer
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
            Node(
                package="bisa",
                executable="frenet_planner",
                name="frenet_planner",
                output="screen",
            ),
            # 3. Local Path Publisher
            Node(
                package="bisa",
                executable="visualize_hvs",
                name="visualize_hvs",
                output="screen",
            ),
            Node(
                package="bisa",
                executable="lane_visualizer.py",
                name="lane_visualizer",
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

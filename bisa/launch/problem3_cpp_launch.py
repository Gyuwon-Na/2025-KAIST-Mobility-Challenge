import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# ==============================================================================
# [1] 차량별 경로(Node Sequence) 데이터베이스 (CAV용)
# ==============================================================================
CAV_PATH_SETTINGS = [
    [21, 51, 46, 40, 43, 9, 56, 59, 18, 21],
    [60, 52, 24, 37, 39, 49, 55, 12, 15, 60],
    [19, 22, 25, 36, 38, 48, 58, 19],
    [16, 61, 47, 41, 42, 8, 11, 16],
]


def load_yaml_file(file_path):
    try:
        with open(file_path, "r") as f:
            return yaml.safe_load(f)
    except Exception as e:
        return {}


def generate_launch_description():
    pkg_dir = get_package_share_directory("bisa")
    config_file = os.path.join(pkg_dir, "config", "cav_config.yaml")
    rviz_config = os.path.join(pkg_dir, "rviz", "bisa.rviz")

    # 1. YAML 파일 로드
    full_config = load_yaml_file(config_file)

    # 공통 파라미터 추출
    try:
        ros_params_dict = full_config["/**"]["ros__parameters"]
        mpc_config_default = ros_params_dict.get("mpc_settings", {})
    except KeyError:
        ros_params_dict = {"cav_ids": [1, 2, 3, 4]}
        mpc_config_default = {}

    # HV 설정 추출
    hv_settings = full_config.get("hv_settings", [])

    # GUI용 슬롯별 파라미터 딕셔너리 생성
    gui_slot_params = {}
    for slot_idx in range(1, 5):  # slot 01 ~ 04
        slot_str = f"{slot_idx:02d}"
        yaml_section = f"mpc_tracker_cav{slot_str}"
        if yaml_section in full_config:
            slot_params = full_config[yaml_section].get("ros__parameters", {})
            for key, value in slot_params.items():
                gui_slot_params[f"slot{slot_str}_{key}"] = value

    nodes = []

    # 동기화 노드
    nodes.append(
        Node(
            package="bisa",
            executable="synchronized_start_node",
            name="synchronized_start_node",
            output="screen",
            parameters=[ros_params_dict],
        )
    )

    # ---------------------------------------------------------
    # 2. 공통 노드 및 GUI 실행
    # ---------------------------------------------------------
    nodes.append(
        Node(
            package="bisa",
            executable="hdmap_visualizer.py",
            name="hdmap_visualizer",
            output="screen",
        )
    )

    # 충돌 방지 노드
    nodes.append(
        Node(
            package="bisa",
            executable="collision_avoidance_node_cpp",
            name="collision_avoidance_node",
            output="screen",
            parameters=[ros_params_dict],
        )
    )

    # ---------------------------------------------------------
    # 3. HV 차량 자동 생성
    # ---------------------------------------------------------
    for hv in hv_settings:
        hv_id = hv["id"]
        hv_path = hv["node_sequence"]

        nodes.append(
            Node(
                package="bisa",
                executable="global_path_pub_multi.py",
                name=f"global_path_pub_hv{hv_id}",
                output="screen",
                parameters=[
                    {"cav_id": hv_id, "node_sequence": hv_path, "rviz_slot": -1}
                ],
                remappings=[("/user_global_path", f"/user_global_path_hv{hv_id}")],
            )
        )

    # ---------------------------------------------------------
    # 4. 동적 CAV 차량 노드 생성 (RViz 슬롯 직접 발행)
    # ---------------------------------------------------------
    active_ids = ros_params_dict.get("cav_ids", [1, 2, 3, 4])

    for index, cav_id in enumerate(active_ids):
        if index >= len(CAV_PATH_SETTINGS):
            continue

        node_seq = CAV_PATH_SETTINGS[index]
        id_str = f"{cav_id:02d}"

        rviz_slot = index
        slot_str = f"{index + 1:02d}"
        yaml_section_name = f"mpc_tracker_cav{slot_str}"

        node_params = {}
        if yaml_section_name in full_config:
            node_params = full_config[yaml_section_name].get("ros__parameters", {})

        # (A) Global Path
        nodes.append(
            Node(
                package="bisa",
                executable="global_path_pub_multi.py",
                name=f"global_path_pub_cav{id_str}",
                output="screen",
                parameters=[
                    {
                        "cav_id": cav_id,
                        "node_sequence": node_seq,
                        "rviz_slot": rviz_slot,
                    }
                ],
                remappings=[("/user_global_path", f"/user_global_path_cav{id_str}")],
            )
        )

        # (B) Local Path
        nodes.append(
            Node(
                package="bisa",
                executable="local_path_pub_cpp",
                name=f"local_path_pub_cav{id_str}",
                output="screen",
                parameters=[
                    ros_params_dict,
                    {
                        "target_cav_id": cav_id,
                        "rviz_slot": rviz_slot,
                    },
                ],
                remappings=[
                    ("/user_global_path", f"/user_global_path_cav{id_str}"),
                    ("/Ego_pose", f"/CAV_{id_str}"),
                    ("/local_path", f"/local_path_cav{id_str}"),
                    ("/car_marker", f"/car_marker_cav{id_str}"),
                    ("/lap_information", f"/lap_info_cav{id_str}"),
                ],
            )
        )

        # (C) MPC Path Tracker
        nodes.append(
            Node(
                package="bisa",
                executable="mpc_path_tracker_cpp",
                name=f"mpc_tracker_cav{id_str}",
                output="screen",
                parameters=[node_params, {"target_cav_id": cav_id}],
                remappings=[
                    ("/local_path", f"/local_path_cav{id_str}"),
                    ("/Ego_pose", f"/CAV_{id_str}"),
                    ("/Accel", f"/CAV_{id_str}_accel_raw"),
                    ("/mpc_predicted_path", f"/mpc_pred_cav{id_str}"),
                ],
            )
        )

    return LaunchDescription(nodes)

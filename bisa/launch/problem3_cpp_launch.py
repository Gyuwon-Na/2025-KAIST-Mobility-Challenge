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
        print(f"[ERROR] Config 파일 읽기 실패: {e}")
        return {}

def generate_launch_description():
    pkg_dir = get_package_share_directory("bisa")
    config_file = os.path.join(pkg_dir, "config", "cav_config.yaml")
    rviz_config = os.path.join(pkg_dir, "rviz", "bisa.rviz")

    # 1. YAML 파일 로드
    full_config = load_yaml_file(config_file)
    
    # 파라미터 추출
    try:
        ros_params_dict = full_config["/**"]["ros__parameters"]
        # MPC 파라미터 분리 (없으면 빈 딕셔너리)
        mpc_config = ros_params_dict.get("mpc_settings", {})
    except KeyError:
        print("[WARN] YAML 구조 에러. 기본값을 사용합니다.")
        ros_params_dict = {"cav_ids": [1, 2, 3, 4]}
        mpc_config = {}

    # HV 설정 추출
    hv_settings = full_config.get("hv_settings", [])

    nodes = []

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

    # [수정] MPC Tuner GUI 실행 (YAML 파라미터 전달)
    # GUI가 실행되면서 이 값들로 초기 트랙바 위치를 잡고, 
    # cav_ids 정보를 통해 누구에게 명령을 보낼지 결정합니다.
    nodes.append(
        Node(
            package="bisa",
            executable="mpc_tuner_gui.py",
            name="mpc_tuner_gui",
            output="screen",
            parameters=[ros_params_dict, mpc_config], # cav_ids와 mpc설정 모두 전달
        )
    )

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
        hv_id = hv['id']
        hv_path = hv['node_sequence']
        
        nodes.append(Node(
            package='bisa', 
            executable='global_path_pub_multi.py',
            name=f'global_path_pub_hv{hv_id}', 
            output='screen',
            parameters=[{'cav_id': hv_id, 'node_sequence': hv_path}],
            remappings=[('/user_global_path', f'/user_global_path_hv{hv_id}')]
        ))

    # ---------------------------------------------------------
    # 4. 동적 CAV 차량 노드 생성
    # ---------------------------------------------------------
    active_ids = ros_params_dict.get("cav_ids", [1, 2, 3, 4])
    print(f"\n========== Launching CAV IDs: {active_ids} ==========\n")

    for index, cav_id in enumerate(active_ids):
        if index >= len(CAV_PATH_SETTINGS):
            continue

        node_seq = CAV_PATH_SETTINGS[index]
        id_str = f"{cav_id:02d}"

        # (A) Global Path
        nodes.append(
            Node(
                package="bisa",
                executable="global_path_pub_multi.py",
                name=f"global_path_pub_cav{id_str}",
                output="screen",
                parameters=[{"cav_id": cav_id, "node_sequence": node_seq}],
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
                    {"target_cav_id": cav_id}
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
        # [수정] YAML에서 읽어온 mpc_config를 직접 전달
        nodes.append(
            Node(
                package="bisa",
                executable="mpc_path_tracker_cpp",
                name=f"mpc_tracker_cav{id_str}",
                output="screen",
                parameters=[mpc_config, {"target_cav_id": cav_id}],
                remappings=[
                    ("/local_path", f"/local_path_cav{id_str}"),
                    ("/Ego_pose", f"/CAV_{id_str}"),
                    ("/Accel", f"/CAV_{id_str}_accel_raw"),
                    ("/mpc_predicted_path", f"/mpc_pred_cav{id_str}"),
                ],
            )
        )

    # 5. RViz2
    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        )
    )

    return LaunchDescription(nodes)
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Accel, PoseStamped
from nav_msgs.msg import Path
import numpy as np
import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from MPC_solver import MPCSolver
import utils

# CSV 파일 경로 설정 [cite: 1]
CSV_PATH = os.path.normpath(os.path.join(current_dir, "../path/problem1-1_CAV1.csv"))

class PathFollowerNode(Node):
    def __init__(self):
        super().__init__("mpc_path_follower")

        # QoS 설정 (매뉴얼 기준 SensorDataQoS 대응) [cite: 282]
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.mpc = MPCSolver()
        # 시뮬레이터 제어 토픽 /Accel [cite: 287]
        self.pub_accel = self.create_publisher(Accel, "/Accel", 10)
        self.sub_pose = self.create_subscription(PoseStamped, "/Ego_pose", self.pose_callback, qos_profile)

        # 경로 시각화용 QoS
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_viz_path = self.create_publisher(Path, "/global_path", latched_qos)
        self.pub_local_path = self.create_publisher(Path, "/mpc_local_path", 10)

        # 데이터 로드
        self.global_path = utils.load_path_csv(CSV_PATH)
        self.last_nearest_idx = 0
        self.current_pose = None
        self.frame_id = "world"

        if len(self.global_path) > 0:
            self.get_logger().info(f"Loaded {len(self.global_path)} points. Ready to drive.")

        # 1초마다 글로벌 패스 재발행 (RViz 표시용)
        self.viz_timer = self.create_timer(1.0, self.publish_global_path)
        # 20Hz 주기로 제어 루프 실행
        self.timer = self.create_timer(0.05, self.control_loop)

    def pose_callback(self, msg):
        self.current_pose = msg
        self.frame_id = msg.header.frame_id

    def publish_global_path(self):
        if len(self.global_path) == 0: return
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        for pt in self.global_path:
            p = PoseStamped()
            p.pose.position.x, p.pose.position.y = pt[0], pt[1]
            path_msg.poses.append(p)
        self.pub_viz_path.publish(path_msg)

    def publish_local_path(self, pred_path, ego):
        if not pred_path: return
        msg = Path()
        msg.header.frame_id = self.frame_id
        rx, ry, ryaw = ego
        c, s = np.cos(ryaw), np.sin(ryaw)
        for lx, ly in pred_path:
            # 로컬 예측 경로를 글로벌 좌표로 역변환하여 표시
            gx = rx + (c * lx - s * ly)
            gy = ry + (s * lx + c * ly)
            p = PoseStamped()
            p.pose.position.x, p.pose.position.y = gx, gy
            msg.poses.append(p)
        self.pub_local_path.publish(msg)

    def control_loop(self):
        if self.current_pose is None or len(self.global_path) == 0: return

        # 1. 현재 차량의 글로벌 상태 (고정된 세계 기준)
        rx = self.current_pose.pose.position.x
        ry = self.current_pose.pose.position.y
        ryaw = utils.euler_from_quaternion(self.current_pose.pose.orientation)

        # 2. Nearest Point 검색
        dists = np.linalg.norm(self.global_path[:, :2] - np.array([rx, ry]), axis=1)
        min_idx = np.argmin(dists)

        # 3. [핵심] 경로를 내 헤딩 기준으로 완전히 '수평' 정렬
        # 코너를 돌면 수직이 되는 게 아니라, 지도가 내 헤딩만큼 반대로 돌아가서 항상 정면이 x축이 됨
        look_ahead = 50 
        pts_global = self.global_path[min_idx : min_idx + look_ahead, :2]
        
        # utils.global_to_local을 통해 pts_local은 이제 '내 앞'이 x축인 좌표계임
        pts_local = utils.global_to_local([rx, ry, ryaw], pts_global)
        
        # 전방 필터링 (내 뒤에 있는 점은 수식 계산에서 제외)
        pts_local = pts_local[pts_local[:, 0] > 0]
        
        if len(pts_local) < 5: return

        # 4. 3차 다항식 피팅 (이 수식은 이제 항상 x축(수평) 근처에서 생성됨)
        coeffs = np.polyfit(pts_local[:, 0], pts_local[:, 1], 3)

        # 5. MPC Solve
        # [변경] 차량의 현재 상태를 [rx, ry, ryaw]가 아니라 [0, 0, 0]으로 넘김
        # 내 위치가 항상 원점이고 방향이 0도라고 가정하면 수직 경로는 존재할 수 없음
        v_f, w_f, pred_path_local = self.mpc.solve([0.0, 0.0, 0.0, self.mpc.prev_v], coeffs)
        
        self.mpc.prev_v = v_f
        self.mpc.prev_w = w_f

        # 시각화 (로컬 패스는 이미 계산됨)
        self.publish_local_path(pred_path_local, [rx, ry, ryaw])
        
        # 제어 명령 발행
        cmd = Accel()
        cmd.linear.x = float(v_f)
        cmd.angular.z = float(w_f)
        self.pub_accel.publish(cmd)
           
    def stop_vehicle(self):
        cmd = Accel()
        cmd.linear.x, cmd.angular.z = 0.0, 0.0
        self.pub_accel.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_vehicle()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
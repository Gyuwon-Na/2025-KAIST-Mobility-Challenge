import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Accel, PoseStamped
from nav_msgs.msg import Path
import numpy as np
import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from MPC_solver import MPCSolver
import utils

CSV_PATH = os.path.normpath(os.path.join(current_dir, '../path/problem1-1_CAV1.csv'))

class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc_path_follower')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.mpc = MPCSolver()
        self.pub_accel = self.create_publisher(Accel, '/Accel', 10)
        self.sub_pose = self.create_subscription(PoseStamped, '/Ego_pose', self.pose_callback, qos_profile)
        
        # Global Path 시각화 (초록색)
        latched_qos = QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_viz_path = self.create_publisher(Path, '/global_path', latched_qos)
        
        # === [추가됨] Local MPC Path 시각화 (파란색) ===
        self.pub_local_path = self.create_publisher(Path, '/mpc_local_path', 10)

        self.global_path = utils.load_path_csv(CSV_PATH)
        self.current_pose = None
        self.frame_id = None
        self.timer = self.create_timer(0.01, self.control_loop) # 100Hz
        
        if len(self.global_path) > 0:
            self.get_logger().info("MPC Node Ready. Waiting for Pose...")

    def pose_callback(self, msg):
        self.current_pose = msg
        if self.frame_id is None:
            self.frame_id = msg.header.frame_id
            self.publish_viz_path()

    def publish_viz_path(self):
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for pt in self.global_path:
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            path_msg.poses.append(pose)
        self.pub_viz_path.publish(path_msg)
        
    def publish_local_path(self, pred_path):
        """MPC가 예측한 미래 궤적을 RViz에 표시"""
        if not pred_path or self.frame_id is None: return
        
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for pt in pred_path:
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            path_msg.poses.append(pose)
            
        self.pub_local_path.publish(path_msg)

    def get_curvature(self, idx, lookahead_steps=5):
        if idx + lookahead_steps >= len(self.global_path): return 0.0
        curr_yaw = self.global_path[idx][2]
        future_yaw = self.global_path[idx+lookahead_steps][2]
        diff = abs(future_yaw - curr_yaw)
        while diff > np.pi: diff -= 2*np.pi
        return abs(diff)

    def control_loop(self):
        if self.current_pose is None or len(self.global_path) == 0: return

        rx = self.current_pose.pose.position.x
        ry = self.current_pose.pose.position.y
        ryaw = utils.euler_from_quaternion(self.current_pose.pose.orientation)
        current_state = [rx, ry, ryaw]

        # 1. Nearest Point 찾기
        dists = np.linalg.norm(self.global_path[:, :2] - np.array([rx, ry]), axis=1)
        min_idx = np.argmin(dists)

        # 2. 전략 선택 (곡률 기반)
        if self.get_curvature(min_idx) > 0.2:
            self.mpc.set_strategy('curve')
        else:
            self.mpc.set_strategy('straight')

        # 3. Reference Path 구성 (거리 기반으로 더 정확하게 추출)
        ref_segment = []
        path_len = len(self.global_path)
        
        # 웨이포인트 간격이 약 0.01m(1cm)라고 가정할 때, 
        # 속도(m/s) * 시간(dt) 만큼 인덱스를 건너뛰어야 물리적으로 맞음
        # 예: 5m/s * 0.1s = 0.5m 이동 -> 약 50 인덱스 필요
        
        current_speed = max(self.mpc.prev_v, 1.0) # 최소 1.0으로 가정
        step_distance = current_speed * self.mpc.dt # 한 스텝당 이동 거리 (m)
        point_spacing = 0.01 # CSV 웨이포인트 간격 (1cm 가정)
        
        index_step = int(step_distance / point_spacing)
        if index_step < 1: index_step = 1

        for i in range(self.mpc.N):
            target_idx = min_idx + (i * index_step)
            if target_idx >= path_len: target_idx = path_len - 1
            ref_segment.append(self.global_path[target_idx])
        
        # 4. MPC Solve (수정된 solve 함수 호출)
        v_cmd, w_cmd, pred_path = self.mpc.solve(current_state, np.array(ref_segment))
        v_final, w_final = self.mpc.apply_smoothing(v_cmd, w_cmd)

        # 5. Local Path 시각화
        self.publish_local_path(pred_path)

        # 6. 제어 명령 발행 및 로그 출력
        cmd = Accel()
        cmd.linear.x = float(v_final)
        cmd.angular.z = float(w_final)
        self.pub_accel.publish(cmd)
        
        # 디버깅: 값이 0이거나 너무 작으면 로그로 확인
        # self.get_logger().info(f"V: {v_final:.2f}, W: {w_final:.2f}, Idx: {min_idx}")

def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
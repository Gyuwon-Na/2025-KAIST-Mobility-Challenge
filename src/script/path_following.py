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
        
        latched_qos = QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_viz_path = self.create_publisher(Path, '/global_path', latched_qos)
        self.pub_local_path = self.create_publisher(Path, '/mpc_local_path', 10)

        self.global_path = utils.load_path_csv(CSV_PATH)
        self.current_pose = None
        self.frame_id = None
        
        # [핵심 추가] 마지막으로 방문한 인덱스를 기억함 (순서 보장용)
        self.last_nearest_idx = 0
        
        self.current_mode = "unknown" 
        self.timer = self.create_timer(0.05, self.control_loop) 
        
        if len(self.global_path) > 0:
            self.get_logger().info(f"MPC Node Ready. Total Waypoints: {len(self.global_path)}")

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

    def get_curvature(self, idx, lookahead_steps=20):
        if idx + lookahead_steps >= len(self.global_path): return 0.0
        curr_yaw = self.global_path[idx][2]
        max_diff = 0.0
        for i in range(1, lookahead_steps, 5): 
            check_idx = idx + i
            if check_idx >= len(self.global_path): break
            future_yaw = self.global_path[check_idx][2]
            diff = abs(future_yaw - curr_yaw)
            while diff > np.pi: diff -= 2*np.pi
            if abs(diff) > max_diff:
                max_diff = abs(diff)
        return max_diff

    def control_loop(self):
        if self.current_pose is None or len(self.global_path) == 0: return

        rx = self.current_pose.pose.position.x
        ry = self.current_pose.pose.position.y
        ryaw = utils.euler_from_quaternion(self.current_pose.pose.orientation)
        current_state = [rx, ry, ryaw]

        # === [핵심 변경 1] 순차적 Nearest Point 찾기 (Window Search) ===
        # 전체 경로를 뒤지는 게 아니라, 이전 위치(last_nearest_idx)부터 
        # 앞으로 50개(약 0.5m ~ 1m) 정도만 뒤집니다.
        # 이렇게 하면 절대 뒷걸음질 치거나 옆길로 새지 않고 인덱스가 1씩 증가합니다.
        
        search_start = self.last_nearest_idx
        # 경로 끝에 다다르면 검색 범위를 끝까지만 제한
        search_end = min(self.last_nearest_idx + 100, len(self.global_path)) 
        
        # 검색 범위 내의 좌표들만 추출
        search_segment = self.global_path[search_start:search_end, :2]
        
        if len(search_segment) == 0:
            # 경로 끝에 도달했거나 에러 상황 -> 멈추거나 마지막 점 유지
            min_idx = len(self.global_path) - 1
        else:
            # 부분 경로 내에서 가장 가까운 점 찾기
            dists = np.linalg.norm(search_segment - np.array([rx, ry]), axis=1)
            local_min_idx = np.argmin(dists)
            
            # 실제 전체 인덱스로 변환
            min_idx = search_start + local_min_idx
            
            # [중요] 상태 업데이트: 다음번엔 여기서부터 찾음
            self.last_nearest_idx = min_idx

        # 2. 모드 결정
        curvature = self.get_curvature(min_idx, lookahead_steps=30)
        new_mode = 'straight'
        if curvature > 0.2: new_mode = 'curve'
        
        if new_mode != self.current_mode:
            self.get_logger().info(f">>> [MODE] {self.current_mode} -> {new_mode} (Curv: {curvature:.4f})")
            self.current_mode = new_mode
            self.mpc.set_strategy(new_mode)

        # 3. Reference Path 구성 (물리 법칙 준수)
        ref_segment = []
        path_len = len(self.global_path)
        
        target_v = self.mpc.target_speed 
        dt_dist = target_v * self.mpc.dt
        point_spacing = 0.01 
        
        step = int(dt_dist / point_spacing)
        if step < 1: step = 1
        
        # [중요] MPC는 미래를 봐야 하므로, 현재 min_idx(순서대로 찾은 것)부터
        # 물리적 거리(step)만큼 띄우면서 점을 골라 담습니다.
        for i in range(self.mpc.N):
            target_idx = min_idx + (i * step)
            if target_idx >= path_len: target_idx = path_len - 1
            ref_segment.append(self.global_path[target_idx])
        
        # 4. Solve
        v_cmd, w_cmd, pred_path = self.mpc.solve(current_state, np.array(ref_segment))
        v_final, w_final = self.mpc.apply_smoothing(v_cmd, w_cmd)

        self.publish_local_path(pred_path)

        cmd = Accel()
        cmd.linear.x = float(v_final)
        cmd.angular.z = float(w_final)
        self.pub_accel.publish(cmd)

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
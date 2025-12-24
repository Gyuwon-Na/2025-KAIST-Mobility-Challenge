import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Accel, PoseStamped
from nav_msgs.msg import Path
import csv
import math
import numpy as np
import os

# current_dir: src/script
current_dir = os.path.dirname(os.path.abspath(__file__))


CSV_PATH = os.path.join(current_dir, '../path/problem1-1_CAV1.csv')
LOOKAHEAD_DIST = 2.5
TARGET_SPEED = 5.0

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower_node')
        
        # 1. QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 2. Publisher & Subscriber
        self.pub_accel = self.create_publisher(Accel, '/Accel', 10)
        
        latched_qos = QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_viz_path = self.create_publisher(Path, '/global_path', latched_qos)
        
        self.sub_pose = self.create_subscription(PoseStamped, '/Ego_pose', self.pose_callback, qos_profile)
        
        # 3. 데이터 초기화
        self.np_path = self.load_path_csv(CSV_PATH)
        self.current_pose = None
        self.detected_frame_id = None 
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.viz_timer = self.create_timer(1.0, self.viz_loop)

        if len(self.np_path) > 0:
            self.get_logger().info(f"Ready. Loaded {len(self.np_path)} points. Waiting for Pose...")
        else:
            self.get_logger().error("Path is empty! Please check CSV file path and content.")

    def load_path_csv(self, file_path):
        """CSV 파일을 읽어오며 헤더와 NaN 값을 모두 처리합니다."""
        path = []
        try:
            with open(file_path, 'r') as f:
                reader = csv.reader(f)
                
                # [중요] 첫 번째 줄(헤더 X, Y) 건너뛰기
                header = next(reader, None)
                
                for i, row in enumerate(reader):
                    # 빈 줄 체크
                    if not row or len(row) < 2: continue
                    
                    try:
                        # 문자열을 실수(float)로 변환
                        s_x = row[0].strip()
                        s_y = row[1].strip()
                        
                        # 빈 문자열 체크
                        if not s_x or not s_y: continue

                        x = float(s_x)
                        y = float(s_y)
                        
                        # [핵심] NaN(숫자 아님)이나 무한대(Inf)가 있으면 건너뜀
                        if math.isnan(x) or math.isnan(y) or math.isinf(x) or math.isinf(y):
                            self.get_logger().warn(f"Skipping invalid data at line {i+2}")
                            continue
                            
                        path.append([x, y])
                    except ValueError:
                        self.get_logger().warn(f"Skipping non-numeric data at line {i+2}: {row}")
                        continue
                        
            return np.array(path)
        except Exception as e:
            self.get_logger().error(f"Failed to load CSV: {e}")
            return np.array([])

    def pose_callback(self, msg):
        if math.isnan(msg.pose.position.x): return
        self.current_pose = msg
        if self.detected_frame_id is None:
            self.detected_frame_id = msg.header.frame_id
            self.get_logger().info(f"Frame ID Detected: {self.detected_frame_id}")

    def euler_from_quaternion(self, q):
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(t3, t4)

    def viz_loop(self):
        if self.detected_frame_id is None or len(self.np_path) == 0:
            return

        path_msg = Path()
        path_msg.header.frame_id = self.detected_frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for pt in self.np_path:
            pose = PoseStamped()
            pose.header.frame_id = self.detected_frame_id
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.orientation.w = 1.0 
            path_msg.poses.append(pose)
            
        self.pub_viz_path.publish(path_msg)

    def control_loop(self):
        if self.current_pose is None or len(self.np_path) == 0:
            return

        curr_x = self.current_pose.pose.position.x
        curr_y = self.current_pose.pose.position.y
        q = self.current_pose.pose.orientation
        curr_yaw = self.euler_from_quaternion(q)

        # 거리 계산
        dists = np.linalg.norm(self.np_path - np.array([curr_x, curr_y]), axis=1)
        min_idx = np.argmin(dists)

        target_idx = min_idx
        found = False
        for i in range(min_idx, len(self.np_path)):
            if np.linalg.norm(self.np_path[i] - np.array([curr_x, curr_y])) > LOOKAHEAD_DIST:
                target_idx = i
                found = True
                break
        
        if not found: target_idx = len(self.np_path) - 1
            
        target_pt = self.np_path[target_idx]
        angle_to_target = math.atan2(target_pt[1] - curr_y, target_pt[0] - curr_x)
        heading_error = angle_to_target - curr_yaw

        while heading_error > math.pi: heading_error -= 2 * math.pi
        while heading_error < -math.pi: heading_error += 2 * math.pi

        # [안전장치] 계산 결과가 안전한지 확인
        steering_cmd = 2.0 * heading_error
        
        # NaN 발생 시 정지
        if math.isnan(steering_cmd) or math.isinf(steering_cmd):
            steering_cmd = 0.0 

        cmd = Accel()
        cmd.linear.x = float(TARGET_SPEED)
        cmd.angular.z = float(steering_cmd)
        
        self.pub_accel.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
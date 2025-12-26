import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Accel, PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Path
import numpy as np
import os
import sys
import csv
import json  # JSON 파싱용

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from MPC_solver import MPCSolver
import utils

CSV_PATH = os.path.normpath(os.path.join(current_dir, "../path/problem1-1_CAV1.csv"))
LOG_PATH = os.path.normpath(os.path.join(current_dir, "trajectory_log.csv"))


class PathFollowerNode(Node):
    def __init__(self):
        super().__init__("mpc_path_follower")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.mpc = MPCSolver()
        self.pub_accel = self.create_publisher(Accel, "/Accel", 10)
        self.pub_mode = self.create_publisher(String, "/mpc_mode", 10)

        # [추가] 튜닝 파라미터 구독
        self.sub_params = self.create_subscription(
            String, "/mpc_params", self.param_callback, 10
        )

        self.sub_pose = self.create_subscription(
            PoseStamped, "/Ego_pose", self.pose_callback, qos_profile
        )
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_viz_path = self.create_publisher(Path, "/global_path", latched_qos)
        self.pub_local_path = self.create_publisher(Path, "/mpc_local_path", 10)

        self.global_path = utils.load_path_csv(CSV_PATH)
        self.last_nearest_idx = 0
        self.current_pose = None
        self.frame_id = "world"

        # [튜닝용 변수] 초기값
        self.look_curve = 35
        self.look_straight = 70

        self.history_x = []
        self.history_y = []
        self.history_mode = []

        if len(self.global_path) > 0:
            self.get_logger().info(f"Loaded {len(self.global_path)} points.")

        self.viz_timer = self.create_timer(1.0, self.publish_global_path)
        self.timer = self.create_timer(0.05, self.control_loop)

    def param_callback(self, msg):
        """GUI에서 보낸 파라미터를 적용"""
        try:
            data = json.loads(msg.data)
            # Look ahead 업데이트
            self.look_curve = data["c_look"]
            self.look_straight = data["s_look"]
            # MPC 내부 파라미터 업데이트
            self.mpc.update_params(data)
        except Exception as e:
            self.get_logger().error(f"Param update failed: {e}")

    def pose_callback(self, msg):
        self.current_pose = msg
        self.frame_id = msg.header.frame_id

    def publish_global_path(self):
        if len(self.global_path) == 0:
            return
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        for pt in self.global_path:
            p = PoseStamped()
            p.pose.position.x, p.pose.position.y = pt[0], pt[1]
            path_msg.poses.append(p)
        self.pub_viz_path.publish(path_msg)

    def publish_local_path(self, pred_path, ego):
        if not pred_path:
            return
        msg = Path()
        msg.header.frame_id = self.frame_id
        rx, ry, ryaw = ego
        c, s = np.cos(ryaw), np.sin(ryaw)
        for lx, ly in pred_path:
            gx = rx + (c * lx - s * ly)
            gy = ry + (s * lx + c * ly)
            p = PoseStamped()
            p.pose.position.x, p.pose.position.y = gx, gy
            msg.poses.append(p)
        self.pub_local_path.publish(msg)

    def control_loop(self):
        if self.current_pose is None or len(self.global_path) == 0:
            return

        rx = self.current_pose.pose.position.x
        ry = self.current_pose.pose.position.y
        ryaw = utils.euler_from_quaternion(self.current_pose.pose.orientation)

        dists = np.linalg.norm(self.global_path[:, :2] - np.array([rx, ry]), axis=1)
        min_idx = np.argmin(dists)

        check_dist = 40
        check_idx = min(min_idx + check_dist, len(self.global_path) - 1)

        yaw_curr = self.global_path[min_idx][2]
        yaw_future = self.global_path[check_idx][2]
        yaw_diff = abs(yaw_future - yaw_curr)
        while yaw_diff > np.pi:
            yaw_diff -= 2 * np.pi

        # [수정] 튜닝된 look_ahead 값 사용
        if abs(yaw_diff) > 0.1:
            mode = "curve"
            look_ahead = self.look_curve
        else:
            mode = "straight"
            look_ahead = self.look_straight

        self.mpc.set_mode(mode)

        mode_msg = String()
        mode_msg.data = mode
        self.pub_mode.publish(mode_msg)

        pts_global = self.global_path[min_idx : min_idx + look_ahead, :2]
        pts_local = utils.global_to_local([rx, ry, ryaw], pts_global)
        pts_local = pts_local[pts_local[:, 0] > 0]

        if len(pts_local) < 5:
            return

        coeffs = np.polyfit(pts_local[:, 0], pts_local[:, 1], 3)
        v_f, w_f, pred_path_local = self.mpc.solve(
            [0.0, 0.0, 0.0, self.mpc.prev_v], coeffs
        )
        self.mpc.prev_v = v_f

        self.history_x.append(rx)
        self.history_y.append(ry)
        self.history_mode.append(mode)

        self.publish_local_path(pred_path_local, [rx, ry, ryaw])

        cmd = Accel()
        cmd.linear.x = float(v_f)
        cmd.angular.z = float(w_f)
        self.pub_accel.publish(cmd)

    def save_trajectory_log(self):
        if not self.history_x:
            return
        try:
            with open(LOG_PATH, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["x", "y", "mode"])
                for x, y, m in zip(self.history_x, self.history_y, self.history_mode):
                    writer.writerow([x, y, m])
            self.get_logger().info(f"Trajectory Log Saved: {LOG_PATH}")
        except Exception as e:
            self.get_logger().error(f"Failed to save log: {e}")

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
        node.save_trajectory_log()
        node.stop_vehicle()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

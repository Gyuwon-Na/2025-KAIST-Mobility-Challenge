import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Accel, PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Path
import numpy as np
import os
import sys
import json

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from MPC_solver import MPCSolver
import utils

CSV_PATH = os.path.normpath(os.path.join(current_dir, "../path/problem1-1_CAV1.csv"))
CONFIG_PATH = os.path.normpath(os.path.join(current_dir, "../mpc_config.json"))


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

        # [수정] 프레임 ID 초기화
        self.frame_id = "world"

        self.load_look_ahead()

        if len(self.global_path) > 0:
            self.get_logger().info(f"Loaded {len(self.global_path)} points.")

        self.viz_timer = self.create_timer(1.0, self.publish_global_path)
        self.timer = self.create_timer(0.05, self.control_loop)

    def load_look_ahead(self):
        try:
            with open(CONFIG_PATH, "r") as f:
                data = json.load(f)
                self.look_curve = data["curve"]["look_ahead"]
                self.look_straight = data["straight"]["look_ahead"]
        except:
            self.look_curve = 35
            self.look_straight = 70

    def param_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.look_curve = data["c_look"]
            self.look_straight = data["s_look"]
            self.mpc.update_params(data)
        except Exception:
            pass

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

        # [수정] 감지 거리를 40 -> 100으로 대폭 늘림
        # 이유: 직선 속도(1.5)가 빠르므로, 훨씬 더 멀리서부터 코너를 감지해야
        # 미리 감속(0.4)하고 핸들을 준비할 수 있음.
        check_dist = 80
        check_idx = min(min_idx + check_dist, len(self.global_path) - 1)

        yaw_curr = self.global_path[min_idx][2]
        yaw_future = self.global_path[check_idx][2]
        yaw_diff = abs(yaw_future - yaw_curr)
        while yaw_diff > np.pi:
            yaw_diff -= 2 * np.pi

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

        # [복구 완료] 불안정한 current_velocity 대신 MPC 내부 상태(prev_v) 사용
        # 이 부분이 변경되어 로컬 패스가 안 떴던 것입니다.
        v_f, w_f, pred_path_local = self.mpc.solve(
            [0.0, 0.0, 0.0, self.mpc.prev_v], coeffs
        )
        self.mpc.prev_v = v_f

        self.publish_local_path(pred_path_local, [rx, ry, ryaw])

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

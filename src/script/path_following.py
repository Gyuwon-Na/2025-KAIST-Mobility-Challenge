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
        self.pub_ref_path = self.create_publisher(Path, "/mpc_ref_points", 10)

        self.global_path = utils.load_path_csv(CSV_PATH)
        self.frame_id = "world"
        self.load_look_ahead()
        self.current_mode_state = "straight"

        self.last_nearest_idx = 0
        self.current_pose = None

        # 리셋 감지용
        self.prev_rx = 0.0
        self.prev_ry = 0.0
        self.is_first_pose = True

        self.viz_timer = self.create_timer(1.0, self.publish_global_path)
        self.timer = self.create_timer(0.05, self.control_loop)

    def load_look_ahead(self):
        try:
            with open(CONFIG_PATH, "r") as f:
                data = json.load(f)
                self.look_curve = data["curve"]["look_ahead"]
                self.look_straight = data["straight"]["look_ahead"]
                self.speed_curve = data["curve"]["speed"]  # [추가] Config에서 속도 읽기
                self.speed_straight = data["straight"]["speed"]
        except:
            self.look_curve = 35
            self.look_straight = 70
            self.speed_curve = 0.4
            self.speed_straight = 1.0

    def param_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.look_curve = data["c_look"]
            self.look_straight = data["s_look"]
            self.speed_curve = data["c_speed"]  # [추가] Tuner 업데이트 반영
            self.speed_straight = data["s_speed"]
            self.mpc.update_params(data)
        except Exception:
            pass

    def pose_callback(self, msg):
        self.current_pose = msg
        self.frame_id = msg.header.frame_id

        curr_x = msg.pose.position.x
        curr_y = msg.pose.position.y

        if not self.is_first_pose:
            dist_jump = np.sqrt(
                (curr_x - self.prev_rx) ** 2 + (curr_y - self.prev_ry) ** 2
            )
            if dist_jump > 2.0:
                self.get_logger().warn(
                    ">>> RESET DETECTED! Clearing Index & MPC Memory. <<<"
                )
                self.mpc.prev_u = np.zeros(self.mpc.N * 2)
                self.mpc.prev_v = 0.0
                self.last_nearest_idx = 0

        self.prev_rx = curr_x
        self.prev_ry = curr_y
        self.is_first_pose = False

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

    def publish_local_path(self, pred_path, ego, pub):
        if pred_path is None or len(pred_path) == 0:
            return
        msg = Path()
        msg.header.frame_id = self.frame_id
        rx, ry, ryaw = ego
        c, s = np.cos(ryaw), np.sin(ryaw)

        for pt in pred_path:
            lx, ly = pt[0], pt[1]
            gx = rx + (c * lx - s * ly)
            gy = ry + (s * lx + c * ly)
            p = PoseStamped()
            p.pose.position.x, p.pose.position.y = gx, gy
            msg.poses.append(p)
        pub.publish(msg)

    def get_reference_trajectory(self, min_idx, target_v):
        """
        [핵심] 입력받은 target_v를 기준으로 점을 추출합니다.
        차가 빨라도 target_v가 0.4면 점이 촘촘하게 찍힙니다.
        """
        ref_traj = []
        dt = 0.05
        N = 12
        v_ref = max(target_v, 0.5)

        current_dist_idx = min_idx
        accumulated_dist = 0.0

        ref_traj.append(self.global_path[min_idx, :2])

        for i in range(1, N):
            target_dist = v_ref * dt
            while True:
                if current_dist_idx >= len(self.global_path) - 1:
                    current_dist_idx = len(self.global_path) - 1
                    break

                p1 = self.global_path[current_dist_idx, :2]
                p2 = self.global_path[current_dist_idx + 1, :2]
                seg_dist = np.linalg.norm(p2 - p1)

                if accumulated_dist + seg_dist >= target_dist:
                    current_dist_idx += 1
                    accumulated_dist = 0
                    break
                else:
                    accumulated_dist += seg_dist
                    current_dist_idx += 1
            ref_traj.append(self.global_path[current_dist_idx, :2])
        return np.array(ref_traj)

    def control_loop(self):
        if self.current_pose is None or len(self.global_path) == 0:
            return

        rx = self.current_pose.pose.position.x
        ry = self.current_pose.pose.position.y
        ryaw = utils.euler_from_quaternion(self.current_pose.pose.orientation)

        # 1. Window Search
        search_len = 50
        start_idx = self.last_nearest_idx
        end_idx = min(start_idx + search_len, len(self.global_path))
        dists = np.linalg.norm(
            self.global_path[start_idx:end_idx, :2] - np.array([rx, ry]), axis=1
        )
        local_min_idx = np.argmin(dists)
        min_idx = start_idx + local_min_idx
        self.last_nearest_idx = min_idx

        # 2. 모드 전환 (히스테리시스)
        check_dist = 120
        check_idx = min(min_idx + check_dist, len(self.global_path) - 1)
        yaw_curr = self.global_path[min_idx][2]
        yaw_future = self.global_path[check_idx][2]
        yaw_diff = abs(yaw_future - yaw_curr)
        while yaw_diff > np.pi:
            yaw_diff -= 2 * np.pi
        yaw_diff = abs(yaw_diff)

        if self.current_mode_state == "straight":
            if yaw_diff > 0.1:
                self.current_mode_state = "curve"
        else:
            if yaw_diff < 0.12:
                self.current_mode_state = "straight"

        mode = self.current_mode_state
        self.mpc.set_mode(mode)
        mode_msg = String()
        mode_msg.data = mode
        self.pub_mode.publish(mode_msg)

        # 3. [핵심] 모드별 목표 속도 설정하여 참조 점 생성
        # 현재 속도(self.mpc.prev_v)가 아니라, '목표 속도'를 넣습니다.
        if mode == "curve":
            target_vel_for_ref = self.speed_curve  # 보통 0.4
        else:
            target_vel_for_ref = self.speed_straight  # 보통 1.0

        ref_traj_global = self.get_reference_trajectory(min_idx, target_vel_for_ref)
        ref_traj_local = utils.global_to_local([rx, ry, ryaw], ref_traj_global)

        self.publish_local_path(ref_traj_local, [rx, ry, ryaw], self.pub_ref_path)

        # 4. MPC Solve
        v_f, w_f, pred_path_local = self.mpc.solve(
            [0.0, 0.0, 0.0, self.mpc.prev_v], ref_traj_local
        )
        self.mpc.prev_v = v_f

        self.publish_local_path(pred_path_local, [rx, ry, ryaw], self.pub_local_path)

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

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import numpy as np
import os
import sys
import csv
import matplotlib.pyplot as plt

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
import utils

# 저장 경로 설정
PATH_DIR = os.path.normpath(os.path.join(current_dir, "../path"))
CSV_LOG_PATH = os.path.join(current_dir, "../log/trajectory_log.csv")
IMG_OUTPUT_PATH = os.path.join(current_dir, "../log/trajectory_result.png")
GLOBAL_PATH_FILE = os.path.join(PATH_DIR, "problem1-1_CAV1.csv")


class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__("trajectory_logger")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub_pose = self.create_subscription(
            PoseStamped, "/Ego_pose", self.pose_callback, qos_profile
        )
        self.sub_mode = self.create_subscription(
            String, "/mpc_mode", self.mode_callback, 10
        )

        self.history_x = []
        self.history_y = []
        self.history_mode = []
        self.current_mode = "straight"

        self.global_path = utils.load_path_csv(GLOBAL_PATH_FILE)
        self.get_logger().info("Logger Ready. Will save to: " + CSV_LOG_PATH)

    def mode_callback(self, msg):
        self.current_mode = msg.data

    def pose_callback(self, msg):
        self.history_x.append(msg.pose.position.x)
        self.history_y.append(msg.pose.position.y)
        self.history_mode.append(self.current_mode)

    def save_results(self):
        if not self.history_x:
            return

        self.get_logger().info("Saving logs...")

        # 1. CSV 저장 (path 폴더)
        try:
            with open(CSV_LOG_PATH, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["x", "y", "mode"])
                for x, y, m in zip(self.history_x, self.history_y, self.history_mode):
                    writer.writerow([x, y, m])
            self.get_logger().info(f"Saved CSV to {CSV_LOG_PATH}")
        except Exception as e:
            self.get_logger().error(f"CSV Save Error: {e}")

        # 2. 이미지 저장 (script 폴더)
        try:
            plt.figure(figsize=(10, 10))
            # ===== Grid (중요) =====
            plt.grid(
                True,
                which="both",
                linestyle="--",
                linewidth=0.5,
                alpha=0.5,
            )

            # minor grid까지 켜면 MPC 미세 차이 비교에 좋음
            plt.minorticks_on()
            plt.grid(
                True,
                which="minor",
                linestyle=":",
                linewidth=0.3,
                alpha=0.3,
            )
            if len(self.global_path) > 0:
                plt.plot(
                    self.global_path[:, 0],
                    self.global_path[:, 1],
                    color="lightgray",
                    linewidth=3,
                )

            hx, hy, hm = (
                np.array(self.history_x),
                np.array(self.history_y),
                np.array(self.history_mode),
            )
            mask_s = hm == "straight"
            mask_c = hm == "curve"

            if np.any(mask_s):
                plt.scatter(hx[mask_s], hy[mask_s], c="black", s=3)
            if np.any(mask_c):
                plt.scatter(hx[mask_c], hy[mask_c], c="gold", s=8)

            plt.axis("equal")
            plt.savefig(IMG_OUTPUT_PATH)
            self.get_logger().info(f"Saved Image to {IMG_OUTPUT_PATH}")
            plt.close()
        except Exception as e:
            self.get_logger().error(f"Image Save Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_results()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

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
import utils  # utils.py 재사용

# 저장 경로 설정
CSV_PATH = os.path.normpath(os.path.join(current_dir, "../path/problem1-1_CAV1.csv"))
LOG_OUTPUT_PATH = os.path.normpath(
    os.path.join(current_dir, "../path/trajectory_log.csv")
)
IMG_OUTPUT_PATH = os.path.normpath(os.path.join(current_dir, "trajectory_result.png"))


class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__("trajectory_logger")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # 데이터 구독
        self.sub_pose = self.create_subscription(
            PoseStamped, "/Ego_pose", self.pose_callback, qos_profile
        )
        self.sub_mode = self.create_subscription(
            String, "/mpc_mode", self.mode_callback, 10
        )

        # 데이터 저장소
        self.history_x = []
        self.history_y = []
        self.history_mode = []
        self.current_mode = "straight"  # 기본값

        # 배경용 글로벌 패스 로드
        self.global_path = utils.load_path_csv(CSV_PATH)
        self.get_logger().info("Trajectory Logger Started. Waiting for data...")

    def mode_callback(self, msg):
        self.current_mode = msg.data

    def pose_callback(self, msg):
        # 위치 데이터가 들어올 때 현재 모드와 함께 저장
        x = msg.pose.position.x
        y = msg.pose.position.y

        self.history_x.append(x)
        self.history_y.append(y)
        self.history_mode.append(self.current_mode)

    def save_results(self):
        if not self.history_x:
            self.get_logger().warn("No data recorded.")
            return

        self.get_logger().info("Saving results...")

        # 1. CSV 저장
        try:
            with open(LOG_OUTPUT_PATH, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["x", "y", "mode"])
                for x, y, m in zip(self.history_x, self.history_y, self.history_mode):
                    writer.writerow([x, y, m])
            self.get_logger().info(f"CSV Saved: {LOG_OUTPUT_PATH}")
        except Exception as e:
            self.get_logger().error(f"Failed to save CSV: {e}")

        # 2. 이미지 생성 및 저장
        try:
            plt.figure(figsize=(12, 12))

            # 배경 (글로벌 패스)
            if len(self.global_path) > 0:
                plt.plot(
                    self.global_path[:, 0],
                    self.global_path[:, 1],
                    color="lightgray",
                    linewidth=3,
                    label="Global Path",
                )

            hx = np.array(self.history_x)
            hy = np.array(self.history_y)
            hm = np.array(self.history_mode)

            # 모드별 시각화
            mask_s = hm == "straight"
            mask_c = hm == "curve"

            if np.any(mask_s):
                plt.scatter(
                    hx[mask_s],
                    hy[mask_s],
                    c="black",
                    s=5,
                    label="Straight Mode",
                    zorder=2,
                )
            if np.any(mask_c):
                plt.scatter(
                    hx[mask_c], hy[mask_c], c="gold", s=10, label="Curve Mode", zorder=3
                )

            plt.legend()
            plt.axis("equal")
            plt.title("Driving Trajectory Analysis")
            plt.grid(True, linestyle="--", alpha=0.5)

            plt.savefig(IMG_OUTPUT_PATH)
            self.get_logger().info(f"Image Saved: {IMG_OUTPUT_PATH}")
            plt.close()  # 메모리 해제

        except Exception as e:
            self.get_logger().error(f"Failed to save Image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_results()  # 종료 시 저장 함수 호출
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

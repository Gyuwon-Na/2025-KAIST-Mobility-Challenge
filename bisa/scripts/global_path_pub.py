#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory


class GlobalPathPublisher(Node):
    def __init__(self):
        super().__init__("global_path_publisher")

        self.get_logger().info("=" * 60)
        self.get_logger().info(" [FIX] Global Path Publisher: FORCING LANE THREE ")
        self.get_logger().info("=" * 60)

        # QoS 설정 (Latched: 늦게 켜진 Rviz에서도 보이도록 설정)
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.global_path_pub = self.create_publisher(Path, "/user_global_path", qos)

        # 경로 로드 및 발행
        self.force_load_lane_three()

    def force_load_lane_three(self):
        # 1. 파일 경로 찾기
        home_dir = os.path.expanduser("~")
        file_path = os.path.join(home_dir, "KAIST/bisa/hdmap_data/lanes.json")

        if not os.path.exists(file_path):
            # 패키지 내부 경로 시도
            try:
                pkg_path = get_package_share_directory("bisa")
                file_path = os.path.join(pkg_path, "hdmap_data", "lanes.json")
            except Exception:
                pass

        if not os.path.exists(file_path):
            self.get_logger().error(
                f"CRITICAL ERROR: Cannot find lanes.json at {file_path}"
            )
            return

        # 2. JSON 로드 및 'three' 키 강제 추출
        try:
            with open(file_path, "r") as f:
                data = json.load(f)

            # [강제] 무조건 'three'만 찾음
            if "three" in data:
                lane_three = data["three"]

                # [디버깅] 터미널에 좌표 출력 (이게 -5.0 근처여야 함)
                if len(lane_three) > 0:
                    start_x = lane_three[0].get("x", 0)
                    start_y = lane_three[0].get("y", 0)
                    self.get_logger().info(
                        f" -> CHECK: Lane 'three' Start Point: X={start_x}, Y={start_y}"
                    )
                    if start_x > -4.0:
                        self.get_logger().warn(
                            "WARNING: This coordinate looks like Lane 1 or 2! Check lanes.json content."
                        )
                    else:
                        self.get_logger().info("OK: Coordinates look like Lane 3.")

                self.publish_path(lane_three)
            else:
                self.get_logger().error("KEY ERROR: 'three' key not found in json!")

        except Exception as e:
            self.get_logger().error(f"Failed to load JSON: {e}")

    def publish_path(self, points):
        msg = Path()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()

        for pt in points:
            if "x" not in pt or "y" not in pt:
                continue

            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(pt["x"])
            pose.pose.position.y = float(pt["y"])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.global_path_pub.publish(msg)
        self.get_logger().info(
            f"✓ Published /user_global_path (Lane 3) with {len(msg.poses)} points."
        )


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

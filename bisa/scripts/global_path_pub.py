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
        self.get_logger().info("Global Path Publisher: FORCE LANE 2 (Index 1)")
        self.get_logger().info("=" * 60)

        # QoS 설정
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.global_path_pub = self.create_publisher(Path, "/user_global_path", qos)

        # 경로 로드 및 발행 (Lane 2 강제)
        self.force_load_lane_two()

    def force_load_lane_two(self):
        # 1. 파일 경로 찾기
        home_dir = os.path.expanduser("~")
        file_path = os.path.join(home_dir, "KAIST/bisa/hdmap_data/lanes.json")

        if not os.path.exists(file_path):
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

        # 2. JSON 로드 및 'two' 키 강제 추출
        try:
            with open(file_path, "r") as f:
                data = json.load(f)

            # [강제] 무조건 'two'만 찾음
            if "two" in data:
                lane_two = data["two"]
                self.get_logger().info(
                    f" -> Loaded Lane 2 with {len(lane_two)} points."
                )
                self.publish_path(lane_two)
            else:
                self.get_logger().error("KEY ERROR: 'two' key not found in json!")

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


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

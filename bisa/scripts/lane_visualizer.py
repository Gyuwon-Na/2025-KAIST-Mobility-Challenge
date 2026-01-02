#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import json
import os


class LaneLoader(Node):
    def __init__(self):
        super().__init__("lane_visualizer")

        # [설정] 파일 경로 (lanes.json)
        home_dir = os.path.expanduser("~")
        self.file_path = os.path.join(home_dir, "KAIST/bisa/hdmap_data/lanes.json")

        # 1. RViz 시각화용 (MarkerArray)
        self.marker_pub = self.create_publisher(MarkerArray, "/hdmap/lane_markers", 10)

        # 2. Planner 제어용 (Path)
        self.pub_one = self.create_publisher(Path, "/hdmap/lane_one", 10)
        self.pub_two = self.create_publisher(Path, "/hdmap/lane_two", 10)
        self.pub_three = self.create_publisher(Path, "/hdmap/lane_three", 10)

        self.timer = self.create_timer(1.0, self.publish_lanes)

        # 데이터 로드
        self.lane_data = self.load_json(self.file_path)
        self.get_logger().info(
            f">>> Simple Lane Loader Started. File: {self.file_path} <<<"
        )

    def load_json(self, filename):
        if not os.path.exists(filename):
            self.get_logger().error(f"[ERROR] 파일 없음: {filename}")
            return {}
        try:
            with open(filename, "r") as f:
                data = json.load(f)
                self.get_logger().info(f"Loaded Keys: {list(data.keys())}")
                return data
        except Exception as e:
            self.get_logger().error(f"JSON Error: {e}")
            return {}

    def create_marker(self, x_list, y_list, ns, r, g, b, id_num):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.ns = ns
        marker.id = id_num
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        marker.color.a = 0.8
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b

        count = min(len(x_list), len(y_list))
        for i in range(count):
            p = Point()
            p.x = float(x_list[i])
            p.y = float(y_list[i])
            p.z = 0.0
            marker.points.append(p)
        return marker

    def create_path(self, x_list, y_list):
        path = Path()
        path.header.frame_id = "world"
        count = min(len(x_list), len(y_list))
        for i in range(count):
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.pose.position.x = float(x_list[i])
            pose.pose.position.y = float(y_list[i])
            path.poses.append(pose)
        return path

    def publish_lanes(self):
        marker_array = MarkerArray()
        idx = 0
        now = self.get_clock().now()

        for key, val in self.lane_data.items():
            xs = []
            ys = []

            if isinstance(val, list):
                for p in val:
                    if "x" in p and "y" in p:
                        xs.append(float(p["x"]))
                        ys.append(float(p["y"]))

            # 데이터가 없으면 건너뜀
            if len(xs) == 0:
                continue

            # 1. 색상 결정
            key_lower = key.lower()
            r, g, b = 1.0, 1.0, 1.0

            if "one" in key_lower:  # Lane 1 -> 빨강
                r, g, b = 1.0, 0.0, 0.0
                path_msg = self.create_path(xs, ys)
                path_msg.header.stamp = now.to_msg()
                self.pub_one.publish(path_msg)

            elif "two" in key_lower:  # Lane 2 -> 초록
                r, g, b = 0.0, 1.0, 0.0
                path_msg = self.create_path(xs, ys)
                path_msg.header.stamp = now.to_msg()
                self.pub_two.publish(path_msg)

            elif "three" in key_lower:  # Lane 3 -> 분홍
                r, g, b = 1.0, 0.0, 1.0
                path_msg = self.create_path(xs, ys)
                path_msg.header.stamp = now.to_msg()
                self.pub_three.publish(path_msg)

            # 2. 마커 생성 및 추가
            marker = self.create_marker(xs, ys, key, r, g, b, idx)
            marker.header.stamp = now.to_msg()
            marker_array.markers.append(marker)
            idx += 1

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = LaneLoader()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

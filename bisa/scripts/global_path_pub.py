#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rclpy
from ament_index_python.packages import get_package_share_directory

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# 경로 import 안전장치
try:
    from mgeo_class_defs import MGeoPlannerMap
    from dijkstra_planner import DijkstraPlanner
except ImportError:
    current_dir = os.path.dirname(os.path.abspath(__file__))
    sys.path.append(current_dir)
    from mgeo_class_defs import MGeoPlannerMap
    from dijkstra_planner import DijkstraPlanner

class GlobalPathPublisher(Node):
    def __init__(self):
        # [핵심] YAML에 있는 임의의 파라미터(problem1, 2...)를 허용하기 위한 옵션
        super().__init__("global_path_publisher", 
                         allow_undeclared_parameters=True, 
                         automatically_declare_parameters_from_overrides=True)

        self.get_logger().info("=" * 60)
        self.get_logger().info("Global Path Publisher (Multi-Scenario Mode)")
        self.get_logger().info("=" * 60)

        # QoS 설정
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.global_path_pub = self.create_publisher(Path, "/user_global_path", qos)

        # 1. HD맵 로드
        try:
            package_path = get_package_share_directory("bisa")
            load_path = os.path.join(package_path, "hdmap_data")
        except Exception as e:
            load_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../hdmap_data")

        if not os.path.exists(load_path):
            self.get_logger().error(f"HD map not found at: {load_path}")
            sys.exit(1)

        self.get_logger().info(f"Loading HD Map from: {load_path}")
        mgeo = MGeoPlannerMap.create_instance_from_json(load_path)
        self.nodes = mgeo.node_set.nodes
        self.links = mgeo.link_set.lines

        self.planner = DijkstraPlanner(
            self.nodes, self.links, enable_lane_change=True, lane_change_distance=0.2
        )

        # 2. [핵심] 파라미터 선택 로직
        # (1) 어떤 문제를 풀 것인지 키값을 가져옵니다. (기본값: problem2)
        target_key = "problem2"
        if self.has_parameter("target_problem"):
            target_key = self.get_parameter("target_problem").get_parameter_value().string_value
        
        self.get_logger().info(f"Selected Scenario: '{target_key}'")

        # (2) 해당 키에 맞는 리스트 데이터를 가져옵니다.
        input_numbers = []
        if self.has_parameter(target_key):
            input_numbers = self.get_parameter(target_key).get_parameter_value().integer_array_value
            # 혹시 정수가 아니라 문자열 등으로 들어올 경우 대비
            if not input_numbers:
                 # ROS2 파라미터 특성상 리스트 파싱이 까다로울 수 있어 예외처리
                 try:
                     val = self.get_parameter(target_key).value
                     if isinstance(val, list):
                         input_numbers = val
                 except:
                     pass
        else:
            self.get_logger().warn(f"Key '{target_key}' not found in parameters! Check path.yaml.")

        if not input_numbers:
            self.get_logger().error(f"Node sequence is empty for '{target_key}'. Aborting path gen.")
            return

        self.get_logger().info(f"Sequence: {input_numbers}")

        # 3. NODE_ 접두사 처리 및 경로 생성
        if self.nodes:
            first_key = next(iter(self.nodes.keys()))
            if isinstance(first_key, str) and str(first_key).startswith("NODE_"):
                self.node_sequence = [f"NODE_{n}" for n in input_numbers]
            else:
                self.node_sequence = input_numbers

        self.global_path_msg = None
        self.calculate_global_path()
        
        self.timer = self.create_timer(1.0, self.publish_path)

    def calculate_global_path(self):
        try:
            result = self.planner.generate_path(self.node_sequence)
            msg = Path()
            msg.header.frame_id = "world"

            for point in result["point_path"]:
                pose = PoseStamped()
                pose.header.frame_id = "world"
                pose.pose.position.x = float(point[0])
                pose.pose.position.y = float(point[1])
                pose.pose.position.z = float(point[2])
                pose.pose.orientation.w = 1.0
                msg.poses.append(pose)

            self.global_path_msg = msg
            self.get_logger().info(f"✓ Path Generated! {len(msg.poses)} points.")

        except Exception as e:
            self.get_logger().error(f"Failed to generate path: {e}")

    def publish_path(self):
        if self.global_path_msg:
            self.global_path_msg.header.stamp = self.get_clock().now().to_msg()
            self.global_path_pub.publish(self.global_path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
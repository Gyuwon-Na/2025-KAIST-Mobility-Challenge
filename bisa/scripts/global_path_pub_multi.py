#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory

from mgeo_class_defs import MGeoPlannerMap
from dijkstra_planner import DijkstraPlanner


class GlobalPathPublisherMulti(Node):
    def __init__(self):
        super().__init__("global_path_publisher_multi")

        # 파라미터 선언
        self.declare_parameter("cav_id", 1)
        self.declare_parameter("node_sequence", [21, 51, 46, 41, 43, 9, 56, 59, 18, 21])
        # ★ [추가] RViz 슬롯 파라미터
        self.declare_parameter("rviz_slot", -1)

        self.cav_id = self.get_parameter("cav_id").get_parameter_value().integer_value
        node_seq_param = (
            self.get_parameter("node_sequence")
            .get_parameter_value()
            .integer_array_value
        )
        self.node_sequence_input = list(node_seq_param)
        self.rviz_slot = (
            self.get_parameter("rviz_slot").get_parameter_value().integer_value
        )

        # self.get_logger().info("=" * 60)
        # self.get_logger().info(
        #     f"Global Path Publisher - CAV{self.cav_id:02d} (RViz Slot: {self.rviz_slot})"
        # )
        # self.get_logger().info(f"Node Sequence: {self.node_sequence_input}")
        # self.get_logger().info("=" * 60)

        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.global_path_pub = self.create_publisher(Path, "/user_global_path", qos)

        self.rviz_global_pub = None
        if self.rviz_slot >= 0:
            rviz_topic = f"/viz/slot{self.rviz_slot}/global_path"
            self.rviz_global_pub = self.create_publisher(Path, rviz_topic, qos)
            # self.get_logger().info(f"RViz topic: {rviz_topic}")

        # HDMap 경로
        try:
            package_share = get_package_share_directory("bisa")
            load_path = os.path.join(package_share, "hdmap_data")
            # self.get_logger().info(f"Loading HD Map from: {load_path}")
        except:
            workspace_src = os.path.join(
                os.path.expanduser("~"), "Mobility_Challenge_Simulator", "src", "bisa"
            )
            load_path = os.path.join(workspace_src, "hdmap_data")
            # self.get_logger().warn(f"Using source directory: {load_path}")

        if not os.path.exists(load_path):
            # self.get_logger().error(f"HD map not found at: {load_path}")
            sys.exit(1)

        mgeo = MGeoPlannerMap.create_instance_from_json(load_path)
        self.nodes = mgeo.node_set.nodes
        self.links = mgeo.link_set.lines

        # self.get_logger().info(
        #     f"Loaded: {len(self.nodes)} nodes, {len(self.links)} links"
        # )

        self.planner = DijkstraPlanner(
            self.nodes, self.links, enable_lane_change=True, lane_change_distance=0.2
        )

        # 노드 시퀀스 변환 (문자열 키 확인)
        if self.nodes:
            first_key = next(iter(self.nodes.keys()))
            if isinstance(first_key, str) and str(first_key).startswith("NODE_"):
                self.node_sequence = [f"NODE_{n}" for n in self.node_sequence_input]
            else:
                self.node_sequence = self.node_sequence_input

        self.global_path_msg = None
        self.calculate_global_path()
        self.timer = self.create_timer(0.1, self.publish_path)

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
            # self.get_logger().info(
            #     f"✔ CAV{self.cav_id:02d} Path Ready! {len(msg.poses)} points"
            # )

        except Exception as e:
            # self.get_logger().error(f"Failed to generate path: {e}")
            import traceback

            traceback.print_exc()
            sys.exit(1)

    def publish_path(self):
        if self.global_path_msg:
            self.global_path_msg.header.stamp = self.get_clock().now().to_msg()
            self.global_path_pub.publish(self.global_path_msg)

            if self.rviz_global_pub:
                self.rviz_global_pub.publish(self.global_path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathPublisherMulti()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

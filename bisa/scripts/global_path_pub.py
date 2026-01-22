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

class GlobalPathPublisher(Node):
    def __init__(self):
        super().__init__('global_path_publisher')
        
        self.get_logger().info("="*60)
        self.get_logger().info("Global Path Publisher (Frame: world)")
        self.get_logger().info("="*60)
        
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.global_path_pub = self.create_publisher(Path, '/user_global_path', qos)
        
        # HDMap 경로 (install 우선)
        try:
            package_share = get_package_share_directory('bisa')
            load_path = os.path.join(package_share, 'hdmap_data')
            self.get_logger().info(f"Loading HD Map from: {load_path}")
        except:
            # Fallback: 소스 디렉토리
            workspace_src = os.path.join(os.path.expanduser('~'), 'Mobility_Challenge_Simulator', 'src', 'bisa')
            load_path = os.path.join(workspace_src, 'hdmap_data')
            self.get_logger().warn(f"Using source directory: {load_path}")
        
        if not os.path.exists(load_path):
            self.get_logger().error(f"HD map not found at: {load_path}")
            sys.exit(1)
        
        mgeo = MGeoPlannerMap.create_instance_from_json(load_path)
        self.nodes = mgeo.node_set.nodes
        self.links = mgeo.link_set.lines
        
        self.get_logger().info(f"Loaded: {len(self.nodes)} nodes, {len(self.links)} links")
        
        self.planner = DijkstraPlanner(
            self.nodes, self.links, 
            enable_lane_change=True, 
            lane_change_distance=0.2
        )
        
        # 노드 시퀀스
        input_numbers = [21, 51, 46, 40, 63, 34, 27, 28, 31, 1, 3, 6, 7, 10, 9, 56, 59, 18, 21]
        
        if self.nodes:
            first_key = next(iter(self.nodes.keys()))
            if isinstance(first_key, str) and str(first_key).startswith("NODE_"):
                self.node_sequence = [f"NODE_{n}" for n in input_numbers]
            else:
                self.node_sequence = input_numbers
        
        self.global_path_msg = None
        self.calculate_global_path()
        self.timer = self.create_timer(0.1, self.publish_path)

    def calculate_global_path(self):
        try:
            result = self.planner.generate_path(self.node_sequence)
            
            msg = Path()
            msg.header.frame_id = 'world'
            
            for point in result['point_path']:
                pose = PoseStamped()
                pose.header.frame_id = 'world'
                pose.pose.position.x = float(point[0])
                pose.pose.position.y = float(point[1])
                pose.pose.position.z = float(point[2])
                pose.pose.orientation.w = 1.0
                msg.poses.append(pose)
            
            self.global_path_msg = msg
            self.get_logger().info(f"✓ Path Ready! {len(msg.poses)} points (Frame: world)")
            
        except Exception as e:
            self.get_logger().error(f"Failed to generate path: {e}")
            import traceback
            traceback.print_exc()
            sys.exit(1)
            
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

if __name__ == '__main__':
    main()

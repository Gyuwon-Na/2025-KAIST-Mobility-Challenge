#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import math


class ObstacleRelay(Node):
    def __init__(self):
        super().__init__("obstacle_relay")

        # [수정 1] 실제 토픽 이름 반영: "_pose" 제거! (/HV_19, /HV_20 ...)
        self.hv_topic_names = [f"/HV_{i}" for i in range(19, 37)]

        self.ROI_FRONT = 30.0
        self.ROI_REAR = 15.0
        self.VEL_STATIC_THRES = 0.1
        self.VEL_SLOW_THRES = 1.5

        # 내 차 위치 (브릿지에서 /CAV_01 -> /Ego_pose로 바꿔서 줌)
        self.ego_pose = None
        self.create_subscription(
            PoseStamped, "/Ego_pose", self.ego_callback, qos_profile_sensor_data
        )

        # 장애물 구독
        self.subs = []
        for topic in self.hv_topic_names:
            self.subs.append(
                self.create_subscription(
                    PoseStamped,
                    topic,
                    lambda msg, t=topic: self.hv_callback(msg, t),
                    qos_profile_sensor_data,
                )
            )

        self.marker_pub = self.create_publisher(MarkerArray, "/obstacles_markers", 10)
        self.obstacles = {}
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Obstacle Relay Started (Real Topic: /HV_xx)")

    def ego_callback(self, msg):
        self.ego_pose = msg.pose

    def hv_callback(self, msg, topic_name):
        # [디버깅] 데이터 들어오면 로그 출력
        self.get_logger().info(f"✅ Data: {topic_name}", throttle_duration_sec=2.0)

        current_time = self.get_clock().now().nanoseconds / 1e9
        x = msg.pose.position.x
        y = msg.pose.position.y
        # PDF 변칙 규칙: z가 Yaw
        yaw = msg.pose.orientation.z

        velocity = 0.0
        if topic_name in self.obstacles:
            prev = self.obstacles[topic_name]
            dt = current_time - prev["time"]
            if dt > 0.001:
                dist = math.sqrt((x - prev["pos"][0]) ** 2 + (y - prev["pos"][1]) ** 2)
                calc_vel = dist / dt
                if calc_vel < 10.0:
                    velocity = (prev["vel"] * 0.4) + (calc_vel * 0.6)
                else:
                    velocity = prev["vel"]

        if velocity < self.VEL_STATIC_THRES:
            obs_type = "STATIC"
        elif velocity < self.VEL_SLOW_THRES:
            obs_type = "SLOW"
        else:
            obs_type = "FAST"

        self.obstacles[topic_name] = {
            "pos": (x, y),
            "yaw": yaw,
            "time": current_time,
            "vel": velocity,
            "type": obs_type,
        }

    def timer_callback(self):
        ego_x, ego_y = 0.0, 0.0
        if self.ego_pose is not None:
            ego_x = self.ego_pose.position.x
            ego_y = self.ego_pose.position.y
        else:
            self.get_logger().warn(
                "Waiting for /Ego_pose...", throttle_duration_sec=2.0
            )

        marker_array = MarkerArray()
        id_cnt = 0

        for topic, data in self.obstacles.items():
            ox, oy = data["pos"]

            # 거리 필터
            dist = math.sqrt((ox - ego_x) ** 2 + (oy - ego_y) ** 2)
            if dist > self.ROI_FRONT:
                continue

            marker = Marker()
            # [수정 2] 실제 데이터의 frame_id인 "world" 사용
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "surrounding_cars"
            marker.id = id_cnt
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = ox
            marker.pose.position.y = oy
            marker.pose.position.z = 0.15

            cy = math.cos(data["yaw"] * 0.5)
            sy = math.sin(data["yaw"] * 0.5)
            marker.pose.orientation.w = cy
            marker.pose.orientation.z = sy

            marker.scale.x = 0.33
            marker.scale.y = 0.15
            marker.scale.z = 0.2

            if data["type"] == "STATIC":
                marker.color.r = 0.5
                marker.color.g = 0.5
                marker.color.b = 0.5
            elif data["type"] == "SLOW":
                marker.color.r = 1.0
                marker.color.g = 0.8
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            marker.color.a = 0.9

            marker_array.markers.append(marker)
            id_cnt += 1

        self.marker_pub.publish(marker_array)


def main():
    rclpy.init()
    node = ObstacleRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

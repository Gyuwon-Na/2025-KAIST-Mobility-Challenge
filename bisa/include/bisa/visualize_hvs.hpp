#ifndef BISA_VISUALIZE_HVS_HPP_
#define BISA_VISUALIZE_HVS_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <functional>
#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float32.hpp"

struct ObstacleData
{
    double x, y, yaw, time, vel;
    std::string type;
};

class ObstacleRelay : public rclcpp::Node
{
public:
    ObstacleRelay();

private:
    void update_dynamic_threshold();

    void ego_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void hv_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const std::string &topic_name);

    void timer_callback();

    double roi_front_;
    double vel_static_thres_;
    double vel_slow_thres_;

    double fast_vel = 0.0;
    double slow_vel = 0.0;

    geometry_msgs::msg::PoseStamped::SharedPtr ego_pose_;
    std::map<std::string, ObstacleData> obstacles_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ego_sub_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> hv_subs_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_slow_vel_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_fast_vel_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
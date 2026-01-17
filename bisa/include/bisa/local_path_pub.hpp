/**
 * @file local_path_pub.hpp
 * @brief Safety-First Local Path Header (Fixed Indices + Zone Safety)
 */

#ifndef BISA_LOCAL_PATH_PUB_HPP_
#define BISA_LOCAL_PATH_PUB_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>
#include <map>
#include <cmath>
#include <string>

#include "bisa/msg/lap_info.hpp"

namespace bisa
{

    enum class LaneID
    {
        LANE_1 = 0,
        LANE_2 = 1,
        LANE_3 = 2,
        NONE = -1
    };

    struct PathPoint
    {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
    };

    struct ObstacleInfo
    {
        double x = 0.0;
        double y = 0.0;
        double vx = 0.0;
        double vy = 0.0;
        LaneID lane = LaneID::NONE;
        double last_seen_sec = 0.0;
    };

    class LocalPathPubCpp : public rclcpp::Node
    {
    public:
        LocalPathPubCpp();
        virtual ~LocalPathPubCpp() = default;

    private:
        // ========================================================================
        // CORE VARIABLES
        // ========================================================================
        const int FIXED_MERGE_IDX = 1500;
        const int FIXED_SPLIT_IDX = 2281;
        const int MERGE_ZONE_THRESHOLD = 40; // +/- 40점 이내는 합류/분기 구간으로 간주

        // ========================================================================
        // CORE FUNCTIONS
        // ========================================================================
        int find_closest_idx_forward(int lane_idx, double x, double y);

        // Lane 판별
        LaneID get_lane_at(double x, double y);
        double get_dist(double x1, double y1, double x2, double y2);
        double normalize_angle(double angle);

        // Zone 판별
        bool in_merge_zone(int idx);
        bool in_merge_gate(int idx);

        // ========================================================================
        // CALLBACKS & LOOPS
        // ========================================================================
        void control_loop();
        void process_lane_path(int lane_idx);
        void publish_lap_info();
        void obstacle_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        // ========================================================================
        // MEMBERS
        // ========================================================================
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_lane_[3];
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_obs_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_env_slow_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_env_fast_;

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_local_path_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_target_vel_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_; // 디버그용
        rclcpp::Publisher<bisa::msg::LapInfo>::SharedPtr lap_info_pub_;

        rclcpp::TimerBase::SharedPtr timer_;

        // Data
        nav_msgs::msg::Path::SharedPtr lane_paths_[3];
        std::vector<PathPoint> processed_lanes_[3];
        std::map<int, ObstacleInfo> obstacles_;

        // Ego State
        bool pose_received_ = false;
        double ego_x_ = 0.0, ego_y_ = 0.0, ego_yaw_ = 0.0, ego_speed_ = 0.0;
        double prev_pose_time_ = 0.0;

        // Index tracking
        int last_closest_idx_ = -1;
        double env_slow_vel_ = 0.0;
        double env_fast_vel_ = 0.0;

        // Lap Info Variables
        int lap_count_ = 0;
        double total_distance_ = 0.0;
        rclcpp::Time lap_start_time_;
        // int prev_track_idx_ = 0; // Lane 2 기준 이전 인덱스
    };

} // namespace bisa

#endif // BISA_LOCAL_PATH_PUB_HPP_
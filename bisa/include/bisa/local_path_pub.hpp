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

    struct MergeGapInfo
    {
        bool gap_available = false;
        double front_dist = 999.0;    // 합류 지점 기준 앞차까지 거리
        double rear_dist = 999.0;     // 합류 지점 기준 뒷차까지 거리
        double front_vel = 0.0;       // 앞차 속도
        double rear_vel = 0.0;        // 뒷차 속도
        double gap_size = 0.0;        // gap 크기
        double recommended_vel = 1.5; // 권장 합류 속도
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
        const int MERGE_ZONE_THRESHOLD = 100; // +/- 40점 이내는 합류/분기 구간으로 간주

        // Merge Zone parameters
        const double MIN_MERGE_GAP = 0.5;   // 최소 합류 gap (m)
        const double SAFE_MERGE_GAP = 0.8;  // 안전 합류 gap (m)
        const double MERGE_LOOKAHEAD = 3.0; // 합류 지점 전후 탐색 범위 (m)

        // 차량 크기
        const double VEH_F = 0.17;  // Ego 앞범퍼까지
        const double VEH_R = 0.16;  // Ego 뒷범퍼까지
        const double VEH_W = 0.075; // Ego 측면까지

        // 상대 차량도 비슷한 크기라 가정
        const double OBS_F = 0.17;
        const double OBS_R = 0.16;
        const double OBS_W = 0.075;

        // 안전 마진 (범퍼 간 최소 거리)
        const double MARGIN_FRONT = 0.5;
        const double MARGIN_REAR = 0.4;
        const double MARGIN_LATERAL = 0.15;

        // 직사각형 크기 = 내 차량 + 상대 차량 + 마진
        const double SAFE_FRONT = VEH_F + OBS_R + MARGIN_FRONT;     // 0.17 + 0.16 + 0.25 = 0.58m
        const double SAFE_REAR = VEH_R + OBS_F + MARGIN_REAR;       // 0.16 + 0.17 + 0.20 = 0.53m
        const double SAFE_LATERAL = VEH_W + OBS_W + MARGIN_LATERAL; // 0.075 + 0.075 + 0.15 = 0.25m

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
        bool in_split_gate(int idx);

        // 합류 Gap 분석
        MergeGapInfo analyze_gap(LaneID target_lane);

        // Lap Info
        int count_lap_idx(int lane_idx, double x, double y);
        void publish_lap_info();

        // ========================================================================
        // CALLBACKS & LOOPS
        // ========================================================================
        void control_loop();
        void process_lane_path(int lane_idx);
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
        rclcpp::Publisher<bisa::msg::LapInfo>::SharedPtr pub_lap_info_;

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
        int prev_track_idx_ = 0; // Lane 2 기준 이전 인덱스
    };

} // namespace bisa

#endif // BISA_LOCAL_PATH_PUB_HPP_
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
        const int MERGE_THRESHOLD = 50; // +/- 40점 이내는 합류/분기 구간으로 간주
        const int SPLIT_THRESHOLD = 50;

        // Merge Zone parameters
        const double MIN_MERGE_GAP = 0.5;   // 최소 합류 gap (m)
        const double SAFE_MERGE_GAP = 0.8;  // 안전 합류 gap (m)
        const double MERGE_LOOKAHEAD = 3.0; // 합류 지점 전후 탐색 범위 (m)
        const double BOOST_DIST = 1.0;      // 앞차와 거리 좁히기 위한 구간 (m)

        // 최소 간격 제한 (하드 코딩)
        const double MIN_FRONT_GAP = 0.3; // 앞차 최소 간격
        const double MIN_REAR_GAP = 0.3;  // 뒷차 최소 간격
        const double COMFORT_FRONT = 0.6; // 앞차 편안한 간격
        const double COMFORT_REAR = 0.5;  // 뒷차 편안한 간격
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
        const double MARGIN_REAR = 0.3;
        const double MARGIN_LATERAL = 0.17;

        // Merge Zone 최소 간격 기준
        const double MERGE_ZONE_MIN_FRONT_GAP = 0.5; // 앞차와 최소 간격 (m)
        const double MERGE_ZONE_MIN_REAR_GAP = 0.2;  // 뒷차와 최소 간격 (m)

        // 직사각형 크기 = 내 차량 + 상대 차량 + 마진
        const double SAFE_FRONT = VEH_F + OBS_R + MARGIN_FRONT;     // 0.17 + 0.16 + 0.25 = 0.58m
        const double SAFE_REAR = VEH_R + OBS_F + MARGIN_REAR;       // 0.16 + 0.17 + 0.20 = 0.53m
        const double SAFE_LATERAL = VEH_W + OBS_W + MARGIN_LATERAL; // 0.075 + 0.075 + 0.15 = 0.3m
        const int LANE3_FORWARD_OFFSET = 10;                        // 튜닝용
        // ========================================================================
        // CORE FUNCTIONS
        // ========================================================================
        int find_closest_idx_forward(int lane_idx, double x, double y);
        int find_closest_idx_forward_heading(int lane_idx, double x, double y, double yaw);

        // Lane 판별
        LaneID get_lane_at(double x, double y);
        double get_dist(double x1, double y1, double x2, double y2);
        double normalize_angle(double angle);

        // Zone 판별
        bool is_initial_merge_safe();
        bool in_merge_zone(int idx);
        bool in_merge_gate(int idx);
        bool in_split_gate(int idx);
        // Merge Zone 속도 계산
        double calculate_merge_zone_velocity(LaneID current_lane, double base_speed);
        // 합류 Gap 분석
        MergeGapInfo analyze_split_gap();
        MergeGapInfo analyze_merge_gap();

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
        int find_closest_forward_idx_in_lane3();
        // Ego State
        bool pose_received_ = false;
        double ego_x_ = 0.0, ego_y_ = 0.0, ego_yaw_ = 0.0, ego_speed_ = 0.0;
        double prev_pose_time_ = 0.0;

        const std::set<int> FAST_HV_IDS = {0, 1, 2, 3};
        const std::set<int> SLOW_HV_IDS = {4, 5, 6, 7, 8, 9, 10, 11};
        bool is_fast_hv(int id);
        bool is_slow_hv(int id);
        // Index tracking
        int last_closest_idx_ = -1;       // lane2 전용
        int last_closest_idx_lane3_ = -1; // lane3 전용 (기존 last_closest_idx_)
        double env_slow_vel_ = 0.0;
        double env_fast_vel_ = 0.0;

        // Lap Info Variables
        int lap_count_ = 0;
        double total_distance_ = 0.0;
        rclcpp::Time lap_start_time_;
        int prev_track_idx_ = 0; // Lane 2 기준 이전 인덱스

        // 멤버 변수 추가 (hpp 파일)
        LaneID current_following_lane_ = LaneID::LANE_2; // 현재 추종 차선
        bool initial_merge_done_ = false;                // 초기 합류 완료 여부

        LaneID current_lane_id = LaneID::LANE_2; // 시작은 무조건 Lane 2
        bool initial_lane_change_done_ = false;  // 차선 변경 완료 여부
        double lane_change_cooldown_ = 0.0;      // 시작 직후 급격한 변경 방지용

        // 차선 변경 판단 임계값
        bool is_lane_change_safe(LaneID target_lane);
        const double LANE_CHANGE_LOOKAHEAD = 0.7;  // 전방 1.5m 여유
        const double LANE_CHANGE_LOOKBEHIND = 0.7; // 후방 1.0m 여유
        const double LANE_CHANGE_COOLDOWN = 2.0;   // 변경 후 최소 유지 시간
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_safety_zone_marker_;
        void publish_safety_zone_visual(bool is_safe);
    };

} // namespace bisa

#endif // BISA_LOCAL_PATH_PUB_HPP_
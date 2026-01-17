/**
 * @file local_path_pub.hpp
 * @brief Safety-First Local Path Publisher Header
 * @version 4.0
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

namespace bisa
{

    // ============================================================================
    // ENUMS
    // ============================================================================

    enum class LaneID
    {
        LANE_1 = 0,
        LANE_2 = 1,
        LANE_3 = 2,
        NONE = -1
    };

    // ============================================================================
    // STRUCTS
    // ============================================================================

    struct PathPoint
    {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        double s = 0.0; // arc length (optional)
    };

    struct ObstacleInfo
    {
        int id = -1;
        double x = 0.0;
        double y = 0.0;
        double vx = 0.0;
        double vy = 0.0;
        double speed = 0.0;
        LaneID lane = LaneID::NONE;
        double last_seen_sec = 0.0;
    };

    // ============================================================================
    // CLASS
    // ============================================================================

    class LocalPathPubCpp : public rclcpp::Node
    {
    public:
        LocalPathPubCpp();

    private:
        // ========================================================================
        // CONSTANTS
        // ========================================================================

        // Speed settings
        static constexpr double CRUISE_SPEED = 1.5; // 일반 주행 속도
        static constexpr double SLOW_SPEED = 0.8;   // 합류/분기 구간 속도
        static constexpr double MIN_SPEED = 0.0;

        // TTC threshold
        static constexpr double TTC_THRESHOLD = 3.0; // 3초 이내 충돌 위험시 정지

        // Path settings
        static constexpr int PATH_POINTS = 80;        // Local path point 수
        static constexpr double LOOKAHEAD_DIST = 2.0; // 전방 주시 거리 (m)

        // ========================================================================
        // ROS INTERFACES
        // ========================================================================

        // Subscribers
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_lane_[3];
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obs_sub_;

        // Publishers
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_vel_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;

        // ========================================================================
        // STATE VARIABLES
        // ========================================================================

        // Lane data
        nav_msgs::msg::Path::SharedPtr lane_paths_[3];
        std::vector<PathPoint> processed_lanes_[3];

        // Ego state
        double ego_x_ = 0.0;
        double ego_y_ = 0.0;
        double ego_yaw_ = 0.0;
        double ego_speed_ = 0.0;
        double prev_pose_time_ = 0.0;
        bool pose_received_ = false;

        // Index tracking (역주행 방지용)
        int last_idx_[3] = {-1, -1, -1};

        // Topology analysis
        bool topology_analyzed_ = false;
        int merge_start_idx_ = -1; // 합류 시작 인덱스
        int merge_end_idx_ = -1;   // 분기(합류 종료) 인덱스

        // Obstacles
        std::map<int, ObstacleInfo> obstacles_;

        // ========================================================================
        // UTILITY FUNCTIONS
        // ========================================================================

        double get_dist(double x1, double y1, double x2, double y2);
        double normalize_angle(double angle);

        // ========================================================================
        // CORE FUNCTIONS
        // ========================================================================

        /**
         * @brief 진행 방향 기반 인덱스 검색 (역주행 방지)
         */
        int find_closest_idx_forward(int lane_idx, double x, double y);

        /**
         * @brief 합류/분기 구간 분석 (1회 실행)
         */
        void analyze_topology();

        /**
         * @brief TTC 계산
         */
        double calculate_ttc(const ObstacleInfo &obs);

        /**
         * @brief 합류 구간 안전성 체크 (Lane 2 HV)
         */
        bool check_merge_safety();

        /**
         * @brief 분기 구간 안전성 체크 (Lane 3 HV)
         */
        bool check_split_safety();

        /**
         * @brief 위치 기반 Lane 판별
         */
        LaneID get_lane_at(double x, double y);

        // ========================================================================
        // CALLBACKS
        // ========================================================================

        void control_loop();
        void process_lane_path(int lane_idx);
        void obstacle_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    };

} // namespace bisa

#endif // BISA_LOCAL_PATH_PUB_HPP_
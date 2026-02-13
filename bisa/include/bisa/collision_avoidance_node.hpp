/**
 * @file collision_avoidance_node.hpp
 * @brief 충돌 방지 노드 v36.0 (Return to Start 기능 통합)
 */

#ifndef BISA_COLLISION_AVOIDANCE_NODE_HPP
#define BISA_COLLISION_AVOIDANCE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>  // [NEW]
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <bisa/msg/lap_info.hpp>

#include <cmath>
#include <vector>
#include <map>
#include <string>
#include <array>
#include <deque>

namespace bisa
{

    struct Point2D
    {
        double x, y;
        Point2D() : x(0), y(0) {}
        Point2D(double x_, double y_) : x(x_), y(y_) {}
        double dist_to(const Point2D &o) const { return std::sqrt((x - o.x) * (x - o.x) + (y - o.y) * (y - o.y)); }
    };

    struct HVState
    {
        Point2D position;
        Point2D rear_point; // HV 뒤쪽 끝점 (글로벌 패스 위)
        double yaw = 0;
        double speed = 0;
        bool valid = false;
        std::vector<Point2D> global_path;
        std::vector<Point2D> dynamic_path;
        size_t current_waypoint = 0;
    };

    struct CAVState
    {
        Point2D position;
        Point2D start_position; // [NEW] 시작 위치 저장
        double yaw = 0;
        double speed = 0;
        std::vector<Point2D> local_path;
        std::vector<Point2D> global_path;
        std::vector<Point2D> dynamic_path;
        std::vector<Point2D> rear_path;
        std::vector<Point2D> full_safety_path;
        size_t current_global_waypoint = 0;
        int remaining_waypoints = 9999;
        double current_velocity = 0;
        bool stop_flag = false;
        bool slow_flag = false;
        bool passed_entry = false;
        bool in_roundabout = false;
        bool hv_danger = false;
        double current_curvature = 0;
        bool correction_done = false;       // 정차 보정 완료 여부
        double correction_start_time = 0.0; // 보정 시작 시간
        
        bool has_start_pose = false;  // [NEW] 시작 위치 초기화 여부
        bool at_start_position = false; // [NEW] 시작 위치 복귀 완료 여부
    };

    class CollisionAvoidanceNode : public rclcpp::Node
    {
    public:
        CollisionAvoidanceNode();
        ~CollisionAvoidanceNode() = default;

    private:
        static constexpr double ROUNDABOUT_CENTER_X = 0.5741;
        static constexpr double ROUNDABOUT_CENTER_Y = -0.2909;
        static constexpr double ROUNDABOUT_RADIUS = 1.9;

        // ★★★ 전방 경로 최소 길이 ★★★
        static constexpr double MIN_PATH_LENGTH = 0.98;
        static constexpr double PATH_OVERLAP_DIST = 0.10;

        static constexpr double CURVATURE_PATH_GAIN = 2.0;
        static constexpr double MAX_CURVE_EXTRA = 1.5;
        static constexpr double CURVATURE_THRESHOLD = 0.3;

        // ★★★ 후방 경로 길이 ★★★
        static constexpr double REAR_PATH_LENGTH = 0.4;

        // ★★★ SAFETY PATH 추가 길이 ★★★
        static constexpr int SAFETY_PATH_EXTRA_POINTS = 30;

        // ★★★ HV 뒤쪽 끝점 오프셋 (글로벌 패스 위에서 뒤로 이동) ★★★
        static constexpr double HV_REAR_OFFSET = 0.25;

        // ★★★ HV 로컬패스 최대 길이 ★★★
        static constexpr double HV_MAX_PATH_LENGTH = 1.45;

        static constexpr double PATH_CROSS_DIST = 0.10;
        static constexpr double COLLISION_RADIUS = 0.15;
        static constexpr double SAFE_STOP_DIST = 0.55;
        static constexpr double EARLY_SLOW_DIST = 1.3;

        static constexpr double NODE_STOP_DIST = 0.55;

        static constexpr int LOOKAHEAD_POINTS = 500;
        static constexpr double MIN_VELOCITY = 0.4;
        static constexpr double ACCEL_RATE = 0.08;
        static constexpr double DECEL_RATE = 0.12;

        // [NEW] Return to Start 관련 상수
        static constexpr double START_POSITION_TOLERANCE = 0.15; // m
        static constexpr double SHUTDOWN_CHECK_INTERVAL = 0.1;   // s

        std::vector<int> hv_ids_;
        std::vector<int> cav_ids_;
        std::vector<int> high_priority_cavs_;

        std::map<int, Point2D> node_positions_;
        std::map<int, int> cav_entry_nodes_;

        std::map<int, HVState> hv_states_;
        std::map<int, CAVState> cav_states_;
        std::map<int, geometry_msgs::msg::Accel> last_accel_;
        std::map<int, std::string> hv_debug_info_;
        std::map<int, std::array<double, 3>> cav_colors_;
        std::vector<Point2D> active_collision_zones_;
        std::map<int, double> prev_dist_to_entry_;

        std::map<int, std::deque<std::tuple<double, double, double>>> hv_history_;

        // [NEW] 복귀 명령 구독자
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr return_command_sub_;

        std::map<int, rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> hv_global_path_subs_;
        std::map<int, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> hv_pose_subs_;
        std::map<int, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> cav_pose_subs_;
        std::map<int, rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> cav_path_subs_;
        std::map<int, rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> cav_global_path_subs_;
        std::map<int, rclcpp::Subscription<bisa::msg::LapInfo>::SharedPtr> cav_lap_subs_;
        std::map<int, rclcpp::Subscription<geometry_msgs::msg::Accel>::SharedPtr> cav_accel_subs_;
        std::map<int, rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr> accel_pubs_;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hv_marker_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cav_marker_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr zone_marker_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr safety_path_marker_pub_;

        std::map<int, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> hv_local_path_pubs_;

        rclcpp::TimerBase::SharedPtr control_timer_;
        rclcpp::TimerBase::SharedPtr vis_timer_;
        rclcpp::TimerBase::SharedPtr debug_timer_;

        // [NEW] Shutdown 상태 관리 변수
        bool shutdown_requested_ = false;
        double shutdown_start_time_ = 0.0;
        bool all_at_start_logged_ = false;

        void init_node_positions();
        void init_hv_global_paths();
        void init_subscribers();
        void init_publishers();
        void init_timers();

        // [NEW] 복귀 명령 콜백 및 핸들러
        void return_command_callback(const std_msgs::msg::Bool::SharedPtr msg);
        void handle_shutdown();
        void check_shutdown_complete();
        bool is_cav_at_start(int cav_id);

        void hv_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int hv_id);
        void hv_global_path_callback(const nav_msgs::msg::Path::SharedPtr msg, int hv_id);
        void cav_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int cav_id);
        void local_path_callback(const nav_msgs::msg::Path::SharedPtr msg, int cav_id);
        void cav_global_path_callback(const nav_msgs::msg::Path::SharedPtr msg, int cav_id);
        void lap_info_callback(const bisa::msg::LapInfo::SharedPtr msg, int cav_id);
        void accel_callback(const geometry_msgs::msg::Accel::SharedPtr msg, int cav_id);

        void update_hv_dynamic_path(int hv_id);
        void update_cav_dynamic_path(int cav_id);
        void update_cav_rear_path(int cav_id);
        void update_cav_full_safety_path(int cav_id);
        double calculate_path_curvature(const std::vector<Point2D> &path, int start_idx, int window);

        // ★★★ 글로벌 패스에서 뒤로 distance만큼 이동한 점 찾기 ★★★
        Point2D get_point_behind_on_path(const std::vector<Point2D> &path, size_t current_idx, double distance) const;

        std::tuple<bool, Point2D, int, int> find_path_overlap(
            const std::vector<Point2D> &path1, const std::vector<Point2D> &path2) const;
        bool check_path_overlap(const std::vector<Point2D> &path1, const std::vector<Point2D> &path2) const;
        bool is_path_clear_of_zone(const std::vector<Point2D> &path, const Point2D &zone, double radius) const;

        int determine_priority(int my_id, int other_id, const Point2D &zone, int my_idx, int other_idx) const;

        std::tuple<bool, int, bool> check_hv_collision(int cav_id);
        std::tuple<bool, int, bool> check_cav_collision(int cav_id);

        Point2D get_roundabout_center() const { return Point2D(ROUNDABOUT_CENTER_X, ROUNDABOUT_CENTER_Y); }
        bool is_high_priority(int cav_id) const
        {
            return std::find(high_priority_cavs_.begin(), high_priority_cavs_.end(), cav_id) != high_priority_cavs_.end();
        }
        double get_current_time() const { return this->now().seconds(); }
        std::string format_cav_id(int id) const { return (id < 10) ? "0" + std::to_string(id) : std::to_string(id); }

        double smooth_velocity(int cav_id, double target);
        void control_loop();

        void publish_visualizations();
        void publish_hv_markers();
        void publish_cav_markers();
        void publish_zone_markers();
        void publish_safety_paths();
        void publish_hv_local_paths();

        void log_hv_debug();
        void load_parameters();
    };

}

#endif

/**
 * @file frenet_planner.cpp
 * @brief Complete Overtake Planner for RC-Scale Autonomous Vehicle (v6.1)
 *
 * v6.1 Changes:
 * - FIX: Front detection now checks LANE (not just distance)
 * - FIX: Added rear threat acceleration
 * - PATH-BASED collision prediction for corners
 * - Corner detection with Lane 1 avoidance
 */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>
#include <map>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

// ============================================================================
// ENUMS AND STRUCTS
// ============================================================================

enum class LaneID
{
    LANE_1 = 0,
    LANE_2 = 1,
    LANE_3 = 2,
    NONE = -1
};

enum class DrivingState
{
    CRUISE,
    PREPARE_OVERTAKE,
    LANE_CHANGING,
    EMERGENCY_ACCEL, // NEW: 후방 위협 시 가속
    EMERGENCY_BRAKE
};

struct ObstacleInfo
{
    int id;
    double x, y;
    double vx, vy;
    double speed;
    LaneID lane;

    // Body frame coordinates
    double rel_x;  // Forward positive
    double rel_y;  // Left positive
    double rel_vx; // Approaching negative

    double last_seen_sec;
    bool is_static;
};

// Surrounding detection result
struct SurroundingStatus
{
    // Front zone (SAME lane only)
    bool front_blocked = false;
    bool front_danger = false;
    bool front_critical = false;
    double front_dist = 999.0;
    double front_speed = 0.0;
    bool front_is_static = false;
    int front_id = -1;

    // Rear zone (SAME lane only)
    bool rear_danger = false;
    double rear_dist = 999.0;
    double rear_speed = 0.0;
    double rear_ttc = 999.0;

    // Left zone (blind spot)
    bool left_clear = true;
    double left_dist = 999.0;

    // Right zone (blind spot)
    bool right_clear = true;
    double right_dist = 999.0;

    // Path-based collision
    bool path_collision = false;
    double path_collision_dist = 999.0;
};

// ============================================================================
// MAIN CLASS
// ============================================================================

class FrenetPlanner : public rclcpp::Node
{
public:
    FrenetPlanner() : Node("frenet_planner")
    {
        RCLCPP_INFO(this->get_logger(), "============================================");
        RCLCPP_INFO(this->get_logger(), "Frenet Planner v6.1 - Lane-Based Detection");
        RCLCPP_INFO(this->get_logger(), "============================================");

        declare_parameters();
        load_parameters();

        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&FrenetPlanner::param_callback, this, std::placeholders::_1));

        setup_ros_interfaces();
        reset_state();

        timer_ = this->create_wall_timer(20ms, std::bind(&FrenetPlanner::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Planner initialized.");
    }

private:
    // ========================================================================
    // VEHICLE DIMENSIONS (RC Scale)
    // ========================================================================

    static constexpr double VEHICLE_LENGTH = 0.33;
    static constexpr double VEHICLE_WIDTH = 0.15;
    static constexpr double VEHICLE_LENGTH_F = 0.17;
    static constexpr double VEHICLE_LENGTH_R = 0.16;
    static constexpr double VEHICLE_HALF_WIDTH = 0.075;

    // Obstacle dimensions
    static constexpr double OBS_LENGTH = 0.33;
    static constexpr double OBS_WIDTH = 0.15;
    static constexpr double OBS_HALF_LENGTH = 0.17;
    static constexpr double OBS_HALF_WIDTH = 0.075;

    // ========================================================================
    // DETECTION ZONES
    // ========================================================================

    static constexpr double FRONT_ZONE_LENGTH = 0.6;
    // ★ FIX: 전방 감지 폭을 차선 폭의 절반으로 제한 (인접 차선 감지 방지)
    static constexpr double FRONT_ZONE_WIDTH = 0.12; // 0.20 → 0.12

    static constexpr double FRONT_SAFE_DIST = 0.45;
    static constexpr double FRONT_DANGER_DIST = 0.30;
    static constexpr double FRONT_CRITICAL_DIST = 0.18;

    static constexpr double REAR_ZONE_LENGTH = 0.8;   // 후방 감지 거리 증가
    static constexpr double REAR_ZONE_WIDTH = 0.12;   // 동일하게 좁게
    static constexpr double REAR_DANGER_DIST = 0.5;   // 후방 위험 거리 증가
    static constexpr double REAR_TTC_THRESHOLD = 2.5; // TTC 임계값

    static constexpr double SIDE_ZONE_START = -0.5;
    static constexpr double SIDE_ZONE_END = 0.5;
    static constexpr double SIDE_ZONE_INNER = 0.10; // 측면 내부 경계
    static constexpr double SIDE_ZONE_OUTER = 0.40; // 측면 외부 경계

    // Lane
    static constexpr double LANE_WIDTH = 0.25;
    static constexpr double LANE_THRESHOLD = 0.20; // 차선 판정 임계값 (더 엄격하게)

    // Static detection
    static constexpr double STATIC_SPEED_THRESHOLD = 0.05;

    // Corner detection
    static constexpr double CORNER_CURVATURE_THRESHOLD = 0.3;

    // Path collision check
    static constexpr double PATH_CHECK_HORIZON = 1.2;
    static constexpr double PATH_COLLISION_RADIUS = 0.22;

    // ========================================================================
    // PARAMETERS
    // ========================================================================

    double wheelbase_ = 0.33;
    double max_steer_ = 0.7;
    double target_speed_ = 0.45;
    double overtake_speed_ = 0.55;
    double accel_speed_ = 0.6; // 후방 위협 시 가속 속도
    double slow_speed_ = 0.25;
    double base_lookahead_ = 0.20;
    double max_lookahead_ = 0.35;
    double steer_alpha_ = 0.4;

    // ========================================================================
    // STATE VARIABLES
    // ========================================================================

    DrivingState current_state_ = DrivingState::CRUISE;
    LaneID current_lane_ = LaneID::LANE_2;
    LaneID target_lane_ = LaneID::LANE_2;
    LaneID original_lane_ = LaneID::LANE_2;

    double lane_change_start_sec_ = 0.0;
    int safe_check_count_ = 0;
    static constexpr int SAFE_CHECK_REQUIRED = 2;

    double ego_x_ = 0.0, ego_y_ = 0.0, ego_yaw_ = 0.0;
    double ego_speed_ = 0.0;
    bool pose_received_ = false;

    double prev_x_ = 0.0, prev_y_ = 0.0;
    double prev_pose_sec_ = 0.0;
    bool prev_pose_valid_ = false;

    int last_closest_idx_[3] = {0, 0, 0};
    double last_steer_ = 0.0;
    bool initial_search_done_ = false;

    // Corner state
    bool is_in_corner_ = false;
    bool corner_approaching_ = false;
    double current_curvature_ = 0.0;

    std::map<int, ObstacleInfo> obstacles_;
    SurroundingStatus current_surrounding_;

    double last_log_sec_ = 0.0;

    // ========================================================================
    // ROS INTERFACES
    // ========================================================================

    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_lane_[3];
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obs_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr accel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path::SharedPtr lane_paths_[3];

    // ========================================================================
    // HELPERS
    // ========================================================================

    double now_sec() { return this->now().seconds(); }

    double normalize_angle(double angle)
    {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }

    int lane_to_int(LaneID lane)
    {
        switch (lane)
        {
        case LaneID::LANE_1:
            return 0;
        case LaneID::LANE_2:
            return 1;
        case LaneID::LANE_3:
            return 2;
        default:
            return 1;
        }
    }

    LaneID int_to_lane(int i)
    {
        switch (i)
        {
        case 0:
            return LaneID::LANE_1;
        case 1:
            return LaneID::LANE_2;
        case 2:
            return LaneID::LANE_3;
        default:
            return LaneID::LANE_2;
        }
    }

    std::string lane_str(LaneID lane)
    {
        switch (lane)
        {
        case LaneID::LANE_1:
            return "L1";
        case LaneID::LANE_2:
            return "L2";
        case LaneID::LANE_3:
            return "L3";
        default:
            return "??";
        }
    }

    std::string state_str(DrivingState s)
    {
        switch (s)
        {
        case DrivingState::CRUISE:
            return "CRUISE";
        case DrivingState::PREPARE_OVERTAKE:
            return "PREPARE";
        case DrivingState::LANE_CHANGING:
            return "CHANGING";
        case DrivingState::EMERGENCY_ACCEL:
            return "ACCEL!";
        case DrivingState::EMERGENCY_BRAKE:
            return "BRAKE!";
        default:
            return "???";
        }
    }

    // ========================================================================
    // INITIALIZATION
    // ========================================================================

    void declare_parameters()
    {
        this->declare_parameter("wheelbase", 0.33);
        this->declare_parameter("max_steer", 0.7);
        this->declare_parameter("target_speed", 0.45);
        this->declare_parameter("overtake_speed", 0.55);
        this->declare_parameter("accel_speed", 0.6);
        this->declare_parameter("slow_speed", 0.25);
        this->declare_parameter("base_lookahead", 0.20);
        this->declare_parameter("max_lookahead", 0.35);
        this->declare_parameter("steer_alpha", 0.4);
        this->declare_parameter("reset_trigger", false);
    }

    void load_parameters()
    {
        this->get_parameter("wheelbase", wheelbase_);
        this->get_parameter("max_steer", max_steer_);
        this->get_parameter("target_speed", target_speed_);
        this->get_parameter("overtake_speed", overtake_speed_);
        this->get_parameter("accel_speed", accel_speed_);
        this->get_parameter("slow_speed", slow_speed_);
        this->get_parameter("base_lookahead", base_lookahead_);
        this->get_parameter("max_lookahead", max_lookahead_);
        this->get_parameter("steer_alpha", steer_alpha_);
    }

    rcl_interfaces::msg::SetParametersResult param_callback(
        const std::vector<rclcpp::Parameter> &params)
    {
        for (const auto &p : params)
        {
            if (p.get_name() == "reset_trigger" && p.as_bool())
                reset_state();
        }
        load_parameters();
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    void setup_ros_interfaces()
    {
        auto map_qos = rclcpp::QoS(10).transient_local();

        std::string topics[3] = {"/hdmap/lane_one", "/hdmap/lane_two", "/hdmap/lane_three"};
        for (int i = 0; i < 3; ++i)
        {
            sub_lane_[i] = this->create_subscription<nav_msgs::msg::Path>(
                topics[i], map_qos,
                [this, i](nav_msgs::msg::Path::SharedPtr msg)
                {
                    lane_paths_[i] = msg;
                    RCLCPP_INFO_ONCE(this->get_logger(), "Lane %d: %zu pts", i + 1, msg->poses.size());
                });
        }

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose", rclcpp::SensorDataQoS(),
            std::bind(&FrenetPlanner::pose_callback, this, std::placeholders::_1));

        obs_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/obstacles_markers", 10,
            std::bind(&FrenetPlanner::obstacle_callback, this, std::placeholders::_1));

        accel_pub_ = this->create_publisher<geometry_msgs::msg::Accel>("/Accel", 10);
        lookahead_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planning/lookahead_point", 10);
        debug_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/planning/debug_markers", 10);
    }

    void reset_state()
    {
        current_state_ = DrivingState::CRUISE;
        current_lane_ = LaneID::LANE_2;
        target_lane_ = LaneID::LANE_2;
        original_lane_ = LaneID::LANE_2;
        lane_change_start_sec_ = 0.0;
        safe_check_count_ = 0;
        ego_speed_ = 0.0;
        for (int i = 0; i < 3; ++i)
            last_closest_idx_[i] = 0;
        last_steer_ = 0.0;
        initial_search_done_ = false;
        prev_pose_valid_ = false;
        is_in_corner_ = false;
        corner_approaching_ = false;
        current_curvature_ = 0.0;
        obstacles_.clear();
        RCLCPP_WARN(this->get_logger(), "=== STATE RESET ===");
    }

    // ========================================================================
    // POSE CALLBACK
    // ========================================================================

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        ego_x_ = msg->pose.position.x;
        ego_y_ = msg->pose.position.y;
        ego_yaw_ = msg->pose.orientation.z;
        pose_received_ = true;

        double current_sec = now_sec();
        if (prev_pose_valid_)
        {
            double dt = current_sec - prev_pose_sec_;
            if (dt > 0.01 && dt < 0.5)
            {
                double dx = ego_x_ - prev_x_;
                double dy = ego_y_ - prev_y_;
                double raw_speed = std::hypot(dx, dy) / dt;

                double move_angle = std::atan2(dy, dx);
                double angle_diff = normalize_angle(move_angle - ego_yaw_);
                if (std::abs(angle_diff) > M_PI / 2)
                    raw_speed = -raw_speed;

                ego_speed_ = ego_speed_ * 0.7 + raw_speed * 0.3;
            }
        }

        prev_x_ = ego_x_;
        prev_y_ = ego_y_;
        prev_pose_sec_ = current_sec;
        prev_pose_valid_ = true;

        current_lane_ = get_lane_at(ego_x_, ego_y_);
    }

    // ========================================================================
    // OBSTACLE CALLBACK
    // ========================================================================

    void obstacle_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        double current_sec = now_sec();

        for (const auto &marker : msg->markers)
        {
            if (marker.ns != "surrounding_cars")
                continue;
            if (marker.type != visualization_msgs::msg::Marker::CUBE)
                continue;

            int id = marker.id;
            double ox = marker.pose.position.x;
            double oy = marker.pose.position.y;

            auto it = obstacles_.find(id);
            if (it != obstacles_.end())
            {
                auto &obs = it->second;
                double dt = current_sec - obs.last_seen_sec;

                if (dt > 0.01 && dt < 1.0)
                {
                    double vx = (ox - obs.x) / dt;
                    double vy = (oy - obs.y) / dt;
                    if (std::hypot(vx, vy) < 5.0)
                    {
                        obs.vx = obs.vx * 0.6 + vx * 0.4;
                        obs.vy = obs.vy * 0.6 + vy * 0.4;
                    }
                }

                obs.x = ox;
                obs.y = oy;
                obs.speed = std::hypot(obs.vx, obs.vy);
                obs.is_static = (obs.speed < STATIC_SPEED_THRESHOLD);
                obs.lane = get_lane_at(ox, oy);
                obs.last_seen_sec = current_sec;
            }
            else
            {
                ObstacleInfo obs;
                obs.id = id;
                obs.x = ox;
                obs.y = oy;
                obs.vx = obs.vy = 0.0;
                obs.speed = 0.0;
                obs.is_static = true;
                obs.lane = get_lane_at(ox, oy);
                obs.last_seen_sec = current_sec;
                obstacles_[id] = obs;
            }
        }

        // Remove stale
        for (auto it = obstacles_.begin(); it != obstacles_.end();)
        {
            if ((current_sec - it->second.last_seen_sec) > 1.0)
                it = obstacles_.erase(it);
            else
                ++it;
        }
    }

    // ========================================================================
    // LANE UTILITIES
    // ========================================================================

    double get_dist_to_lane(int lane_idx, double x, double y)
    {
        if (lane_idx < 0 || lane_idx > 2)
            return 999.0;
        if (!lane_paths_[lane_idx] || lane_paths_[lane_idx]->poses.empty())
            return 999.0;

        const auto &poses = lane_paths_[lane_idx]->poses;
        double min_d = 999.0;
        size_t step = std::max(size_t(1), poses.size() / 500);

        for (size_t i = 0; i < poses.size(); i += step)
        {
            double d = std::hypot(poses[i].pose.position.x - x,
                                  poses[i].pose.position.y - y);
            if (d < min_d)
                min_d = d;
        }
        return min_d;
    }

    LaneID get_lane_at(double x, double y)
    {
        double d[3];
        for (int i = 0; i < 3; ++i)
            d[i] = get_dist_to_lane(i, x, y);

        int best = 0;
        for (int i = 1; i < 3; ++i)
            if (d[i] < d[best])
                best = i;

        if (d[best] > LANE_THRESHOLD)
            return LaneID::NONE;

        return int_to_lane(best);
    }

    int find_closest_idx(int lane_idx, double x, double y)
    {
        if (lane_idx < 0 || lane_idx > 2)
            return 0;
        if (!lane_paths_[lane_idx] || lane_paths_[lane_idx]->poses.empty())
            return 0;

        const auto &poses = lane_paths_[lane_idx]->poses;
        int n = poses.size();
        int best = last_closest_idx_[lane_idx];
        double min_d = 999.0;

        bool need_global = !initial_search_done_ ||
                           current_state_ == DrivingState::LANE_CHANGING;

        if (need_global)
        {
            for (int i = 0; i < n; ++i)
            {
                double d = std::hypot(poses[i].pose.position.x - x,
                                      poses[i].pose.position.y - y);
                if (d < min_d)
                {
                    min_d = d;
                    best = i;
                }
            }
        }
        else
        {
            for (int i = 0; i < 200; ++i)
            {
                int idx = (last_closest_idx_[lane_idx] + i) % n;
                double d = std::hypot(poses[idx].pose.position.x - x,
                                      poses[idx].pose.position.y - y);
                if (d < min_d)
                {
                    min_d = d;
                    best = idx;
                }
            }
        }

        last_closest_idx_[lane_idx] = best;
        return best;
    }

    // ========================================================================
    // CURVATURE & CORNER DETECTION
    // ========================================================================

    double compute_curvature(int lane_idx, int idx, int window = 10)
    {
        if (lane_idx < 0 || lane_idx > 2)
            return 0.0;
        if (!lane_paths_[lane_idx] || lane_paths_[lane_idx]->poses.size() < 30)
            return 0.0;

        const auto &poses = lane_paths_[lane_idx]->poses;
        int n = poses.size();

        int i0 = ((idx - window) % n + n) % n;
        int i1 = idx;
        int i2 = (idx + window) % n;

        double x0 = poses[i0].pose.position.x, y0 = poses[i0].pose.position.y;
        double x1 = poses[i1].pose.position.x, y1 = poses[i1].pose.position.y;
        double x2 = poses[i2].pose.position.x, y2 = poses[i2].pose.position.y;

        double a = std::hypot(x1 - x0, y1 - y0);
        double b = std::hypot(x2 - x1, y2 - y1);
        double c = std::hypot(x0 - x2, y0 - y2);

        double s = (a + b + c) / 2.0;
        double area_sq = s * (s - a) * (s - b) * (s - c);

        if (area_sq <= 0 || a < 0.001 || b < 0.001 || c < 0.001)
            return 0.0;

        double area = std::sqrt(area_sq);
        double curv = 4.0 * area / (a * b * c);

        double cross = (x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1);
        return (cross >= 0) ? curv : -curv;
    }

    void update_corner_state(int lane_idx, int closest)
    {
        if (lane_idx < 0 || lane_idx > 2)
            return;
        if (!lane_paths_[lane_idx] || lane_paths_[lane_idx]->poses.empty())
            return;

        int n = lane_paths_[lane_idx]->poses.size();

        // Current curvature
        current_curvature_ = std::abs(compute_curvature(lane_idx, closest, 10));
        is_in_corner_ = (current_curvature_ > CORNER_CURVATURE_THRESHOLD);

        // Look ahead for approaching corner
        int preview_idx = (closest + 40) % n;
        double ahead_curv = std::abs(compute_curvature(lane_idx, preview_idx, 10));
        corner_approaching_ = (ahead_curv > CORNER_CURVATURE_THRESHOLD);
    }

    // ========================================================================
    // BODY FRAME TRANSFORM
    // ========================================================================

    void update_all_obstacles_body_frame()
    {
        double cos_yaw = std::cos(ego_yaw_);
        double sin_yaw = std::sin(ego_yaw_);

        for (auto &[id, obs] : obstacles_)
        {
            double dx = obs.x - ego_x_;
            double dy = obs.y - ego_y_;

            obs.rel_x = dx * cos_yaw + dy * sin_yaw;
            obs.rel_y = -dx * sin_yaw + dy * cos_yaw;
            obs.rel_vx = (obs.vx * cos_yaw + obs.vy * sin_yaw) - ego_speed_;
        }
    }

    // ========================================================================
    // PATH-BASED COLLISION CHECK
    // ========================================================================

    bool check_path_collision(int lane_idx, double &collision_dist)
    {
        collision_dist = 999.0;

        if (lane_idx < 0 || lane_idx > 2)
            return false;
        if (!lane_paths_[lane_idx] || lane_paths_[lane_idx]->poses.empty())
            return false;

        const auto &poses = lane_paths_[lane_idx]->poses;
        int n = poses.size();
        int closest = find_closest_idx(lane_idx, ego_x_, ego_y_);

        double check_speed = std::max(ego_speed_, 0.2);
        double total_dist = 0.0;
        double max_check_dist = check_speed * PATH_CHECK_HORIZON;

        for (int i = 1; i < 60 && total_dist < max_check_dist; ++i)
        {
            int idx = (closest + i) % n;
            int prev_idx = (closest + i - 1) % n;

            double px = poses[idx].pose.position.x;
            double py = poses[idx].pose.position.y;

            double seg_dist = std::hypot(
                px - poses[prev_idx].pose.position.x,
                py - poses[prev_idx].pose.position.y);
            total_dist += seg_dist;

            double t = total_dist / check_speed;

            for (const auto &[id, obs] : obstacles_)
            {
                double obs_x = obs.x + obs.vx * t;
                double obs_y = obs.y + obs.vy * t;

                double dist = std::hypot(px - obs_x, py - obs_y);

                if (dist < PATH_COLLISION_RADIUS)
                {
                    if (total_dist < collision_dist)
                    {
                        collision_dist = total_dist;
                    }
                    return true;
                }
            }
        }

        return false;
    }

    // ========================================================================
    // ★ LANE-BASED COLLISION DETECTION (v6.1 핵심 수정)
    // ========================================================================

    SurroundingStatus check_all_zones()
    {
        SurroundingStatus status;
        double current_sec = now_sec();

        for (const auto &[id, obs] : obstacles_)
        {
            if ((current_sec - obs.last_seen_sec) > 0.5)
                continue;

            double rx = obs.rel_x;
            double ry = obs.rel_y;

            // ============================================================
            // ★ FRONT ZONE: 반드시 같은 차선의 장애물만 체크!
            // ============================================================
            if (obs.lane == current_lane_) // ★ 핵심: 같은 차선만!
            {
                // 전방에 있는지?
                if (rx > -OBS_HALF_LENGTH && rx < FRONT_ZONE_LENGTH + OBS_HALF_LENGTH)
                {
                    // 차선 중심 기준 좌우 범위 내?
                    if (std::abs(ry) < FRONT_ZONE_WIDTH + OBS_HALF_WIDTH)
                    {
                        double effective_dist = rx - OBS_HALF_LENGTH;

                        if (effective_dist < status.front_dist && effective_dist > -0.1)
                        {
                            status.front_dist = effective_dist;
                            status.front_speed = obs.speed;
                            status.front_is_static = obs.is_static;
                            status.front_id = id;

                            if (effective_dist < FRONT_SAFE_DIST)
                                status.front_blocked = true;
                            if (effective_dist < FRONT_DANGER_DIST)
                                status.front_danger = true;
                            if (effective_dist < FRONT_CRITICAL_DIST)
                                status.front_critical = true;
                        }
                    }
                }

                // ============================================================
                // ★ REAR ZONE: 같은 차선의 후방 장애물
                // ============================================================
                if (rx < OBS_HALF_LENGTH && rx > -(REAR_ZONE_LENGTH + OBS_HALF_LENGTH))
                {
                    if (std::abs(ry) < REAR_ZONE_WIDTH + OBS_HALF_WIDTH)
                    {
                        double effective_dist = -rx - OBS_HALF_LENGTH;

                        if (effective_dist > 0 && effective_dist < status.rear_dist)
                        {
                            status.rear_dist = effective_dist;
                            status.rear_speed = obs.speed;

                            // TTC (Time To Collision) 계산
                            double closing_speed = -obs.rel_vx; // 접근 속도
                            if (closing_speed > 0.05)
                            {
                                status.rear_ttc = effective_dist / closing_speed;
                                if (status.rear_ttc < REAR_TTC_THRESHOLD)
                                {
                                    status.rear_danger = true;
                                }
                            }
                        }
                    }
                }
            }

            // ============================================================
            // SIDE ZONES (Blind spots) - 인접 차선 장애물
            // ============================================================
            // 측면은 "다른 차선" 장애물 체크
            if (obs.lane != current_lane_ && obs.lane != LaneID::NONE)
            {
                if (rx > SIDE_ZONE_START - OBS_HALF_LENGTH &&
                    rx < SIDE_ZONE_END + OBS_HALF_LENGTH)
                {
                    // Left (내 왼쪽에 있는 장애물)
                    if (ry > 0)
                    {
                        double lat_dist = ry - OBS_HALF_WIDTH;
                        if (lat_dist < SIDE_ZONE_OUTER && lat_dist > -0.1)
                        {
                            status.left_clear = false;
                            status.left_dist = std::min(status.left_dist, lat_dist);
                        }
                    }
                    // Right (내 오른쪽에 있는 장애물)
                    else
                    {
                        double lat_dist = -ry - OBS_HALF_WIDTH;
                        if (lat_dist < SIDE_ZONE_OUTER && lat_dist > -0.1)
                        {
                            status.right_clear = false;
                            status.right_dist = std::min(status.right_dist, lat_dist);
                        }
                    }
                }
            }
        }

        // Path collision check
        int path_lane = lane_to_int(target_lane_);
        double path_coll_dist;
        status.path_collision = check_path_collision(path_lane, path_coll_dist);
        status.path_collision_dist = path_coll_dist;

        return status;
    }

    // ========================================================================
    // GEOMETRY HELPER
    // ========================================================================

    bool is_target_left(LaneID target_lane)
    {
        int t_idx = lane_to_int(target_lane);
        if (!lane_paths_[t_idx] || lane_paths_[t_idx]->poses.empty())
            return false;

        int closest = find_closest_idx(t_idx, ego_x_, ego_y_);
        double tx = lane_paths_[t_idx]->poses[closest].pose.position.x;
        double ty = lane_paths_[t_idx]->poses[closest].pose.position.y;

        double dx = tx - ego_x_;
        double dy = ty - ego_y_;

        double head_x = std::cos(ego_yaw_);
        double head_y = std::sin(ego_yaw_);

        double cross_product = head_x * dy - head_y * dx;
        return cross_product > 0.0;
    }

    // ========================================================================
    // LANE CHANGE SAFETY CHECK
    // ========================================================================

    bool is_lane_blocked_by_static(LaneID lane)
    {
        const double STATIC_BLOCK_DIST = 2.0;

        for (const auto &[id, obs] : obstacles_)
        {
            if (obs.lane != lane)
                continue;
            if (!obs.is_static)
                continue;

            double dist = std::hypot(obs.x - ego_x_, obs.y - ego_y_);
            double dx = obs.x - ego_x_;
            double dy = obs.y - ego_y_;
            double forward = dx * std::cos(ego_yaw_) + dy * std::sin(ego_yaw_);

            if (dist < STATIC_BLOCK_DIST && forward > -0.3)
            {
                return true;
            }
        }
        return false;
    }

    bool is_overtake_safe(LaneID target)
    {
        bool going_left = is_target_left(target);

        // Blind spot check
        if (going_left && !current_surrounding_.left_clear)
            return false;
        if (!going_left && !current_surrounding_.right_clear)
            return false;

        // Path collision check
        double coll_dist;
        int target_idx = lane_to_int(target);
        if (check_path_collision(target_idx, coll_dist))
        {
            if (coll_dist < 0.4)
                return false;
        }

        // Target lane obstacles check
        for (const auto &[id, obs] : obstacles_)
        {
            if (obs.lane != target)
                continue;

            double rx = obs.rel_x;

            // Front
            if (rx > -OBS_HALF_LENGTH && rx < FRONT_ZONE_LENGTH)
            {
                double dist = rx - OBS_HALF_LENGTH;
                if (dist < FRONT_CRITICAL_DIST)
                    return false;
            }

            // Rear approaching fast
            if (rx < 0 && rx > -REAR_ZONE_LENGTH)
            {
                double closing = -obs.rel_vx;
                if (closing > 0.1)
                {
                    double dist = -rx;
                    double ttc = dist / closing;
                    if (ttc < 1.5)
                        return false;
                }
            }
        }

        return true;
    }

    // ========================================================================
    // OVERTAKE LANE SELECTION
    // ========================================================================

    LaneID choose_overtake_lane()
    {
        int current_idx = lane_to_int(current_lane_);

        std::vector<int> candidates;
        bool avoid_lane1 = is_in_corner_ || corner_approaching_;

        if (current_idx == 0) // L1
        {
            candidates = {1, 2};
        }
        else if (current_idx == 1) // L2
        {
            if (avoid_lane1)
            {
                candidates = {2}; // Corner: only L3
            }
            else
            {
                candidates = {2, 0}; // Prefer L3
            }
        }
        else // L3
        {
            candidates = {1};
        }

        for (int cand : candidates)
        {
            LaneID target = int_to_lane(cand);

            if (avoid_lane1 && cand == 0)
                continue;

            if (is_lane_blocked_by_static(target))
                continue;

            double coll_dist;
            if (check_path_collision(cand, coll_dist) && coll_dist < 0.4)
                continue;

            if (is_overtake_safe(target))
                return target;
        }

        return current_lane_;
    }

    // ========================================================================
    // STATE MACHINE (v6.1: 후방 위협 대응 추가)
    // ========================================================================

    void update_state_machine()
    {
        double current_sec = now_sec();
        const auto &s = current_surrounding_;

        // ★ 후방 위협 체크 (모든 상태에서)
        if (s.rear_danger && current_state_ != DrivingState::EMERGENCY_ACCEL)
        {
            // 전방이 막히지 않았으면 가속
            if (!s.front_critical && s.front_dist > FRONT_DANGER_DIST)
            {
                current_state_ = DrivingState::EMERGENCY_ACCEL;
                RCLCPP_WARN(this->get_logger(), "REAR THREAT! TTC=%.1f, Accelerating!", s.rear_ttc);
                return;
            }
            // 전방도 막혔으면 차선 변경
            else
            {
                LaneID escape = choose_overtake_lane();
                if (escape != current_lane_ && is_overtake_safe(escape))
                {
                    target_lane_ = escape;
                    current_state_ = DrivingState::LANE_CHANGING;
                    lane_change_start_sec_ = current_sec;
                    RCLCPP_WARN(this->get_logger(), "SANDWICHED! Escape to %s", lane_str(escape).c_str());
                    return;
                }
            }
        }

        // Lane 1 코너 접근 시 탈출
        if (current_lane_ == LaneID::LANE_1 && corner_approaching_ &&
            current_state_ == DrivingState::CRUISE)
        {
            if (is_overtake_safe(LaneID::LANE_2))
            {
                target_lane_ = LaneID::LANE_2;
                current_state_ = DrivingState::LANE_CHANGING;
                lane_change_start_sec_ = current_sec;
                RCLCPP_WARN(this->get_logger(), "Corner ahead! Escaping L1 -> L2");
                return;
            }
        }

        switch (current_state_)
        {
        case DrivingState::CRUISE:
        {
            bool need_overtake = s.front_blocked || s.front_danger;

            if (s.path_collision && s.path_collision_dist < 0.5)
                need_overtake = true;

            if (need_overtake)
            {
                LaneID overtake = choose_overtake_lane();
                if (overtake != current_lane_)
                {
                    target_lane_ = overtake;
                    original_lane_ = current_lane_;

                    if (s.front_is_static)
                    {
                        current_state_ = DrivingState::LANE_CHANGING;
                        lane_change_start_sec_ = current_sec;
                        RCLCPP_WARN(this->get_logger(), "STATIC! Immediate: %s -> %s",
                                    lane_str(current_lane_).c_str(), lane_str(target_lane_).c_str());
                    }
                    else
                    {
                        current_state_ = DrivingState::PREPARE_OVERTAKE;
                        safe_check_count_ = 0;
                    }
                }
                else if (s.front_critical)
                {
                    current_state_ = DrivingState::EMERGENCY_BRAKE;
                }
            }
            break;
        }

        case DrivingState::PREPARE_OVERTAKE:
        {
            if (is_overtake_safe(target_lane_))
            {
                safe_check_count_++;
                if (safe_check_count_ >= SAFE_CHECK_REQUIRED)
                {
                    current_state_ = DrivingState::LANE_CHANGING;
                    lane_change_start_sec_ = current_sec;
                }
            }
            else
            {
                safe_check_count_ = std::max(0, safe_check_count_ - 1);
            }

            if (!s.front_blocked && s.front_dist > FRONT_SAFE_DIST)
            {
                current_state_ = DrivingState::CRUISE;
                target_lane_ = current_lane_;
            }
            break;
        }

        case DrivingState::LANE_CHANGING:
        {
            if (current_lane_ == target_lane_)
            {
                double elapsed = current_sec - lane_change_start_sec_;
                if (elapsed > 0.3)
                {
                    current_state_ = DrivingState::CRUISE;
                    RCLCPP_INFO(this->get_logger(), "Lane change complete: %s",
                                lane_str(current_lane_).c_str());
                }
            }

            if ((current_sec - lane_change_start_sec_) > 5.0)
            {
                current_state_ = DrivingState::CRUISE;
                target_lane_ = current_lane_;
            }
            break;
        }

        case DrivingState::EMERGENCY_ACCEL:
        {
            // 후방 위협 해소?
            if (!s.rear_danger || s.rear_dist > REAR_DANGER_DIST * 1.5)
            {
                current_state_ = DrivingState::CRUISE;
                RCLCPP_INFO(this->get_logger(), "Rear clear, back to cruise");
            }
            // 전방 막힘?
            else if (s.front_danger)
            {
                LaneID escape = choose_overtake_lane();
                if (escape != current_lane_ && is_overtake_safe(escape))
                {
                    target_lane_ = escape;
                    current_state_ = DrivingState::LANE_CHANGING;
                    lane_change_start_sec_ = current_sec;
                }
            }
            break;
        }

        case DrivingState::EMERGENCY_BRAKE:
        {
            LaneID escape = choose_overtake_lane();
            if (escape != current_lane_ && is_overtake_safe(escape))
            {
                target_lane_ = escape;
                current_state_ = DrivingState::LANE_CHANGING;
                lane_change_start_sec_ = current_sec;
            }

            if (s.front_dist > FRONT_SAFE_DIST)
            {
                current_state_ = DrivingState::CRUISE;
            }
            break;
        }
        }
    }

    // ========================================================================
    // PATH FOLLOWING
    // ========================================================================

    int find_lookahead_idx(int lane_idx, int closest, double dist)
    {
        if (lane_idx < 0 || lane_idx > 2)
            return 0;
        if (!lane_paths_[lane_idx] || lane_paths_[lane_idx]->poses.empty())
            return 0;

        const auto &poses = lane_paths_[lane_idx]->poses;
        int n = poses.size();

        for (int i = 0; i < 100; ++i)
        {
            int idx = (closest + i) % n;
            double d = std::hypot(poses[idx].pose.position.x - ego_x_,
                                  poses[idx].pose.position.y - ego_y_);
            if (d >= dist)
                return idx;
        }

        return (closest + 20) % n;
    }

    double compute_cross_track_error(int lane_idx, int closest)
    {
        if (lane_idx < 0 || lane_idx > 2)
            return 0.0;
        if (!lane_paths_[lane_idx] || lane_paths_[lane_idx]->poses.empty())
            return 0.0;

        const auto &poses = lane_paths_[lane_idx]->poses;
        int n = poses.size();

        double cx = poses[closest].pose.position.x;
        double cy = poses[closest].pose.position.y;

        int next = (closest + 3) % n;
        double path_yaw = std::atan2(
            poses[next].pose.position.y - cy,
            poses[next].pose.position.x - cx);

        double dx = ego_x_ - cx;
        double dy = ego_y_ - cy;

        return -dx * std::sin(path_yaw) + dy * std::cos(path_yaw);
    }

    // ========================================================================
    // STEERING CONTROL
    // ========================================================================

    // ========================================================================
    // STEERING CONTROL (v6.1.1: 코너 차선변경 조향 강화)
    // ========================================================================

    // ========================================================================
    // STEERING CONTROL (수정됨: 코너 탈출 안정성 및 헤딩 정렬 강화)
    // ========================================================================

    double compute_steering(int lane_idx, int closest, int lookahead)
    {
        if (lane_idx < 0 || lane_idx > 2)
            return 0.0;
        if (!lane_paths_[lane_idx] || lane_paths_[lane_idx]->poses.empty())
            return 0.0;

        const auto &poses = lane_paths_[lane_idx]->poses;
        int n = poses.size();

        double tx = poses[lookahead].pose.position.x;
        double ty = poses[lookahead].pose.position.y;
        double dx = tx - ego_x_;
        double dy = ty - ego_y_;
        double dist = std::hypot(dx, dy);
        double target_yaw = std::atan2(dy, dx);

        int next = (lookahead + 5) % n;
        double road_yaw = std::atan2(
            poses[next].pose.position.y - ty,
            poses[next].pose.position.x - tx);

        double heading_err = normalize_angle(target_yaw - ego_yaw_);
        double road_err = normalize_angle(road_yaw - ego_yaw_);

        double cte = compute_cross_track_error(lane_idx, closest);

        int preview = (closest + 20) % n;
        double curv = compute_curvature(lane_idx, preview, 10);
        bool is_corner = std::abs(curv) > CORNER_CURVATURE_THRESHOLD;

        double steer = 0.0;
        double steer_pp = std::atan2(2.0 * wheelbase_ * std::sin(heading_err),
                                     std::max(dist, 0.1));

        // 코너 + 차선 변경 (기존 유지)
        if (is_corner && current_state_ == DrivingState::LANE_CHANGING)
        {
            double k_cte = 12.0;
            double cte_steer = std::atan2(k_cte * cte, std::max(ego_speed_, 0.3));
            cte_steer = std::clamp(cte_steer, -0.7, 0.7);
            double feedforward = 0.8 * curv * wheelbase_;
            steer = steer_pp * 0.3 + road_err * 0.8 + cte_steer * 1.0 + feedforward;
        }
        // 일반 코너 (기존 유지)
        else if (is_corner)
        {
            double k_cte = 8.0;
            double cte_steer = std::atan2(k_cte * cte, std::max(ego_speed_, 0.5));
            cte_steer = std::clamp(cte_steer, -0.6, 0.6);
            double feedforward = 0.7 * curv * wheelbase_;
            steer = steer_pp * 0.15 + road_err * 1.2 + cte_steer * 0.6 + feedforward;
        }
        // 차선 변경 (기존 유지)
        else if (current_state_ == DrivingState::LANE_CHANGING)
        {
            double k_cte = 8.0;
            double cte_steer = std::atan2(k_cte * cte, std::max(ego_speed_, 0.5));
            cte_steer = std::clamp(cte_steer, -0.6, 0.6);
            steer = steer_pp * 0.2 + road_err * 0.2 + cte_steer * 0.8;
        }
        // [핵심 수정] 직선 주행 및 코너 탈출 구간 (Normal Driving)
        else
        {
            // 1. CTE 이득 강화 (4.0 -> 7.0)
            // 코너 탈출 후 밖으로 밀려난 상태를 빠르게 중앙으로 복구합니다.
            double k_cte = 7.0;
            double cte_steer = std::atan2(k_cte * cte, std::max(ego_speed_, 0.5));
            cte_steer = std::clamp(cte_steer, -0.6, 0.6);

            // 2. 가중치 재조정 (헤딩 정렬 최우선)
            // steer_pp (점 추종): 0.3 -> 0.2 (줄임: 진동 방지)
            // road_err (도로 정렬): 0.4 -> 0.8 (대폭 늘림: 차선과 평행 유지 강화)
            // cte_steer (위치 보정): 0.7 -> 0.9 (늘림: 차선 중앙 복귀 강화)

            steer = steer_pp * 0.2 + road_err * 0.8 + cte_steer * 0.9;
        }

        // Emergency correction
        if (std::abs(cte) > 0.12)
        {
            double emergency_steer = (cte > 0) ? -0.4 : 0.4;
            double weight = is_corner ? 0.3 : 0.5;
            steer = steer * (1.0 - weight) + emergency_steer * weight;
        }

        // Smoothing
        double alpha;
        if (is_corner && current_state_ == DrivingState::LANE_CHANGING)
            alpha = 0.6;
        else if (current_state_ == DrivingState::LANE_CHANGING)
            alpha = 0.3;
        else if (is_corner)
            alpha = 0.4;
        else
            alpha = 0.45; // [수정] 직선 구간 반응성 약간 향상 (0.4 -> 0.45)

        steer = steer * alpha + last_steer_ * (1.0 - alpha);

        return std::clamp(steer, -max_steer_, max_steer_);
    }

    // ========================================================================
    // VELOCITY CONTROL (v6.1: 후방 위협 가속)
    // ========================================================================

    double compute_velocity()
    {
        const auto &s = current_surrounding_;
        double vel = target_speed_;

        switch (current_state_)
        {
        case DrivingState::CRUISE:
            if (s.front_blocked)
            {
                if (s.front_is_static)
                {
                    vel = slow_speed_;
                }
                else
                {
                    double gap = s.front_dist - FRONT_CRITICAL_DIST;
                    vel = std::min(target_speed_, std::max(s.front_speed, gap * 2.0));
                    vel = std::max(vel, 0.1);
                }
            }
            if (s.path_collision && s.path_collision_dist < 0.5)
            {
                vel = std::min(vel, slow_speed_);
            }
            break;

        case DrivingState::PREPARE_OVERTAKE:
            if (s.front_is_static)
                vel = slow_speed_;
            else
            {
                vel = std::min(target_speed_, s.front_speed + 0.05);
                vel = std::max(vel, slow_speed_);
            }
            break;

        case DrivingState::LANE_CHANGING:
            vel = overtake_speed_;
            // 후방에 빠른 차가 있으면 더 가속
            if (s.rear_danger || s.rear_ttc < 3.0)
            {
                vel = std::max(vel, accel_speed_);
            }
            break;

        case DrivingState::EMERGENCY_ACCEL:
            // ★ 후방 위협 시 최대 가속
            vel = accel_speed_;
            // 전방이 막히면 약간 감속
            if (s.front_blocked)
            {
                vel = std::min(vel, target_speed_);
            }
            break;

        case DrivingState::EMERGENCY_BRAKE:
            vel = slow_speed_;
            if (s.front_dist < FRONT_CRITICAL_DIST * 0.5)
                vel = 0.1;
            break;
        }

        // Corner speed limit
        if (is_in_corner_ && current_state_ == DrivingState::LANE_CHANGING)
        {
            vel = std::min(vel, 0.35); // 코너 차선변경은 0.35 제한
        }
        else if (is_in_corner_)
        {
            vel = std::min(vel, 0.4);
        }

        return std::clamp(vel, 0.1, 0.7);
    }

    // ========================================================================
    // MAIN CONTROL LOOP
    // ========================================================================

    void control_loop()
    {
        if (!pose_received_)
            return;

        bool ready = true;
        for (int i = 0; i < 3; ++i)
        {
            if (!lane_paths_[i] || lane_paths_[i]->poses.empty())
            {
                ready = false;
                break;
            }
        }
        if (!ready)
            return;

        if (!initial_search_done_)
        {
            for (int i = 0; i < 3; ++i)
                find_closest_idx(i, ego_x_, ego_y_);
            initial_search_done_ = true;
            RCLCPP_INFO(this->get_logger(), "Initial search done. Lane: %s",
                        lane_str(current_lane_).c_str());
        }

        if (current_lane_ == LaneID::NONE)
        {
            current_lane_ = get_lane_at(ego_x_, ego_y_);
            if (current_lane_ == LaneID::NONE)
                current_lane_ = LaneID::LANE_2;
            target_lane_ = current_lane_;
        }

        // Update body frame
        update_all_obstacles_body_frame();

        // Path selection
        int path_lane_idx = lane_to_int(
            (current_state_ == DrivingState::LANE_CHANGING) ? target_lane_ : current_lane_);

        int closest = find_closest_idx(path_lane_idx, ego_x_, ego_y_);

        // Update corner state
        update_corner_state(path_lane_idx, closest);

        // Check all zones
        current_surrounding_ = check_all_zones();

        // State machine
        update_state_machine();

        // Logging
        double current_sec = now_sec();
        if ((current_sec - last_log_sec_) > 1.0)
        {
            RCLCPP_INFO(this->get_logger(),
                        "[%s] %s%s | Spd=%.2f | F=%.2f(%s) R=%.2f(ttc=%.1f) | L=%s R=%s",
                        state_str(current_state_).c_str(),
                        lane_str(current_lane_).c_str(),
                        (target_lane_ != current_lane_) ? ("->" + lane_str(target_lane_)).c_str() : "",
                        ego_speed_,
                        current_surrounding_.front_dist,
                        current_surrounding_.front_is_static ? "S" : "M",
                        current_surrounding_.rear_dist,
                        current_surrounding_.rear_ttc,
                        current_surrounding_.left_clear ? "OK" : "X",
                        current_surrounding_.right_clear ? "OK" : "X");
            last_log_sec_ = current_sec;
        }

        // Lookahead
        int preview = (closest + 25) % lane_paths_[path_lane_idx]->poses.size();
        double curv = std::abs(compute_curvature(path_lane_idx, preview, 10));

        double lookahead_dist = (curv > CORNER_CURVATURE_THRESHOLD) ? 0.12 : base_lookahead_;
        if (current_state_ == DrivingState::LANE_CHANGING)
            lookahead_dist = 0.15;
        lookahead_dist = std::clamp(lookahead_dist, 0.08, max_lookahead_);

        int lookahead_idx = find_lookahead_idx(path_lane_idx, closest, lookahead_dist);

        // Control
        double steer = compute_steering(path_lane_idx, closest, lookahead_idx);
        double vel = compute_velocity();

        last_steer_ = steer;

        // Publish
        geometry_msgs::msg::Accel cmd;
        cmd.linear.x = vel;
        cmd.angular.z = steer;
        accel_pub_->publish(cmd);

        // Visualization
        publish_visualization(path_lane_idx, lookahead_idx);
    }

    // ========================================================================
    // VISUALIZATION
    // ========================================================================

    void publish_visualization(int lane_idx, int lookahead_idx)
    {
        if (lane_idx < 0 || lane_idx > 2)
            return;
        if (!lane_paths_[lane_idx])
            return;

        const auto &poses = lane_paths_[lane_idx]->poses;
        if (lookahead_idx >= (int)poses.size())
            return;

        visualization_msgs::msg::MarkerArray markers;
        int id = 0;

        double cos_yaw = std::cos(ego_yaw_);
        double sin_yaw = std::sin(ego_yaw_);

        tf2::Quaternion q;
        q.setRPY(0, 0, ego_yaw_);
        auto quat_msg = tf2::toMsg(q);

        // FRONT ZONE
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "world";
            m.header.stamp = this->now();
            m.ns = "zones";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::CUBE;
            m.action = visualization_msgs::msg::Marker::ADD;

            double center_x = FRONT_ZONE_LENGTH / 2;
            m.pose.position.x = ego_x_ + cos_yaw * center_x;
            m.pose.position.y = ego_y_ + sin_yaw * center_x;
            m.pose.position.z = 0.02;
            m.pose.orientation = quat_msg;

            m.scale.x = FRONT_ZONE_LENGTH;
            m.scale.y = FRONT_ZONE_WIDTH * 2;
            m.scale.z = 0.01;

            if (current_surrounding_.front_critical || current_surrounding_.path_collision)
            {
                m.color.r = 1.0;
                m.color.g = 0.0;
                m.color.b = 0.0;
            }
            else if (current_surrounding_.front_danger)
            {
                m.color.r = 1.0;
                m.color.g = 0.5;
                m.color.b = 0.0;
            }
            else if (current_surrounding_.front_blocked)
            {
                m.color.r = 1.0;
                m.color.g = 1.0;
                m.color.b = 0.0;
            }
            else
            {
                m.color.r = 0.0;
                m.color.g = 1.0;
                m.color.b = 0.0;
            }

            m.color.a = 0.4;
            m.lifetime = rclcpp::Duration::from_seconds(0.1);
            markers.markers.push_back(m);
        }

        // REAR ZONE
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "world";
            m.header.stamp = this->now();
            m.ns = "zones";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::CUBE;
            m.action = visualization_msgs::msg::Marker::ADD;

            double center_x = -REAR_ZONE_LENGTH / 2;
            m.pose.position.x = ego_x_ + cos_yaw * center_x;
            m.pose.position.y = ego_y_ + sin_yaw * center_x;
            m.pose.position.z = 0.02;
            m.pose.orientation = quat_msg;

            m.scale.x = REAR_ZONE_LENGTH;
            m.scale.y = REAR_ZONE_WIDTH * 2;
            m.scale.z = 0.01;

            if (current_surrounding_.rear_danger)
            {
                m.color.r = 1.0;
                m.color.g = 0.0;
                m.color.b = 0.0;
            }
            else
            {
                m.color.r = 1.0;
                m.color.g = 0.5;
                m.color.b = 0.0;
            }

            m.color.a = 0.3;
            m.lifetime = rclcpp::Duration::from_seconds(0.1);
            markers.markers.push_back(m);
        }

        // LEFT BLIND SPOT
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "world";
            m.header.stamp = this->now();
            m.ns = "zones";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::CUBE;
            m.action = visualization_msgs::msg::Marker::ADD;

            double center_x = (SIDE_ZONE_START + SIDE_ZONE_END) / 2;
            double center_y = (SIDE_ZONE_INNER + SIDE_ZONE_OUTER) / 2;
            m.pose.position.x = ego_x_ + cos_yaw * center_x - sin_yaw * center_y;
            m.pose.position.y = ego_y_ + sin_yaw * center_x + cos_yaw * center_y;
            m.pose.position.z = 0.02;
            m.pose.orientation = quat_msg;

            m.scale.x = SIDE_ZONE_END - SIDE_ZONE_START;
            m.scale.y = SIDE_ZONE_OUTER - SIDE_ZONE_INNER;
            m.scale.z = 0.01;

            if (!current_surrounding_.left_clear)
            {
                m.color.r = 1.0;
                m.color.g = 0.0;
                m.color.b = 0.0;
            }
            else
            {
                m.color.r = 0.0;
                m.color.g = 0.5;
                m.color.b = 1.0;
            }

            m.color.a = 0.4;
            m.lifetime = rclcpp::Duration::from_seconds(0.1);
            markers.markers.push_back(m);
        }

        // RIGHT BLIND SPOT
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "world";
            m.header.stamp = this->now();
            m.ns = "zones";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::CUBE;
            m.action = visualization_msgs::msg::Marker::ADD;

            double center_x = (SIDE_ZONE_START + SIDE_ZONE_END) / 2;
            double center_y = -(SIDE_ZONE_INNER + SIDE_ZONE_OUTER) / 2;
            m.pose.position.x = ego_x_ + cos_yaw * center_x - sin_yaw * center_y;
            m.pose.position.y = ego_y_ + sin_yaw * center_x + cos_yaw * center_y;
            m.pose.position.z = 0.02;
            m.pose.orientation = quat_msg;

            m.scale.x = SIDE_ZONE_END - SIDE_ZONE_START;
            m.scale.y = SIDE_ZONE_OUTER - SIDE_ZONE_INNER;
            m.scale.z = 0.01;

            if (!current_surrounding_.right_clear)
            {
                m.color.r = 1.0;
                m.color.g = 0.0;
                m.color.b = 0.0;
            }
            else
            {
                m.color.r = 0.0;
                m.color.g = 0.5;
                m.color.b = 1.0;
            }

            m.color.a = 0.4;
            m.lifetime = rclcpp::Duration::from_seconds(0.1);
            markers.markers.push_back(m);
        }

        // STATE TEXT
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "world";
            m.header.stamp = this->now();
            m.ns = "state";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = ego_x_;
            m.pose.position.y = ego_y_;
            m.pose.position.z = 0.4;
            m.scale.z = 0.1;
            m.color.r = m.color.g = m.color.b = m.color.a = 1.0;

            std::stringstream ss;
            ss << state_str(current_state_);
            if (is_in_corner_)
                ss << " [C]";
            ss << "\n"
               << std::fixed << std::setprecision(2) << ego_speed_ << " m/s\n"
               << lane_str(current_lane_);
            if (target_lane_ != current_lane_)
                ss << "->" << lane_str(target_lane_);
            m.text = ss.str();
            m.lifetime = rclcpp::Duration::from_seconds(0.1);
            markers.markers.push_back(m);
        }

        debug_pub_->publish(markers);

        // Lookahead marker
        visualization_msgs::msg::Marker lm;
        lm.header.frame_id = "world";
        lm.header.stamp = this->now();
        lm.ns = "lookahead";
        lm.id = 0;
        lm.type = visualization_msgs::msg::Marker::SPHERE;
        lm.action = visualization_msgs::msg::Marker::ADD;
        lm.pose.position.x = poses[lookahead_idx].pose.position.x;
        lm.pose.position.y = poses[lookahead_idx].pose.position.y;
        lm.pose.position.z = 0.15;
        lm.scale.x = lm.scale.y = lm.scale.z = 0.08;
        lm.color.r = 0.0;
        lm.color.g = 1.0;
        lm.color.b = 0.5;
        lm.color.a = 1.0;
        lm.lifetime = rclcpp::Duration::from_seconds(0.1);
        lookahead_pub_->publish(lm);
    }
};

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrenetPlanner>());
    rclcpp::shutdown();
    return 0;
}
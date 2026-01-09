/**
 * @file frenet_planner.cpp
 * @brief Advanced Lane Change Planner for RC-Scale Autonomous Vehicle (v4.0)
 *
 * Key Features:
 * - Static obstacle detection and avoidance
 * - Emergency escape logic with timeout
 * - Lane boundary enforcement
 * - Corner heading alignment
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
    LANE_1 = 0, // Innermost - Static HVs
    LANE_2 = 1, // Middle - Slow Dynamic HVs
    LANE_3 = 2, // Outermost - Fast Dynamic HVs
    NONE = -1
};

enum class DrivingState
{
    CRUISE,
    PREPARE_CHANGE,
    LANE_CHANGING,
    EMERGENCY_ACCEL,
    EMERGENCY_BRAKE,
    OVERTAKE_STATIC // Static �μ븷臾� 異붿썡 以�
};

struct ObstacleInfo
{
    int id;
    double x, y;
    double vx, vy;
    double speed;
    LaneID lane;
    double rel_x, rel_y;
    double rel_vx;
    double last_seen_sec;
    bool is_static;
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
        RCLCPP_INFO(this->get_logger(), "Frenet Planner v4.0 - Overtake Algorithm");
        RCLCPP_INFO(this->get_logger(), "============================================");

        declare_parameters();
        load_parameters();

        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&FrenetPlanner::param_callback, this, std::placeholders::_1));

        setup_ros_interfaces();
        reset_state();

        timer_ = this->create_wall_timer(20ms, std::bind(&FrenetPlanner::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Planner initialized. Waiting for data...");
    }

private:
    // ========================================================================
    // CONSTANTS
    // ========================================================================

    static constexpr double VEHICLE_LENGTH = 0.33;
    static constexpr double VEHICLE_WIDTH = 0.15;
    static constexpr double VEHICLE_LENGTH_F = 0.17;
    static constexpr double VEHICLE_LENGTH_R = 0.16;

    // Lane
    static constexpr double LANE_WIDTH = 0.25;
    static constexpr double LANE_THRESHOLD = 0.35;
    static constexpr double LANE_HARD_LIMIT = 0.10; // 李⑥꽑 以묒떖�먯꽌 理쒕� �댄깉 嫄곕━

    // Safety
    // Safety distances
    static constexpr double FRONT_SAFE_DIST = 0.5;
    static constexpr double FRONT_CRITICAL_DIST = 0.3;
    static constexpr double FRONT_EMERGENCY_DIST = 0.2;
    static constexpr double REAR_DANGER_DIST = 0.5;  // 0.35 -> 0.5 利앷�
    static constexpr double BLIND_SPOT_LENGTH = 0.5; // 0.35 -> 0.5 利앷�
    static constexpr double SIDE_MARGIN = 0.25;      // 0.25 -> 0.3 利앷�

    // Static obstacle
    static constexpr double STATIC_SPEED_THRESHOLD = 0.05;
    static constexpr double EMERGENCY_TIMEOUT = 1.0;

    // ========================================================================
    // PARAMETERS
    // ========================================================================

    double wheelbase_ = 0.33;
    double max_steer_ = 0.7;
    double target_speed_ = 0.45;
    double boost_speed_ = 0.6;
    double base_lookahead_ = 0.25;
    double max_lookahead_ = 0.4;
    double weight_heading_ = 0.5;
    double weight_path_ = 0.5;
    double steer_alpha_ = 0.35;

    // ========================================================================
    // STATE VARIABLES
    // ========================================================================

    DrivingState current_state_ = DrivingState::CRUISE;
    LaneID current_lane_ = LaneID::LANE_2;
    LaneID target_lane_ = LaneID::LANE_2;

    bool lane_change_committed_ = false;
    double lane_change_start_sec_ = 0.0;
    int safe_check_count_ = 0;
    static constexpr int SAFE_CHECK_REQUIRED = 3;

    double ego_x_ = 0.0, ego_y_ = 0.0, ego_yaw_ = 0.0;
    double ego_speed_ = 0.0;
    bool pose_received_ = false;

    double prev_x_ = 0.0, prev_y_ = 0.0;
    double prev_pose_sec_ = 0.0;
    bool prev_pose_valid_ = false;

    int last_closest_idx_ = 0;
    int last_closest_idx_lane1_ = 0;
    int last_closest_idx_lane2_ = 0;
    int last_closest_idx_lane3_ = 0;
    double last_steer_ = 0.0;

    bool initial_search_done_ = false;
    double emergency_start_sec_ = 0.0;
    int static_obstacle_id_ = -1;

    std::map<int, ObstacleInfo> obstacles_;

    double last_log_sec_ = 0.0;
    double last_detail_log_sec_ = 0.0;

    // ========================================================================
    // ROS INTERFACES
    // ========================================================================

    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_lane_1_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_lane_2_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_lane_3_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obs_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr accel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path::SharedPtr path_lane_1_;
    nav_msgs::msg::Path::SharedPtr path_lane_2_;
    nav_msgs::msg::Path::SharedPtr path_lane_3_;

    // ========================================================================
    // HELPER
    // ========================================================================

    double now_sec()
    {
        return this->now().seconds();
    }

    double normalize_angle(double angle)
    {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }

    // ========================================================================
    // INITIALIZATION
    // ========================================================================

    void declare_parameters()
    {
        this->declare_parameter("wheelbase", 0.33);
        this->declare_parameter("max_steer", 0.7);
        this->declare_parameter("target_speed", 0.45);
        this->declare_parameter("boost_speed", 0.6);
        this->declare_parameter("base_lookahead", 0.25);
        this->declare_parameter("max_lookahead", 0.4);
        this->declare_parameter("weight_heading", 0.5);
        this->declare_parameter("weight_path", 0.5);
        this->declare_parameter("steer_alpha", 0.35);
        this->declare_parameter("reset_trigger", false);
    }

    void load_parameters()
    {
        this->get_parameter("wheelbase", wheelbase_);
        this->get_parameter("max_steer", max_steer_);
        this->get_parameter("target_speed", target_speed_);
        this->get_parameter("boost_speed", boost_speed_);
        this->get_parameter("base_lookahead", base_lookahead_);
        this->get_parameter("max_lookahead", max_lookahead_);
        this->get_parameter("weight_heading", weight_heading_);
        this->get_parameter("weight_path", weight_path_);
        this->get_parameter("steer_alpha", steer_alpha_);
    }

    rcl_interfaces::msg::SetParametersResult param_callback(
        const std::vector<rclcpp::Parameter> &params)
    {
        for (const auto &p : params)
        {
            if (p.get_name() == "reset_trigger" && p.as_bool())
            {
                reset_state();
            }
        }
        load_parameters();
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    void setup_ros_interfaces()
    {
        auto map_qos = rclcpp::QoS(10).transient_local();

        sub_lane_1_ = this->create_subscription<nav_msgs::msg::Path>(
            "/hdmap/lane_one", map_qos,
            [this](nav_msgs::msg::Path::SharedPtr msg)
            {
                path_lane_1_ = msg;
                RCLCPP_INFO_ONCE(this->get_logger(), "Lane 1 received: %zu points", msg->poses.size());
            });
        sub_lane_2_ = this->create_subscription<nav_msgs::msg::Path>(
            "/hdmap/lane_two", map_qos,
            [this](nav_msgs::msg::Path::SharedPtr msg)
            {
                path_lane_2_ = msg;
                RCLCPP_INFO_ONCE(this->get_logger(), "Lane 2 received: %zu points", msg->poses.size());
            });
        sub_lane_3_ = this->create_subscription<nav_msgs::msg::Path>(
            "/hdmap/lane_three", map_qos,
            [this](nav_msgs::msg::Path::SharedPtr msg)
            {
                path_lane_3_ = msg;
                RCLCPP_INFO_ONCE(this->get_logger(), "Lane 3 received: %zu points", msg->poses.size());
            });

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
        lane_change_committed_ = false;
        lane_change_start_sec_ = 0.0;
        safe_check_count_ = 0;
        ego_speed_ = 0.0;
        last_closest_idx_ = 0;
        last_closest_idx_lane1_ = 0;
        last_closest_idx_lane2_ = 0;
        last_closest_idx_lane3_ = 0;
        last_steer_ = 0.0;
        initial_search_done_ = false;
        emergency_start_sec_ = 0.0;
        static_obstacle_id_ = -1;
        prev_pose_valid_ = false;
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
                double dist = std::hypot(dx, dy);
                double raw_speed = dist / dt;

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

        current_lane_ = get_lane_at_position(ego_x_, ego_y_);
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
                obs.lane = get_lane_at_position(ox, oy);
                obs.last_seen_sec = current_sec;
                obs.is_static = (obs.speed < STATIC_SPEED_THRESHOLD);
            }
            else
            {
                ObstacleInfo obs;
                obs.id = id;
                obs.x = ox;
                obs.y = oy;
                obs.vx = 0.0;
                obs.vy = 0.0;
                obs.speed = 0.0;
                obs.lane = get_lane_at_position(ox, oy);
                obs.last_seen_sec = current_sec;
                obs.is_static = true;
                obstacles_[id] = obs;
            }
        }

        for (auto it = obstacles_.begin(); it != obstacles_.end();)
        {
            if ((current_sec - it->second.last_seen_sec) > 1.0)
                it = obstacles_.erase(it);
            else
                ++it;
        }
    }

    // ========================================================================
    // LANE DETECTION
    // ========================================================================

    double get_dist_to_lane(const nav_msgs::msg::Path::SharedPtr &path, double x, double y)
    {
        if (!path || path->poses.empty())
            return 999.0;

        double min_d = 999.0;
        size_t step = std::max(size_t(1), path->poses.size() / 500);
        for (size_t i = 0; i < path->poses.size(); i += step)
        {
            double d = std::hypot(path->poses[i].pose.position.x - x,
                                  path->poses[i].pose.position.y - y);
            if (d < min_d)
                min_d = d;
        }
        return min_d;
    }

    LaneID get_lane_at_position(double x, double y)
    {
        double d1 = get_dist_to_lane(path_lane_1_, x, y);
        double d2 = get_dist_to_lane(path_lane_2_, x, y);
        double d3 = get_dist_to_lane(path_lane_3_, x, y);

        double min_d = std::min({d1, d2, d3});

        if (min_d > LANE_THRESHOLD)
            return LaneID::NONE;

        if (d1 <= d2 && d1 <= d3)
            return LaneID::LANE_1;
        if (d2 <= d1 && d2 <= d3)
            return LaneID::LANE_2;
        return LaneID::LANE_3;
    }

    nav_msgs::msg::Path::SharedPtr get_lane_path(LaneID lane)
    {
        switch (lane)
        {
        case LaneID::LANE_1:
            return path_lane_1_;
        case LaneID::LANE_2:
            return path_lane_2_;
        case LaneID::LANE_3:
            return path_lane_3_;
        default:
            return path_lane_2_;
        }
    }

    std::string lane_to_string(LaneID lane)
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
            return "NONE";
        }
    }

    // ========================================================================
    // CROSS-TRACK ERROR
    // ========================================================================

    double compute_cross_track_error(const nav_msgs::msg::Path::SharedPtr &path, int closest)
    {
        if (!path || path->poses.empty())
            return 0.0;

        int n = path->poses.size();
        double cx = path->poses[closest].pose.position.x;
        double cy = path->poses[closest].pose.position.y;

        int next = (closest + 3) % n;
        double path_dx = path->poses[next].pose.position.x - cx;
        double path_dy = path->poses[next].pose.position.y - cy;
        double path_yaw = std::atan2(path_dy, path_dx);

        double dx = ego_x_ - cx;
        double dy = ego_y_ - cy;

        return -dx * std::sin(path_yaw) + dy * std::cos(path_yaw);
    }

    // ========================================================================
    // BODY FRAME TRANSFORM
    // ========================================================================

    void update_obstacle_body_frame(ObstacleInfo &obs)
    {
        double dx = obs.x - ego_x_;
        double dy = obs.y - ego_y_;
        double cos_yaw = std::cos(ego_yaw_);
        double sin_yaw = std::sin(ego_yaw_);

        obs.rel_x = dx * cos_yaw + dy * sin_yaw;
        obs.rel_y = -dx * sin_yaw + dy * cos_yaw;
        obs.rel_vx = (obs.vx * cos_yaw + obs.vy * sin_yaw) - ego_speed_;
    }

    // ========================================================================
    // THREAT ASSESSMENT
    // ========================================================================

    struct LaneStatus
    {
        bool blocked_front = false;
        bool blocked_by_static = false;
        bool danger_rear = false;
        bool blind_spot = false;
        double front_dist = 999.0;
        double front_speed = 0.0;
        double rear_dist = 999.0;
        double rear_speed = 0.0;
        int front_obs_id = -1;
        bool front_is_static = false;
    };

    LaneStatus assess_lane(LaneID lane, bool check_side = false)
    {
        LaneStatus status;
        double current_sec = now_sec();

        for (auto &[id, obs] : obstacles_)
        {
            if ((current_sec - obs.last_seen_sec) > 0.5)
                continue;

            update_obstacle_body_frame(obs);

            bool is_on_lane = (obs.lane == lane);
            bool is_adjacent = false;

            if (!is_on_lane)
            {
                double d = get_dist_to_lane(get_lane_path(lane), obs.x, obs.y);
                is_adjacent = (d < LANE_THRESHOLD * 1.5);
            }

            if (!is_on_lane && !is_adjacent)
                continue;

            double lateral_dist = std::abs(obs.rel_y);

            // Front obstacle (always check)
            if (obs.rel_x > -VEHICLE_LENGTH_R && obs.rel_x < 4.0)
            {
                if (lateral_dist < SIDE_MARGIN)
                {
                    if (obs.rel_x < status.front_dist)
                    {
                        status.front_dist = obs.rel_x;
                        status.front_speed = obs.speed;
                        status.front_obs_id = id;
                        status.front_is_static = obs.is_static;

                        if (obs.rel_x < FRONT_SAFE_DIST)
                        {
                            status.blocked_front = true;
                        }
                    }
                }
            }

            // Rear obstacle (always check)
            if (obs.rel_x < VEHICLE_LENGTH_F && obs.rel_x > -3.0)
            {
                if (lateral_dist < SIDE_MARGIN)
                {
                    double rear_d = -obs.rel_x;
                    if (rear_d > 0 && rear_d < status.rear_dist)
                    {
                        status.rear_dist = rear_d;
                        status.rear_speed = obs.speed;

                        if (rear_d < REAR_DANGER_DIST && obs.speed > ego_speed_ + 0.1)
                        {
                            status.danger_rear = true;
                        }
                    }
                }
            }

            // Side/Blind spot - 李⑥꽑 蹂�寃� �쒖뿉留� 泥댄겕!
            if (check_side)
            {
                // Side-Rear (�꾩륫諛�) - 鍮좊Ⅴ寃� �묎렐�섎뒗 李⑤웾
                if (obs.rel_x > -1.5 && obs.rel_x < 0.8)
                {
                    if (lateral_dist > 0.1 && lateral_dist < 0.5)
                    {
                        double approach_speed = -obs.rel_vx;
                        if (approach_speed > 0.1 || std::abs(obs.rel_x) < 0.5)
                        {
                            status.blind_spot = true;
                        }
                    }
                }

                // Blind spot (李⑤웾 諛붾줈 ��)
                if (std::abs(obs.rel_x) < BLIND_SPOT_LENGTH && lateral_dist < SIDE_MARGIN * 1.3)
                {
                    status.blind_spot = true;
                }
            }
        }

        return status;
    }
    // ========================================================================
    // LANE CHANGE DECISION
    // ========================================================================

    LaneID choose_overtake_lane()
    {
        std::vector<LaneID> candidates;

        if (current_lane_ == LaneID::LANE_1)
        {
            candidates = {LaneID::LANE_2};
        }
        else if (current_lane_ == LaneID::LANE_2)
        {
            candidates = {LaneID::LANE_3, LaneID::LANE_1};
        }
        else
        {
            candidates = {LaneID::LANE_2};
        }

        for (LaneID cand : candidates)
        {
            LaneStatus status = assess_lane(cand);

            if (!status.blind_spot && !status.blocked_front && status.front_dist > FRONT_SAFE_DIST)
            {
                return cand;
            }
        }

        for (LaneID cand : candidates)
        {
            LaneStatus status = assess_lane(cand);
            if (!status.blind_spot && status.front_dist > FRONT_CRITICAL_DIST)
            {
                return cand;
            }
        }

        return current_lane_;
    }

    bool is_lane_change_safe(LaneID target, bool emergency = false)
    {
        LaneStatus status = assess_lane(target, true);

        // Blind spot (includes side-rear) - always check
        if (status.blind_spot)
            return false;

        if (emergency)
        {
            // Emergency: only blind spot check + minimum front space
            if (status.front_dist < VEHICLE_LENGTH * 0.8)
                return false;
            return true;
        }

        // Normal safety check
        if (status.danger_rear)
            return false;
        if (status.front_dist < FRONT_CRITICAL_DIST)
            return false;

        // 異붽�: target lane �꾨갑�먯꽌 鍮좊Ⅴ寃� �묎렐�섎뒗 李⑤웾 泥댄겕
        for (const auto &[id, obs] : obstacles_)
        {
            if (obs.lane != target)
                continue;

            // �꾨갑 李⑤웾�� �묎렐 �쒓컙(TTC) 怨꾩궛
            double dx = obs.x - ego_x_;
            double dy = obs.y - ego_y_;
            double rel_speed = obs.speed - ego_speed_;

            // �꾨갑�먯꽌 鍮좊Ⅴ寃� �묎렐�섎㈃ �꾪뿕
            if (rel_speed > 0.15) // �곷� �띾룄 0.15m/s �댁긽�쇰줈 �묎렐
            {
                double dist = std::hypot(dx, dy);
                double ttc = dist / rel_speed; // Time to collision

                if (ttc < 2.0) // 2珥� �대궡 異⑸룎 �덉긽
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                         "TTC warning: %.1fs from obs %d", ttc, id);
                    return false;
                }
            }
        }

        // Future collision check
        for (double t = 0.2; t <= 1.5; t += 0.2)
        {
            double my_future_x = ego_x_ + ego_speed_ * std::cos(ego_yaw_) * t;
            double my_future_y = ego_y_ + ego_speed_ * std::sin(ego_yaw_) * t;

            for (const auto &[id, obs] : obstacles_)
            {
                double obs_future_x = obs.x + obs.vx * t;
                double obs_future_y = obs.y + obs.vy * t;

                double dist = std::hypot(my_future_x - obs_future_x, my_future_y - obs_future_y);
                if (dist < VEHICLE_LENGTH * 0.9)
                {
                    return false;
                }
            }
        }

        return true;
    }

    // ========================================================================
    // STATE MACHINE
    // ========================================================================

    void update_state_machine()
    {
        LaneStatus current_status = assess_lane(current_lane_, false);
        double current_sec = now_sec();

        if ((current_sec - last_detail_log_sec_) > 1.5)
        {
            RCLCPP_INFO(this->get_logger(),
                        "[%s] Lane=%s | Spd=%.2f | Front=%.2f(static=%d) | Obs=%zu",
                        state_to_string(current_state_).c_str(),
                        lane_to_string(current_lane_).c_str(),
                        ego_speed_,
                        current_status.front_dist,
                        current_status.front_is_static ? 1 : 0,
                        obstacles_.size());
            last_detail_log_sec_ = current_sec;
        }

        switch (current_state_)
        {
        case DrivingState::CRUISE:
        {
            if (current_status.danger_rear)
            {
                current_state_ = DrivingState::EMERGENCY_ACCEL;
                RCLCPP_WARN(this->get_logger(), "REAR THREAT! Accelerating...");
                break;
            }

            if (current_status.blocked_by_static && current_status.front_dist < FRONT_SAFE_DIST)
            {
                LaneID overtake = choose_overtake_lane();
                if (overtake != current_lane_)
                {
                    target_lane_ = overtake;
                    current_state_ = DrivingState::OVERTAKE_STATIC;
                    static_obstacle_id_ = current_status.front_obs_id;
                    RCLCPP_INFO(this->get_logger(), "STATIC detected! Overtaking %s -> %s",
                                lane_to_string(current_lane_).c_str(),
                                lane_to_string(target_lane_).c_str());
                    break;
                }
            }

            if (current_status.blocked_front && !current_status.blocked_by_static)
            {
                if (current_status.front_speed < ego_speed_ - 0.05 ||
                    current_status.front_dist < FRONT_CRITICAL_DIST)
                {
                    LaneID best = choose_overtake_lane();
                    if (best != current_lane_)
                    {
                        target_lane_ = best;
                        current_state_ = DrivingState::PREPARE_CHANGE;
                        safe_check_count_ = 0;
                        RCLCPP_INFO(this->get_logger(), "Preparing overtake: %s -> %s",
                                    lane_to_string(current_lane_).c_str(),
                                    lane_to_string(target_lane_).c_str());
                    }
                }
            }

            if (current_status.front_dist < FRONT_EMERGENCY_DIST && !current_status.blocked_by_static)
            {
                current_state_ = DrivingState::EMERGENCY_BRAKE;
                emergency_start_sec_ = current_sec;
                RCLCPP_WARN(this->get_logger(), "EMERGENCY BRAKE! Front=%.2f", current_status.front_dist);
            }
            break;
        }

        case DrivingState::OVERTAKE_STATIC:
        {
            if (is_lane_change_safe(target_lane_))
            {
                safe_check_count_++;
                if (safe_check_count_ >= 2)
                {
                    current_state_ = DrivingState::LANE_CHANGING;
                    lane_change_committed_ = true;
                    lane_change_start_sec_ = current_sec;
                    RCLCPP_INFO(this->get_logger(), "Overtake COMMITTED: %s -> %s",
                                lane_to_string(current_lane_).c_str(),
                                lane_to_string(target_lane_).c_str());
                }
            }
            else
            {
                safe_check_count_ = std::max(0, safe_check_count_ - 1);
            }
            break;
        }

        case DrivingState::PREPARE_CHANGE:
        {
            if (is_lane_change_safe(target_lane_))
            {
                safe_check_count_++;
                if (safe_check_count_ >= SAFE_CHECK_REQUIRED)
                {
                    current_state_ = DrivingState::LANE_CHANGING;
                    lane_change_committed_ = true;
                    lane_change_start_sec_ = current_sec;
                    RCLCPP_INFO(this->get_logger(), "Lane change COMMITTED: %s -> %s",
                                lane_to_string(current_lane_).c_str(),
                                lane_to_string(target_lane_).c_str());
                }
            }
            else
            {
                safe_check_count_ = std::max(0, safe_check_count_ - 1);
            }

            if (safe_check_count_ == 0 && !current_status.blocked_front)
            {
                current_state_ = DrivingState::CRUISE;
                target_lane_ = current_lane_;
                RCLCPP_INFO(this->get_logger(), "Lane change cancelled - clear ahead");
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
                    lane_change_committed_ = false;
                    static_obstacle_id_ = -1;
                    RCLCPP_INFO(this->get_logger(), "Lane change COMPLETE: now in %s",
                                lane_to_string(current_lane_).c_str());
                }
            }

            double elapsed = current_sec - lane_change_start_sec_;
            if (elapsed > 5.0)
            {
                current_state_ = DrivingState::CRUISE;
                lane_change_committed_ = false;
                target_lane_ = current_lane_;
                static_obstacle_id_ = -1;
                RCLCPP_WARN(this->get_logger(), "Lane change TIMEOUT");
            }
            break;
        }

        case DrivingState::EMERGENCY_ACCEL:
        {
            if (!current_status.danger_rear && current_status.rear_dist > REAR_DANGER_DIST)
            {
                current_state_ = DrivingState::CRUISE;
                RCLCPP_INFO(this->get_logger(), "Rear clear, resuming cruise");
            }

            if (current_status.front_dist < FRONT_CRITICAL_DIST)
            {
                LaneID escape = choose_overtake_lane();
                if (escape != current_lane_ && is_lane_change_safe(escape))
                {
                    target_lane_ = escape;
                    current_state_ = DrivingState::LANE_CHANGING;
                    lane_change_committed_ = true;
                    lane_change_start_sec_ = current_sec;
                }
            }
            break;
        }

        case DrivingState::EMERGENCY_BRAKE:
        {
            double elapsed = current_sec - emergency_start_sec_;

            if (current_status.front_dist > FRONT_SAFE_DIST)
            {
                current_state_ = DrivingState::CRUISE;
                RCLCPP_INFO(this->get_logger(), "Front clear, resuming cruise");
                break;
            }

            if (elapsed > EMERGENCY_TIMEOUT || current_status.blocked_by_static)
            {
                LaneID escape = choose_overtake_lane();
                if (escape != current_lane_)
                {
                    LaneStatus escape_status = assess_lane(escape);
                    if (!escape_status.blind_spot)
                    {
                        target_lane_ = escape;
                        current_state_ = DrivingState::LANE_CHANGING;
                        lane_change_committed_ = true;
                        lane_change_start_sec_ = current_sec;
                        RCLCPP_WARN(this->get_logger(), "ESCAPE! Forcing lane change to %s",
                                    lane_to_string(escape).c_str());
                    }
                }
            }
            break;
        }
        }
    }

    std::string state_to_string(DrivingState s)
    {
        switch (s)
        {
        case DrivingState::CRUISE:
            return "CRUISE";
        case DrivingState::PREPARE_CHANGE:
            return "PREPARE";
        case DrivingState::LANE_CHANGING:
            return "CHANGING";
        case DrivingState::EMERGENCY_ACCEL:
            return "ACCEL!";
        case DrivingState::EMERGENCY_BRAKE:
            return "BRAKE!";
        case DrivingState::OVERTAKE_STATIC:
            return "OVERTAKE";
        default:
            return "???";
        }
    }

    // ========================================================================
    // PATH FOLLOWING
    // ========================================================================

    int find_closest_index(const nav_msgs::msg::Path::SharedPtr &path, LaneID lane)
    {
        if (!path || path->poses.empty())
            return 0;

        int n = path->poses.size();

        int last_idx = 0;
        switch (lane)
        {
        case LaneID::LANE_1:
            last_idx = last_closest_idx_lane1_;
            break;
        case LaneID::LANE_2:
            last_idx = last_closest_idx_lane2_;
            break;
        case LaneID::LANE_3:
            last_idx = last_closest_idx_lane3_;
            break;
        default:
            last_idx = last_closest_idx_;
            break;
        }

        int best = last_idx;
        double min_d = 999.0;

        bool need_global = !initial_search_done_ ||
                           current_state_ == DrivingState::LANE_CHANGING ||
                           current_lane_ != target_lane_;

        if (need_global)
        {
            for (int i = 0; i < n; ++i)
            {
                double d = std::hypot(path->poses[i].pose.position.x - ego_x_,
                                      path->poses[i].pose.position.y - ego_y_);
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
                int idx = (last_idx + i) % n;
                double d = std::hypot(path->poses[idx].pose.position.x - ego_x_,
                                      path->poses[idx].pose.position.y - ego_y_);
                if (d < min_d)
                {
                    min_d = d;
                    best = idx;
                }
            }
        }

        switch (lane)
        {
        case LaneID::LANE_1:
            last_closest_idx_lane1_ = best;
            break;
        case LaneID::LANE_2:
            last_closest_idx_lane2_ = best;
            break;
        case LaneID::LANE_3:
            last_closest_idx_lane3_ = best;
            break;
        default:
            break;
        }

        return best;
    }

    int find_lookahead_index(const nav_msgs::msg::Path::SharedPtr &path, int closest, double dist)
    {
        if (!path || path->poses.empty())
            return 0;

        int n = path->poses.size();

        for (int i = 0; i < 150; ++i)
        {
            int idx = (closest + i) % n;
            double d = std::hypot(path->poses[idx].pose.position.x - ego_x_,
                                  path->poses[idx].pose.position.y - ego_y_);
            if (d >= dist)
                return idx;
        }

        return (closest + 30) % n;
    }

    // ========================================================================
    // CURVATURE
    // ========================================================================

    double compute_curvature(const nav_msgs::msg::Path::SharedPtr &path, int idx, int window = 10)
    {
        if (!path || path->poses.size() < 30)
            return 0.0;

        int n = path->poses.size();

        int i0 = ((idx - window) % n + n) % n;
        int i1 = idx;
        int i2 = (idx + window) % n;

        double x0 = path->poses[i0].pose.position.x;
        double y0 = path->poses[i0].pose.position.y;
        double x1 = path->poses[i1].pose.position.x;
        double y1 = path->poses[i1].pose.position.y;
        double x2 = path->poses[i2].pose.position.x;
        double y2 = path->poses[i2].pose.position.y;

        double a = std::hypot(x1 - x0, y1 - y0);
        double b = std::hypot(x2 - x1, y2 - y1);
        double c = std::hypot(x0 - x2, y0 - y2);

        double s = (a + b + c) / 2.0;
        double area_sq = s * (s - a) * (s - b) * (s - c);

        if (area_sq <= 0 || a < 0.001 || b < 0.001 || c < 0.001)
            return 0.0;

        double area = std::sqrt(area_sq);
        double curvature = 4.0 * area / (a * b * c);

        double cross = (x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1);
        return (cross >= 0) ? curvature : -curvature;
    }

    // ========================================================================
    // STEERING
    // ========================================================================

    double compute_steering(const nav_msgs::msg::Path::SharedPtr &path, int closest, int lookahead)
    {
        if (!path || path->poses.empty())
            return 0.0;

        int n = path->poses.size();

        // === Lookahead point ===
        double tx = path->poses[lookahead].pose.position.x;
        double ty = path->poses[lookahead].pose.position.y;

        double dx = tx - ego_x_;
        double dy = ty - ego_y_;
        double dist = std::hypot(dx, dy);
        double target_yaw = std::atan2(dy, dx);

        // === Road direction ===
        int next = (lookahead + 5) % n;
        double road_dx = path->poses[next].pose.position.x - tx;
        double road_dy = path->poses[next].pose.position.y - ty;
        double road_yaw = std::atan2(road_dy, road_dx);

        // === Errors ===
        double heading_err = normalize_angle(target_yaw - ego_yaw_);
        double road_err = normalize_angle(road_yaw - ego_yaw_);

        // === Cross-track error ===
        double cross_track = compute_cross_track_error(path, closest);

        // === Curvature ===
        double curvature = compute_curvature(path, closest, 10);
        double curvature_ahead = compute_curvature(path, (closest + 15) % n, 10);
        double max_curv = std::max(std::abs(curvature), std::abs(curvature_ahead));
        bool is_corner = max_curv > 0.5;

        // === Steering calculation ===
        double steer_pp = std::atan2(2.0 * wheelbase_ * std::sin(heading_err), std::max(dist, 0.08));

        double steer_cmd = 0.0;

        if (is_corner)
        {
            // 肄붾꼫: feedforward + road alignment 媛뺥솕
            double k_curv = 0.5;
            double feedforward = k_curv * curvature_ahead * wheelbase_;
            steer_cmd = steer_pp * 0.4 + road_err * 0.9 + feedforward;
        }
        else
        {
            // 吏곸꽑: Pure pursuit
            steer_cmd = steer_pp * 0.6 + road_err * 0.4;
        }

        // === 李⑥꽑 �댄깉 蹂댁젙 ===
        if (std::abs(cross_track) > LANE_HARD_LIMIT)
        {
            double correction = -cross_track * 2.5;
            correction = std::clamp(correction, -0.25, 0.25);
            steer_cmd += correction;
        }

        // === Smoothing ===
        double alpha = is_corner ? 0.5 : 0.3;
        steer_cmd = steer_cmd * alpha + last_steer_ * (1.0 - alpha);

        steer_cmd = std::clamp(steer_cmd, -max_steer_, max_steer_);

        return steer_cmd;
    }

    // ========================================================================
    // VELOCITY
    // ========================================================================

    double compute_velocity()
    {
        LaneStatus status = assess_lane(current_lane_, false);
        double vel = target_speed_;

        switch (current_state_)
        {
        case DrivingState::CRUISE:
        {
            if (status.blocked_front && !status.blocked_by_static)
            {
                double gap = status.front_dist - FRONT_CRITICAL_DIST;
                vel = std::min(target_speed_, std::max(status.front_speed * 0.9, gap * 2.0));
            }
            break;
        }

        case DrivingState::OVERTAKE_STATIC:
        {
            vel = std::max(0.2, target_speed_ * 0.5);
            break;
        }

        case DrivingState::PREPARE_CHANGE:
        {
            LaneStatus target_status = assess_lane(target_lane_);

            if (target_status.rear_speed > ego_speed_)
            {
                vel = boost_speed_;
            }
            else if (status.blocked_front)
            {
                vel = std::max(0.25, status.front_speed * 0.9);
            }
            break;
        }

        case DrivingState::LANE_CHANGING:
        {
            vel = boost_speed_;
            break;
        }

        case DrivingState::EMERGENCY_ACCEL:
        {
            vel = std::min(1.0, boost_speed_ * 1.2);
            break;
        }

        case DrivingState::EMERGENCY_BRAKE:
        {
            if (status.front_dist < FRONT_EMERGENCY_DIST * 0.5)
                vel = 0.1;
            else
                vel = std::max(0.15, (status.front_dist - 0.1) * 1.5);
            break;
        }
        }

        return std::clamp(vel, 0.0, 1.2);
    }

    // ========================================================================
    // MAIN CONTROL LOOP
    // ========================================================================

    void control_loop()
    {
        if (!pose_received_)
            return;

        if (!initial_search_done_)
        {
            if (path_lane_1_ && !path_lane_1_->poses.empty())
                find_closest_index(path_lane_1_, LaneID::LANE_1);
            if (path_lane_2_ && !path_lane_2_->poses.empty())
                find_closest_index(path_lane_2_, LaneID::LANE_2);
            if (path_lane_3_ && !path_lane_3_->poses.empty())
                find_closest_index(path_lane_3_, LaneID::LANE_3);

            initial_search_done_ = true;
            RCLCPP_INFO(this->get_logger(), "Initial search completed");
        }

        if (current_lane_ == LaneID::NONE)
        {
            double d1 = get_dist_to_lane(path_lane_1_, ego_x_, ego_y_);
            double d2 = get_dist_to_lane(path_lane_2_, ego_x_, ego_y_);
            double d3 = get_dist_to_lane(path_lane_3_, ego_x_, ego_y_);

            if (d1 <= d2 && d1 <= d3)
                current_lane_ = LaneID::LANE_1;
            else if (d2 <= d1 && d2 <= d3)
                current_lane_ = LaneID::LANE_2;
            else
                current_lane_ = LaneID::LANE_3;

            target_lane_ = current_lane_;
        }

        nav_msgs::msg::Path::SharedPtr path = get_lane_path(target_lane_);
        if (!path || path->poses.empty())
            path = get_lane_path(current_lane_);
        if (!path || path->poses.empty())
            path = path_lane_2_;
        if (!path || path->poses.empty())
            return;

        update_state_machine();

        LaneID path_lane = (current_state_ == DrivingState::LANE_CHANGING ||
                            current_state_ == DrivingState::OVERTAKE_STATIC)
                               ? target_lane_
                               : current_lane_;
        int closest = find_closest_index(path, path_lane);
        last_closest_idx_ = closest;

        int preview_idx = (closest + 20) % path->poses.size();
        double curvature = compute_curvature(path, preview_idx, 10);
        bool is_corner = std::abs(curvature) > 0.5;

        double lookahead_dist;
        if (is_corner)
            lookahead_dist = 0.12;
        else if (current_state_ == DrivingState::LANE_CHANGING ||
                 current_state_ == DrivingState::OVERTAKE_STATIC)
            lookahead_dist = 0.18;
        else
            lookahead_dist = base_lookahead_;

        lookahead_dist = std::clamp(lookahead_dist, 0.08, max_lookahead_);

        int lookahead_idx = find_lookahead_index(path, closest, lookahead_dist);

        double steer = compute_steering(path, closest, lookahead_idx);
        last_steer_ = steer;

        double vel = compute_velocity();

        if (is_corner && current_state_ != DrivingState::EMERGENCY_ACCEL)
        {
            vel = std::min(vel, 0.35);
        }

        geometry_msgs::msg::Accel cmd;
        cmd.linear.x = vel;
        cmd.angular.z = steer;
        accel_pub_->publish(cmd);

        publish_debug_visualization(path, lookahead_idx);
    }

    // ========================================================================
    // VISUALIZATION
    // ========================================================================

    void publish_debug_visualization(const nav_msgs::msg::Path::SharedPtr &path, int lookahead_idx)
    {
        if (!path || lookahead_idx >= (int)path->poses.size())
            return;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "lookahead";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = path->poses[lookahead_idx].pose.position.x;
        marker.pose.position.y = path->poses[lookahead_idx].pose.position.y;
        marker.pose.position.z = 0.2;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1;

        switch (current_state_)
        {
        case DrivingState::CRUISE:
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            break;
        case DrivingState::PREPARE_CHANGE:
        case DrivingState::LANE_CHANGING:
            marker.color.r = 0.0;
            marker.color.g = 0.5;
            marker.color.b = 1.0;
            break;
        case DrivingState::OVERTAKE_STATIC:
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            break;
        case DrivingState::EMERGENCY_ACCEL:
            marker.color.r = 1.0;
            marker.color.g = 0.5;
            marker.color.b = 0.0;
            break;
        case DrivingState::EMERGENCY_BRAKE:
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            break;
        }
        marker.color.a = 0.9;
        marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        lookahead_pub_->publish(marker);

        visualization_msgs::msg::MarkerArray debug_markers;
        int id = 0;

        double cos_yaw = std::cos(ego_yaw_);
        double sin_yaw = std::sin(ego_yaw_);

        tf2::Quaternion q;
        q.setRPY(0, 0, ego_yaw_);
        auto quat_msg = tf2::toMsg(q);

        visualization_msgs::msg::Marker front_zone;
        front_zone.header.frame_id = "world";
        front_zone.header.stamp = this->now();
        front_zone.ns = "safety";
        front_zone.id = id++;
        front_zone.type = visualization_msgs::msg::Marker::CUBE;
        front_zone.action = visualization_msgs::msg::Marker::ADD;
        front_zone.pose.position.x = ego_x_ + cos_yaw * (VEHICLE_LENGTH_F + FRONT_SAFE_DIST / 2);
        front_zone.pose.position.y = ego_y_ + sin_yaw * (VEHICLE_LENGTH_F + FRONT_SAFE_DIST / 2);
        front_zone.pose.position.z = 0.02;
        front_zone.pose.orientation = quat_msg;
        front_zone.scale.x = FRONT_SAFE_DIST;
        front_zone.scale.y = SIDE_MARGIN * 2;
        front_zone.scale.z = 0.01;
        front_zone.color.r = 0.0;
        front_zone.color.g = 1.0;
        front_zone.color.b = 0.0;
        front_zone.color.a = 0.3;
        front_zone.lifetime = rclcpp::Duration::from_seconds(0.1);
        debug_markers.markers.push_back(front_zone);

        visualization_msgs::msg::Marker text;
        text.header.frame_id = "world";
        text.header.stamp = this->now();
        text.ns = "state";
        text.id = id++;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position.x = ego_x_;
        text.pose.position.y = ego_y_;
        text.pose.position.z = 0.4;
        text.scale.z = 0.12;
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 1.0;
        text.color.a = 1.0;

        std::stringstream ss;
        ss << state_to_string(current_state_) << "\n"
           << std::fixed << std::setprecision(2) << ego_speed_ << " m/s\n"
           << lane_to_string(current_lane_);
        if (target_lane_ != current_lane_)
        {
            ss << "->" << lane_to_string(target_lane_);
        }
        text.text = ss.str();
        text.lifetime = rclcpp::Duration::from_seconds(0.1);
        debug_markers.markers.push_back(text);

        debug_pub_->publish(debug_markers);
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

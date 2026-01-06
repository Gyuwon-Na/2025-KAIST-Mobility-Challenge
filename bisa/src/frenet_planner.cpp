/**
 * @file frenet_planner.cpp
 * @brief Advanced Lane Change Planner for RC-Scale Autonomous Vehicle (v2.2)
 *
 * Fixes:
 * - Time source mismatch error (critical bug fix)
 * - Yaw angle extraction (orientation.z is Euler yaw)
 * - Separate front/rear safety zone visualization
 * - More aggressive lane change triggering
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
    LANE_2 = 1, // Middle - Slow Dynamic HVs (CAV starts here)
    LANE_3 = 2, // Outermost - Fast Dynamic HVs
    NONE = -1
};

enum class DrivingState
{
    CRUISE,          // Normal driving
    PREPARE_CHANGE,  // Preparing lane change
    LANE_CHANGING,   // Executing lane change
    EMERGENCY_ACCEL, // Rear threat - speed up
    EMERGENCY_BRAKE  // Front collision - brake
};

struct ObstacleInfo
{
    int id;
    double x, y;          // Global position
    double vx, vy;        // Global velocity
    double speed;         // Absolute speed
    LaneID lane;          // Which lane
    double rel_x, rel_y;  // Body frame position
    double rel_vx;        // Relative velocity (body frame)
    double last_seen_sec; // Use double instead of rclcpp::Time to avoid time source issues
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
        RCLCPP_INFO(this->get_logger(), "Frenet Planner v2.2 - Time Bug Fixed");
        RCLCPP_INFO(this->get_logger(), "============================================");

        declare_parameters();
        load_parameters();

        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&FrenetPlanner::param_callback, this, std::placeholders::_1));

        setup_ros_interfaces();
        reset_state();

        // 50Hz control loop
        timer_ = this->create_wall_timer(20ms, std::bind(&FrenetPlanner::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Planner initialized. Waiting for data...");
    }

private:
    // ========================================================================
    // CONSTANTS (RC Car Scale)
    // ========================================================================

    static constexpr double VEHICLE_LENGTH = 0.33;
    static constexpr double VEHICLE_WIDTH = 0.15;
    static constexpr double VEHICLE_LENGTH_F = 0.17;
    static constexpr double VEHICLE_LENGTH_R = 0.16;

    // Lane detection
    static constexpr double LANE_WIDTH = 0.25;
    static constexpr double LANE_THRESHOLD = 0.20;

    // Safety distances
    static constexpr double FRONT_SAFE_DIST = 0.6;
    static constexpr double FRONT_CRITICAL_DIST = 0.35;
    static constexpr double FRONT_EMERGENCY_DIST = 0.25;
    static constexpr double REAR_DANGER_DIST = 0.4;
    static constexpr double BLIND_SPOT_LENGTH = 0.4;
    static constexpr double SIDE_MARGIN = 0.3;

    // ========================================================================
    // PARAMETERS
    // ========================================================================

    double wheelbase_ = 0.33;
    double max_steer_ = 0.52;
    double target_speed_ = 0.8;
    double boost_speed_ = 1.0;
    double base_lookahead_ = 0.4;
    double max_lookahead_ = 0.8;
    double weight_heading_ = 0.6;
    double weight_path_ = 0.4;
    double steer_alpha_ = 0.2;

    // ========================================================================
    // STATE VARIABLES
    // ========================================================================

    DrivingState current_state_ = DrivingState::CRUISE;
    LaneID current_lane_ = LaneID::LANE_2;
    LaneID target_lane_ = LaneID::LANE_2;

    // Lane change
    bool lane_change_committed_ = false;
    double lane_change_start_sec_ = 0.0;
    int safe_check_count_ = 0;
    static constexpr int SAFE_CHECK_REQUIRED = 3;

    // Pose
    double ego_x_ = 0.0, ego_y_ = 0.0, ego_yaw_ = 0.0;
    double ego_speed_ = 0.0;
    bool pose_received_ = false;

    // Velocity estimation
    double prev_x_ = 0.0, prev_y_ = 0.0;
    double prev_pose_sec_ = 0.0;
    bool prev_pose_valid_ = false;

    // Path following
    int last_closest_idx_ = 0;
    double last_steer_ = 0.0;

    // Obstacles
    std::map<int, ObstacleInfo> obstacles_;

    // Logging (use double for time to avoid time source issues)
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
    // HELPER: Get current time as double (seconds)
    // ========================================================================

    double now_sec()
    {
        return this->now().seconds();
    }

    // ========================================================================
    // INITIALIZATION
    // ========================================================================

    void declare_parameters()
    {
        this->declare_parameter("wheelbase", 0.33);
        this->declare_parameter("max_steer", 0.52);
        this->declare_parameter("target_speed", 0.8);
        this->declare_parameter("boost_speed", 1.0);
        this->declare_parameter("base_lookahead", 0.4);
        this->declare_parameter("max_lookahead", 0.8);
        this->declare_parameter("weight_heading", 0.6);
        this->declare_parameter("weight_path", 0.4);
        this->declare_parameter("steer_alpha", 0.2);
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
        last_steer_ = 0.0;
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

        // CRITICAL: orientation.z contains Euler yaw directly!
        ego_yaw_ = msg->pose.orientation.z;

        pose_received_ = true;

        // Estimate velocity using double for time
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

                // Direction check
                double move_angle = std::atan2(dy, dx);
                double angle_diff = move_angle - ego_yaw_;
                while (angle_diff > M_PI)
                    angle_diff -= 2 * M_PI;
                while (angle_diff < -M_PI)
                    angle_diff += 2 * M_PI;

                if (std::abs(angle_diff) > M_PI / 2)
                    raw_speed = -raw_speed;

                ego_speed_ = ego_speed_ * 0.7 + raw_speed * 0.3;
            }
        }

        prev_x_ = ego_x_;
        prev_y_ = ego_y_;
        prev_pose_sec_ = current_sec;
        prev_pose_valid_ = true;

        // Update current lane
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
                        obs.vx = obs.vx * 0.5 + vx * 0.5;
                        obs.vy = obs.vy * 0.5 + vy * 0.5;
                    }
                }

                obs.x = ox;
                obs.y = oy;
                obs.speed = std::hypot(obs.vx, obs.vy);
                obs.lane = get_lane_at_position(ox, oy);
                obs.last_seen_sec = current_sec;
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
                obstacles_[id] = obs;
            }
        }

        // Remove stale obstacles
        double now = current_sec;
        for (auto it = obstacles_.begin(); it != obstacles_.end();)
        {
            if ((now - it->second.last_seen_sec) > 1.0)
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
        for (size_t i = 0; i < path->poses.size(); i += 10)
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

        if (d1 == min_d)
            return LaneID::LANE_1;
        if (d2 == min_d)
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
            return "L1(Static)";
        case LaneID::LANE_2:
            return "L2(Slow)";
        case LaneID::LANE_3:
            return "L3(Fast)";
        default:
            return "NONE";
        }
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
        bool danger_rear = false;
        bool blind_spot = false;
        double front_dist = 999.0;
        double front_speed = 0.0;
        double rear_dist = 999.0;
        double rear_speed = 0.0;
        int front_obs_id = -1;
    };

    LaneStatus assess_lane(LaneID lane)
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

            // Front obstacle
            if (obs.rel_x > -VEHICLE_LENGTH_R && obs.rel_x < 5.0)
            {
                if (lateral_dist < SIDE_MARGIN)
                {
                    if (obs.rel_x < status.front_dist)
                    {
                        status.front_dist = obs.rel_x;
                        status.front_speed = obs.speed;
                        status.front_obs_id = id;

                        if (obs.rel_x < FRONT_SAFE_DIST)
                        {
                            status.blocked_front = true;
                        }
                    }
                }
            }

            // Rear obstacle
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

            // Blind spot
            if (std::abs(obs.rel_x) < BLIND_SPOT_LENGTH && lateral_dist < SIDE_MARGIN * 1.5)
            {
                status.blind_spot = true;
            }
        }

        return status;
    }

    // ========================================================================
    // LANE CHANGE DECISION
    // ========================================================================

    LaneID choose_best_lane()
    {
        LaneStatus status_current = assess_lane(current_lane_);

        if (!status_current.blocked_front && status_current.front_dist > FRONT_SAFE_DIST)
        {
            return current_lane_;
        }

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

        LaneID best = current_lane_;
        double best_score = -100.0;

        double current_score = status_current.front_dist;
        if (status_current.blocked_front)
            current_score -= 2.0;

        for (LaneID cand : candidates)
        {
            LaneStatus status = assess_lane(cand);

            if (status.blind_spot)
                continue;
            if (status.danger_rear)
                continue;

            double score = status.front_dist;

            if (!status.blocked_front && status.front_dist > 2.0)
            {
                score += 3.0;
            }

            if (cand == LaneID::LANE_1)
            {
                score -= 1.0;
            }

            if (cand == LaneID::LANE_3 && ego_speed_ > 0.6)
            {
                score += 0.5;
            }

            if (score > best_score && score > current_score + 0.3)
            {
                best_score = score;
                best = cand;
            }
        }

        return best;
    }

    bool is_lane_change_safe(LaneID target)
    {
        LaneStatus status = assess_lane(target);

        if (status.blind_spot)
            return false;
        if (status.danger_rear)
            return false;
        if (status.front_dist < FRONT_CRITICAL_DIST)
            return false;

        for (double t = 0.1; t <= 1.5; t += 0.2)
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
        LaneStatus current_status = assess_lane(current_lane_);
        double current_sec = now_sec();

        // Periodic logging
        if ((current_sec - last_detail_log_sec_) > 2.0)
        {
            RCLCPP_INFO(this->get_logger(),
                        "[%s] Lane=%s | Speed=%.2f | Front=%.2f@%.2f | Rear=%.2f@%.2f | Obs=%zu",
                        state_to_string(current_state_).c_str(),
                        lane_to_string(current_lane_).c_str(),
                        ego_speed_,
                        current_status.front_dist, current_status.front_speed,
                        current_status.rear_dist, current_status.rear_speed,
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
                RCLCPP_WARN(this->get_logger(), "⚠️ REAR THREAT! Accelerating...");
                break;
            }

            if (current_status.front_dist < FRONT_EMERGENCY_DIST)
            {
                current_state_ = DrivingState::EMERGENCY_BRAKE;
                RCLCPP_WARN(this->get_logger(), "🛑 EMERGENCY BRAKE! Front=%.2f", current_status.front_dist);
                break;
            }

            if (current_status.blocked_front ||
                (current_status.front_dist < FRONT_SAFE_DIST && current_status.front_speed < ego_speed_ - 0.05))
            {
                LaneID best = choose_best_lane();
                if (best != current_lane_)
                {
                    target_lane_ = best;
                    current_state_ = DrivingState::PREPARE_CHANGE;
                    safe_check_count_ = 0;
                    RCLCPP_INFO(this->get_logger(), "🔄 Preparing lane change: %s -> %s",
                                lane_to_string(current_lane_).c_str(),
                                lane_to_string(target_lane_).c_str());
                }
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
                    RCLCPP_INFO(this->get_logger(), "✅ Lane change COMMITTED: %s -> %s",
                                lane_to_string(current_lane_).c_str(),
                                lane_to_string(target_lane_).c_str());
                }
            }
            else
            {
                safe_check_count_ = std::max(0, safe_check_count_ - 1);
            }

            if (safe_check_count_ == 0)
            {
                if (!current_status.blocked_front && current_status.front_dist > FRONT_SAFE_DIST)
                {
                    current_state_ = DrivingState::CRUISE;
                    target_lane_ = current_lane_;
                    RCLCPP_INFO(this->get_logger(), "Lane change aborted - current lane OK");
                    break;
                }
            }

            if (current_status.front_dist < FRONT_EMERGENCY_DIST)
            {
                current_state_ = DrivingState::EMERGENCY_BRAKE;
            }
            else if (current_status.danger_rear)
            {
                current_state_ = DrivingState::EMERGENCY_ACCEL;
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
                    RCLCPP_INFO(this->get_logger(), "✅ Lane change COMPLETE: now in %s",
                                lane_to_string(current_lane_).c_str());
                }
            }

            double elapsed = current_sec - lane_change_start_sec_;
            if (elapsed > 4.0)
            {
                current_state_ = DrivingState::CRUISE;
                lane_change_committed_ = false;
                target_lane_ = current_lane_;
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
                LaneID escape = choose_best_lane();
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
            if (current_status.front_dist > FRONT_SAFE_DIST)
            {
                current_state_ = DrivingState::CRUISE;
                RCLCPP_INFO(this->get_logger(), "Front clear, resuming cruise");
            }

            LaneID escape = choose_best_lane();
            if (escape != current_lane_ && is_lane_change_safe(escape))
            {
                target_lane_ = escape;
                current_state_ = DrivingState::PREPARE_CHANGE;
                safe_check_count_ = SAFE_CHECK_REQUIRED - 1;
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
        default:
            return "???";
        }
    }

    // ========================================================================
    // PATH FOLLOWING
    // ========================================================================

    int find_closest_index(const nav_msgs::msg::Path::SharedPtr &path)
    {
        if (!path || path->poses.empty())
            return 0;

        int n = path->poses.size();
        int best = last_closest_idx_;
        double min_d = 999.0;

        int start = std::max(0, last_closest_idx_ - 10);
        int end = std::min(n, last_closest_idx_ + 100);

        for (int i = start; i < end; ++i)
        {
            int idx = i % n;
            double d = std::hypot(path->poses[idx].pose.position.x - ego_x_,
                                  path->poses[idx].pose.position.y - ego_y_);
            if (d < min_d)
            {
                min_d = d;
                best = idx;
            }
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

        return (closest + 50) % n;
    }

    double compute_steering(const nav_msgs::msg::Path::SharedPtr &path, int closest, int lookahead)
    {
        if (!path || path->poses.empty())
            return 0.0;

        int n = path->poses.size();

        double tx = path->poses[lookahead].pose.position.x;
        double ty = path->poses[lookahead].pose.position.y;

        int next = (lookahead + 5) % n;
        double road_dx = path->poses[next].pose.position.x - tx;
        double road_dy = path->poses[next].pose.position.y - ty;
        double road_yaw = std::atan2(road_dy, road_dx);

        double dx = tx - ego_x_;
        double dy = ty - ego_y_;
        double target_yaw = std::atan2(dy, dx);
        double dist = std::hypot(dx, dy);

        double heading_err = target_yaw - ego_yaw_;
        while (heading_err > M_PI)
            heading_err -= 2 * M_PI;
        while (heading_err < -M_PI)
            heading_err += 2 * M_PI;

        double road_err = road_yaw - ego_yaw_;
        while (road_err > M_PI)
            road_err -= 2 * M_PI;
        while (road_err < -M_PI)
            road_err += 2 * M_PI;

        double steer_pp = std::atan2(2.0 * wheelbase_ * std::sin(heading_err), std::max(dist, 0.1));
        double steer_cmd = steer_pp * weight_path_ + road_err * weight_heading_;

        steer_cmd = steer_cmd * steer_alpha_ + last_steer_ * (1.0 - steer_alpha_);
        steer_cmd = std::clamp(steer_cmd, -max_steer_, max_steer_);

        return steer_cmd;
    }

    // ========================================================================
    // VELOCITY CONTROL
    // ========================================================================

    double compute_velocity()
    {
        LaneStatus status = assess_lane(current_lane_);
        double vel = target_speed_;

        switch (current_state_)
        {
        case DrivingState::CRUISE:
        {
            if (status.blocked_front)
            {
                double gap = status.front_dist - FRONT_CRITICAL_DIST;
                vel = std::min(target_speed_, std::max(status.front_speed, gap * 1.5));
            }
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
                vel = std::max(0.3, status.front_speed);
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
            vel = std::min(1.3, boost_speed_ * 1.3);
            break;
        }

        case DrivingState::EMERGENCY_BRAKE:
        {
            if (status.front_dist < FRONT_EMERGENCY_DIST * 0.7)
                vel = 0.0;
            else
                vel = std::max(0.0, (status.front_dist - 0.15) * 2.0);
            break;
        }
        }

        return std::clamp(vel, 0.0, 1.5);
    }

    // ========================================================================
    // MAIN CONTROL LOOP
    // ========================================================================

    void control_loop()
    {
        if (!pose_received_)
            return;

        nav_msgs::msg::Path::SharedPtr path = get_lane_path(target_lane_);
        if (!path || path->poses.empty())
        {
            path = get_lane_path(current_lane_);
        }
        if (!path || path->poses.empty())
        {
            path = path_lane_2_;
        }
        if (!path || path->poses.empty())
            return;

        update_state_machine();

        int closest = find_closest_index(path);
        last_closest_idx_ = closest;

        double path_error = std::hypot(
            path->poses[closest].pose.position.x - ego_x_,
            path->poses[closest].pose.position.y - ego_y_);

        double lookahead_dist = base_lookahead_ + path_error * 0.5;
        lookahead_dist = std::clamp(lookahead_dist, base_lookahead_, max_lookahead_);

        int lookahead_idx = find_lookahead_index(path, closest, lookahead_dist);

        double steer = compute_steering(path, closest, lookahead_idx);
        last_steer_ = steer;

        double vel = compute_velocity();

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

        // Lookahead marker
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
        marker.scale.x = marker.scale.y = marker.scale.z = 0.12;

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

        // Debug markers
        visualization_msgs::msg::MarkerArray debug_markers;
        int id = 0;

        double cos_yaw = std::cos(ego_yaw_);
        double sin_yaw = std::sin(ego_yaw_);

        tf2::Quaternion q;
        q.setRPY(0, 0, ego_yaw_);
        auto quat_msg = tf2::toMsg(q);

        // Front safety zone (GREEN)
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

        // Rear safety zone (ORANGE)
        visualization_msgs::msg::Marker rear_zone;
        rear_zone.header.frame_id = "world";
        rear_zone.header.stamp = this->now();
        rear_zone.ns = "safety";
        rear_zone.id = id++;
        rear_zone.type = visualization_msgs::msg::Marker::CUBE;
        rear_zone.action = visualization_msgs::msg::Marker::ADD;
        rear_zone.pose.position.x = ego_x_ - cos_yaw * (VEHICLE_LENGTH_R + REAR_DANGER_DIST / 2);
        rear_zone.pose.position.y = ego_y_ - sin_yaw * (VEHICLE_LENGTH_R + REAR_DANGER_DIST / 2);
        rear_zone.pose.position.z = 0.02;
        rear_zone.pose.orientation = quat_msg;
        rear_zone.scale.x = REAR_DANGER_DIST;
        rear_zone.scale.y = SIDE_MARGIN * 2;
        rear_zone.scale.z = 0.01;
        rear_zone.color.r = 1.0;
        rear_zone.color.g = 0.5;
        rear_zone.color.b = 0.0;
        rear_zone.color.a = 0.3;
        rear_zone.lifetime = rclcpp::Duration::from_seconds(0.1);
        debug_markers.markers.push_back(rear_zone);

        // State text
        visualization_msgs::msg::Marker text;
        text.header.frame_id = "world";
        text.header.stamp = this->now();
        text.ns = "state";
        text.id = id++;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position.x = ego_x_;
        text.pose.position.y = ego_y_;
        text.pose.position.z = 0.5;
        text.scale.z = 0.15;
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
            ss << " -> " << lane_to_string(target_lane_);
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
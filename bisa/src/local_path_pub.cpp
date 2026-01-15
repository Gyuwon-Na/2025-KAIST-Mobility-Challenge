/**
 * @file local_path_pub.cpp
 * @brief Integrated Local Path Publisher with Hybrid Velocity Logic & Merge Safety
 */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <limits>
#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <sstream>
#include <iomanip>

// ============================================================================
// ENUMS AND STRUCTS
// ============================================================================

enum class LaneID
{
    LANE_1 = 0, // Left
    LANE_2 = 1, // Center
    LANE_3 = 2, // Right
    NONE = -1
};

enum class DrivingState
{
    CRUISE,
    WAIT_FOR_GAP,
    LANE_CHANGING,
    EMERGENCY_ACCEL,
    RETURN_TO_CENTER
};

struct ObstacleInfo
{
    int id;
    double x, y;
    double vx, vy;
    double speed;
    LaneID lane;
    double rel_x, rel_y, rel_vx;
    double last_seen_sec;
    bool is_static;
};

struct SurroundingStatus
{
    bool front_blocked = false;
    bool front_danger = false;
    bool front_critical = false;
    double front_dist = 999.0;
    double front_speed = 0.0;
    bool front_is_static = false;
    int front_id = -1;

    bool rear_danger = false;
    double rear_dist = 999.0;
    double rear_speed = 0.0;
    double rear_ttc = 999.0;

    bool left_clear = true;
    double left_dist = 999.0;
    bool right_clear = true;
    double right_dist = 999.0;

    bool path_collision = false;
    double path_collision_dist = 999.0;
};

struct PathPoint
{
    double x, y, yaw;
    double s;
};

namespace bisa
{
    class LocalPathPubCpp : public rclcpp::Node
    {
    public:
        LocalPathPubCpp() : Node("local_path_pub")
        {
            RCLCPP_INFO(this->get_logger(), "============================================");
            RCLCPP_INFO(this->get_logger(), "Local Path Publisher v5.3 (Fixed Constants)");
            RCLCPP_INFO(this->get_logger(), "============================================");

            auto qos = rclcpp::QoS(10).transient_local();

            sub_lane_[0] = this->create_subscription<nav_msgs::msg::Path>("/hdmap/lane_one", qos,
                                                                          [this](nav_msgs::msg::Path::SharedPtr msg)
                                                                          { lane_paths_[0] = msg; process_lane_path(0); });
            sub_lane_[1] = this->create_subscription<nav_msgs::msg::Path>("/hdmap/lane_two", qos,
                                                                          [this](nav_msgs::msg::Path::SharedPtr msg)
                                                                          { lane_paths_[1] = msg; process_lane_path(1); });
            sub_lane_[2] = this->create_subscription<nav_msgs::msg::Path>("/hdmap/lane_three", qos,
                                                                          [this](nav_msgs::msg::Path::SharedPtr msg)
                                                                          { lane_paths_[2] = msg; process_lane_path(2); });

            obs_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
                "/obstacles_markers", 10, std::bind(&LocalPathPubCpp::obstacle_callback, this, std::placeholders::_1));

            pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/Ego_pose", rclcpp::SensorDataQoS(), std::bind(&LocalPathPubCpp::pose_callback, this, std::placeholders::_1));

            local_pub_ = this->create_publisher<nav_msgs::msg::Path>("/local_path", 10);
            target_vel_pub_ = this->create_publisher<std_msgs::msg::Float32>("/planning/target_v", 10);
            debug_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/planning/debug_markers", 10);

            timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&LocalPathPubCpp::control_loop, this));
        }

    private:
        // ========================================================================
        // CONSTANTS (FIXED: Missing constants added)
        // ========================================================================
        static constexpr double VEHICLE_LENGTH = 0.33;
        static constexpr double VEHICLE_WIDTH = 0.15;
        static constexpr double OBS_HALF_LENGTH = 0.17; // [FIX] Added
        static constexpr double OBS_HALF_WIDTH = 0.075; // [FIX] Added

        static constexpr double FRONT_ZONE_LENGTH = 0.8;
        static constexpr double FRONT_ZONE_WIDTH = 0.12;
        static constexpr double FRONT_SAFE_DIST = 0.50;
        static constexpr double FRONT_DANGER_DIST = 0.35;
        static constexpr double FRONT_CRITICAL_DIST = 0.20;

        static constexpr double REAR_ZONE_LENGTH = 1.0;
        static constexpr double REAR_ZONE_WIDTH = 0.12;
        static constexpr double REAR_DANGER_DIST = 0.5;
        static constexpr double REAR_TTC_THRESHOLD = 2.0;

        static constexpr double SIDE_ZONE_START = -0.5;
        static constexpr double SIDE_ZONE_END = 0.6;
        static constexpr double SIDE_ZONE_OUTER = 0.35;

        static constexpr double LANE_WIDTH = 0.25;
        static constexpr double LANE_THRESHOLD = 0.20;

        static constexpr double CRUISE_SPEED = 0.5;
        static constexpr double OVERTAKE_SPEED = 0.6;
        static constexpr double ACCEL_SPEED = 0.7;
        static constexpr double SLOW_SPEED = 0.3;
        static constexpr double MIN_SPEED = 0.15;
        static constexpr double TIME_GAP = 1.2;

        static constexpr double LANE_CHANGE_FORWARD_DIST = 0.8;
        static constexpr double LANE_CHANGE_TIME = 1.5;
        static constexpr double CORNER_CURVATURE_THRESHOLD = 0.3;
        static constexpr double PATH_CHECK_HORIZON = 1.5;
        static constexpr double PATH_COLLISION_RADIUS = 0.22;
        static constexpr double STATIC_SPEED_THRESHOLD = 0.08;

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_lane_[3];
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obs_sub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_vel_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        nav_msgs::msg::Path::SharedPtr lane_paths_[3];
        std::vector<PathPoint> processed_lanes_[3];
        double ego_x_ = 0.0, ego_y_ = 0.0, ego_yaw_ = 0.0, ego_speed_ = 0.0;
        bool pose_received_ = false;
        double prev_x_ = 0.0, prev_y_ = 0.0, prev_pose_sec_ = 0.0;
        bool prev_pose_valid_ = false;

        LaneID current_lane_ = LaneID::LANE_2;
        LaneID target_lane_ = LaneID::LANE_2;
        DrivingState current_state_ = DrivingState::CRUISE;

        bool is_lane_changing_ = false;
        double lane_change_start_sec_ = 0.0;
        double lane_change_progress_ = 0.0;
        int safe_check_count_ = 0;
        static constexpr int SAFE_CHECK_REQUIRED = 3;

        bool is_in_corner_ = false;
        bool corner_approaching_ = false;
        double current_curvature_ = 0.0;

        std::map<int, ObstacleInfo> obstacles_;
        SurroundingStatus current_surrounding_;

        int last_closest_idx_[3] = {0, 0, 0};
        bool initial_search_done_ = false;
        double last_log_sec_ = 0.0;

        // ========================================================================
        // UTILS
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
            if (lane == LaneID::LANE_1)
                return 0;
            if (lane == LaneID::LANE_3)
                return 2;
            return 1;
        }

        LaneID int_to_lane(int i)
        {
            if (i == 0)
                return LaneID::LANE_1;
            if (i == 2)
                return LaneID::LANE_3;
            return LaneID::LANE_2;
        }

        std::string lane_str(LaneID lane)
        {
            if (lane == LaneID::LANE_1)
                return "L1";
            if (lane == LaneID::LANE_3)
                return "L3";
            return "L2";
        }

        std::string state_str(DrivingState s)
        {
            if (s == DrivingState::CRUISE)
                return "CRUISE";
            if (s == DrivingState::WAIT_FOR_GAP)
                return "WAIT";
            if (s == DrivingState::LANE_CHANGING)
                return "LC";
            if (s == DrivingState::EMERGENCY_ACCEL)
                return "ACCEL";
            return "RET";
        }

        // ========================================================================
        // CALLBACKS
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
                    if (std::abs(normalize_angle(move_angle - ego_yaw_)) > M_PI / 2)
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

        void process_lane_path(int lane_idx)
        {
            if (!lane_paths_[lane_idx] || lane_paths_[lane_idx]->poses.empty())
                return;
            processed_lanes_[lane_idx].clear();
            const auto &poses = lane_paths_[lane_idx]->poses;
            double cumulative_s = 0.0;
            for (size_t i = 0; i < poses.size(); ++i)
            {
                PathPoint pt;
                pt.x = poses[i].pose.position.x;
                pt.y = poses[i].pose.position.y;
                if (i > 0)
                    cumulative_s += std::hypot(pt.x - processed_lanes_[lane_idx][i - 1].x, pt.y - processed_lanes_[lane_idx][i - 1].y);
                pt.s = cumulative_s;
                if (i < poses.size() - 1)
                    pt.yaw = std::atan2(poses[i + 1].pose.position.y - pt.y, poses[i + 1].pose.position.x - pt.x);
                else if (i > 0)
                    pt.yaw = processed_lanes_[lane_idx][i - 1].yaw;
                else
                    pt.yaw = 0.0;
                processed_lanes_[lane_idx].push_back(pt);
            }
        }

        void obstacle_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
        {
            double current_sec = now_sec();
            for (const auto &marker : msg->markers)
            {
                if (marker.ns != "surrounding_cars" || marker.type != visualization_msgs::msg::Marker::CUBE)
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
                    obs.vx = 0;
                    obs.vy = 0;
                    obs.speed = 0;
                    obs.is_static = true;
                    obs.lane = get_lane_at(ox, oy);
                    obs.last_seen_sec = current_sec;
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
        // CORE FUNCTIONS
        // ========================================================================
        double get_dist_to_lane(int lane_idx, double x, double y)
        {
            if (lane_idx < 0 || lane_idx > 2 || processed_lanes_[lane_idx].empty())
                return 999.0;
            const auto &lane = processed_lanes_[lane_idx];
            double min_d = 999.0;
            size_t step = std::max(size_t(1), lane.size() / 500);
            for (size_t i = 0; i < lane.size(); i += step)
            {
                double d = std::hypot(lane[i].x - x, lane[i].y - y);
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

        int find_closest_idx(int lane_idx, double x, double y, bool global_search = false)
        {
            if (lane_idx < 0 || lane_idx > 2 || processed_lanes_[lane_idx].empty())
                return 0;
            const auto &lane = processed_lanes_[lane_idx];
            int n = lane.size();
            int best = last_closest_idx_[lane_idx];
            double min_d = 999.0;

            if (global_search || !initial_search_done_)
            {
                for (int i = 0; i < n; ++i)
                {
                    double d = std::hypot(lane[i].x - x, lane[i].y - y);
                    if (d < min_d)
                    {
                        min_d = d;
                        best = i;
                    }
                }
            }
            else
            {
                for (int i = -30; i < 200; ++i)
                {
                    int idx = (last_closest_idx_[lane_idx] + i + n) % n;
                    double d = std::hypot(lane[idx].x - x, lane[idx].y - y);
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

        int find_index_at_distance(int lane_idx, int start_idx, double target_dist)
        {
            if (processed_lanes_[lane_idx].empty())
                return start_idx;
            const auto &lane = processed_lanes_[lane_idx];
            int n = lane.size();
            double accumulated_dist = 0.0;
            for (int i = 0; i < 500; ++i)
            {
                int idx = (start_idx + i) % n;
                int next = (start_idx + i + 1) % n;
                accumulated_dist += std::hypot(lane[next].x - lane[idx].x, lane[next].y - lane[idx].y);
                if (accumulated_dist >= target_dist)
                    return next;
            }
            return (start_idx + 100) % n;
        }

        void update_corner_state(int lane_idx, int closest)
        {
            if (lane_idx < 0 || lane_idx > 2 || processed_lanes_[lane_idx].empty())
                return;
            int n = processed_lanes_[lane_idx].size();
            auto get_pt = [&](int i)
            { return processed_lanes_[lane_idx][(i + n) % n]; };
            auto curv = [&](int i)
            {
                auto p0 = get_pt(i - 10), p1 = get_pt(i), p2 = get_pt(i + 10);
                double a = std::hypot(p1.x - p0.x, p1.y - p0.y);
                double b = std::hypot(p2.x - p1.x, p2.y - p1.y);
                double c = std::hypot(p0.x - p2.x, p0.y - p2.y);
                double s = (a + b + c) / 2.0;
                double area = std::sqrt(std::max(0.0, s * (s - a) * (s - b) * (s - c)));
                return (a * b * c > 0.001) ? 4.0 * area / (a * b * c) : 0.0;
            };
            current_curvature_ = curv(closest);
            is_in_corner_ = (current_curvature_ > CORNER_CURVATURE_THRESHOLD);
            corner_approaching_ = (curv(closest + 40) > CORNER_CURVATURE_THRESHOLD);
        }

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

        bool check_path_collision(int lane_idx, double &collision_dist)
        {
            collision_dist = 999.0;
            if (lane_idx < 0 || lane_idx > 2 || processed_lanes_[lane_idx].empty())
                return false;

            const auto &lane = processed_lanes_[lane_idx];
            int n = lane.size();
            int closest = find_closest_idx(lane_idx, ego_x_, ego_y_);
            double check_speed = std::max(ego_speed_, 0.2);
            double total_dist = 0.0;
            double max_check_dist = check_speed * PATH_CHECK_HORIZON;

            for (int i = 1; i < 80 && total_dist < max_check_dist; ++i)
            {
                int idx = (closest + i) % n;
                int prev_idx = (closest + i - 1) % n;
                double seg_dist = std::hypot(lane[idx].x - lane[prev_idx].x, lane[idx].y - lane[prev_idx].y);
                total_dist += seg_dist;
                double t = total_dist / check_speed;

                for (const auto &[id, obs] : obstacles_)
                {
                    double obs_x = obs.x + obs.vx * t;
                    double obs_y = obs.y + obs.vy * t;
                    if (std::hypot(lane[idx].x - obs_x, lane[idx].y - obs_y) < PATH_COLLISION_RADIUS)
                    {
                        if (total_dist < collision_dist)
                            collision_dist = total_dist;
                        return true;
                    }
                }
            }
            return false;
        }

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

                if (obs.lane == current_lane_)
                {
                    if (rx > -OBS_HALF_LENGTH && rx < FRONT_ZONE_LENGTH + OBS_HALF_LENGTH)
                    {
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
                    if (rx < OBS_HALF_LENGTH && rx > -(REAR_ZONE_LENGTH + OBS_HALF_LENGTH))
                    {
                        if (std::abs(ry) < REAR_ZONE_WIDTH + OBS_HALF_WIDTH)
                        {
                            double effective_dist = -rx - OBS_HALF_LENGTH;
                            if (effective_dist > 0 && effective_dist < status.rear_dist)
                            {
                                status.rear_dist = effective_dist;
                                status.rear_speed = obs.speed;
                                if (-obs.rel_vx > 0.05)
                                {
                                    status.rear_ttc = effective_dist / -obs.rel_vx;
                                    if (status.rear_ttc < REAR_TTC_THRESHOLD)
                                        status.rear_danger = true;
                                }
                            }
                        }
                    }
                }

                if (obs.lane != current_lane_ && obs.lane != LaneID::NONE)
                {
                    if (rx > SIDE_ZONE_START - OBS_HALF_LENGTH && rx < SIDE_ZONE_END + OBS_HALF_LENGTH)
                    {
                        double lat_dist = (ry > 0) ? ry - OBS_HALF_WIDTH : -ry - OBS_HALF_WIDTH;
                        if (lat_dist < SIDE_ZONE_OUTER && lat_dist > -0.1)
                        {
                            if (ry > 0)
                            {
                                status.left_clear = false;
                                status.left_dist = std::min(status.left_dist, lat_dist);
                            }
                            else
                            {
                                status.right_clear = false;
                                status.right_dist = std::min(status.right_dist, lat_dist);
                            }
                        }
                    }
                }
            }

            double pd;
            status.path_collision = check_path_collision(lane_to_int(target_lane_), pd);
            status.path_collision_dist = pd;
            return status;
        }

        bool is_target_left(LaneID target)
        {
            int idx = lane_to_int(target);
            if (processed_lanes_[idx].empty())
                return false;
            int closest = find_closest_idx(idx, ego_x_, ego_y_);
            double dx = processed_lanes_[idx][closest].x - ego_x_;
            double dy = processed_lanes_[idx][closest].y - ego_y_;
            return (std::cos(ego_yaw_) * dy - std::sin(ego_yaw_) * dx) > 0.0;
        }

        bool is_lane_blocked_by_static(LaneID lane)
        {
            for (const auto &[id, obs] : obstacles_)
            {
                if (obs.lane != lane || !obs.is_static)
                    continue;
                double dist = std::hypot(obs.x - ego_x_, obs.y - ego_y_);
                double forward = (obs.x - ego_x_) * std::cos(ego_yaw_) + (obs.y - ego_y_) * std::sin(ego_yaw_);
                if (dist < 2.0 && forward > -0.3)
                    return true;
            }
            return false;
        }

        bool is_overtake_safe(LaneID target)
        {
            bool left = is_target_left(target);
            if ((left && !current_surrounding_.left_clear) || (!left && !current_surrounding_.right_clear))
                return false;

            double coll_dist;
            if (check_path_collision(lane_to_int(target), coll_dist) && coll_dist < 0.5)
                return false;

            for (const auto &[id, obs] : obstacles_)
            {
                if (obs.lane != target)
                    continue;
                if (obs.rel_x > -OBS_HALF_LENGTH && obs.rel_x < FRONT_ZONE_LENGTH && (obs.rel_x - OBS_HALF_LENGTH) < FRONT_CRITICAL_DIST)
                    return false;
                if (obs.rel_x < 0 && obs.rel_x > -REAR_ZONE_LENGTH && -obs.rel_vx > 0.1 && (-obs.rel_x / -obs.rel_vx) < 1.5)
                    return false;
            }
            return true;
        }

        bool are_lanes_overlapping(LaneID id1, LaneID id2)
        {
            int idx1 = lane_to_int(id1), idx2 = lane_to_int(id2);
            if (processed_lanes_[idx1].empty() || processed_lanes_[idx2].empty())
                return true;

            int closest1 = find_closest_idx(idx1, ego_x_, ego_y_);
            int n1 = processed_lanes_[idx1].size();

            for (int offset = 0; offset <= 50; offset += 10)
            {
                int curr_idx = (closest1 + offset) % n1;
                double x1 = processed_lanes_[idx1][curr_idx].x;
                double y1 = processed_lanes_[idx1][curr_idx].y;

                int closest2 = find_closest_idx(idx2, x1, y1, true);
                double dist = std::hypot(x1 - processed_lanes_[idx2][closest2].x, y1 - processed_lanes_[idx2][closest2].y);

                if (dist < 0.25)
                    return true;
            }
            return false;
        }

        LaneID choose_overtake_lane()
        {
            int c_idx = lane_to_int(current_lane_);
            std::vector<int> cands;
            if (c_idx == 0)
                cands = {1, 2};
            else if (c_idx == 1)
                cands = {2, 0};
            else
                cands = {1};

            bool corner = is_in_corner_ || corner_approaching_;

            for (int cand : cands)
            {
                LaneID tgt = int_to_lane(cand);
                if (are_lanes_overlapping(current_lane_, tgt))
                    continue;
                if (is_lane_blocked_by_static(tgt))
                    continue;
                if (corner && cand == 0 && tgt != LaneID::LANE_1)
                    continue;

                double cd;
                if (check_path_collision(cand, cd) && cd < 0.5)
                    continue;

                return tgt;
            }
            return current_lane_;
        }

        std::pair<double, double> check_lane_front(LaneID lane)
        {
            double min_d = 999.0, spd = 0.0;
            for (const auto &[id, obs] : obstacles_)
            {
                if (obs.lane != lane)
                    continue;
                double d = std::hypot(obs.x - ego_x_, obs.y - ego_y_);
                if (((obs.x - ego_x_) * std::cos(ego_yaw_) + (obs.y - ego_y_) * std::sin(ego_yaw_)) > 0 && d < min_d)
                {
                    min_d = d;
                    spd = obs.speed;
                }
            }
            return {min_d, spd};
        }

        void update_state_machine()
        {
            double cur_sec = now_sec();
            const auto &s = current_surrounding_;

            if (s.rear_danger && current_state_ != DrivingState::EMERGENCY_ACCEL && current_state_ != DrivingState::LANE_CHANGING)
            {
                if (!s.front_critical && s.front_dist > FRONT_DANGER_DIST)
                {
                    current_state_ = DrivingState::EMERGENCY_ACCEL;
                }
                else
                {
                    LaneID esc = choose_overtake_lane();
                    if (esc != current_lane_ && is_overtake_safe(esc))
                    {
                        target_lane_ = esc;
                        current_state_ = DrivingState::LANE_CHANGING;
                        is_lane_changing_ = true;
                        lane_change_start_sec_ = cur_sec;
                        lane_change_progress_ = 0.0;
                    }
                }
            }

            if (current_lane_ == LaneID::LANE_1 && corner_approaching_ && current_state_ == DrivingState::CRUISE)
            {
                if (is_overtake_safe(LaneID::LANE_2) && !are_lanes_overlapping(LaneID::LANE_1, LaneID::LANE_2))
                {
                    target_lane_ = LaneID::LANE_2;
                    current_state_ = DrivingState::LANE_CHANGING;
                    is_lane_changing_ = true;
                    lane_change_start_sec_ = cur_sec;
                    lane_change_progress_ = 0.0;
                }
            }

            switch (current_state_)
            {
            case DrivingState::CRUISE:
                if (s.front_blocked || (s.path_collision && s.path_collision_dist < 0.6))
                {
                    LaneID target = choose_overtake_lane();
                    if (target != current_lane_)
                    {
                        target_lane_ = target;
                        if (is_overtake_safe(target_lane_) && !s.front_is_static)
                        {
                            current_state_ = DrivingState::LANE_CHANGING;
                            is_lane_changing_ = true;
                            lane_change_start_sec_ = cur_sec;
                            lane_change_progress_ = 0.0;
                        }
                        else
                        {
                            current_state_ = DrivingState::WAIT_FOR_GAP;
                            safe_check_count_ = 0;
                        }
                    }
                }
                if (current_lane_ != LaneID::LANE_2 && !s.front_blocked && current_state_ == DrivingState::CRUISE)
                {
                    auto check = check_lane_front(LaneID::LANE_2);
                    if (check.first > FRONT_SAFE_DIST * 1.5 && is_overtake_safe(LaneID::LANE_2) && !are_lanes_overlapping(current_lane_, LaneID::LANE_2))
                    {
                        target_lane_ = LaneID::LANE_2;
                        current_state_ = DrivingState::RETURN_TO_CENTER;
                        is_lane_changing_ = true;
                        lane_change_start_sec_ = cur_sec;
                        lane_change_progress_ = 0.0;
                    }
                }
                break;
            case DrivingState::WAIT_FOR_GAP:
                if (is_overtake_safe(target_lane_))
                {
                    if (++safe_check_count_ >= SAFE_CHECK_REQUIRED)
                    {
                        current_state_ = DrivingState::LANE_CHANGING;
                        is_lane_changing_ = true;
                        lane_change_start_sec_ = cur_sec;
                        lane_change_progress_ = 0.0;
                    }
                }
                else
                    safe_check_count_ = 0;
                if (!s.front_blocked && s.front_dist > FRONT_SAFE_DIST * 1.5)
                {
                    current_state_ = DrivingState::CRUISE;
                    target_lane_ = current_lane_;
                }
                break;
            case DrivingState::LANE_CHANGING:
            case DrivingState::RETURN_TO_CENTER:
                lane_change_progress_ = std::min(1.0, (cur_sec - lane_change_start_sec_) / LANE_CHANGE_TIME);
                if (current_lane_ == target_lane_)
                {
                    if ((cur_sec - lane_change_start_sec_) > 0.3)
                    {
                        current_state_ = DrivingState::CRUISE;
                        is_lane_changing_ = false;
                    }
                }
                if ((cur_sec - lane_change_start_sec_) > 4.0)
                {
                    current_state_ = DrivingState::CRUISE;
                    is_lane_changing_ = false;
                    target_lane_ = current_lane_;
                }
                break;
            case DrivingState::EMERGENCY_ACCEL:
                if (!s.rear_danger || s.rear_dist > REAR_DANGER_DIST * 1.5)
                    current_state_ = DrivingState::CRUISE;
                else if (s.front_danger)
                {
                    LaneID esc = choose_overtake_lane();
                    if (esc != current_lane_ && is_overtake_safe(esc))
                    {
                        target_lane_ = esc;
                        current_state_ = DrivingState::LANE_CHANGING;
                        is_lane_changing_ = true;
                        lane_change_start_sec_ = cur_sec;
                        lane_change_progress_ = 0.0;
                    }
                }
                break;
            }
        }

        // ========================================================================
        // [FIX] HYBRID SMART VELOCITY (With MPC Handoff)
        // ========================================================================
        double compute_smart_velocity()
        {
            const auto &s = current_surrounding_;
            double target_v = 999.0;
            bool need_intervention = false;

            if (s.front_blocked || s.front_danger)
            {
                need_intervention = true;
                if (s.front_critical)
                {
                    target_v = MIN_SPEED;
                }
                else if (s.front_is_static)
                {
                    double ratio = (s.front_dist - FRONT_CRITICAL_DIST) / (FRONT_SAFE_DIST - FRONT_CRITICAL_DIST);
                    ratio = std::clamp(ratio, 0.0, 1.0);
                    target_v = MIN_SPEED + ratio * (SLOW_SPEED - MIN_SPEED);
                }
                else
                {
                    target_v = std::max(s.front_speed * 0.95, MIN_SPEED);
                }
            }

            double v_rear_min = 0.0;
            if (s.rear_danger)
            {
                need_intervention = true;
                if (s.rear_ttc < 1.5 || s.rear_dist < 0.4)
                    v_rear_min = ACCEL_SPEED;
                else if (s.rear_ttc < 2.5)
                    v_rear_min = std::max(s.rear_speed, CRUISE_SPEED);
                else
                    v_rear_min = s.rear_speed * 0.9;
            }

            if (current_state_ == DrivingState::WAIT_FOR_GAP)
            {
                need_intervention = true;
                bool target_left = is_target_left(target_lane_);
                bool side_blocked = target_left ? !s.left_clear : !s.right_clear;
                if (side_blocked)
                    target_v = std::min(target_v, SLOW_SPEED);
            }

            if (!need_intervention)
                return 999.0;

            if (v_rear_min > target_v && !s.front_critical)
                target_v = v_rear_min;

            if (is_in_corner_)
                target_v = std::min(target_v, 0.5);

            return std::clamp(target_v, MIN_SPEED, ACCEL_SPEED);
        }

        std::vector<PathPoint> generate_lane_change_path()
        {
            std::vector<PathPoint> traj;
            int f_idx = lane_to_int(current_lane_), t_idx = lane_to_int(target_lane_);
            if (processed_lanes_[f_idx].empty() || processed_lanes_[t_idx].empty())
                return traj;
            const auto &fl = processed_lanes_[f_idx];
            const auto &tl = processed_lanes_[t_idx];
            int n_f = fl.size();
            int closest = find_closest_idx(f_idx, ego_x_, ego_y_);
            double rem = std::max(LANE_CHANGE_FORWARD_DIST * (1.0 - lane_change_progress_), 0.3);
            int end_f = find_index_at_distance(f_idx, closest, rem);

            for (int i = 0; i <= 50; ++i)
            {
                double t = (double)i / 50.0;
                double blend = t * t * t * (t * (6 * t - 15) + 10);
                double idx_r = t * (end_f - closest);
                int fi = (closest + (int)idx_r + n_f) % n_f;
                int ti = find_closest_idx(t_idx, fl[fi].x, fl[fi].y, true);
                PathPoint pt;
                pt.x = fl[fi].x * (1 - blend) + tl[ti].x * blend;
                pt.y = fl[fi].y * (1 - blend) + tl[ti].y * blend;
                if (i < 50)
                {
                    double nt = (double)(i + 1) / 50.0;
                    double nb = nt * nt * nt * (nt * (6 * nt - 15) + 10);
                    double nir = nt * (end_f - closest);
                    int nfi = (closest + (int)nir + n_f) % n_f;
                    int nti = find_closest_idx(t_idx, fl[nfi].x, fl[nfi].y, true);
                    double nx = fl[nfi].x * (1 - nb) + tl[nti].x * nb;
                    double ny = fl[nfi].y * (1 - nb) + tl[nti].y * nb;
                    pt.yaw = std::atan2(ny - pt.y, nx - pt.x);
                }
                else
                    pt.yaw = tl[ti].yaw;
                traj.push_back(pt);
            }
            return traj;
        }

        void control_loop()
        {
            if (!pose_received_)
                return;
            bool rdy = true;
            for (int i = 0; i < 3; ++i)
                if (processed_lanes_[i].empty())
                    rdy = false;
            if (!rdy)
                return;

            if (!initial_search_done_)
            {
                for (int i = 0; i < 3; ++i)
                    find_closest_idx(i, ego_x_, ego_y_, true);
                initial_search_done_ = true;
            }

            if (current_lane_ == LaneID::NONE)
            {
                current_lane_ = get_lane_at(ego_x_, ego_y_);
                if (current_lane_ == LaneID::NONE)
                    current_lane_ = LaneID::LANE_2;
                target_lane_ = current_lane_;
            }

            update_all_obstacles_body_frame();

            int p_idx = lane_to_int(is_lane_changing_ ? target_lane_ : current_lane_);
            int cls = find_closest_idx(p_idx, ego_x_, ego_y_);
            update_corner_state(p_idx, cls);

            current_surrounding_ = check_all_zones();
            update_state_machine();

            if ((now_sec() - last_log_sec_) > 1.0)
            {
                RCLCPP_INFO(this->get_logger(), "[%s] %s%s | V=%.2f | F=%.2f",
                            state_str(current_state_).c_str(), lane_str(current_lane_).c_str(),
                            (target_lane_ != current_lane_) ? ("->" + lane_str(target_lane_)).c_str() : "",
                            ego_speed_, current_surrounding_.front_dist);
                last_log_sec_ = now_sec();
            }

            nav_msgs::msg::Path path_msg;
            path_msg.header.frame_id = "world";
            path_msg.header.stamp = this->now();

            if (is_lane_changing_)
            {
                auto tr = generate_lane_change_path();
                for (const auto &p : tr)
                {
                    geometry_msgs::msg::PoseStamped ps;
                    ps.header = path_msg.header;
                    ps.pose.position.x = p.x;
                    ps.pose.position.y = p.y;
                    tf2::Quaternion q;
                    q.setRPY(0, 0, p.yaw);
                    ps.pose.orientation = tf2::toMsg(q);
                    path_msg.poses.push_back(ps);
                }
                if (!tr.empty())
                {
                    auto lp = tr.back();
                    int ti = lane_to_int(target_lane_);
                    int tcl = find_closest_idx(ti, lp.x, lp.y, true);
                    const auto &tl = processed_lanes_[ti];
                    for (int i = 1; i < 80; ++i)
                    {
                        int idx = (tcl + i) % tl.size();
                        geometry_msgs::msg::PoseStamped ps;
                        ps.header = path_msg.header;
                        ps.pose.position.x = tl[idx].x;
                        ps.pose.position.y = tl[idx].y;
                        tf2::Quaternion q;
                        q.setRPY(0, 0, tl[idx].yaw);
                        ps.pose.orientation = tf2::toMsg(q);
                        path_msg.poses.push_back(ps);
                    }
                }
            }
            else
            {
                int idx = lane_to_int(current_lane_);
                const auto &ln = processed_lanes_[idx];
                for (int i = 0; i < 100; ++i)
                {
                    int k = (cls + i) % ln.size();
                    geometry_msgs::msg::PoseStamped ps;
                    ps.header = path_msg.header;
                    ps.pose.position.x = ln[k].x;
                    ps.pose.position.y = ln[k].y;
                    tf2::Quaternion q;
                    q.setRPY(0, 0, ln[k].yaw);
                    ps.pose.orientation = tf2::toMsg(q);
                    path_msg.poses.push_back(ps);
                }
            }
            local_pub_->publish(path_msg);

            std_msgs::msg::Float32 vm;
            vm.data = compute_smart_velocity();
            target_vel_pub_->publish(vm);
            publish_debug_markers();
        }

        void publish_debug_markers()
        {
            visualization_msgs::msg::MarkerArray ma;
            int id = 0;
            double cy = std::cos(ego_yaw_), sy = std::sin(ego_yaw_);
            tf2::Quaternion q;
            q.setRPY(0, 0, ego_yaw_);
            auto qm = tf2::toMsg(q);

            visualization_msgs::msg::Marker m;
            m.header.frame_id = "world";
            m.header.stamp = this->now();
            m.ns = "zones";
            m.type = visualization_msgs::msg::Marker::CUBE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.z = 0.02;
            m.pose.orientation = qm;

            m.id = id++;
            m.scale.x = FRONT_ZONE_LENGTH;
            m.scale.y = FRONT_ZONE_WIDTH * 2;
            m.scale.z = 0.01;
            m.pose.position.x = ego_x_ + cy * FRONT_ZONE_LENGTH / 2;
            m.pose.position.y = ego_y_ + sy * FRONT_ZONE_LENGTH / 2;
            if (current_surrounding_.front_blocked)
            {
                m.color.r = 1.0;
                m.color.g = 0.0;
            }
            else
            {
                m.color.r = 0.0;
                m.color.g = 1.0;
            }
            m.color.b = 0.0;
            m.color.a = 0.3;
            ma.markers.push_back(m);

            m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            m.ns = "txt";
            m.id = id++;
            m.pose.position.x = ego_x_;
            m.pose.position.y = ego_y_;
            m.pose.position.z = 0.5;
            m.scale.z = 0.15;
            m.color.r = 1.0;
            m.color.g = 1.0;
            m.color.b = 1.0;
            m.color.a = 1.0;
            std::stringstream ss;
            ss << state_str(current_state_);
            if (is_in_corner_)
                ss << "[C]";
            ss << "\n"
               << lane_str(current_lane_);
            if (target_lane_ != current_lane_)
                ss << "->" << lane_str(target_lane_);
            m.text = ss.str();
            ma.markers.push_back(m);
            debug_pub_->publish(ma);
        }
    };
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bisa::LocalPathPubCpp>());
    rclcpp::shutdown();
    return 0;
}
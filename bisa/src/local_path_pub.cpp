/**
 * @file local_path_pub.cpp
 * @brief Integrated Local Path Publisher with Merge Detection & Priority Fix
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
    PREPARE_OVERTAKE,
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

    double rel_x;
    double rel_y;
    double rel_vx;

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
            RCLCPP_INFO(this->get_logger(), "Local Path Publisher v3.1 (Merge Safety Fix)");
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
        // CONSTANTS
        // ========================================================================
        static constexpr double VEHICLE_LENGTH = 0.33;
        static constexpr double VEHICLE_WIDTH = 0.15;

        static constexpr double OBS_HALF_LENGTH = 0.17;
        static constexpr double OBS_HALF_WIDTH = 0.075;

        static constexpr double FRONT_ZONE_LENGTH = 0.8;
        static constexpr double FRONT_ZONE_WIDTH = 0.12;
        static constexpr double FRONT_SAFE_DIST = 0.70;
        static constexpr double FRONT_DANGER_DIST = 0.5;
        static constexpr double FRONT_CRITICAL_DIST = 0.35;

        static constexpr double REAR_ZONE_LENGTH = 1.0;
        static constexpr double REAR_ZONE_WIDTH = 0.12;
        static constexpr double REAR_DANGER_DIST = 0.5;
        static constexpr double REAR_TTC_THRESHOLD = 2.0;

        static constexpr double SIDE_ZONE_START = -0.5;
        static constexpr double SIDE_ZONE_END = 0.6;
        static constexpr double SIDE_ZONE_INNER = 0.10;
        static constexpr double SIDE_ZONE_OUTER = 0.35;

        static constexpr double LANE_WIDTH = 0.25;
        static constexpr double LANE_THRESHOLD = 0.20;
        // [New] 합류 구간 감지 임계값 (두 차선 간격이 이 값보다 작으면 같은 차선으로 간주)
        static constexpr double MERGE_THRESHOLD = 0.15;

        static constexpr double CRUISE_SPEED = 1.5;
        static constexpr double OVERTAKE_SPEED = 2.0;
        static constexpr double ACCEL_SPEED = 3.0;
        static constexpr double SLOW_SPEED = 0.8;
        static constexpr double MIN_SPEED = 0.0;

        static constexpr double LANE_CHANGE_FORWARD_DIST = 0.8;
        static constexpr double LANE_CHANGE_TIME = 1.5;

        static constexpr double CORNER_CURVATURE_THRESHOLD = 0.3;
        static constexpr double PATH_CHECK_HORIZON = 1.5;
        static constexpr double PATH_COLLISION_RADIUS = 0.22;
        static constexpr double STATIC_SPEED_THRESHOLD = 0.08;

        // ROS INTERFACES
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_lane_[3];
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obs_sub_;

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_vel_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        // STATE VARIABLES
        nav_msgs::msg::Path::SharedPtr lane_paths_[3];
        std::vector<PathPoint> processed_lanes_[3];

        double ego_x_ = 0.0, ego_y_ = 0.0, ego_yaw_ = 0.0;
        double ego_speed_ = 0.0;
        bool pose_received_ = false;
        double prev_x_ = 0.0, prev_y_ = 0.0;
        double prev_pose_sec_ = 0.0;
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
        // UTILITY FUNCTIONS
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
                return "LC";
            case DrivingState::EMERGENCY_ACCEL:
                return "ACCEL!";
            case DrivingState::RETURN_TO_CENTER:
                return "RETURN";
            default:
                return "???";
            }
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
                {
                    double dx = pt.x - processed_lanes_[lane_idx][i - 1].x;
                    double dy = pt.y - processed_lanes_[lane_idx][i - 1].y;
                    cumulative_s += std::hypot(dx, dy);
                }
                pt.s = cumulative_s;

                if (i < poses.size() - 1)
                {
                    double dx = poses[i + 1].pose.position.x - pt.x;
                    double dy = poses[i + 1].pose.position.y - pt.y;
                    pt.yaw = std::atan2(dy, dx);
                }
                else if (i > 0)
                {
                    pt.yaw = processed_lanes_[lane_idx][i - 1].yaw;
                }
                else
                {
                    pt.yaw = 0.0;
                }
                processed_lanes_[lane_idx].push_back(pt);
            }
        }

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

            for (auto it = obstacles_.begin(); it != obstacles_.end();)
            {
                if ((current_sec - it->second.last_seen_sec) > 1.0)
                    it = obstacles_.erase(it);
                else
                    ++it;
            }
        }

        // ========================================================================
        // LANE UTILITIES & MERGE CHECK
        // ========================================================================

        double get_dist_to_lane(int lane_idx, double x, double y)
        {
            if (lane_idx < 0 || lane_idx > 2)
                return 999.0;
            if (processed_lanes_[lane_idx].empty())
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

        // [New] 두 차선이 물리적으로 얼마나 가까운지 확인 (합류 구간 감지용)
        bool are_lanes_merged(LaneID lane_a, LaneID lane_b)
        {
            if (lane_a == lane_b)
                return true;

            int idx_a = lane_to_int(lane_a);
            int idx_b = lane_to_int(lane_b);

            if (processed_lanes_[idx_a].empty() || processed_lanes_[idx_b].empty())
                return false;

            // 현재 ego 위치 기준 Lane A의 가장 가까운 점 찾기
            int closest_a = find_closest_idx(idx_a, ego_x_, ego_y_);
            double ax = processed_lanes_[idx_a][closest_a].x;
            double ay = processed_lanes_[idx_a][closest_a].y;

            // 그 지점에서 Lane B까지의 거리 측정
            int closest_b = find_closest_idx(idx_b, ax, ay, true);
            double bx = processed_lanes_[idx_b][closest_b].x;
            double by = processed_lanes_[idx_b][closest_b].y;

            double dist = std::hypot(ax - bx, ay - by);

            // 차선 간격이 임계값보다 작으면 '합류됨(사실상 같은 차선)'으로 판단
            return (dist < MERGE_THRESHOLD);
        }

        int find_closest_idx(int lane_idx, double x, double y, bool global_search = false)
        {
            if (lane_idx < 0 || lane_idx > 2)
                return 0;
            if (processed_lanes_[lane_idx].empty())
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

        double compute_curvature(int lane_idx, int idx, int window = 10)
        {
            if (lane_idx < 0 || lane_idx > 2)
                return 0.0;
            if (processed_lanes_[lane_idx].size() < 30)
                return 0.0;

            const auto &lane = processed_lanes_[lane_idx];
            int n = lane.size();
            int i0 = ((idx - window) % n + n) % n;
            int i1 = idx;
            int i2 = (idx + window) % n;

            double a = std::hypot(lane[i1].x - lane[i0].x, lane[i1].y - lane[i0].y);
            double b = std::hypot(lane[i2].x - lane[i1].x, lane[i2].y - lane[i1].y);
            double c = std::hypot(lane[i0].x - lane[i2].x, lane[i0].y - lane[i2].y);
            double s = (a + b + c) / 2.0;
            double area = std::sqrt(std::max(0.0, s * (s - a) * (s - b) * (s - c)));

            if (a < 0.001 || b < 0.001 || c < 0.001)
                return 0.0;
            return 4.0 * area / (a * b * c);
        }

        void update_corner_state(int lane_idx, int closest)
        {
            current_curvature_ = compute_curvature(lane_idx, closest, 10);
            is_in_corner_ = (current_curvature_ > CORNER_CURVATURE_THRESHOLD);

            int preview_idx = (closest + 40) % processed_lanes_[lane_idx].size();
            double ahead_curv = compute_curvature(lane_idx, preview_idx, 10);
            corner_approaching_ = (ahead_curv > CORNER_CURVATURE_THRESHOLD);
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

            // 경로상의 점들을 순회하며 충돌 검사
            for (int i = 1; i < 80 && total_dist < max_check_dist; ++i)
            {
                int idx = (closest + i) % n;
                int prev_idx = (closest + i - 1) % n;
                double seg_dist = std::hypot(lane[idx].x - lane[prev_idx].x, lane[idx].y - lane[prev_idx].y);
                total_dist += seg_dist;
                double t = total_dist / check_speed;

                for (const auto &[id, obs] : obstacles_)
                {
                    // [수정] 무시 로직 제거: 모든 장애물에 대해 엄격하게 검사
                    double obs_x = obs.x + obs.vx * t;
                    double obs_y = obs.y + obs.vy * t;

                    // 내 차량(경로점)과 장애물 간의 거리
                    if (std::hypot(lane[idx].x - obs_x, lane[idx].y - obs_y) < PATH_COLLISION_RADIUS)
                    {
                        if (total_dist < collision_dist)
                            collision_dist = total_dist;
                        return true; // 충돌 감지
                    }
                }
            }
            return false;
        }

        SurroundingStatus check_all_zones()
        {
            SurroundingStatus status;
            double current_sec = now_sec();
            int path_lane = lane_to_int(target_lane_);
            double path_coll_dist;

            for (const auto &[id, obs] : obstacles_)
            {
                if ((current_sec - obs.last_seen_sec) > 0.5)
                    continue;
                double rx = obs.rel_x;
                double ry = obs.rel_y;

                if (obs.lane == current_lane_)
                {
                    if (rx > -OBS_HALF_LENGTH && rx < FRONT_ZONE_LENGTH + OBS_HALF_LENGTH && std::abs(ry) < FRONT_ZONE_WIDTH + OBS_HALF_WIDTH)
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
                    if (rx < OBS_HALF_LENGTH && rx > -(REAR_ZONE_LENGTH + OBS_HALF_LENGTH) && std::abs(ry) < REAR_ZONE_WIDTH + OBS_HALF_WIDTH)
                    {
                        double effective_dist = -rx - OBS_HALF_LENGTH;
                        if (effective_dist > 0 && effective_dist < status.rear_dist)
                        {
                            status.rear_dist = effective_dist;
                            status.rear_speed = obs.speed;
                            double closing_speed = -obs.rel_vx;
                            if (closing_speed > 0.05)
                            {
                                status.rear_ttc = effective_dist / closing_speed;
                                if (status.rear_ttc < REAR_TTC_THRESHOLD)
                                    status.rear_danger = true;
                            }
                        }
                    }
                }
                if (obs.lane != current_lane_ && obs.lane != LaneID::NONE)
                {
                    if (rx > SIDE_ZONE_START - OBS_HALF_LENGTH && rx < SIDE_ZONE_END + OBS_HALF_LENGTH)
                    {
                        if (ry > 0)
                        { // Left
                            double lat_dist = ry - OBS_HALF_WIDTH;
                            if (lat_dist < SIDE_ZONE_OUTER && lat_dist > -0.1)
                            {
                                status.left_clear = false;
                                status.left_dist = std::min(status.left_dist, lat_dist);
                            }
                        }
                        else
                        { // Right
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
            // [수정] ignore 인자 제거
            status.path_collision = check_path_collision(path_lane, path_coll_dist);
            status.path_collision_dist = path_coll_dist;
            return status;
        }

        bool is_target_left(LaneID target)
        {
            int t_idx = lane_to_int(target);
            if (processed_lanes_[t_idx].empty())
                return false;
            int closest = find_closest_idx(t_idx, ego_x_, ego_y_);
            double tx = processed_lanes_[t_idx][closest].x;
            double ty = processed_lanes_[t_idx][closest].y;
            double cross = std::cos(ego_yaw_) * (ty - ego_y_) - std::sin(ego_yaw_) * (tx - ego_x_);
            return cross > 0.0;
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
            bool going_left = is_target_left(target);
            if (going_left && !current_surrounding_.left_clear)
                return false;
            if (!going_left && !current_surrounding_.right_clear)
                return false;

            double coll_dist;
            int target_idx = lane_to_int(target);

            // [수정] ignore_id 인자 제거 및 엄격한 검사 수행
            // 앞차와 안전거리를(0.5m) 유지하고 있다면, 충돌반경(0.22m) 내에 들어오지 않으므로
            // 정상적인 경우라면 여기서 false가 나오지 않음.
            if (check_path_collision(target_idx, coll_dist))
            {
                if (coll_dist < 0.5)
                    return false;
            }

            for (const auto &[id, obs] : obstacles_)
            {
                if (obs.lane != target)
                    continue;
                double rx = obs.rel_x;
                // Target Lane 전방 체크
                if (rx > -OBS_HALF_LENGTH && rx < FRONT_ZONE_LENGTH)
                {
                    if (rx - OBS_HALF_LENGTH < FRONT_CRITICAL_DIST)
                        return false;
                }
                // Target Lane 후방 체크
                if (rx < 0 && rx > -REAR_ZONE_LENGTH)
                {
                    double closing = -obs.rel_vx;
                    if (closing > 0.1 && (-rx / closing) < 1.5)
                        return false;
                }
            }
            return true;
        }

        // ========================================================================
        // LANE SELECTION (MODIFIED FOR MERGE SCENARIO)
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
                // [수정 사항 1] Priority Swap: 합류 구간 위험 방지를 위해 L1(좌측)을 우선 탐색
                if (avoid_lane1)
                    candidates = {2};
                else
                    candidates = {0, 2}; // Left first (0), then Right (2)
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
                if (are_lanes_merged(current_lane_, target))
                    continue;
                if (is_lane_blocked_by_static(target))
                    continue;

                // [수정] ignore_id 제거
                double coll_dist;
                if (check_path_collision(cand, coll_dist) && coll_dist < 0.5)
                    continue;

                if (is_overtake_safe(target))
                    return target;
            }
            return current_lane_;
        }

        // ========================================================================
        // STATE MACHINE
        // ========================================================================

        void update_state_machine()
        {
            double current_sec = now_sec();
            const auto &s = current_surrounding_;

            if (s.rear_danger && current_state_ != DrivingState::EMERGENCY_ACCEL && current_state_ != DrivingState::LANE_CHANGING)
            {
                if (!s.front_critical && s.front_dist > FRONT_DANGER_DIST)
                {
                    current_state_ = DrivingState::EMERGENCY_ACCEL;
                    return;
                }
                else
                {
                    LaneID escape = choose_overtake_lane();
                    if (escape != current_lane_ && is_overtake_safe(escape))
                    {
                        target_lane_ = escape;
                        current_state_ = DrivingState::LANE_CHANGING;
                        is_lane_changing_ = true;
                        lane_change_start_sec_ = current_sec;
                        lane_change_progress_ = 0.0;
                        return;
                    }
                }
            }

            if (current_lane_ == LaneID::LANE_1 && corner_approaching_ && current_state_ == DrivingState::CRUISE)
            {
                if (is_overtake_safe(LaneID::LANE_2) && !are_lanes_merged(current_lane_, LaneID::LANE_2))
                {
                    target_lane_ = LaneID::LANE_2;
                    current_state_ = DrivingState::LANE_CHANGING;
                    is_lane_changing_ = true;
                    lane_change_start_sec_ = current_sec;
                    lane_change_progress_ = 0.0;
                    return;
                }
            }

            switch (current_state_)
            {
            case DrivingState::CRUISE:
            {
                bool need_overtake = s.front_blocked || s.front_danger;
                if (s.path_collision && s.path_collision_dist < 0.6)
                    need_overtake = true;

                if (need_overtake)
                {
                    LaneID overtake = choose_overtake_lane();
                    if (overtake != current_lane_)
                    {
                        target_lane_ = overtake;
                        if (s.front_is_static || s.front_critical)
                        {
                            current_state_ = DrivingState::LANE_CHANGING;
                            is_lane_changing_ = true;
                            lane_change_start_sec_ = current_sec;
                            lane_change_progress_ = 0.0;
                        }
                        else
                        {
                            current_state_ = DrivingState::PREPARE_OVERTAKE;
                            safe_check_count_ = 0;
                        }
                    }
                }

                if (current_lane_ != LaneID::LANE_2 && !need_overtake)
                {
                    auto center_check = check_lane_front(LaneID::LANE_2);
                    if (center_check.first > FRONT_SAFE_DIST * 1.5 && is_overtake_safe(LaneID::LANE_2) && !are_lanes_merged(current_lane_, LaneID::LANE_2))
                    {
                        target_lane_ = LaneID::LANE_2;
                        current_state_ = DrivingState::RETURN_TO_CENTER;
                        is_lane_changing_ = true;
                        lane_change_start_sec_ = current_sec;
                        lane_change_progress_ = 0.0;
                    }
                }
                break;
            }

            case DrivingState::PREPARE_OVERTAKE:
            {
                if (is_overtake_safe(target_lane_) && !are_lanes_merged(current_lane_, target_lane_))
                {
                    safe_check_count_++;
                    if (safe_check_count_ >= SAFE_CHECK_REQUIRED)
                    {
                        current_state_ = DrivingState::LANE_CHANGING;
                        is_lane_changing_ = true;
                        lane_change_start_sec_ = current_sec;
                        lane_change_progress_ = 0.0;
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
            case DrivingState::RETURN_TO_CENTER:
            {
                double elapsed = current_sec - lane_change_start_sec_;
                lane_change_progress_ = std::min(1.0, elapsed / LANE_CHANGE_TIME);

                if (current_lane_ == target_lane_)
                {
                    if (elapsed > 0.3)
                    {
                        current_state_ = DrivingState::CRUISE;
                        is_lane_changing_ = false;
                    }
                }
                if (elapsed > 4.0)
                {
                    current_state_ = DrivingState::CRUISE;
                    is_lane_changing_ = false;
                    target_lane_ = current_lane_;
                }
                break;
            }

            case DrivingState::EMERGENCY_ACCEL:
            {
                if (!s.rear_danger || s.rear_dist > REAR_DANGER_DIST * 1.5)
                {
                    current_state_ = DrivingState::CRUISE;
                }
                else if (s.front_danger)
                {
                    LaneID escape = choose_overtake_lane();
                    if (escape != current_lane_ && is_overtake_safe(escape))
                    {
                        target_lane_ = escape;
                        current_state_ = DrivingState::LANE_CHANGING;
                        is_lane_changing_ = true;
                        lane_change_start_sec_ = current_sec;
                        lane_change_progress_ = 0.0;
                    }
                }
                break;
            }
            }
        }

        std::pair<double, double> check_lane_front(LaneID lane)
        {
            double min_dist = 999.0;
            double obs_speed = 0.0;
            for (const auto &[id, obs] : obstacles_)
            {
                if (obs.lane != lane)
                    continue;
                double dist = std::hypot(obs.x - ego_x_, obs.y - ego_y_);
                double dx = obs.x - ego_x_;
                double dy = obs.y - ego_y_;
                double forward = dx * std::cos(ego_yaw_) + dy * std::sin(ego_yaw_);
                if (forward > 0 && dist < min_dist)
                {
                    min_dist = dist;
                    obs_speed = obs.speed;
                }
            }
            return {min_dist, obs_speed};
        }

        double compute_velocity()
        {
            const auto &s = current_surrounding_;
            double vel = CRUISE_SPEED; // 기본적으로 고속(MPC 맡김)

            // 1. 상황별 속도 제한 (Planner의 안전 로직 개입)
            switch (current_state_)
            {
            case DrivingState::CRUISE:
                if (s.front_blocked)
                {
                    if (s.front_is_static)
                    {
                        // 정지 차량 발견 시: 거리에 비례해서 멈추도록 설정
                        double dist_ratio = std::max(0.0, s.front_dist - 0.5) / 5.0; // 5m 전부터 감속
                        vel = std::min(SLOW_SPEED, dist_ratio * CRUISE_SPEED);
                    }
                    else
                    {
                        // 동적 장애물: 앞차 속도에 맞추거나 안전거리 유지
                        double gap = s.front_dist - FRONT_CRITICAL_DIST; // 여유 거리
                        double safe_v = std::max(0.0, gap * 1.5);        // P제어 느낌으로 거리 비례 속도

                        // 앞차 속도와 거리 비례 속도 중 작은 것 선택 (안전 우선)
                        vel = std::min(CRUISE_SPEED, std::max(s.front_speed, safe_v));
                    }
                }

                // 경로상 충돌 위험이 있으면 급감속
                if (s.path_collision && s.path_collision_dist < 1.0)
                {
                    double collision_ratio = std::max(0.0, s.path_collision_dist - 0.3);
                    vel = std::min(vel, collision_ratio * 2.0); // 충돌 지점 가까워질수록 0에 수렴
                }
                break;

            case DrivingState::PREPARE_OVERTAKE:
                // 추월 준비 중에는 앞차 속도에 맞춤
                if (!s.front_is_static)
                    vel = std::min(CRUISE_SPEED, s.front_speed);
                else
                    vel = SLOW_SPEED;
                break;

            case DrivingState::LANE_CHANGING:
            case DrivingState::RETURN_TO_CENTER:
                // 차선 변경 중에는 기본적으로 가속하되, 코너라면 감속
                vel = OVERTAKE_SPEED;
                break;

            case DrivingState::EMERGENCY_ACCEL:
                vel = ACCEL_SPEED;
                // 앞이 막혀있는데 뒤때문에 가속해야 하는 상황 -> 앞차 안 박을 정도로만 가속
                if (s.front_blocked)
                    vel = std::min(vel, std::max(s.front_speed, 1.0));
                break;
            }

            // 2. 물리적 제약 (코너링)
            // 차선 변경 중이 아닐 때, 코너에서는 안전을 위해 Planner가 상한선을 그어줌
            if (is_in_corner_)
            {
                // 코너에서는 1.2m/s 정도로 제한 (MPC가 더 줄일 수도 있음)
                vel = std::min(vel, 1.2);
            }

            // 3. 최종 클램핑
            // MPC가 YAML에 설정된 max_velocity(3.0)를 넘지 않도록 여기서도 한번 더 체크해주면 좋음
            return std::clamp(vel, MIN_SPEED, 3.0);
        }

        std::vector<PathPoint> generate_lane_change_path()
        {
            std::vector<PathPoint> trajectory;
            int from_idx = lane_to_int(current_lane_);
            int to_idx = lane_to_int(target_lane_);
            if (processed_lanes_[from_idx].empty() || processed_lanes_[to_idx].empty())
                return trajectory;

            const auto &from_lane = processed_lanes_[from_idx];
            const auto &to_lane = processed_lanes_[to_idx];
            int n_from = from_lane.size();

            int from_closest = find_closest_idx(from_idx, ego_x_, ego_y_);
            double remaining_forward = LANE_CHANGE_FORWARD_DIST * (1.0 - lane_change_progress_);
            remaining_forward = std::max(remaining_forward, 0.3);

            int end_idx_from = find_index_at_distance(from_idx, from_closest, remaining_forward);
            int num_points = 50;

            for (int i = 0; i <= num_points; ++i)
            {
                double t = static_cast<double>(i) / num_points;
                double lateral_blend = t * t * t * (t * (6.0 * t - 15.0) + 10.0);
                double idx_ratio = t * (end_idx_from - from_closest);
                int from_idx_interp = (from_closest + static_cast<int>(idx_ratio) + n_from) % n_from;
                int to_idx_interp = find_closest_idx(to_idx, from_lane[from_idx_interp].x, from_lane[from_idx_interp].y, true);

                PathPoint pt;
                pt.x = from_lane[from_idx_interp].x * (1.0 - lateral_blend) + to_lane[to_idx_interp].x * lateral_blend;
                pt.y = from_lane[from_idx_interp].y * (1.0 - lateral_blend) + to_lane[to_idx_interp].y * lateral_blend;

                if (i < num_points)
                {
                    double next_t = static_cast<double>(i + 1) / num_points;
                    double next_lateral = next_t * next_t * next_t * (next_t * (6.0 * next_t - 15.0) + 10.0);
                    double next_idx_ratio = next_t * (end_idx_from - from_closest);
                    int next_from = (from_closest + static_cast<int>(next_idx_ratio) + n_from) % n_from;
                    int next_to = find_closest_idx(to_idx, from_lane[next_from].x, from_lane[next_from].y, true);
                    double next_x = from_lane[next_from].x * (1.0 - next_lateral) + to_lane[next_to].x * next_lateral;
                    double next_y = from_lane[next_from].y * (1.0 - next_lateral) + to_lane[next_to].y * next_lateral;
                    pt.yaw = std::atan2(next_y - pt.y, next_x - pt.x);
                }
                else
                {
                    pt.yaw = to_lane[to_idx_interp].yaw;
                }
                pt.s = t * remaining_forward;
                trajectory.push_back(pt);
            }
            return trajectory;
        }

        void control_loop()
        {
            if (!pose_received_)
                return;
            bool ready = true;
            for (int i = 0; i < 3; ++i)
                if (processed_lanes_[i].empty())
                {
                    ready = false;
                    break;
                }
            if (!ready)
                return;

            if (!initial_search_done_)
            {
                for (int i = 0; i < 3; ++i)
                    find_closest_idx(i, ego_x_, ego_y_, true);
                initial_search_done_ = true;
                RCLCPP_INFO(this->get_logger(), "Initial search done.");
            }

            if (current_lane_ == LaneID::NONE)
            {
                current_lane_ = get_lane_at(ego_x_, ego_y_);
                if (current_lane_ == LaneID::NONE)
                    current_lane_ = LaneID::LANE_2;
                target_lane_ = current_lane_;
            }

            update_all_obstacles_body_frame();

            int path_lane_idx = lane_to_int(is_lane_changing_ ? target_lane_ : current_lane_);
            int closest = find_closest_idx(path_lane_idx, ego_x_, ego_y_);
            update_corner_state(path_lane_idx, closest);

            current_surrounding_ = check_all_zones();
            update_state_machine();

            double current_sec = now_sec();
            if ((current_sec - last_log_sec_) > 1.0)
            {
                RCLCPP_INFO(this->get_logger(), "[%s] %s%s | Spd=%.2f",
                            state_str(current_state_).c_str(),
                            lane_str(current_lane_).c_str(),
                            (target_lane_ != current_lane_) ? ("->" + lane_str(target_lane_)).c_str() : "",
                            ego_speed_);
                last_log_sec_ = current_sec;
            }

            nav_msgs::msg::Path local_path_msg;
            local_path_msg.header.frame_id = "world";
            local_path_msg.header.stamp = this->now();

            if (is_lane_changing_)
            {
                auto lc_path = generate_lane_change_path();
                for (const auto &pt : lc_path)
                {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = local_path_msg.header;
                    pose.pose.position.x = pt.x;
                    pose.pose.position.y = pt.y;
                    pose.pose.position.z = 0.0;
                    tf2::Quaternion q;
                    q.setRPY(0, 0, pt.yaw);
                    pose.pose.orientation = tf2::toMsg(q);
                    local_path_msg.poses.push_back(pose);
                }
                if (!lc_path.empty())
                {
                    const auto &last_pt = lc_path.back();
                    int to_idx = lane_to_int(target_lane_);
                    int target_closest = find_closest_idx(to_idx, last_pt.x, last_pt.y, true);
                    const auto &to_lane = processed_lanes_[to_idx];
                    int n = to_lane.size();
                    for (int i = 1; i < 80; ++i)
                    {
                        int idx = (target_closest + i) % n;
                        geometry_msgs::msg::PoseStamped pose;
                        pose.header = local_path_msg.header;
                        pose.pose.position.x = to_lane[idx].x;
                        pose.pose.position.y = to_lane[idx].y;
                        pose.pose.position.z = 0.0;
                        tf2::Quaternion q;
                        q.setRPY(0, 0, to_lane[idx].yaw);
                        pose.pose.orientation = tf2::toMsg(q);
                        local_path_msg.poses.push_back(pose);
                    }
                }
            }
            else
            {
                int lane_idx = lane_to_int(current_lane_);
                const auto &lane = processed_lanes_[lane_idx];
                int n = lane.size();
                for (int i = 0; i < 100; ++i)
                {
                    int idx = (closest + i) % n;
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = local_path_msg.header;
                    pose.pose.position.x = lane[idx].x;
                    pose.pose.position.y = lane[idx].y;
                    pose.pose.position.z = 0.0;
                    tf2::Quaternion q;
                    q.setRPY(0, 0, lane[idx].yaw);
                    pose.pose.orientation = tf2::toMsg(q);
                    local_path_msg.poses.push_back(pose);
                }
            }
            local_pub_->publish(local_path_msg);

            double target_vel = compute_velocity();
            std_msgs::msg::Float32 vel_msg;
            vel_msg.data = target_vel;
            target_vel_pub_->publish(vel_msg);

            publish_debug_markers();
        }

        void publish_debug_markers()
        {
            visualization_msgs::msg::MarkerArray markers;
            int id = 0;
            double cos_yaw = std::cos(ego_yaw_);
            double sin_yaw = std::sin(ego_yaw_);
            tf2::Quaternion q;
            q.setRPY(0, 0, ego_yaw_);
            auto quat_msg = tf2::toMsg(q);

            {
                visualization_msgs::msg::Marker m;
                m.header.frame_id = "world";
                m.header.stamp = this->now();
                m.ns = "zones";
                m.id = id++;
                m.type = visualization_msgs::msg::Marker::CUBE;
                m.action = visualization_msgs::msg::Marker::ADD;
                double cx = FRONT_ZONE_LENGTH / 2;
                m.pose.position.x = ego_x_ + cos_yaw * cx;
                m.pose.position.y = ego_y_ + sin_yaw * cx;
                m.pose.position.z = 0.02;
                m.pose.orientation = quat_msg;
                m.scale.x = FRONT_ZONE_LENGTH;
                m.scale.y = FRONT_ZONE_WIDTH * 2;
                m.scale.z = 0.01;
                m.color.a = 0.4;
                m.color.g = 1.0;
                if (current_surrounding_.front_critical)
                {
                    m.color.r = 1.0;
                    m.color.g = 0.0;
                }
                markers.markers.push_back(m);
            }
            debug_pub_->publish(markers);
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
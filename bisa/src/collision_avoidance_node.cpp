/**
 * @file collision_avoidance_node.cpp
 * @brief 충돌 방지 노드
 */

#include "bisa/collision_avoidance_node.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>
#include <iomanip>

using namespace std::chrono_literals;

namespace bisa
{
    // ============================================================================
    // Sigmoid Velocity Control System
    // ============================================================================
    struct SigmoidVelocityState
    {
        double target_velocity;
        double initial_velocity;
        double phase_time;
        bool is_accelerating;
        bool is_decelerating;

        SigmoidVelocityState()
            : target_velocity(0.0),
              initial_velocity(0.0),
              phase_time(0.0),
              is_accelerating(false),
              is_decelerating(false)
        {
        }
    };

    static std::map<int, SigmoidVelocityState> sigmoid_velocity_states_;

    static double sigmoid_fn(double t, double k, double t_center)
    {
        double exponent = -k * (t - t_center);
        if (exponent > 20.0)
            return 0.0;
        if (exponent < -20.0)
            return 1.0;
        return 1.0 / (1.0 + std::exp(exponent));
    }

    static double compute_sigmoid_vel(int cav_id, double current_vel, double target_vel, double dt)
    {
        auto &sig = sigmoid_velocity_states_[cav_id];
        bool is_hard_stop = (target_vel < 0.001);
        const double K_ACCEL = 5.0;
        const double K_DECEL = is_hard_stop ? 8.0 : 0.5;
        const double TRANS_TIME = is_hard_stop ? 0.2 : 0.4;

        if (std::abs(target_vel - sig.target_velocity) > 0.01)
        {
            sig.initial_velocity = current_vel;
            sig.target_velocity = target_vel;
            sig.phase_time = 0.0;

            if (target_vel > current_vel)
            {
                sig.is_accelerating = true;
                sig.is_decelerating = false;
            }
            else if (target_vel < current_vel)
            {
                sig.is_accelerating = false;
                sig.is_decelerating = true;
            }
            else
            {
                sig.is_accelerating = false;
                sig.is_decelerating = false;
            }
        }

        sig.phase_time += dt;
        double result_vel = current_vel;

        if (sig.is_accelerating)
        {
            double t_center = TRANS_TIME / 2.0;
            double progress = sigmoid_fn(sig.phase_time, K_ACCEL, t_center);
            result_vel = sig.initial_velocity + (sig.target_velocity - sig.initial_velocity) * progress;
            if (sig.phase_time >= TRANS_TIME)
            {
                sig.is_accelerating = false;
                result_vel = sig.target_velocity;
            }
        }
        else if (sig.is_decelerating)
        {
            double t_center = TRANS_TIME / 2.0;
            double progress = sigmoid_fn(sig.phase_time, K_DECEL, t_center);
            result_vel = sig.initial_velocity - (sig.initial_velocity - sig.target_velocity) * progress;
            double time_mult = is_hard_stop ? 1.0 : 2.0;
            if (sig.phase_time >= TRANS_TIME * time_mult)
            {
                sig.is_decelerating = false;
                result_vel = sig.target_velocity;
            }
        }
        else
        {
            result_vel = sig.target_velocity;
        }

        if (result_vel < 0.001)
            result_vel = 0.0;
        return result_vel;
    }

    // ============================================================================
    // CollisionAvoidanceNode Implementation
    // ============================================================================

    CollisionAvoidanceNode::CollisionAvoidanceNode()
        : Node("collision_avoidance_node")
    {
        load_parameters();
        init_node_positions();
        init_hv_global_paths();
        init_subscribers();
        init_publishers();
        init_timers();
    }

    void CollisionAvoidanceNode::load_parameters()
    {
        this->declare_parameter("hv_ids", std::vector<int64_t>{19, 20});
        std::vector<int64_t> hv_ids_64 = this->get_parameter("hv_ids").as_integer_array();
        hv_ids_.clear();
        for (const auto &id : hv_ids_64)
            hv_ids_.push_back(static_cast<int>(id));

        this->declare_parameter("cav_ids", std::vector<int64_t>{1, 2, 3, 4});
        std::vector<int64_t> cav_ids_64 = this->get_parameter("cav_ids").as_integer_array();
        cav_ids_.clear();
        for (const auto &id : cav_ids_64)
            cav_ids_.push_back(static_cast<int>(id));

        this->declare_parameter("high_priority_ids", std::vector<int64_t>{1, 2});
        std::vector<int64_t> pri_ids_64 = this->get_parameter("high_priority_ids").as_integer_array();
        high_priority_cavs_.clear();
        for (const auto &id : pri_ids_64)
            high_priority_cavs_.push_back(static_cast<int>(id));
    }

    void CollisionAvoidanceNode::init_node_positions()
    {
        node_positions_[36] = Point2D(1.30833333333333, 1.4);
        node_positions_[37] = Point2D(1.55833333333333, 1.4);
        node_positions_[46] = Point2D(0.06444444444444, -0.10833333);
        node_positions_[47] = Point2D(0.06444444444444, -0.35833333);
        node_positions_[38] = Point2D(0.9419234046606328, 0.7247577813792205);
        node_positions_[41] = Point2D(0.9419234046606328, -0.72475778137922);
        node_positions_[62] = Point2D(0.2, 0.0);
        cav_entry_nodes_[1] = 46;
        cav_entry_nodes_[2] = 37;
        cav_entry_nodes_[3] = 36;
        cav_entry_nodes_[4] = 47;

        std::vector<std::array<double, 3>> palette = {
            {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, 1.0, 0.0}, {1.0, 1.0, 0.0}};

        cav_colors_.clear();
        for (size_t i = 0; i < cav_ids_.size(); ++i)
        {
            int id = cav_ids_[i];
            if (i < palette.size())
                cav_colors_[id] = palette[i];
            else
                cav_colors_[id] = {1.0, 1.0, 1.0};
        }

        for (int cav_id : cav_ids_)
            prev_dist_to_entry_[cav_id] = 999.0;
    }

    void CollisionAvoidanceNode::init_hv_global_paths()
    {
        double cx = ROUNDABOUT_CENTER_X, cy = ROUNDABOUT_CENTER_Y, r = 0.75;
        std::vector<Point2D> fallback_path;
        int num_points = static_cast<int>(2 * M_PI * r / 0.02);
        for (int i = 0; i < num_points; ++i)
        {
            double angle = 2 * M_PI * i / num_points;
            fallback_path.push_back(Point2D(cx + r * std::cos(-angle + M_PI / 2), cy + r * std::sin(-angle + M_PI / 2)));
        }
        for (int hv_id : hv_ids_)
        {
            hv_states_[hv_id].global_path = fallback_path;
            hv_states_[hv_id].current_waypoint = 0;
        }
    }

    void CollisionAvoidanceNode::init_subscribers()
    {
        rmw_qos_profile_t sensor_qos_profile = rmw_qos_profile_sensor_data;
        auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(sensor_qos_profile), sensor_qos_profile);

        for (int hv_id : hv_ids_)
        {
            hv_states_[hv_id].valid = false;
            hv_pose_subs_[hv_id] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/HV_" + std::to_string(hv_id), sensor_qos,
                [this, hv_id](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
                { this->hv_pose_callback(msg, hv_id); });
            hv_global_path_subs_[hv_id] = this->create_subscription<nav_msgs::msg::Path>(
                "/user_global_path_hv" + std::to_string(hv_id), rclcpp::QoS(10).transient_local(),
                [this, hv_id](const nav_msgs::msg::Path::SharedPtr msg)
                { this->hv_global_path_callback(msg, hv_id); });
        }

        for (int cav_id : cav_ids_)
        {
            cav_states_[cav_id] = CAVState();
            last_accel_[cav_id] = geometry_msgs::msg::Accel();
            hv_debug_info_[cav_id] = "";
            std::string cav_str = format_cav_id(cav_id);

            cav_pose_subs_[cav_id] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/CAV_" + cav_str, sensor_qos,
                [this, cav_id](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
                { this->cav_pose_callback(msg, cav_id); });
            cav_path_subs_[cav_id] = this->create_subscription<nav_msgs::msg::Path>(
                "/local_path_cav" + cav_str, 10,
                [this, cav_id](const nav_msgs::msg::Path::SharedPtr msg)
                { this->local_path_callback(msg, cav_id); });
            cav_global_path_subs_[cav_id] = this->create_subscription<nav_msgs::msg::Path>(
                "/user_global_path_cav" + cav_str, rclcpp::QoS(10).transient_local(),
                [this, cav_id](const nav_msgs::msg::Path::SharedPtr msg)
                { this->cav_global_path_callback(msg, cav_id); });
            cav_lap_subs_[cav_id] = this->create_subscription<bisa::msg::LapInfo>(
                "/lap_info_cav" + cav_str, 10,
                [this, cav_id](const bisa::msg::LapInfo::SharedPtr msg)
                { this->lap_info_callback(msg, cav_id); });
            cav_accel_subs_[cav_id] = this->create_subscription<geometry_msgs::msg::Accel>(
                "/CAV_" + cav_str + "_accel_sync", 10,
                [this, cav_id](const geometry_msgs::msg::Accel::SharedPtr msg)
                { this->accel_callback(msg, cav_id); });
        }
    }

    void CollisionAvoidanceNode::init_publishers()
    {
        auto qos = rclcpp::QoS(10);
        for (int cav_id : cav_ids_)
        {
            accel_pubs_[cav_id] = this->create_publisher<geometry_msgs::msg::Accel>("/CAV_" + format_cav_id(cav_id) + "_accel", 10);
        }
        for (int hv_id : hv_ids_)
        {
            hv_local_path_pubs_[hv_id] = this->create_publisher<nav_msgs::msg::Path>("/local_path_hv" + std::to_string(hv_id), 10);
        }
        hv_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/hv_markers", qos);
        cav_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cav_markers", qos);
        zone_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/collision_zones", qos);
        safety_path_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/safety_paths", qos);
    }

    void CollisionAvoidanceNode::init_timers()
    {
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(25),
            std::bind(&CollisionAvoidanceNode::control_loop, this));

        vis_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CollisionAvoidanceNode::publish_visualizations, this));

        debug_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&CollisionAvoidanceNode::log_hv_debug, this));
    }

    void CollisionAvoidanceNode::hv_global_path_callback(const nav_msgs::msg::Path::SharedPtr msg, int hv_id)
    {
        auto &state = hv_states_[hv_id];
        state.global_path.clear();
        for (const auto &pose : msg->poses)
        {
            state.global_path.push_back(Point2D(pose.pose.position.x, pose.pose.position.y));
        }
    }

    void CollisionAvoidanceNode::hv_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int hv_id)
    {
        auto &state = hv_states_[hv_id];
        double x = msg->pose.position.x, y = msg->pose.position.y;
        double current_time = get_current_time();

        if (hv_history_[hv_id].size() >= 2)
        {
            auto &[prev_time, prev_x, prev_y] = hv_history_[hv_id].back();
            double dt = current_time - prev_time;
            if (dt > 0.01)
            {
                double new_speed = std::sqrt((x - prev_x) * (x - prev_x) + (y - prev_y) * (y - prev_y)) / dt;
                state.speed = 0.7 * state.speed + 0.3 * std::min(new_speed, 3.0);
            }
        }
        hv_history_[hv_id].push_back({current_time, x, y});
        if (hv_history_[hv_id].size() > 15)
            hv_history_[hv_id].pop_front();

        state.position = Point2D(x, y);
        state.yaw = msg->pose.orientation.z;
        state.valid = true;

        if (!state.global_path.empty())
        {
            double min_dist = 1e9;
            size_t closest = 0;
            for (size_t i = 0; i < state.global_path.size(); ++i)
            {
                double d = state.position.dist_to(state.global_path[i]);
                if (d < min_dist)
                {
                    min_dist = d;
                    closest = i;
                }
            }
            state.current_waypoint = closest;
            state.rear_point = get_point_behind_on_path(state.global_path, closest, HV_REAR_OFFSET);
        }

        update_hv_dynamic_path(hv_id);
    }

    Point2D CollisionAvoidanceNode::get_point_behind_on_path(const std::vector<Point2D> &path, size_t current_idx, double distance) const
    {
        if (path.empty())
            return Point2D();

        size_t total = path.size();
        double accumulated_dist = 0;
        size_t target_idx = current_idx;

        for (size_t i = 1; i < total && accumulated_dist < distance; ++i)
        {
            size_t curr_idx = (current_idx + total - i) % total;
            size_t prev_idx = (current_idx + total - i + 1) % total;
            double d = path[curr_idx].dist_to(path[prev_idx]);
            accumulated_dist += d;
            target_idx = curr_idx;
        }

        return path[target_idx];
    }

    void CollisionAvoidanceNode::cav_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int cav_id)
    {
        auto &state = cav_states_[cav_id];
        Point2D new_pos(msg->pose.position.x, msg->pose.position.y);

        double dt = 0.001;
        if (state.position.x != 0 || state.position.y != 0)
        {
            double dx = new_pos.x - state.position.x, dy = new_pos.y - state.position.y;
            double new_speed = std::sqrt(dx * dx + dy * dy) / dt;
            state.speed = 0.9 * state.speed + 0.1 * std::min(new_speed, 3.0);
        }
        state.position = new_pos;
        state.yaw = msg->pose.orientation.z;

        Point2D entry_pos = node_positions_[cav_entry_nodes_[cav_id]];
        double dist_to_center = state.position.dist_to(get_roundabout_center());
        double dist_entry_to_center = entry_pos.dist_to(get_roundabout_center());
        if (dist_to_center < dist_entry_to_center - 0.3)
            state.passed_entry = true;
        if (dist_to_center > ROUNDABOUT_RADIUS)
            state.passed_entry = false;
        state.in_roundabout = dist_to_center < ROUNDABOUT_RADIUS - 0.2;

        if (!state.global_path.empty())
        {
            double min_dist = 1e9;
            size_t closest = state.current_global_waypoint;
            size_t total = state.global_path.size();
            int search_window = 200;
            for (int i = -search_window; i <= search_window; ++i)
            {
                long idx_temp = static_cast<long>(state.current_global_waypoint) + i;
                if (idx_temp < 0)
                    idx_temp += total;
                else if (idx_temp >= static_cast<long>(total))
                    idx_temp %= total;
                size_t idx = static_cast<size_t>(idx_temp);
                double d = state.position.dist_to(state.global_path[idx]);
                if (d < min_dist)
                {
                    min_dist = d;
                    closest = idx;
                }
            }
            if (min_dist > 2.0)
            {
                min_dist = 1e9;
                for (size_t i = 0; i < total; ++i)
                {
                    double d = state.position.dist_to(state.global_path[i]);
                    if (d < min_dist)
                    {
                        min_dist = d;
                        closest = i;
                    }
                }
            }
            state.current_global_waypoint = closest;
        }

        update_cav_dynamic_path(cav_id);
        update_cav_rear_path(cav_id);
        update_cav_full_safety_path(cav_id);
    }

    void CollisionAvoidanceNode::local_path_callback(const nav_msgs::msg::Path::SharedPtr msg, int cav_id)
    {
        auto &state = cav_states_[cav_id];
        state.local_path.clear();
        int count = std::min(static_cast<int>(msg->poses.size()), LOOKAHEAD_POINTS);
        for (int i = 0; i < count; ++i)
        {
            state.local_path.push_back(Point2D(msg->poses[i].pose.position.x, msg->poses[i].pose.position.y));
        }
        update_cav_dynamic_path(cav_id);
        update_cav_full_safety_path(cav_id);
    }

    void CollisionAvoidanceNode::cav_global_path_callback(const nav_msgs::msg::Path::SharedPtr msg, int cav_id)
    {
        auto &state = cav_states_[cav_id];
        state.global_path.clear();
        for (const auto &pose : msg->poses)
        {
            state.global_path.push_back(Point2D(pose.pose.position.x, pose.pose.position.y));
        }
    }

    void CollisionAvoidanceNode::lap_info_callback(const bisa::msg::LapInfo::SharedPtr msg, int cav_id)
    {
        auto &state = cav_states_[cav_id];
        state.remaining_waypoints = msg->total_waypoints - msg->current_waypoint;
        if (msg->current_waypoint < 3)
            state.passed_entry = false;
    }

    void CollisionAvoidanceNode::accel_callback(const geometry_msgs::msg::Accel::SharedPtr msg, int cav_id)
    {
        last_accel_[cav_id] = *msg;
    }

    double CollisionAvoidanceNode::calculate_path_curvature(const std::vector<Point2D> &path, int start_idx, int window)
    {
        if (path.size() < 3)
            return 0.0;
        int end_idx = std::min(start_idx + window, (int)path.size() - 1);
        int mid_idx = (start_idx + end_idx) / 2;
        if (end_idx - start_idx < 2)
            return 0.0;
        const Point2D &A = path[start_idx], &B = path[mid_idx], &C = path[end_idx];
        double area = std::abs((B.x - A.x) * (C.y - A.y) - (C.x - A.x) * (B.y - A.y)) / 2.0;
        double AB = A.dist_to(B), BC = B.dist_to(C), CA = C.dist_to(A);
        double denom = AB * BC * CA;
        return (denom < 0.0001) ? 0.0 : 4.0 * area / denom;
    }

    void CollisionAvoidanceNode::update_hv_dynamic_path(int hv_id)
    {
        auto &state = hv_states_[hv_id];
        state.dynamic_path.clear();
        if (!state.valid || state.global_path.empty())
            return;

        size_t total = state.global_path.size();
        size_t start_idx = state.current_waypoint;
        double rear_accumulated = 0;
        std::vector<Point2D> rear_segment;
        for (size_t i = 1; i < total && rear_accumulated < HV_REAR_OFFSET; ++i)
        {
            size_t curr_idx = (start_idx + total - i) % total;
            size_t prev_idx = (start_idx + total - i + 1) % total;
            double d = state.global_path[curr_idx].dist_to(state.global_path[prev_idx]);
            rear_accumulated += d;
            rear_segment.insert(rear_segment.begin(), state.global_path[curr_idx]);
        }
        for (const auto &p : rear_segment)
            state.dynamic_path.push_back(p);
        state.dynamic_path.push_back(state.position);

        double path_length = std::min(std::max(state.speed, MIN_PATH_LENGTH), HV_MAX_PATH_LENGTH);
        double front_accumulated = 0;
        for (size_t i = 0; i < total && front_accumulated < path_length; ++i)
        {
            size_t curr_idx = (start_idx + i) % total;
            state.dynamic_path.push_back(state.global_path[curr_idx]);
            if (i > 0)
            {
                size_t prev_idx = (start_idx + i - 1) % total;
                front_accumulated += state.global_path[curr_idx].dist_to(state.global_path[prev_idx]);
            }
        }
    }

    void CollisionAvoidanceNode::update_cav_dynamic_path(int cav_id)
    {
        auto &state = cav_states_[cav_id];
        state.dynamic_path.clear();
        if (state.local_path.empty())
            return;
        double curvature = calculate_path_curvature(state.local_path, 0, 50);
        state.current_curvature = curvature;
        double extra = (curvature >= CURVATURE_THRESHOLD) ? std::min(curvature * CURVATURE_PATH_GAIN, MAX_CURVE_EXTRA) : 0.0;
        double path_length = std::max(state.speed, MIN_PATH_LENGTH) + extra;
        double accumulated_dist = 0;
        state.dynamic_path.push_back(state.local_path[0]);
        for (size_t i = 1; i < state.local_path.size(); ++i)
        {
            double d = state.local_path[i].dist_to(state.local_path[i - 1]);
            accumulated_dist += d;
            state.dynamic_path.push_back(state.local_path[i]);
            if (accumulated_dist >= path_length)
                break;
        }

        if (!state.global_path.empty() && SAFETY_PATH_EXTRA_POINTS > 0)
        {
            Point2D last_point = state.dynamic_path.back();
            double min_dist = 1e9;
            size_t closest_idx = 0;
            for (size_t i = 0; i < state.global_path.size(); ++i)
            {
                double d = last_point.dist_to(state.global_path[i]);
                if (d < min_dist)
                {
                    min_dist = d;
                    closest_idx = i;
                }
            }
            size_t total = state.global_path.size();
            for (int i = 1; i <= SAFETY_PATH_EXTRA_POINTS; ++i)
            {
                size_t idx = (closest_idx + i) % total;
                state.dynamic_path.push_back(state.global_path[idx]);
            }
        }
    }

    void CollisionAvoidanceNode::update_cav_rear_path(int cav_id)
    {
        auto &state = cav_states_[cav_id];
        state.rear_path.clear();
        if (state.global_path.empty())
            return;
        size_t total = state.global_path.size();
        size_t current_idx = state.current_global_waypoint;
        state.rear_path.push_back(state.position);
        double accumulated_dist = 0;
        for (size_t i = 1; i < total && accumulated_dist < REAR_PATH_LENGTH; ++i)
        {
            size_t curr_idx = (current_idx + total - i) % total;
            size_t prev_idx = (current_idx + total - i + 1) % total;
            double d = state.global_path[curr_idx].dist_to(state.global_path[prev_idx]);
            accumulated_dist += d;
            state.rear_path.insert(state.rear_path.begin(), state.global_path[curr_idx]);
            if (state.rear_path.size() > 200)
                break;
        }
    }

    void CollisionAvoidanceNode::update_cav_full_safety_path(int cav_id)
    {
        auto &state = cav_states_[cav_id];
        state.full_safety_path.clear();
        for (const auto &p : state.rear_path)
            state.full_safety_path.push_back(p);
        for (size_t i = 1; i < state.dynamic_path.size(); ++i)
            state.full_safety_path.push_back(state.dynamic_path[i]);
    }

    std::tuple<bool, Point2D, int, int> CollisionAvoidanceNode::find_path_overlap(
        const std::vector<Point2D> &path1, const std::vector<Point2D> &path2) const
    {
        if (path1.empty() || path2.empty())
            return {false, Point2D(), -1, -1};
        const double OVERLAP_DIST = 0.10;
        for (size_t i = 0; i < path1.size(); ++i)
        {
            for (size_t j = 0; j < path2.size(); ++j)
            {
                double d = path1[i].dist_to(path2[j]);
                if (d < OVERLAP_DIST)
                {
                    Point2D zone((path1[i].x + path2[j].x) / 2, (path1[i].y + path2[j].y) / 2);
                    return {true, zone, (int)i, (int)j};
                }
            }
        }
        return {false, Point2D(), -1, -1};
    }

    bool CollisionAvoidanceNode::check_path_overlap(const std::vector<Point2D> &path1, const std::vector<Point2D> &path2) const
    {
        auto [overlaps, zone, idx1, idx2] = find_path_overlap(path1, path2);
        return overlaps;
    }

    bool CollisionAvoidanceNode::is_path_clear_of_zone(const std::vector<Point2D> &path, const Point2D &zone, double radius) const
    {
        for (const auto &p : path)
        {
            if (p.dist_to(zone) < radius)
                return false;
        }
        return true;
    }

    std::tuple<bool, int, bool> CollisionAvoidanceNode::check_hv_collision(int cav_id)
    {
        auto &cav = cav_states_[cav_id];
        if (cav.dynamic_path.empty())
            return {false, -1, false};

        Point2D cav_tip = cav.dynamic_path.back();
        const double HV_OVERLAP_DIST = 0.15;
        const double TIP_ZONE_LENGTH = 0.35;

        size_t behind_tip_idx = cav.dynamic_path.size() - 1;
        double accumulated = 0;
        for (int i = (int)cav.dynamic_path.size() - 1; i > 0; --i)
        {
            accumulated += cav.dynamic_path[i].dist_to(cav.dynamic_path[i - 1]);
            if (accumulated >= TIP_ZONE_LENGTH)
            {
                behind_tip_idx = i - 1;
                break;
            }
        }
        if (behind_tip_idx >= cav.dynamic_path.size())
            behind_tip_idx = 0;

        // ★★★ CAV 전방 마진 인덱스 계산 ★★★
        // CAV 중심점(dynamic_path[0])에서 전방 CAV_FRONT_MARGIN 만큼의 인덱스
        size_t front_margin_idx = 0;
        double front_accumulated = 0;
        for (size_t i = 1; i < cav.dynamic_path.size(); ++i)
        {
            front_accumulated += cav.dynamic_path[i].dist_to(cav.dynamic_path[i - 1]);
            if (front_accumulated >= CAV_FRONT_MARGIN)
            {
                front_margin_idx = i;
                break;
            }
        }
        // 경로가 짧아서 CAV_FRONT_MARGIN에 도달하지 못하면 전체를 앞쪽으로 간주
        if (front_margin_idx == 0 && cav.dynamic_path.size() > 1)
            front_margin_idx = cav.dynamic_path.size() - 1;

        for (int hv_id : hv_ids_)
        {
            auto &hv = hv_states_[hv_id];
            if (!hv.valid)
                continue;

            double dist_to_hv_rear = cav_tip.dist_to(hv.rear_point);
            double dist_to_hv_center = cav_tip.dist_to(hv.position);

            double min_dist_to_tip_zone = 1e9;
            double min_dist_to_behind_zone = 1e9;

            // ★★★ 새로운 변수: 전방 마진 기준 앞/뒤 ★★★
            double min_dist_to_front_of_margin = 1e9;  // front_margin_idx 이후 (차량 앞쪽)
            double min_dist_to_rear_of_margin = 1e9;   // front_margin_idx 이전 (차량 뒤쪽)

            for (const auto &hp : hv.dynamic_path)
            {
                // 기존 로직: tip 기준 앞/뒤
                for (size_t i = behind_tip_idx + 1; i < cav.dynamic_path.size(); ++i)
                {
                    double d = hp.dist_to(cav.dynamic_path[i]);
                    if (d < min_dist_to_tip_zone)
                        min_dist_to_tip_zone = d;
                }
                for (size_t i = 0; i <= behind_tip_idx; ++i)
                {
                    double d = hp.dist_to(cav.dynamic_path[i]);
                    if (d < min_dist_to_behind_zone)
                        min_dist_to_behind_zone = d;
                }

                // ★★★ 새로운 로직: 전방 마진 기준 앞/뒤 ★★★
                // front_margin_idx 이후 = 차량 앞쪽 (CAV 중심 + 0.17m 이후)
                for (size_t i = front_margin_idx; i < cav.dynamic_path.size(); ++i)
                {
                    double d = hp.dist_to(cav.dynamic_path[i]);
                    if (d < min_dist_to_front_of_margin)
                        min_dist_to_front_of_margin = d;
                }
                // front_margin_idx 이전 = 차량 뒤쪽 (CAV 중심 + 0.17m 이전)
                for (size_t i = 0; i < front_margin_idx; ++i)
                {
                    double d = hp.dist_to(cav.dynamic_path[i]);
                    if (d < min_dist_to_rear_of_margin)
                        min_dist_to_rear_of_margin = d;
                }
            }

            bool hv_near_tip = (min_dist_to_tip_zone < HV_OVERLAP_DIST);
            bool hv_near_behind = (min_dist_to_behind_zone < HV_OVERLAP_DIST);
            bool hv_from_behind = hv_near_behind && (min_dist_to_behind_zone <= min_dist_to_tip_zone);

            // ★★★ 새로운 판단: HV가 CAV 전방마진 기준 뒤쪽에 먼저 닿는지 ★★★
            // HV 로컬패스가 CAV 중심+0.17m 지점보다 뒤쪽에 먼저 닿으면 CAV는 계속 주행
            bool hv_behind_front_margin = (min_dist_to_rear_of_margin < HV_OVERLAP_DIST) && 
                                          (min_dist_to_rear_of_margin < min_dist_to_front_of_margin);

            bool passed_entry = cav.passed_entry;

            if (!passed_entry)
            {
                // ★★★ 수정: hv_behind_front_margin이면 HV가 뒤에서 오는 것이므로 무시 ★★★
                if (hv_behind_front_margin)
                {
                    hv_debug_info_[cav_id] = "HV" + std::to_string(hv_id) + " behind(margin)→GO";
                    continue;  // 이 HV는 뒤에 있으므로 무시하고 다음 HV 검사
                }

                if (hv_near_tip && !hv_from_behind)
                {
                    hv_debug_info_[cav_id] = "HV" + std::to_string(hv_id) + " wait(pre)";
                    cav.hv_danger = true;
                    return {true, hv_id, false};
                }
            }
            else
            {
                // ★★★ 수정: hv_behind_front_margin이면 HV가 뒤에서 오는 것이므로 무시 ★★★
                if (hv_behind_front_margin)
                {
                    hv_debug_info_[cav_id] = "HV" + std::to_string(hv_id) + " behind(margin)→GO";
                    continue;  // 이 HV는 뒤에 있으므로 무시하고 다음 HV 검사
                }

                if (hv_from_behind)
                    continue;
                if (dist_to_hv_rear < HV_OVERLAP_DIST)
                {
                    hv_debug_info_[cav_id] = "HV" + std::to_string(hv_id) + " follow";
                    cav.hv_danger = true;
                    return {false, hv_id, true};
                }
                if (dist_to_hv_center < HV_OVERLAP_DIST)
                {
                    hv_debug_info_[cav_id] = "HV" + std::to_string(hv_id) + " center";
                    cav.hv_danger = true;
                    return {true, hv_id, false};
                }
                if (hv_near_tip)
                {
                    hv_debug_info_[cav_id] = "HV" + std::to_string(hv_id) + " path";
                    cav.hv_danger = true;
                    return {true, hv_id, false};
                }
            }
        }
        hv_debug_info_[cav_id] = "GO";
        cav.hv_danger = false;
        return {false, -1, false};
    }

    int CollisionAvoidanceNode::determine_priority(int my_id, int other_id,
                                                   const Point2D &zone, int my_idx, int other_idx) const
    {
        const auto &my = cav_states_.at(my_id);
        const auto &other = cav_states_.at(other_id);

        double my_dist_to_zone = my.position.dist_to(zone);
        double other_dist_to_zone = other.position.dist_to(zone);

        int my_rear_size = (int)my.rear_path.size();
        int other_rear_size = (int)other.rear_path.size();
        bool my_passed_zone = (my_idx < my_rear_size);
        bool other_passed_zone = (other_idx < other_rear_size);

        // 기존 로직: 한 쪽이 이미 지나갔으면 그쪽 우선
        if (my_passed_zone && !other_passed_zone)
            return my_id;
        if (other_passed_zone && !my_passed_zone)
            return other_id;

        // ===== 새로 추가: 둘 다 대기 중일 때 시간 기반 판단 =====
        if (!my_passed_zone && !other_passed_zone)
        {
            // 도착 시간 예측 (거리 / 속도)
            double my_time = my_dist_to_zone / std::max(my.speed, 0.1);
            double other_time = other_dist_to_zone / std::max(other.speed, 0.1);

            const double TIME_THRESHOLD = 1.0; // 1초 차이면 확실히 구분
            if (my_time < other_time - TIME_THRESHOLD)
                return my_id;
            if (other_time < my_time - TIME_THRESHOLD)
                return other_id;
        }
        // ===== 추가 끝 =====

        // 기존 거리 기반 로직
        const double DIST_THRESHOLD = 0.05;
        if (my_dist_to_zone < other_dist_to_zone - DIST_THRESHOLD)
            return my_id;
        if (other_dist_to_zone < my_dist_to_zone - DIST_THRESHOLD)
            return other_id;

        // 기존 그룹/웨이포인트 기반 로직
        int my_group = is_high_priority(my_id) ? 1 : 2;
        int other_group = is_high_priority(other_id) ? 1 : 2;

        if (my_group < other_group)
            return my_id;
        if (other_group < my_group)
            return other_id;
        if (my.remaining_waypoints > other.remaining_waypoints)
            return my_id;
        if (other.remaining_waypoints > my.remaining_waypoints)
            return other_id;

        return (my_id < other_id) ? my_id : other_id;
    }

    std::tuple<bool, int, bool> CollisionAvoidanceNode::check_cav_collision(int cav_id)
    {
        auto &my = cav_states_[cav_id];
        if (my.full_safety_path.size() < 2)
            return {false, -1, false};

        // 충돌 정보 구조체
        struct ConflictInfo
        {
            int other_id;
            Point2D zone;
            int my_idx;
            int other_idx;
        };

        std::vector<ConflictInfo> conflicts;

        // 1단계: 모든 잠재적 충돌 수집
        for (int other_id : cav_ids_)
        {
            if (other_id == cav_id)
                continue;
            auto &other = cav_states_[other_id];
            if (other.full_safety_path.size() < 2)
                continue;

            auto [overlaps, zone, my_idx, other_idx] =
                find_path_overlap(my.full_safety_path, other.full_safety_path);

            if (!overlaps)
                continue;

            // passed zone 체크
            int my_rear_size = (int)my.rear_path.size();
            int other_rear_size = (int)other.rear_path.size();
            bool my_passed = (my_idx < my_rear_size);
            bool other_passed = (other_idx < other_rear_size);

            // 내가 이미 지나간 충돌은 무시
            if (my_passed)
                continue;

            // 상대가 이미 지나간 충돌은 무시
            if (other_passed)
                continue;

            // 둘 다 아직 도달하지 않은 충돌 → 추가
            conflicts.push_back({other_id, zone, my_idx, other_idx});
        }

        // 충돌이 없으면 통과
        if (conflicts.empty())
        {
            return {false, -1, false};
        }

        // 2단계: 모든 충돌 차량 중 최고 우선순위 결정
        int highest_priority_cav = cav_id;
        Point2D critical_zone = conflicts[0].zone;
        int critical_my_idx = conflicts[0].my_idx;
        int critical_other_idx = conflicts[0].other_idx;

        for (const auto &conflict : conflicts)
        {
            int my_comparison_idx = (highest_priority_cav == cav_id) ? conflict.my_idx : critical_other_idx;
            int other_comparison_idx = (highest_priority_cav == cav_id) ? conflict.other_idx : critical_my_idx;

            int priority = determine_priority(
                highest_priority_cav,
                conflict.other_id,
                conflict.zone,
                my_comparison_idx,
                other_comparison_idx);

            if (priority != highest_priority_cav)
            {
                highest_priority_cav = conflict.other_id;
                critical_zone = conflict.zone;
                critical_my_idx = conflict.my_idx;
                critical_other_idx = conflict.other_idx;
            }
        }

        // 3단계: 최종 판정
        if (highest_priority_cav != cav_id)
        {
            active_collision_zones_ = {critical_zone};
            return {true, highest_priority_cav, false};
        }

        return {false, -1, false};
    }

    double CollisionAvoidanceNode::smooth_velocity(int cav_id, double target)
    {
        auto &state = cav_states_[cav_id];
        double smoothed = compute_sigmoid_vel(cav_id, state.current_velocity, target, 0.02);
        state.current_velocity = smoothed;
        return smoothed;
    }

    void CollisionAvoidanceNode::control_loop()
    {
        for (int cav_id : cav_ids_)
        {
            auto &state = cav_states_[cav_id];

            // 이하 일반 충돌 회피 로직
            bool should_stop = false, should_slow = false;
            std::string reason;

            auto [hv_stop, hv_id, hv_slow] = check_hv_collision(cav_id);
            if (hv_stop)
            {
                should_stop = true;
                reason = "HV" + std::to_string(hv_id);
            }
            else if (hv_slow)
            {
                should_slow = true;
                reason = "HV" + std::to_string(hv_id) + " follow";
            }

            if (!should_stop)
            {
                auto [cav_stop, other_id, cav_slow] = check_cav_collision(cav_id);
                if (cav_stop)
                {
                    should_stop = true;
                    reason = "CAV" + format_cav_id(other_id);
                }
                else if (cav_slow && !should_slow)
                {
                    should_slow = true;
                }
            }

            state.stop_flag = should_stop;
            state.slow_flag = should_slow;

            geometry_msgs::msg::Accel msg;
            auto &raw = last_accel_[cav_id];

            double target_v;
            if (should_stop)
            {
                target_v = 0.0;
            }
            else if (should_slow)
            {
                double hv_speed = 0.3;
                for (int hid : hv_ids_)
                {
                    if (hv_states_[hid].valid)
                    {
                        double d = state.position.dist_to(hv_states_[hid].position);
                        if (d < 0.5)
                        {
                            hv_speed = std::max(hv_states_[hid].speed, 0.2);
                            break;
                        }
                    }
                }
                target_v = std::min(hv_speed, raw.linear.x);
            }
            else
            {
                target_v = raw.linear.x;
            }

            double smooth_v = smooth_velocity(cav_id, target_v);
            msg.linear.x = smooth_v;
            msg.angular.z = (smooth_v > 0.05) ? raw.angular.z : 0.0;

            if (should_stop)
            {
                if (smooth_v > 0.15)
                {
                    msg.linear.x = smooth_v;
                    msg.angular.z = 0.0;
                    state.correction_done = false;
                }
                else if (smooth_v > 0.02 && !state.correction_done)
                {
                    double brake_force = -0.15 * (smooth_v / 0.15);
                    if (state.correction_start_time == 0.0)
                        state.correction_start_time = get_current_time();
                    double elapsed = get_current_time() - state.correction_start_time;

                    if (elapsed < 0.03)
                    {
                        msg.linear.x = brake_force;
                        msg.angular.z = 0.0;
                    }
                    else
                    {
                        msg.linear.x = 0.0;
                        msg.angular.z = 0.0;
                        state.correction_done = true;
                    }
                }
                else
                {
                    msg.linear.x = 0.0;
                    msg.angular.z = 0.0;
                    state.correction_done = true;
                }
            }
            else if (!should_stop)
            {
                state.correction_done = false;
                state.correction_start_time = 0.0;
            }

            accel_pubs_[cav_id]->publish(msg);
        }
    }

    void CollisionAvoidanceNode::publish_visualizations()
    {
        publish_hv_markers();
        publish_cav_markers();
        publish_zone_markers();
        publish_safety_paths();
        publish_hv_local_paths();
    }

    void CollisionAvoidanceNode::publish_hv_local_paths()
    {
        for (int hv_id : hv_ids_)
        {
            const auto &hv = hv_states_[hv_id];
            nav_msgs::msg::Path msg;
            msg.header.frame_id = "world";
            msg.header.stamp = this->now();
            for (const auto &p : hv.dynamic_path)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = msg.header;
                pose.pose.position.x = p.x;
                pose.pose.position.y = p.y;
                pose.pose.orientation.w = 1.0;
                msg.poses.push_back(pose);
            }
            hv_local_path_pubs_[hv_id]->publish(msg);
        }
    }

    void CollisionAvoidanceNode::publish_hv_markers()
    {
        visualization_msgs::msg::MarkerArray markers;
        int idx = 0;
        for (int hv_id : hv_ids_)
        {
            const auto &s = hv_states_[hv_id];
            if (!s.valid)
                continue;

            visualization_msgs::msg::Marker rear;
            rear.header.frame_id = "world";
            rear.header.stamp = this->now();
            rear.ns = "hv_rear";
            rear.id = idx++;
            rear.type = visualization_msgs::msg::Marker::SPHERE;
            rear.action = visualization_msgs::msg::Marker::ADD;
            rear.pose.position.x = s.rear_point.x;
            rear.pose.position.y = s.rear_point.y;
            rear.pose.position.z = 0.08;
            rear.pose.orientation.w = 1.0;
            rear.scale.x = 0.08;
            rear.scale.y = 0.08;
            rear.scale.z = 0.08;
            rear.color.r = 1.0;
            rear.color.g = 0.0;
            rear.color.b = 0.0;
            rear.color.a = 1.0;
            markers.markers.push_back(rear);

            visualization_msgs::msg::Marker center;
            center.header.frame_id = "world";
            center.header.stamp = this->now();
            center.ns = "hv_center";
            center.id = idx++;
            center.type = visualization_msgs::msg::Marker::SPHERE;
            center.action = visualization_msgs::msg::Marker::ADD;
            center.pose.position.x = s.position.x;
            center.pose.position.y = s.position.y;
            center.pose.position.z = 0.08;
            center.pose.orientation.w = 1.0;
            center.scale.x = 0.05;
            center.scale.y = 0.05;
            center.scale.z = 0.05;
            center.color.r = 1.0;
            center.color.g = 1.0;
            center.color.b = 1.0;
            center.color.a = 0.8;
            markers.markers.push_back(center);
        }
        hv_marker_pub_->publish(markers);
    }

    void CollisionAvoidanceNode::publish_cav_markers()
    {
        visualization_msgs::msg::MarkerArray markers;
        for (int cav_id : cav_ids_)
        {
            const auto &s = cav_states_[cav_id];
            visualization_msgs::msg::Marker l;
            l.header.frame_id = "world";
            l.header.stamp = this->now();
            l.ns = "cav_label";
            l.id = cav_id;
            l.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            l.action = visualization_msgs::msg::Marker::ADD;
            l.pose.position.x = s.position.x;
            l.pose.position.y = s.position.y;
            l.pose.position.z = 0.35;
            l.scale.z = 0.08;
            std::string st = s.stop_flag ? "STOP" : (s.slow_flag ? "SLOW" : "GO");
            std::string grp = is_high_priority(cav_id) ? "H" : "L";
            std::ostringstream oss;
            oss << "C" << cav_id << "[" << grp << "] " << st;
            if (s.hv_danger)
                oss << " HV!";
            l.text = oss.str();
            l.color.r = s.stop_flag ? 1.0 : (s.slow_flag ? 1.0 : 0.3);
            l.color.g = s.stop_flag ? 0.3 : (s.slow_flag ? 0.8 : 1.0);
            l.color.b = 0.3;
            l.color.a = 1.0;
            markers.markers.push_back(l);
        }
        cav_marker_pub_->publish(markers);
    }

    void CollisionAvoidanceNode::publish_zone_markers()
    {
        visualization_msgs::msg::MarkerArray markers;
        int idx = 0;
        for (const auto &zone : active_collision_zones_)
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "world";
            m.header.stamp = this->now();
            m.ns = "collision_zone";
            m.id = idx++;
            m.type = visualization_msgs::msg::Marker::CYLINDER;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = zone.x;
            m.pose.position.y = zone.y;
            m.pose.position.z = 0.01;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.20;
            m.scale.y = 0.20;
            m.scale.z = 0.02;
            m.color.r = 1.0;
            m.color.g = 0.5;
            m.color.a = 0.6;
            m.lifetime = rclcpp::Duration::from_seconds(0.2);
            markers.markers.push_back(m);
        }
        zone_marker_pub_->publish(markers);
    }

    void CollisionAvoidanceNode::publish_safety_paths()
    {
        visualization_msgs::msg::MarkerArray markers;
        int idx = 0;

        for (int hv_id : hv_ids_)
        {
            const auto &hv = hv_states_[hv_id];
            if (!hv.valid || hv.dynamic_path.size() < 2)
                continue;
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "world";
            m.header.stamp = this->now();
            m.ns = "hv_local_path";
            m.id = idx++;
            m.type = visualization_msgs::msg::Marker::LINE_STRIP;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.scale.x = 0.03;
            m.color.r = 0.6;
            m.color.g = 0.6;
            m.color.b = 0.6;
            m.color.a = 0.9;
            m.lifetime = rclcpp::Duration::from_seconds(0.1);
            for (const auto &p : hv.dynamic_path)
            {
                geometry_msgs::msg::Point pt;
                pt.x = p.x;
                pt.y = p.y;
                pt.z = 0.02;
                m.points.push_back(pt);
            }
            markers.markers.push_back(m);
        }

        for (int cav_id : cav_ids_)
        {
            const auto &cav = cav_states_[cav_id];
            const auto &c = cav_colors_[cav_id];

            if (cav.full_safety_path.size() >= 2)
            {
                visualization_msgs::msg::Marker m;
                m.header.frame_id = "world";
                m.header.stamp = this->now();
                m.ns = "cav_safety_path";
                m.id = idx++;
                m.type = visualization_msgs::msg::Marker::LINE_STRIP;
                m.action = visualization_msgs::msg::Marker::ADD;
                m.scale.x = 0.05;
                m.color.r = c[0];
                m.color.g = c[1];
                m.color.b = c[2];
                m.color.a = 0.8;
                m.lifetime = rclcpp::Duration::from_seconds(0.1);
                for (const auto &p : cav.full_safety_path)
                {
                    geometry_msgs::msg::Point pt;
                    pt.x = p.x;
                    pt.y = p.y;
                    pt.z = 0.03;
                    m.points.push_back(pt);
                }
                markers.markers.push_back(m);
            }

            if (!cav.dynamic_path.empty())
            {
                visualization_msgs::msg::Marker tip;
                tip.header.frame_id = "world";
                tip.header.stamp = this->now();
                tip.ns = "cav_tip";
                tip.id = idx++;
                tip.type = visualization_msgs::msg::Marker::SPHERE;
                tip.action = visualization_msgs::msg::Marker::ADD;
                const auto &end_pt = cav.dynamic_path.back();
                tip.pose.position.x = end_pt.x;
                tip.pose.position.y = end_pt.y;
                tip.pose.position.z = 0.05;
                tip.pose.orientation.w = 1.0;
                tip.scale.x = 0.06;
                tip.scale.y = 0.06;
                tip.scale.z = 0.06;
                tip.color.r = c[0];
                tip.color.g = c[1];
                tip.color.b = c[2];
                tip.color.a = 1.0;
                tip.lifetime = rclcpp::Duration::from_seconds(0.1);
                markers.markers.push_back(tip);
            }
        }
        safety_path_marker_pub_->publish(markers);
    }

    void CollisionAvoidanceNode::log_hv_debug()
    {
        for (int hv_id : hv_ids_)
        {
            const auto &hv = hv_states_[hv_id];
            if (hv.valid)
            {
                // debug
            }
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bisa::CollisionAvoidanceNode>());
    rclcpp::shutdown();
    return 0;
}
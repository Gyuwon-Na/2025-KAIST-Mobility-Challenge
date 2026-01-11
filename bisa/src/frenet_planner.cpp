/**
 * @file frenet_planner.cpp
 * @brief Complete Overtake Planner for RC-Scale Autonomous Vehicle (v5.0)
 *
 * Key Features:
 * - Rectangular collision detection (Front/Rear/Left/Right)
 * - Debug markers match detection zones exactly
 * - Strong lane center keeping
 * - Safe overtaking with blind spot check
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
    EMERGENCY
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
    // Front zone
    bool front_blocked = false;
    bool front_danger = false;
    bool front_critical = false;
    double front_dist = 999.0;
    double front_speed = 0.0;
    bool front_is_static = false;
    int front_id = -1;

    // Rear zone
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
        RCLCPP_INFO(this->get_logger(), "Frenet Planner v5.0 - Complete Overtake");
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
    static constexpr double VEHICLE_LENGTH_F = 0.17; // Front from center
    static constexpr double VEHICLE_LENGTH_R = 0.16; // Rear from center
    static constexpr double VEHICLE_HALF_WIDTH = 0.075;

    // Obstacle dimensions (approximate)
    static constexpr double OBS_LENGTH = 0.33;
    static constexpr double OBS_WIDTH = 0.15;
    static constexpr double OBS_HALF_LENGTH = 0.17;
    static constexpr double OBS_HALF_WIDTH = 0.075;

    // ========================================================================
    // DETECTION ZONES (Rectangular - matches debug markers)
    // ========================================================================

    // Front zone: rectangle ahead of vehicle
    static constexpr double FRONT_ZONE_LENGTH = 0.5; // How far ahead to check
    static constexpr double FRONT_ZONE_WIDTH = 0.20; // Half-width of front zone

    // Danger/Critical thresholds within front zone
    static constexpr double FRONT_SAFE_DIST = 0.45;
    static constexpr double FRONT_DANGER_DIST = 0.30;
    static constexpr double FRONT_CRITICAL_DIST = 0.18;

    // Rear zone
    static constexpr double REAR_ZONE_LENGTH = 0.5;
    static constexpr double REAR_ZONE_WIDTH = 0.20;
    static constexpr double REAR_DANGER_DIST = 0.35;

    // Side zones (blind spots)
    static constexpr double SIDE_ZONE_LENGTH = 1.0; // From rear to front
    static constexpr double SIDE_ZONE_START = -0.5; // Start behind center
    static constexpr double SIDE_ZONE_END = 0.5;    // End ahead of center
    static constexpr double SIDE_ZONE_INNER = 0.08; // Inner boundary
    static constexpr double SIDE_ZONE_OUTER = 0.4;  // Outer boundary

    // Lane
    static constexpr double LANE_WIDTH = 0.35;
    static constexpr double LANE_THRESHOLD = 0.30;

    // Static detection
    static constexpr double STATIC_SPEED_THRESHOLD = 0.05;

    // ========================================================================
    // PARAMETERS
    // ========================================================================

    double wheelbase_ = 0.33;
    double max_steer_ = 0.7;
    double target_speed_ = 0.4;
    double overtake_speed_ = 0.5;
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
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr car_pub_;

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
        case DrivingState::EMERGENCY:
            return "EMERGENCY";
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
        this->declare_parameter("target_speed", 0.4);
        this->declare_parameter("overtake_speed", 0.5);
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
        car_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/car_marker", 10);
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
        publish_car_marker();
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

            // Body frame: x forward, y left
            obs.rel_x = dx * cos_yaw + dy * sin_yaw;
            obs.rel_y = -dx * sin_yaw + dy * cos_yaw;
            obs.rel_vx = (obs.vx * cos_yaw + obs.vy * sin_yaw) - ego_speed_;
        }
    }

    // ========================================================================
    // GEOMETRY HELPER (신규 추가)
    // ========================================================================

    // 목표 차선이 내 차 기준으로 왼쪽에 있는가? (True=Left, False=Right)
    bool is_target_left(LaneID target_lane)
    {
        // 1. 목표 차선의 현재 내 위치 근처 좌표를 가져옴
        int t_idx = lane_to_int(target_lane);
        if (!lane_paths_[t_idx] || lane_paths_[t_idx]->poses.empty())
            return false; // 데이터 없으면 기본값

        // 내 차와 가장 가까운 목표 차선상의 점 찾기 (Closest Index)
        int closest = find_closest_idx(t_idx, ego_x_, ego_y_);
        double tx = lane_paths_[t_idx]->poses[closest].pose.position.x;
        double ty = lane_paths_[t_idx]->poses[closest].pose.position.y;

        // 2. 내 차에서 목표 지점까지의 벡터
        double dx = tx - ego_x_;
        double dy = ty - ego_y_;

        // 3. 내 차의 헤딩 벡터 (cos, sin)
        double head_x = std::cos(ego_yaw_);
        double head_y = std::sin(ego_yaw_);

        // 4. 2D 외적 (Cross Product) 계산: (Head_x * dy) - (Head_y * dx)
        // 결과가 양수(+)면 목표가 왼쪽, 음수(-)면 오른쪽
        double cross_product = head_x * dy - head_y * dx;

        return cross_product > 0.0;
    }

    // ========================================================================
    // RECTANGULAR COLLISION DETECTION
    // ========================================================================

    // ========================================================================
    // COLLISION DETECTION (수정됨: 코너링 대응 광각 감지 추가)
    // ========================================================================

    SurroundingStatus check_all_zones()
    {
        SurroundingStatus status;
        double current_sec = now_sec();

        // [설정] 코너링 안전 파라미터
        // 내 차 반경 1.2m 이내, 전방 120도(좌우 60도) 부채꼴 영역 감시
        const double PROXIMITY_DIST = 1.2;
        const double PROXIMITY_ANGLE = 60.0 * M_PI / 180.0; // +/- 60도

        for (const auto &[id, obs] : obstacles_)
        {
            if ((current_sec - obs.last_seen_sec) > 0.5)
                continue;

            double rx = obs.rel_x; // 내 차 기준 앞쪽 거리
            double ry = obs.rel_y; // 내 차 기준 왼쪽 거리

            // 장애물까지의 직선 거리 및 각도 계산
            double dist = std::hypot(rx, ry);
            double angle = std::atan2(ry, rx); // 내 차 헤딩 기준 장애물 각도

            // ============================================================
            // 1. [신규] 광각 근접 감지 (Wide-Angle Check)
            // 직사각형 박스가 아니라, 내 차 앞 '부채꼴' 영역을 감시합니다.
            // 코너를 돌 때 측면 전방(A필러 방향)에 있는 장애물을 잡습니다.
            // ============================================================
            bool is_wide_threat = false;

            // 거리가 가깝고 && 각도가 전방 시야각 안에 있다면
            if (dist < PROXIMITY_DIST && std::abs(angle) < PROXIMITY_ANGLE)
            {
                // 단, 내 차 뒤쪽(-90~90도 범위를 벗어난)은 제외해야 함
                // atan2 결과가 전방(-60~+60)이므로 이미 뒤쪽은 제외됨.
                is_wide_threat = true;

                // 로그 출력 (디버깅용)
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                      "Corner Threat! ID=%d, Dist=%.2f, Ang=%.1f deg",
                                      id, dist, angle * 180.0 / M_PI);
            }

            // ============================================================
            // 2. FRONT ZONE (기존 직사각형 박스)
            // ============================================================
            bool is_box_threat = false;
            if (rx > -OBS_HALF_LENGTH && rx < FRONT_ZONE_LENGTH + OBS_HALF_LENGTH)
            {
                if (std::abs(ry) < FRONT_ZONE_WIDTH + OBS_HALF_WIDTH)
                {
                    is_box_threat = true;
                }
            }

            // ============================================================
            // 3. 통합 위협 판정
            // ============================================================
            if (is_box_threat || is_wide_threat)
            {
                // 실제 물리적 거리(dist)에서 차량/장애물 크기 보정
                double effective_dist = std::max(0.0, dist - OBS_HALF_LENGTH - VEHICLE_LENGTH_F);

                // 광각 감지로 잡힌 경우, 거리를 좀 더 보수적으로(가깝게) 인식시킴
                if (is_wide_threat && !is_box_threat)
                {
                    effective_dist *= 0.8; // 위험도 가중치
                }

                if (effective_dist < status.front_dist)
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

            // ============================================================
            // REAR ZONE (기존 유지)
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
                        // TTC check
                        double closing_speed = -obs.rel_vx;
                        if (closing_speed > 0.05)
                        {
                            status.rear_ttc = effective_dist / closing_speed;
                            if (status.rear_ttc < 2.0)
                                status.rear_danger = true;
                        }
                    }
                }
            }

            // ============================================================
            // SIDE ZONES (기존 유지 - 지난번 수정된 파라미터 적용됨)
            // ============================================================
            // Left Blind Spot
            if (rx > SIDE_ZONE_START - OBS_HALF_LENGTH && rx < SIDE_ZONE_END + OBS_HALF_LENGTH)
            {
                if (ry > SIDE_ZONE_INNER - OBS_HALF_WIDTH && ry < SIDE_ZONE_OUTER + OBS_HALF_WIDTH)
                {
                    status.left_clear = false;
                    status.left_dist = std::min(status.left_dist, ry - OBS_HALF_WIDTH);
                }
            }
            // Right Blind Spot
            if (rx > SIDE_ZONE_START - OBS_HALF_LENGTH && rx < SIDE_ZONE_END + OBS_HALF_LENGTH)
            {
                if (ry < -(SIDE_ZONE_INNER - OBS_HALF_WIDTH) && ry > -(SIDE_ZONE_OUTER + OBS_HALF_WIDTH))
                {
                    status.right_clear = false;
                    status.right_dist = std::min(status.right_dist, -ry - OBS_HALF_WIDTH);
                }
            }
        }

        return status;
    }
    // ========================================================================
    // LANE CHANGE SAFETY CHECK
    // ========================================================================

    // ========================================================================
    // LANE CHANGE SAFETY CHECK (수정됨: 주행 방향 무관하게 작동)
    // ========================================================================

    bool is_overtake_safe(LaneID target)
    {
        // [수정] ID 비교 대신 기하학적 위치로 방향 판별
        bool going_left = is_target_left(target);

        // 디버깅용 로그 (방향이 헷갈릴 때 주석 해제하여 확인)
        // RCLCPP_INFO(this->get_logger(), "Checking %s -> %s | Direction: %s",
        //             lane_str(current_lane_).c_str(), lane_str(target).c_str(),
        //             going_left ? "LEFT" : "RIGHT");

        // Check blind spot on the relevant side
        if (going_left && !current_surrounding_.left_clear)
        {
            RCLCPP_DEBUG(this->get_logger(), "Left blind spot blocked");
            return false;
        }
        if (!going_left && !current_surrounding_.right_clear)
        {
            RCLCPP_DEBUG(this->get_logger(), "Right blind spot blocked");
            return false;
        }

        // Check target lane status (기존 코드 유지)
        for (const auto &[id, obs] : obstacles_)
        {
            if (obs.lane != target)
                continue;

            double rx = obs.rel_x;

            // Front obstacle in target lane
            if (rx > -OBS_HALF_LENGTH && rx < FRONT_ZONE_LENGTH)
            {
                double dist = rx - OBS_HALF_LENGTH;
                if (dist < FRONT_CRITICAL_DIST)
                    return false;
            }

            // Rear approaching fast in target lane
            if (rx < 0 && rx > -REAR_ZONE_LENGTH)
            {
                double closing = -obs.rel_vx;
                if (closing > 0.15)
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

    // ========================================================================
    // STATIC OBSTACLE LONG-RANGE CHECK (신규 추가)
    // ========================================================================

    // 특정 차선에 정지 장애물(Static)이 멀리라도 있는지 확인 (3.0m)
    bool is_lane_blocked_by_static(LaneID lane)
    {
        int target_lane_idx = lane_to_int(lane);

        // 정지 장애물 감지 거리 (안전 거리 0.6m보다 훨씬 길게 설정)
        const double STATIC_BLOCK_DIST = 3.0;

        for (const auto &[id, obs] : obstacles_)
        {
            // 1. 해당 차선에 있는 장애물만 확인
            if (lane_to_int(obs.lane) != target_lane_idx)
                continue;

            // 2. 정지 장애물(Static)인지 확인
            if (!obs.is_static)
                continue;

            // 3. 거리 및 방향 확인 (Body Frame 기준)
            double dx = obs.x - ego_x_;
            double dy = obs.y - ego_y_;

            // 내 차 진행 방향(Heading) 기준 앞쪽 거리 계산 (Dot Product)
            double forward = dx * std::cos(ego_yaw_) + dy * std::sin(ego_yaw_);

            // 내 차 뒤쪽(-0.5m)은 무시하고, 앞쪽 3.0m 이내에 있으면 '봉쇄'로 판단
            if (forward > -0.5 && forward < STATIC_BLOCK_DIST)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "[BLOCK] Lane %s blocked by STATIC obs %d at %.2fm",
                                     lane_str(lane).c_str(), id, forward);
                return true;
            }
        }
        return false;
    }

    // ========================================================================
    // OVERTAKE LANE SELECTION (수정됨: Static 차량 있는 차선 원천 봉쇄)
    // ========================================================================

    LaneID choose_overtake_lane()
    {
        int current_idx = lane_to_int(current_lane_);

        // Priority: LANE_3 > LANE_2 > LANE_1
        // (바깥쪽 차선을 선호하도록 후보 순서 배치)
        std::vector<int> candidates;

        if (current_idx == 0) // L1 -> L2, L3
            candidates = {1, 2};
        else if (current_idx == 1) // L2 -> L3, L1
            candidates = {2, 0};
        else // L3 -> L2
            candidates = {1};

        for (int cand : candidates)
        {
            LaneID target = int_to_lane(cand);

            // [핵심 수정] 1. 정지 장애물(Static)이 있는 차선인가?
            // 3.0m 앞에라도 정지 차량이 있다면, 거기로 들어가면 갇히므로 아예 배제함.
            if (is_lane_blocked_by_static(target))
            {
                continue;
            }

            // 2. 일반적인 안전 체크 (움직이는 차, 사각지대 등)
            if (is_overtake_safe(target))
            {
                return target;
            }
        }

        // 갈 곳이 없으면 현재 차선 유지
        return current_lane_;
    }
    // ========================================================================
    // STATE MACHINE
    // ========================================================================

    // ========================================================================
    // STATE MACHINE (수정됨: 정지 장애물 즉시 회피)
    // ========================================================================

    void update_state_machine()
    {
        double current_sec = now_sec();
        const auto &s = current_surrounding_;

        // (로그 출력 부분은 생략 - 기존 유지)

        switch (current_state_)
        {
        case DrivingState::CRUISE:
        {
            if (s.front_blocked || s.front_danger)
            {
                LaneID overtake = choose_overtake_lane();
                if (overtake != current_lane_)
                {
                    target_lane_ = overtake;
                    original_lane_ = current_lane_;

                    // [수정] 정지 장애물(Static)이면 기다리지 않고 즉시 차선 변경 시작!
                    if (s.front_is_static)
                    {
                        current_state_ = DrivingState::LANE_CHANGING;
                        lane_change_start_sec_ = current_sec;
                        RCLCPP_WARN(this->get_logger(), "STATIC OBSTACLE! Immediate Overtake: %s -> %s",
                                    lane_str(current_lane_).c_str(), lane_str(target_lane_).c_str());
                    }
                    else
                    {
                        // 움직이는 차라면 안전 확인 절차 거침
                        current_state_ = DrivingState::PREPARE_OVERTAKE;
                        safe_check_count_ = 0;
                    }
                }
                else if (s.front_critical)
                {
                    current_state_ = DrivingState::EMERGENCY;
                }
            }
            break;
        }

        case DrivingState::PREPARE_OVERTAKE:
        {
            // (기존 로직 유지, 단 정지 장애물인 경우 위에서 바로 LANE_CHANGING으로 넘어감)
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
            // 차선 변경 완료 조건
            if (current_lane_ == target_lane_)
            {
                double elapsed = current_sec - lane_change_start_sec_;
                // 정지 장애물 회피 시에는 조금 더 빨리 완료 처리해도 됨
                if (elapsed > 0.3)
                {
                    current_state_ = DrivingState::CRUISE;
                    RCLCPP_INFO(this->get_logger(), "Lane change complete");
                }
            }
            // 타임아웃
            if ((current_sec - lane_change_start_sec_) > 5.0) // 4.0 -> 5.0 여유 증가
            {
                current_state_ = DrivingState::CRUISE;
                target_lane_ = current_lane_;
            }
            break;
        }

        case DrivingState::EMERGENCY:
        {
            LaneID escape = choose_overtake_lane();
            if (escape != current_lane_ && is_overtake_safe(escape))
            {
                target_lane_ = escape;
                current_state_ = DrivingState::LANE_CHANGING;
                lane_change_start_sec_ = current_sec;
                RCLCPP_WARN(this->get_logger(), "Emergency Escape! -> %s", lane_str(escape).c_str());
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

    double compute_steering(int lane_idx, int closest, int lookahead)
    {
        if (lane_idx < 0 || lane_idx > 2)
            return 0.0;
        if (!lane_paths_[lane_idx] || lane_paths_[lane_idx]->poses.empty())
            return 0.0;

        const auto &poses = lane_paths_[lane_idx]->poses;
        int n = poses.size();

        // 1. Target Point & Pure Pursuit Info
        double tx = poses[lookahead].pose.position.x;
        double ty = poses[lookahead].pose.position.y;
        double dx = tx - ego_x_;
        double dy = ty - ego_y_;
        double dist = std::hypot(dx, dy);
        double target_yaw = std::atan2(dy, dx); // Lookahead 점을 향하는 각도

        // 2. Road Direction (Lookahead 점에서의 도로 접선 방향)
        // v2 로직 참고: 조금 더 먼 미래의 점을 잡아 도로 흐름을 파악
        int next = (lookahead + 5) % n;
        double road_yaw = std::atan2(
            poses[next].pose.position.y - ty,
            poses[next].pose.position.x - tx);

        // 3. Errors
        double heading_err = normalize_angle(target_yaw - ego_yaw_); // 점을 향하는 각도 오차
        double road_err = normalize_angle(road_yaw - ego_yaw_);      // 도로 방향과의 정렬 오차

        // 4. Cross-track Error (CTE)
        double cte = compute_cross_track_error(lane_idx, closest);

        // 5. Curvature (Lookahead 기준 곡률)
        int preview = (closest + 20) % n;
        double curv = compute_curvature(lane_idx, preview, 10);
        bool is_corner = std::abs(curv) > 0.4;

        // 6. Steering Calculation
        double steer = 0.0;

        // Pure Pursuit Component
        double steer_pp = std::atan2(2.0 * wheelbase_ * std::sin(heading_err),
                                     std::max(dist, 0.1));

        // CTE Correction Gain Setting
        double k_cte = 4.0;
        if (current_state_ == DrivingState::LANE_CHANGING)
        {
            k_cte = 8.0;
        }
        else if (is_corner)
        {
            k_cte = 8.0; // [수정] 코너에서도 6.0 -> 8.0으로 CTE 보정 강화 (밀림 방지)
        }

        double cte_steer = std::atan2(k_cte * cte, std::max(ego_speed_, 0.5));
        cte_steer = std::clamp(cte_steer, -0.6, 0.6);

        if (is_corner)
        {
            // [수정: 코너링 로직 강화]
            // v2: road_err * 0.9 사용
            // v5(New): road_err * 1.2로 강화하여 차선과 평행을 맞추려는 힘을 극대화
            // steer_pp 비중을 줄여서(0.15) 점만 쫓다가 궤적을 이탈하는 현상 방지

            // Feedforward: 곡률에 따라 미리 꺾어주는 힘 강화 (0.5 -> 0.7)
            double feedforward = 0.7 * curv * wheelbase_;

            // 조합: Road Alignment(1.2) + Position Correction(0.6) + Feedforward
            steer = steer_pp * 0.15 + road_err * 1.2 + cte_steer * 0.6 + feedforward;
        }
        else if (current_state_ == DrivingState::LANE_CHANGING)
        {
            // Lane Change: CTE(위치) 우선
            steer = steer_pp * 0.2 + road_err * 0.2 + cte_steer * 0.8;
        }
        else
        {
            // Normal Cruise
            steer = steer_pp * 0.3 + road_err * 0.4 + cte_steer * 0.7;
        }

        // Emergency Correction (차선 이탈 방지)
        if (std::abs(cte) > 0.12)
        {
            double emergency_steer = (cte > 0) ? -0.4 : 0.4;
            // 코너에서는 비상 조향의 비중을 조금 낮춰 부드럽게 개입
            double weight = is_corner ? 0.3 : 0.5;
            steer = steer * (1.0 - weight) + emergency_steer * weight;
        }

        // Smoothing
        // 코너에서는 반응성을 위해 alpha를 낮춤 (과거 값 영향 감소)
        double alpha = (current_state_ == DrivingState::LANE_CHANGING) ? 0.2 : (is_corner ? 0.4 : 0.4);
        steer = steer * alpha + last_steer_ * (1.0 - alpha);

        return std::clamp(steer, -max_steer_, max_steer_);
    }
    // ========================================================================
    // VELOCITY CONTROL
    // ========================================================================

    // ========================================================================
    // VELOCITY CONTROL (수정됨: 정지 장애물 대응)
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
                // [수정] 정지한 장애물(Static)인 경우:
                // 브레이크를 밟지 않고 회피 기동을 위해 속도를 유지함 (멈추면 못 빠져나감)
                if (s.front_is_static)
                {
                    vel = slow_speed_; // 최소 회피 속도 유지 (0.25 m/s)
                }
                else
                {
                    // 움직이는 차라면 안전거리 유지하며 감속
                    double gap = s.front_dist - FRONT_CRITICAL_DIST;
                    vel = std::min(target_speed_, std::max(s.front_speed, gap * 2.0));
                    vel = std::max(vel, 0.0); // 완전히 멈출 수도 있음
                }
            }
            break;

        case DrivingState::PREPARE_OVERTAKE:
            // [수정] 정지 장애물이면 감속 없이 바로 차선 변경 준비
            if (s.front_is_static)
            {
                vel = slow_speed_;
            }
            else
            {
                vel = std::min(target_speed_, s.front_speed + 0.05);
                vel = std::max(vel, slow_speed_);
            }
            break;

        case DrivingState::LANE_CHANGING:
            // 차선 변경 중에는 과감하게 가속
            vel = overtake_speed_;
            break;

        case DrivingState::EMERGENCY:
            // [수정] 긴급 상황이라도 정지 장애물이면 멈추지 않고 탈출 시도
            if (s.front_is_static)
            {
                vel = slow_speed_;
            }
            else
            {
                vel = slow_speed_;
                if (s.front_dist < FRONT_CRITICAL_DIST * 0.5)
                    vel = 0.0; // 움직이는 차가 너무 가까우면 정지
            }
            break;
        }

        return std::clamp(vel, 0.0, 1.0); // 0.1 -> 0.0 하한선 조정
    }

    // ========================================================================
    // MAIN CONTROL LOOP
    // ========================================================================

    void control_loop()
    {
        if (!pose_received_)
            return;

        // Check lanes ready
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

        // Initial search
        if (!initial_search_done_)
        {
            for (int i = 0; i < 3; ++i)
                find_closest_idx(i, ego_x_, ego_y_);
            initial_search_done_ = true;
            RCLCPP_INFO(this->get_logger(), "Initial search done. Lane: %s",
                        lane_str(current_lane_).c_str());
        }

        // Handle NONE lane
        if (current_lane_ == LaneID::NONE)
        {
            current_lane_ = get_lane_at(ego_x_, ego_y_);
            if (current_lane_ == LaneID::NONE)
                current_lane_ = LaneID::LANE_2;
            target_lane_ = current_lane_;
        }

        // Update body frame for all obstacles
        update_all_obstacles_body_frame();

        // Check all zones
        current_surrounding_ = check_all_zones();

        // State machine
        update_state_machine();

        // Select path
        int path_lane_idx = lane_to_int(
            (current_state_ == DrivingState::LANE_CHANGING) ? target_lane_ : current_lane_);

        // Path following
        int closest = find_closest_idx(path_lane_idx, ego_x_, ego_y_);

        // Curvature check
        int preview = (closest + 25) % lane_paths_[path_lane_idx]->poses.size();
        double curv = compute_curvature(path_lane_idx, preview, 10);
        bool is_corner = std::abs(curv) > 0.4;

        // Lookahead
        double lookahead_dist = is_corner ? 0.12 : base_lookahead_;
        if (current_state_ == DrivingState::LANE_CHANGING)
            lookahead_dist = 0.15;
        lookahead_dist = std::clamp(lookahead_dist, 0.08, max_lookahead_);

        int lookahead_idx = find_lookahead_idx(path_lane_idx, closest, lookahead_dist);

        // Control
        double steer = compute_steering(path_lane_idx, closest, lookahead_idx);
        double vel = compute_velocity();

        // Corner speed limit
        if (is_corner)
            vel = std::min(vel, 0.35);

        last_steer_ = steer;

        // Publish command
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

    void publish_car_marker()
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "world";
        m.header.stamp = this->now();
        m.ns = "ego_car";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;

        m.pose.position.x = ego_x_;
        m.pose.position.y = ego_y_;
        m.pose.position.z = 0.1;

        tf2::Quaternion q;
        q.setRPY(0, 0, ego_yaw_);
        m.pose.orientation = tf2::toMsg(q);

        m.scale.x = 0.4;
        m.scale.y = 0.2;
        m.scale.z = 0.2;

        m.color.r = 0.0;
        m.color.g = 0.0;
        m.color.b = 1.0;
        m.color.a = 0.8;

        m.lifetime = rclcpp::Duration::from_seconds(0.2);
        car_pub_->publish(m);
    }

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

        // ============ FRONT ZONE (Green/Yellow/Red) ============
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "world";
            m.header.stamp = this->now();
            m.ns = "zones";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::CUBE;
            m.action = visualization_msgs::msg::Marker::ADD;

            double center_x = FRONT_ZONE_LENGTH / 2;
            m.pose.position.x = ego_x_ + cos_yaw * center_x - sin_yaw * 0;
            m.pose.position.y = ego_y_ + sin_yaw * center_x + cos_yaw * 0;
            m.pose.position.z = 0.02;
            m.pose.orientation = quat_msg;

            m.scale.x = FRONT_ZONE_LENGTH;
            m.scale.y = FRONT_ZONE_WIDTH * 2;
            m.scale.z = 0.01;

            if (current_surrounding_.front_critical)
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

        // ============ REAR ZONE (Orange) ============
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

        // ============ LEFT BLIND SPOT (Blue) ============
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

        // ============ RIGHT BLIND SPOT (Blue) ============
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

        // ============ STATE TEXT ============
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
            ss << state_str(current_state_) << "\n"
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
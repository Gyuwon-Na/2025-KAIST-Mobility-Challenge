/**
 * @file local_path_pub.cpp
 * @brief Safety-First Local Path Publisher
 *        - Lane 3 고정 주행
 *        - 합류/분기 구간 TTC 기반 안전 체크
 *        - 역주행 방지 (진행 방향 기반 인덱스 검색)
 * @version 4.0
 */

#include "bisa/local_path_pub.hpp"

namespace bisa
{

    LocalPathPubCpp::LocalPathPubCpp() : Node("local_path_pub")
    {
        RCLCPP_INFO(this->get_logger(), "============================================");
        RCLCPP_INFO(this->get_logger(), "Local Path: SAFETY MODE v4.0");
        RCLCPP_INFO(this->get_logger(), "  - Lane 3 Fixed Path Following");
        RCLCPP_INFO(this->get_logger(), "  - TTC-based Merge/Split Safety Check");
        RCLCPP_INFO(this->get_logger(), "  - Anti-Reverse Direction Logic");
        RCLCPP_INFO(this->get_logger(), "============================================");

        auto qos = rclcpp::QoS(10).transient_local();

        // Lane Subscriptions
        sub_lane_[0] = this->create_subscription<nav_msgs::msg::Path>(
            "/hdmap/lane_one", qos,
            [this](nav_msgs::msg::Path::SharedPtr msg)
            { lane_paths_[0] = msg; process_lane_path(0); });

        sub_lane_[1] = this->create_subscription<nav_msgs::msg::Path>(
            "/hdmap/lane_two", qos,
            [this](nav_msgs::msg::Path::SharedPtr msg)
            { lane_paths_[1] = msg; process_lane_path(1); });

        sub_lane_[2] = this->create_subscription<nav_msgs::msg::Path>(
            "/hdmap/lane_three", qos,
            [this](nav_msgs::msg::Path::SharedPtr msg)
            { lane_paths_[2] = msg; process_lane_path(2); });

        // Obstacles & Pose
        obs_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/obstacles_markers", 10,
            std::bind(&LocalPathPubCpp::obstacle_callback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose", rclcpp::SensorDataQoS(),
            std::bind(&LocalPathPubCpp::pose_callback, this, std::placeholders::_1));

        // Publishers
        local_pub_ = this->create_publisher<nav_msgs::msg::Path>("/local_path", 10);
        target_vel_pub_ = this->create_publisher<std_msgs::msg::Float32>("/planning/target_v", 10);
        debug_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/planning/debug", 10);

        // Timer (20ms -> 50Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&LocalPathPubCpp::control_loop, this));
    }

    // ============================================================================
    // UTILITY FUNCTIONS
    // ============================================================================

    double LocalPathPubCpp::get_dist(double x1, double y1, double x2, double y2)
    {
        return std::hypot(x1 - x2, y1 - y2);
    }

    double LocalPathPubCpp::normalize_angle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    // ============================================================================
    // [핵심] 진행 방향 기반 인덱스 검색 - 역주행 방지
    // ============================================================================

    int LocalPathPubCpp::find_closest_idx_forward(int lane_idx, double x, double y)
    {
        if (processed_lanes_[lane_idx].empty())
            return 0;

        const auto &lane = processed_lanes_[lane_idx];
        int n = lane.size();

        // 첫 검색이거나 인덱스가 초기화 안됨
        if (last_idx_[lane_idx] < 0)
        {
            // Global search (초기 1회)
            int best = 0;
            double min_d = 1e9;
            for (int i = 0; i < n; ++i)
            {
                double d = get_dist(x, y, lane[i].x, lane[i].y);
                if (d < min_d)
                {
                    min_d = d;
                    best = i;
                }
            }
            last_idx_[lane_idx] = best;
            return best;
        }

        // [핵심] 이전 인덱스 기준으로 "앞으로만" 검색 (역주행 방지)
        // 검색 범위: last_idx - 5 ~ last_idx + 100 (뒤로는 조금만, 앞으로는 많이)
        int best = last_idx_[lane_idx];
        double min_d = 1e9;

        for (int offset = -5; offset < 100; ++offset)
        {
            int idx = (last_idx_[lane_idx] + offset + n) % n;
            double d = get_dist(x, y, lane[idx].x, lane[idx].y);

            // 거리가 가장 가까운 점 선택
            // 단, 너무 뒤로 가지 않도록 offset이 음수일 때는 penalty 적용
            double penalty = (offset < 0) ? 0.1 * std::abs(offset) : 0.0;

            if (d + penalty < min_d)
            {
                min_d = d;
                best = idx;
            }
        }

        // 인덱스가 급격히 감소하면 (역주행 징후) 무시
        int idx_diff = best - last_idx_[lane_idx];
        if (idx_diff < -10 && idx_diff > -(n - 100))
        {
            // 역주행 방지: 이전 인덱스 유지하고 조금만 증가
            best = (last_idx_[lane_idx] + 1) % n;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "[ANTI-REVERSE] Index jump detected! Forcing forward.");
        }

        last_idx_[lane_idx] = best;
        return best;
    }

    // ============================================================================
    // 합류/분기 구간 분석
    // ============================================================================

    void LocalPathPubCpp::analyze_topology()
    {
        if (processed_lanes_[1].empty() || processed_lanes_[2].empty())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Cannot analyze topology: Lane data missing.");
            return;
        }
        if (topology_analyzed_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Topology already analyzed.");
            return;
        }

        const auto &l3 = processed_lanes_[2]; // Lane 3 (Main Path)
        const auto &l2 = processed_lanes_[1]; // Lane 2 (Conflict Lane)

        // Lane 3의 각 점이 Lane 2와 얼마나 가까운지 분석
        std::vector<double> dist_to_l2(l3.size(), 999.0);

        for (size_t i = 0; i < l3.size(); ++i)
        {
            double min_d = 999.0;
            for (size_t j = 0; j < l2.size(); ++j)
            {
                double d = get_dist(l3[i].x, l3[i].y, l2[j].x, l2[j].y);
                if (d < min_d)
                    min_d = d;
            }
            dist_to_l2[i] = min_d;
        }

        // 합류/분기 구간 감지 (거리 < MERGE_DIST_THRESHOLD)
        const double MERGE_DIST_THRESHOLD = 0.01; // 30cm 이내면 합류 구간

        merge_start_idx_ = -1;
        merge_end_idx_ = -1;

        // 상태: 0=분리, 1=합류
        int state = (dist_to_l2[0] < MERGE_DIST_THRESHOLD) ? 1 : 0;

        for (size_t i = 1; i < l3.size(); ++i)
        {
            bool merged = (dist_to_l2[i] < MERGE_DIST_THRESHOLD);

            RCLCPP_INFO(this->get_logger(), "Idx %zu: Dist to L2=%.2f, Merged=%d", i, dist_to_l2[i], merged);

            if (state == 0 && merged)
            {
                // 분리 → 합류 (합류 시작점)
                merge_start_idx_ = i;
                RCLCPP_INFO(this->get_logger(), "[TOPOLOGY] Merge START at idx %d (%.2f, %.2f)",
                            (int)i, l3[i].x, l3[i].y);
                state = 1;
            }
            else if (state == 1 && !merged)
            {
                // 합류 → 분리 (분기점)
                merge_end_idx_ = i;
                RCLCPP_INFO(this->get_logger(), "[TOPOLOGY] Merge END (Split) at idx %d (%.2f, %.2f)",
                            (int)i, l3[i].x, l3[i].y);
                state = 0;
                break; // 첫 번째 합류/분기 구간만 분석
            }
        }

        // 유효성 체크
        if (merge_start_idx_ >= 0 && merge_end_idx_ < 0)
        {
            // 끝까지 합류 상태면 마지막을 분기점으로
            merge_end_idx_ = l3.size() - 1;
        }

        RCLCPP_INFO(this->get_logger(), "[TOPOLOGY] Analysis Complete: merge=[%d, %d]",
                    merge_start_idx_, merge_end_idx_);

        topology_analyzed_ = true;
    }

    // ============================================================================
    // TTC 기반 안전성 체크
    // ============================================================================

    double LocalPathPubCpp::calculate_ttc(const ObstacleInfo &obs)
    {
        double dx = obs.x - ego_x_;
        double dy = obs.y - ego_y_;
        double dist = std::hypot(dx, dy);

        if (dist < 0.01)
            return 0.0; // 이미 충돌

        // 차량 헤딩 방향
        double heading_x = std::cos(ego_yaw_);
        double heading_y = std::sin(ego_yaw_);

        // 내적으로 전방 여부 확인 (음수면 후방)
        double forward_dot = dx * heading_x + dy * heading_y;
        if (forward_dot < 0)
            return 999.0; // 후방 장애물은 무시

        // 상대 속도 계산
        double ego_vx = ego_speed_ * heading_x;
        double ego_vy = ego_speed_ * heading_y;
        double rel_vx = obs.vx - ego_vx;
        double rel_vy = obs.vy - ego_vy;

        // Closing speed (양수면 가까워지는 중)
        double closing_speed = -(rel_vx * dx + rel_vy * dy) / dist;

        if (closing_speed <= 0.01)
            return 999.0; // 멀어지거나 정지

        return dist / closing_speed;
    }

    bool LocalPathPubCpp::check_merge_safety()
    {
        // 합류 구간: Lane 2의 HV와 충돌 위험 체크
        double min_ttc = 999.0;
        int danger_id = -1;

        for (const auto &[id, obs] : obstacles_)
        {
            // Lane 2 장애물만 체크
            if (obs.lane != LaneID::LANE_2)
                continue;

            double ttc = calculate_ttc(obs);
            if (ttc < min_ttc)
            {
                min_ttc = ttc;
                danger_id = id;
            }
        }

        if (min_ttc < TTC_THRESHOLD)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "[MERGE DANGER] L2 HV id=%d, TTC=%.2f < %.1f", danger_id, min_ttc, TTC_THRESHOLD);
            return false;
        }
        return true;
    }

    bool LocalPathPubCpp::check_split_safety()
    {
        // 분기 구간: Lane 3의 HV와 충돌 위험 체크
        double min_ttc = 999.0;
        int danger_id = -1;

        for (const auto &[id, obs] : obstacles_)
        {
            // Lane 3 장애물만 체크
            if (obs.lane != LaneID::LANE_3)
                continue;

            double ttc = calculate_ttc(obs);
            if (ttc < min_ttc)
            {
                min_ttc = ttc;
                danger_id = id;
            }
        }

        if (min_ttc < TTC_THRESHOLD)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "[SPLIT DANGER] L3 HV id=%d, TTC=%.2f < %.1f", danger_id, min_ttc, TTC_THRESHOLD);
            return false;
        }
        return true;
    }

    // ============================================================================
    // Lane 판별
    // ============================================================================

    LaneID LocalPathPubCpp::get_lane_at(double x, double y)
    {
        double min_d2 = 999.0, min_d3 = 999.0;

        // Lane 2
        if (!processed_lanes_[1].empty())
        {
            for (const auto &pt : processed_lanes_[1])
            {
                double d = get_dist(x, y, pt.x, pt.y);
                if (d < min_d2)
                    min_d2 = d;
            }
        }

        // Lane 3
        if (!processed_lanes_[2].empty())
        {
            for (const auto &pt : processed_lanes_[2])
            {
                double d = get_dist(x, y, pt.x, pt.y);
                if (d < min_d3)
                    min_d3 = d;
            }
        }

        // 더 가까운 차선 반환 (threshold 내)
        const double LANE_THRESHOLD = 0.5;
        if (min_d2 < min_d3 && min_d2 < LANE_THRESHOLD)
            return LaneID::LANE_2;
        if (min_d3 < min_d2 && min_d3 < LANE_THRESHOLD)
            return LaneID::LANE_3;
        if (min_d2 < LANE_THRESHOLD)
            return LaneID::LANE_2;
        if (min_d3 < LANE_THRESHOLD)
            return LaneID::LANE_3;

        return LaneID::NONE;
    }

    // ============================================================================
    // MAIN CONTROL LOOP
    // ============================================================================

    void LocalPathPubCpp::control_loop()
    {
        if (!pose_received_)
            return;
        if (processed_lanes_[2].empty())
            return;

        // 1. 토폴로지 분석 (1회)
        analyze_topology();

        // 2. 현재 위치 인덱스 (역주행 방지 로직 적용)
        int ego_idx = find_closest_idx_forward(2, ego_x_, ego_y_);
        const auto &lane3 = processed_lanes_[2];
        int n = lane3.size();

        // 3. 기본 속도
        double target_vel = CRUISE_SPEED;
        std::string state_str = "CRUISE";

        // 4. 합류/분기 구간 체크
        bool in_merge_zone = false;
        bool approaching_merge = false;
        bool in_split_zone = false;
        bool approaching_split = false;

        if (merge_start_idx_ >= 0 && merge_end_idx_ >= 0)
        {
            // 합류 구간 접근 체크 (앞으로 LOOKAHEAD_DIST 이내)
            double dist_to_merge_start = 0.0;
            for (int i = ego_idx; i < ego_idx + 100 && i < ego_idx + n; ++i)
            {
                int idx = i % n;
                int next_idx = (i + 1) % n;
                dist_to_merge_start += get_dist(lane3[idx].x, lane3[idx].y,
                                                lane3[next_idx].x, lane3[next_idx].y);
                if (idx == merge_start_idx_)
                {
                    approaching_merge = (dist_to_merge_start < LOOKAHEAD_DIST);
                    break;
                }
            }

            // 합류 구간 내부인지 체크
            if (merge_start_idx_ < merge_end_idx_)
            {
                in_merge_zone = (ego_idx >= merge_start_idx_ && ego_idx < merge_end_idx_);
            }
            else
            {
                // wrap-around case
                in_merge_zone = (ego_idx >= merge_start_idx_ || ego_idx < merge_end_idx_);
            }

            // 분기점 접근 체크
            double dist_to_split = 0.0;
            for (int i = ego_idx; i < ego_idx + 100 && i < ego_idx + n; ++i)
            {
                int idx = i % n;
                int next_idx = (i + 1) % n;
                dist_to_split += get_dist(lane3[idx].x, lane3[idx].y,
                                          lane3[next_idx].x, lane3[next_idx].y);
                if (idx == merge_end_idx_)
                {
                    approaching_split = (dist_to_split < LOOKAHEAD_DIST);
                    in_split_zone = (dist_to_split < LOOKAHEAD_DIST * 0.5);
                    break;
                }
            }
        }

        // 5. 안전성 체크 및 속도 조절
        if (approaching_merge || in_merge_zone)
        {
            if (!check_merge_safety())
            {
                target_vel = 0.0;
                state_str = "WAIT_MERGE";
            }
            else
            {
                target_vel = SLOW_SPEED;
                state_str = "MERGE";
            }
        }

        if (approaching_split || in_split_zone)
        {
            if (!check_split_safety())
            {
                target_vel = 0.0;
                state_str = "WAIT_SPLIT";
            }
            else if (state_str != "WAIT_MERGE")
            {
                target_vel = SLOW_SPEED;
                state_str = "SPLIT";
            }
        }

        // 6. Local Path 생성 (Lane 3 기반)
        nav_msgs::msg::Path local_path_msg;
        local_path_msg.header.frame_id = "world";
        local_path_msg.header.stamp = this->now();

        for (int i = 0; i < PATH_POINTS; ++i)
        {
            int idx = (ego_idx + i) % n;
            geometry_msgs::msg::PoseStamped pose;
            pose.header = local_path_msg.header;
            pose.pose.position.x = lane3[idx].x;
            pose.pose.position.y = lane3[idx].y;
            pose.pose.position.z = 0.0;

            // Orientation 계산 (다음 점 방향)
            int next_idx = (idx + 1) % n;
            double yaw = std::atan2(lane3[next_idx].y - lane3[idx].y,
                                    lane3[next_idx].x - lane3[idx].x);
            pose.pose.orientation.z = std::sin(yaw / 2.0);
            pose.pose.orientation.w = std::cos(yaw / 2.0);

            local_path_msg.poses.push_back(pose);
        }

        // 7. Publish
        local_pub_->publish(local_path_msg);

        std_msgs::msg::Float32 v_msg;
        v_msg.data = static_cast<float>(target_vel);
        target_vel_pub_->publish(v_msg);

        // 8. Logging
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "[%s] idx=%d, vel=%.2f, ego_spd=%.2f",
                             state_str.c_str(), ego_idx, target_vel, ego_speed_);
    }

    // ============================================================================
    // CALLBACKS
    // ============================================================================

    void LocalPathPubCpp::process_lane_path(int lane_idx)
    {
        if (!lane_paths_[lane_idx] || lane_paths_[lane_idx]->poses.empty())
            return;

        processed_lanes_[lane_idx].clear();

        for (const auto &p : lane_paths_[lane_idx]->poses)
        {
            PathPoint pt;
            pt.x = p.pose.position.x;
            pt.y = p.pose.position.y;
            processed_lanes_[lane_idx].push_back(pt);
        }

        // RCLCPP_INFO(this->get_logger(), "Lane %d loaded: %zu points",
        //             lane_idx, processed_lanes_[lane_idx].size());

        // 인덱스 초기화
        last_idx_[lane_idx] = -1;
        topology_analyzed_ = false;
    }

    void LocalPathPubCpp::obstacle_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        double now_s = this->now().seconds();

        for (const auto &mk : msg->markers)
        {
            if (mk.ns != "surrounding_cars")
                continue;
            if (mk.type != visualization_msgs::msg::Marker::CUBE)
                continue;

            int id = mk.id;
            ObstacleInfo &obs = obstacles_[id];

            // 속도 계산 (위치 미분)
            if (obs.last_seen_sec > 0)
            {
                double dt = now_s - obs.last_seen_sec;
                if (dt > 0.01 && dt < 1.0)
                {
                    double new_vx = (mk.pose.position.x - obs.x) / dt;
                    double new_vy = (mk.pose.position.y - obs.y) / dt;

                    // Low-pass filter
                    obs.vx = obs.vx * 0.5 + new_vx * 0.5;
                    obs.vy = obs.vy * 0.5 + new_vy * 0.5;
                }
            }

            obs.id = id;
            obs.x = mk.pose.position.x;
            obs.y = mk.pose.position.y;
            obs.speed = std::hypot(obs.vx, obs.vy);
            obs.lane = get_lane_at(obs.x, obs.y);
            obs.last_seen_sec = now_s;
        }

        // Stale obstacle 제거
        for (auto it = obstacles_.begin(); it != obstacles_.end();)
        {
            if ((now_s - it->second.last_seen_sec) > 2.0)
                it = obstacles_.erase(it);
            else
                ++it;
        }
    }

    void LocalPathPubCpp::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        double now_s = this->now().seconds();

        double new_x = msg->pose.position.x;
        double new_y = msg->pose.position.y;

        // Yaw from quaternion (simplified: assuming 2D)
        // 시뮬레이터에서 orientation.z가 yaw일 수 있음
        double siny = 2.0 * (msg->pose.orientation.w * msg->pose.orientation.z +
                             msg->pose.orientation.x * msg->pose.orientation.y);
        double cosy = 1.0 - 2.0 * (msg->pose.orientation.y * msg->pose.orientation.y +
                                   msg->pose.orientation.z * msg->pose.orientation.z);
        ego_yaw_ = std::atan2(siny, cosy);

        // Speed 계산
        if (prev_pose_time_ > 0)
        {
            double dt = now_s - prev_pose_time_;
            if (dt > 0.01 && dt < 0.5)
            {
                double dist = get_dist(new_x, new_y, ego_x_, ego_y_);
                double raw_speed = dist / dt;

                // 진행 방향 확인 (헤딩과 이동 방향 비교)
                double move_angle = std::atan2(new_y - ego_y_, new_x - ego_x_);
                double angle_diff = normalize_angle(move_angle - ego_yaw_);
                if (std::abs(angle_diff) > M_PI / 2.0)
                    raw_speed = -raw_speed; // 후진 감지

                ego_speed_ = ego_speed_ * 0.7 + raw_speed * 0.3; // Low-pass filter
            }
        }

        ego_x_ = new_x;
        ego_y_ = new_y;
        prev_pose_time_ = now_s;
        pose_received_ = true;
    }

} // namespace bisa

// ============================================================================
// MAIN
// ============================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bisa::LocalPathPubCpp>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
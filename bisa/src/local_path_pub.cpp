/**
 * @file local_path_pub.cpp
 * @brief Safety-First Local Path: Consistent Zone-Based Safety
 */

#include "bisa/local_path_pub.hpp"
#include <limits>
#include <algorithm>
#include <cmath>

namespace bisa
{
    // [핵심] 절대 변하지 않는 고정 좌표 인덱스 (일관성 보장)
    static const int FIXED_MERGE_IDX = 1500;
    static const int FIXED_SPLIT_IDX = 2281;

    LocalPathPubCpp::LocalPathPubCpp() : Node("local_path_pub")
    {
        RCLCPP_INFO(this->get_logger(), "============================================");
        RCLCPP_INFO(this->get_logger(), "Local Path: FIXED INDICES + ZONE SAFETY");
        RCLCPP_INFO(this->get_logger(), " - Merge Index: %d", FIXED_MERGE_IDX);
        RCLCPP_INFO(this->get_logger(), " - Split Index: %d", FIXED_SPLIT_IDX);
        RCLCPP_INFO(this->get_logger(), "============================================");

        auto qos = rclcpp::QoS(10).transient_local();

        // Subscribers
        sub_lane_[0] = this->create_subscription<nav_msgs::msg::Path>(
            "/hdmap/lane_one", qos, [this](nav_msgs::msg::Path::SharedPtr msg)
            { lane_paths_[0] = msg; process_lane_path(0); });
        sub_lane_[1] = this->create_subscription<nav_msgs::msg::Path>(
            "/hdmap/lane_two", qos, [this](nav_msgs::msg::Path::SharedPtr msg)
            { lane_paths_[1] = msg; process_lane_path(1); });
        sub_lane_[2] = this->create_subscription<nav_msgs::msg::Path>(
            "/hdmap/lane_three", qos, [this](nav_msgs::msg::Path::SharedPtr msg)
            { lane_paths_[2] = msg; process_lane_path(2); });

        sub_obs_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/obstacles_markers", 10, std::bind(&LocalPathPubCpp::obstacle_callback, this, std::placeholders::_1));
        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose", rclcpp::SensorDataQoS(), std::bind(&LocalPathPubCpp::pose_callback, this, std::placeholders::_1));

        sub_env_slow_ = this->create_subscription<std_msgs::msg::Float32>(
            "/env/slow_vel", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            { env_slow_vel_ = msg->data; });

        sub_env_fast_ = this->create_subscription<std_msgs::msg::Float32>(
            "/env/fast_vel", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            { env_fast_vel_ = msg->data; });

        // Publishers
        pub_local_path_ = this->create_publisher<nav_msgs::msg::Path>("/local_path", 10);
        pub_target_vel_ = this->create_publisher<std_msgs::msg::Float32>("/planning/target_v", 10);
        pub_debug_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/merge_zone", 10);

        // Timer
        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&LocalPathPubCpp::control_loop, this));
    }

    // ========================================================================
    // UTILS
    // ========================================================================
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

    // [기존 코드 유지] 사용자님이 원하시는 전방 탐색 로직
    int LocalPathPubCpp::find_closest_idx_forward(int lane_idx, double x, double y)
    {
        if (processed_lanes_[lane_idx].empty())
            return 0;
        int path_size = processed_lanes_[lane_idx].size();

        // 처음 시작할 때
        if (last_closest_idx_ < 0)
        {
            double min_d = 1e9;
            int min_idx = 0;
            for (int i = 0; i < path_size; ++i)
            {
                double d = get_dist(x, y, processed_lanes_[lane_idx][i].x, processed_lanes_[lane_idx][i].y);
                if (d < min_d)
                {
                    min_d = d;
                    min_idx = i;
                }
            }
            last_closest_idx_ = min_idx;
            return min_idx;
        }

        // 주행 중: 이전 인덱스부터 앞으로 검색 (Window Search)
        int search_len = 200; // 검색 범위
        int best_idx = last_closest_idx_;
        double min_d = 1e9;

        for (int i = 0; i < search_len; ++i)
        {
            int idx = (last_closest_idx_ + i) % path_size;
            double d = get_dist(x, y, processed_lanes_[lane_idx][idx].x, processed_lanes_[lane_idx][idx].y);
            if (d < min_d)
            {
                min_d = d;
                best_idx = idx;
            }
        }
        last_closest_idx_ = best_idx;
        return best_idx;
    }

    LaneID LocalPathPubCpp::get_lane_at(double x, double y)
    {
        // 간단한 최근접 차선 판별
        double d2 = 1e9, d3 = 1e9;

        // Lane 2 거리 (샘플링 최적화)
        if (!processed_lanes_[1].empty())
        {
            for (size_t i = 0; i < processed_lanes_[1].size(); i += 10)
            {
                double d = get_dist(x, y, processed_lanes_[1][i].x, processed_lanes_[1][i].y);
                if (d < d2)
                    d2 = d;
            }
        }
        // Lane 3 거리
        if (!processed_lanes_[2].empty())
        {
            for (size_t i = 0; i < processed_lanes_[2].size(); i += 10)
            {
                double d = get_dist(x, y, processed_lanes_[2][i].x, processed_lanes_[2][i].y);
                if (d < d3)
                    d3 = d;
            }
        }

        if (d2 < d3 && d2 < 1.0)
            return LaneID::LANE_2;
        if (d3 <= d2 && d3 < 1.0)
            return LaneID::LANE_3;
        return LaneID::NONE;
    }

    // [핵심 변경] Zone 기반 안전성 체크 (방향 무관, 위치만 봄)
    bool LocalPathPubCpp::check_zone_safety(double center_x, double center_y, double radius, LaneID target_lane)
    {
        for (auto const &[id, obs] : obstacles_)
        {
            if (obs.lane != target_lane)
                continue;

            // 해당 구역(반경 radius) 안에 차가 들어오면 무조건 위험
            double d = get_dist(center_x, center_y, obs.x, obs.y);
            if (d < radius)
            {
                return false; // UNSAFE
            }
        }
        return true; // SAFE
    }

    bool LocalPathPubCpp::in_merge_zone(int idx)
    {
        return (idx >= FIXED_MERGE_IDX && idx <= FIXED_SPLIT_IDX);
    }

    // ========================================================================
    // MAIN LOOP
    // ========================================================================
    void LocalPathPubCpp::control_loop()
    {
        if (!pose_received_)
            return;
        if (processed_lanes_[2].empty())
            return;

        // 1. 현재 위치 찾기 (사용자님 로직 사용)
        LaneID current_lane_id = get_lane_at(ego_x_, ego_y_);

        if (current_lane_id == LaneID::NONE || current_lane_id == LaneID::LANE_1)
        {
            current_lane_id = LaneID::LANE_3;
        }

        int ego_idx = find_closest_idx_forward(2, ego_x_, ego_y_);

        // 2. 목표 속도 결정 (내가 위치한 차선 기준)
        double follow_speed = 2.0;
        if (current_lane_id == LaneID::LANE_2)
        {
            follow_speed = (env_slow_vel_ > 0.1) ? env_slow_vel_ : 1.5;
        }
        else
        {
            follow_speed = (env_fast_vel_ > 0.1) ? env_fast_vel_ : 2.0;
        }

        // 3. 전방 장애물 거리 탐색 (내 차선 기준)
        double min_dist_ahead = 999.0;
        bool obj_found = false;

        for (auto const &[id, obs] : obstacles_)
        {
            if (obs.lane != current_lane_id)
                continue;

            double dx = obs.x - ego_x_;
            double dy = obs.y - ego_y_;
            // 로컬 좌표계 변환 (전방 거리)
            double local_x = dx * std::cos(ego_yaw_) + dy * std::sin(ego_yaw_);

            // 전방 0.0m ~ 20m 탐색
            if (local_x > 0.0 && local_x < 2.0)
            {
                double dist = std::hypot(dx, dy);
                if (dist < min_dist_ahead)
                {
                    min_dist_ahead = dist;
                    obj_found = true;
                }
            }
        }

        // 4. 속도 발행
        double target_vel = follow_speed;

        if (obj_found)
        {
            if (min_dist_ahead < 1.0)
            {
                // [Case 1] 장애물 매우 가까움 (ACC 감속)
                // [Case 1-1] 너무 가까우면(0.4m) 더 감속
                if (min_dist_ahead <= 0.4)
                {
                    target_vel = follow_speed * 0.7; // 간격 벌리기
                }
                else
                {
                    target_vel = follow_speed; // 그룹 속도 유지
                }
            }
            else
            {
                // [Case 2] 장애물이 있지만 멀리 있음 (1.0m 이상)
                // [Case 2-1] 합류 구간이 아닐 때만 2.0으로 증속
                if (!in_merge_zone(ego_idx))
                {
                    target_vel = 2.0;
                }

                // [Case 2-2] 합류 구간이면??
            }
        }
        else
        {
            // [Case 3] 장애물 없음
            // [Case 3-1] 합류 구간이 아닐 때만 2.0으로 증속
            if (!in_merge_zone(ego_idx))
            {
                target_vel = 2.0;
            }

            // [Case 3-2] 합류 구간이면??
        }

        const auto &lane3 = processed_lanes_[2];
        int path_size = lane3.size();

        // --------------------------------------------------------------------
        // [FIX] Path Generation (최소 길이 보장)
        // 속도가 줄어도 경로가 너무 짧아지지 않도록 최소 50개 점은 무조건 생성
        // --------------------------------------------------------------------
        nav_msgs::msg::Path local_path_msg;
        local_path_msg.header.frame_id = "world";
        local_path_msg.header.stamp = this->now();

        // 기존 로직: 속도 비례 Lookahead
        double current_speed = std::abs(ego_speed_);
        // 최소 5m(50개)는 무조건 보장, 속도 빠르면 더 길게
        int lookahead_steps = std::max(50, static_cast<int>((current_speed * 1.5) / 0.1));
        // 상한선 설정 (너무 길면 불필요)
        lookahead_steps = std::min(lookahead_steps, 150);

        for (int i = 0; i < lookahead_steps; ++i)
        {
            int idx = (ego_idx + i) % path_size; // Loop 처리

            geometry_msgs::msg::PoseStamped pose;
            pose.header = local_path_msg.header;
            pose.pose.position.x = lane3[idx].x;
            pose.pose.position.y = lane3[idx].y;
            pose.pose.orientation.w = 1.0;
            local_path_msg.poses.push_back(pose);
        }

        pub_local_path_->publish(local_path_msg);

        std_msgs::msg::Float32 v_msg;
        v_msg.data = target_vel;
        pub_target_vel_->publish(v_msg);

        // [시각화] 고정 포인트 표시 (제대로 작동하는지 눈으로 확인용)
        visualization_msgs::msg::MarkerArray markers;
        auto now = this->now();

        // Merge (Red)
        visualization_msgs::msg::Marker m1;
        m1.header.frame_id = "world";
        m1.header.stamp = now;
        m1.id = 0;
        m1.type = visualization_msgs::msg::Marker::SPHERE;
        m1.action = visualization_msgs::msg::Marker::ADD;
        m1.pose.position.x = lane3[FIXED_MERGE_IDX].x;
        m1.pose.position.y = lane3[FIXED_MERGE_IDX].y;
        m1.pose.position.z = 0.5;
        m1.scale.x = 1.0;
        m1.scale.y = 1.0;
        m1.scale.z = 1.0;
        m1.color.r = 1.0;
        m1.color.a = 0.6;
        markers.markers.push_back(m1);

        // Split (Blue)
        visualization_msgs::msg::Marker m2;
        m2 = m1;
        m2.id = 1;
        m2.pose.position.x = lane3[FIXED_SPLIT_IDX].x;
        m2.pose.position.y = lane3[FIXED_SPLIT_IDX].y;
        m2.color.r = 0.0;
        m2.color.b = 1.0;
        markers.markers.push_back(m2);

        pub_debug_->publish(markers);
    }

    // ========================================================================
    // CALLBACK IMPL
    // ========================================================================
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
    }

    void LocalPathPubCpp::obstacle_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        double now_s = this->now().seconds();
        for (const auto &mk : msg->markers)
        {
            if (mk.type != visualization_msgs::msg::Marker::CUBE)
                continue;
            int id = mk.id;
            ObstacleInfo &obs = obstacles_[id];

            if (obs.last_seen_sec > 0)
            {
                double dt = now_s - obs.last_seen_sec;
                if (dt > 0.01)
                {
                    obs.vx = (mk.pose.position.x - obs.x) / dt;
                    obs.vy = (mk.pose.position.y - obs.y) / dt;
                }
            }
            obs.x = mk.pose.position.x;
            obs.y = mk.pose.position.y;
            obs.lane = get_lane_at(obs.x, obs.y);
            obs.last_seen_sec = now_s;
        }
    }

    void LocalPathPubCpp::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        double now_s = this->now().seconds();
        double new_x = msg->pose.position.x;
        double new_y = msg->pose.position.y;

        // Yaw calculation
        double siny = 2.0 * (msg->pose.orientation.w * msg->pose.orientation.z + msg->pose.orientation.x * msg->pose.orientation.y);
        double cosy = 1.0 - 2.0 * (msg->pose.orientation.y * msg->pose.orientation.y + msg->pose.orientation.z * msg->pose.orientation.z);
        ego_yaw_ = std::atan2(siny, cosy);

        // Speed calculation
        if (prev_pose_time_ > 0)
        {
            double dt = now_s - prev_pose_time_;
            if (dt > 0.01 && dt < 0.5)
            {
                double dist = get_dist(new_x, new_y, ego_x_, ego_y_);
                double raw_speed = dist / dt;
                // Simple Filter
                ego_speed_ = ego_speed_ * 0.7 + raw_speed * 0.3;
            }
        }
        ego_x_ = new_x;
        ego_y_ = new_y;
        prev_pose_time_ = now_s;
        pose_received_ = true;
    }

} // namespace bisa

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bisa::LocalPathPubCpp>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
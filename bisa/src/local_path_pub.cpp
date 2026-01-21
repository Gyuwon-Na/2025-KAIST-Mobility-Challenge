/**
 * @file local_path_pub.cpp
 * @brief Force Lane 2 Tracking + NO Corner Slowdown (Raw Speed)
 */

#include "bisa/local_path_pub.hpp"
#include <limits>
#include <algorithm>
#include <cmath>

namespace bisa
{
    // 디버그용 마커 인덱스 (예: 합류 구간)
    static const int FIXED_MERGE_IDX = 1500;

    LocalPathPubCpp::LocalPathPubCpp() : Node("local_path_pub")
    {
        RCLCPP_INFO(this->get_logger(), "============================================");
        RCLCPP_INFO(this->get_logger(), "Local Path: LANE 2 ONLY | RAW SPEED MODE");
        RCLCPP_INFO(this->get_logger(), " - Corner Slowdown: REMOVED");
        RCLCPP_INFO(this->get_logger(), " - Velocity: Strictly follows env_slow_vel");
        RCLCPP_INFO(this->get_logger(), "============================================");

        auto qos = rclcpp::QoS(10).transient_local();

        // --------------------------------------------------------
        // Subscribers
        // --------------------------------------------------------
        sub_lane_[0] = this->create_subscription<nav_msgs::msg::Path>(
            "/hdmap/lane_one", qos, [this](nav_msgs::msg::Path::SharedPtr msg)
            { lane_paths_[0] = msg; process_lane_path(0); });

        // [핵심] Lane 2 데이터 수신
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

        // [속도 수신] visualize_hvs에서 계산된 속도 수신
        sub_env_slow_ = this->create_subscription<std_msgs::msg::Float32>(
            "/env/slow_vel", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            {
                // 유효한 값(0.1 이상)만 업데이트
                if (msg->data > 0.1)
                {
                    env_slow_vel_ = msg->data;
                }
            });

        // --------------------------------------------------------
        // Publishers
        // --------------------------------------------------------
        pub_local_path_ = this->create_publisher<nav_msgs::msg::Path>("/local_path", 10);
        pub_target_vel_ = this->create_publisher<std_msgs::msg::Float32>("/planning/target_v", 10);
        pub_debug_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/markers", 10);

        // Timer (50Hz)
        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&LocalPathPubCpp::control_loop, this));
    }

    // ========================================================================
    // UTILS
    // ========================================================================
    double LocalPathPubCpp::get_dist(double x1, double y1, double x2, double y2)
    {
        return std::hypot(x1 - x2, y1 - y2);
    }

    // [최적화] 인덱스 추적 및 자동 복구 (멈춤 현상 해결)
    int LocalPathPubCpp::find_closest_idx_forward(int lane_idx, double x, double y)
    {
        if (processed_lanes_[lane_idx].empty())
            return 0;

        int path_size = processed_lanes_[lane_idx].size();

        // 1. 초기화 또는 리셋 상태 (전역 검색)
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

        // 2. 주행 중 Window Search (범위 400으로 확장)
        int search_len = 400;
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

        // [안전장치] 만약 가장 가까운 점도 3m 이상 멀어졌다면 -> 트래킹 실패로 간주하고 리셋
        if (min_d > 3.0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Tracking Lost (Dist: %.2fm). Resetting Search...", min_d);
            last_closest_idx_ = -1; // 다음 루프에서 전역 검색 수행
            return best_idx;
        }

        last_closest_idx_ = best_idx;
        return best_idx;
    }

    LaneID LocalPathPubCpp::get_lane_at(double x, double y)
    {
        return LaneID::LANE_2;
    }

    bool LocalPathPubCpp::check_zone_safety(double center_x, double center_y, double radius, LaneID target_lane)
    {
        return true;
    }

    // ========================================================================
    // MAIN LOOP
    // ========================================================================
    void LocalPathPubCpp::control_loop()
    {
        if (!pose_received_)
            return;

        // Lane 2 (Index 1) 데이터 체크
        if (processed_lanes_[1].empty())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for Lane 2 Data...");
            return;
        }

        // 1. 무조건 Lane 2 타겟 설정
        int target_lane_idx = 1;

        // 2. 현재 위치 인덱스 찾기
        int ego_idx = find_closest_idx_forward(target_lane_idx, ego_x_, ego_y_);

        // --------------------------------------------------------------------
        // [수정 완료] 속도 설정 (코너 감속 로직 제거)
        // --------------------------------------------------------------------
        // 곡률 계산, 코너링 감속 등 모든 로직을 배제하고
        // 오직 수신된 env_slow_vel 값만 사용합니다.

        double target_vel = env_slow_vel_;

        // (안전장치) 만약 토픽이 아직 안 들어왔거나 0이라면 기본 속도 1.0 부여 (멈춤 방지)
        if (target_vel < 0.1)
        {
            target_vel = 1.0;
        }

        // 3. Local Path 생성
        const auto &target_lane = processed_lanes_[target_lane_idx];
        int path_size = target_lane.size();

        nav_msgs::msg::Path local_path_msg;
        local_path_msg.header.frame_id = "world";
        local_path_msg.header.stamp = this->now();

        // 현재 속도에 맞춰 Lookahead 거리 조절 (너무 짧으면 MPC가 불안정할 수 있음)
        double current_speed = std::abs(ego_speed_);
        int lookahead_steps = std::max(40, static_cast<int>((current_speed * 1.5) / 0.1));
        lookahead_steps = std::min(lookahead_steps, 150);

        // MPC가 바로 추종할 수 있게 현재 위치보다 살짝 앞(+2)부터 경로 제공
        int start_offset = 2;

        for (int i = 0; i < lookahead_steps; ++i)
        {
            int idx = (ego_idx + start_offset + i) % path_size;

            geometry_msgs::msg::PoseStamped pose;
            pose.header = local_path_msg.header;
            pose.pose.position.x = target_lane[idx].x;
            pose.pose.position.y = target_lane[idx].y;
            pose.pose.orientation.w = 1.0;
            local_path_msg.poses.push_back(pose);
        }

        pub_local_path_->publish(local_path_msg);

        // 4. 목표 속도 발행
        std_msgs::msg::Float32 v_msg;
        v_msg.data = target_vel;
        pub_target_vel_->publish(v_msg);
    }

    // ========================================================================
    // CALLBACKS
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
        // Lane 2 강제 추종 모드이므로 장애물 회피 로직은 생략
    }

    void LocalPathPubCpp::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        double now_s = this->now().seconds();
        double new_x = msg->pose.position.x;
        double new_y = msg->pose.position.y;

        if (prev_pose_time_ > 0)
        {
            double dt = now_s - prev_pose_time_;
            if (dt > 0.01 && dt < 0.5)
            {
                double dist = get_dist(new_x, new_y, ego_x_, ego_y_);
                double raw_speed = dist / dt;
                ego_speed_ = ego_speed_ * 0.7 + raw_speed * 0.3; // Low-pass filter
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
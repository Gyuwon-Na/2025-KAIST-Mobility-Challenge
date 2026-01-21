/**
 * @file local_path_pub.cpp
 * @brief Force Lane 2 + Chase Logic (Speed up when closer than n)
 */

#include "bisa/local_path_pub.hpp"
#include <limits>
#include <algorithm>
#include <cmath>

namespace bisa
{
    // [설정] 추격(Chase) 거리 설정
    static const double CHASE_DIST_N = 1.0;  // "n (m) 이하"의 기준 값
    static const double SAFETY_MARGIN = 0.6; // 최소 안전 거리

    LocalPathPubCpp::LocalPathPubCpp() : Node("local_path_pub")
    {
        RCLCPP_INFO(this->get_logger(), "============================================");
        RCLCPP_INFO(this->get_logger(), "Local Path: CHASE MODE ACTIVATED");
        RCLCPP_INFO(this->get_logger(), " - If dist <= %.1fm : Speed UP (x1.2)", CHASE_DIST_N);
        RCLCPP_INFO(this->get_logger(), " - If dist <= %.1fm : Speed DOWN (Safety)", SAFETY_MARGIN);
        RCLCPP_INFO(this->get_logger(), "============================================");

        auto qos = rclcpp::QoS(10).transient_local();

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
            {
                if (msg->data > 0.1)
                    env_slow_vel_ = msg->data;
            });

        pub_local_path_ = this->create_publisher<nav_msgs::msg::Path>("/local_path", 10);
        pub_target_vel_ = this->create_publisher<std_msgs::msg::Float32>("/planning/target_v", 10);
        pub_debug_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/markers", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&LocalPathPubCpp::control_loop, this));
    }

    // ========================================================================
    // UTILS
    // ========================================================================
    double LocalPathPubCpp::get_dist(double x1, double y1, double x2, double y2)
    {
        return std::hypot(x1 - x2, y1 - y2);
    }

    int LocalPathPubCpp::find_closest_idx_forward(int lane_idx, double x, double y)
    {
        if (processed_lanes_[lane_idx].empty())
            return 0;
        int path_size = processed_lanes_[lane_idx].size();

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

        if (min_d > 3.0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Tracking Lost (Dist: %.2fm). Reset...", min_d);
            last_closest_idx_ = -1;
            return best_idx;
        }

        last_closest_idx_ = best_idx;
        return best_idx;
    }

    // [New] 전방 장애물 거리 계산 함수
    double LocalPathPubCpp::calc_front_dist(int lane_idx, int ego_idx)
    {
        double min_front_dist = 999.0;
        const auto &lane_pts = processed_lanes_[lane_idx];
        int path_size = lane_pts.size();

        // 현재 Lane 2에 매핑된 장애물만 검색하지 않고,
        // 편의상 등록된 모든 장애물과 Ego 위치를 비교하여 전방 판단
        for (auto const &[id, obs] : obstacles_)
        {
            // 1. Ego와의 거리
            double dist = get_dist(ego_x_, ego_y_, obs.x, obs.y);

            // 2. 벡터 내적을 통한 전방 판별 (간이 방식)
            double dx = obs.x - ego_x_;
            double dy = obs.y - ego_y_;
            double ego_yaw_vec_x = std::cos(ego_yaw_);
            double ego_yaw_vec_y = std::sin(ego_yaw_);

            double dot = dx * ego_yaw_vec_x + dy * ego_yaw_vec_y;

            // 전방에 있고(dot > 0), 거리가 가까우면
            if (dot > 0 && dist < min_front_dist)
            {
                min_front_dist = dist;
            }
        }
        return min_front_dist;
    }

    // [Warning Fix] 사용하지 않는 파라미터 처리
    LaneID LocalPathPubCpp::get_lane_at(double x, double y)
    {
        (void)x;
        (void)y;
        return LaneID::LANE_2;
    }

    bool LocalPathPubCpp::check_zone_safety(double center_x, double center_y, double radius, LaneID target_lane)
    {
        (void)center_x;
        (void)center_y;
        (void)radius;
        (void)target_lane;
        return true;
    }

    // ========================================================================
    // MAIN LOOP
    // ========================================================================
    void LocalPathPubCpp::control_loop()
    {
        if (!pose_received_)
            return;
        if (processed_lanes_[1].empty())
            return;

        int target_lane_idx = 1; // Lane 2
        int ego_idx = find_closest_idx_forward(target_lane_idx, ego_x_, ego_y_);

        // 1. 기본 속도 설정
        double target_vel = (env_slow_vel_ > 0.1) ? env_slow_vel_ : 1.0;

        // 2. [요청사항] n (m) 이하일 때 속도 증가 로직 (Chase Logic)
        double front_dist = calc_front_dist(target_lane_idx, ego_idx);

        // n 미터 안으로 들어오면?
        if (front_dist <= CHASE_DIST_N)
        {
            if (front_dist > SAFETY_MARGIN)
            {
                // [Chase Mode] 안전 거리보다는 멀고, n보다는 가까움 -> 바짝 붙기 위해 속도 증가!
                // 예: n=15m, 현재=10m -> 1.2배 가속
                target_vel *= 1.2;

                // 디버깅 로그
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "CHASE MODE: Dist %.1fm <= %.1fm | Speed UP (%.1f -> %.1f)",
                                     front_dist, CHASE_DIST_N, env_slow_vel_, target_vel);
            }
        }
        // 만약 n보다 멀다면 그냥 기본 env_slow_vel 유지

        // 3. Path Generation
        const auto &target_lane = processed_lanes_[target_lane_idx];
        int path_size = target_lane.size();

        nav_msgs::msg::Path local_path_msg;
        local_path_msg.header.frame_id = "world";
        local_path_msg.header.stamp = this->now();

        double current_speed = std::abs(ego_speed_);
        int lookahead_steps = std::max(40, static_cast<int>((current_speed * 1.5) / 0.1));
        lookahead_steps = std::min(lookahead_steps, 150);
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

        // 4. Target Velocity Publish
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
        // [Warning Fix] 파라미터 사용
        (void)msg;

        // 실제 장애물 정보 업데이트 (calc_front_dist에서 사용하기 위해 저장)
        double now_s = this->now().seconds();
        for (const auto &mk : msg->markers)
        {
            if (mk.type != visualization_msgs::msg::Marker::CUBE)
                continue;
            int id = mk.id;
            ObstacleInfo &obs = obstacles_[id];

            obs.x = mk.pose.position.x;
            obs.y = mk.pose.position.y;
            obs.last_seen_sec = now_s;
        }
    }

    void LocalPathPubCpp::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        double now_s = this->now().seconds();
        double new_x = msg->pose.position.x;
        double new_y = msg->pose.position.y;

        // Yaw from Quaternion
        double qx = msg->pose.orientation.x;
        double qy = msg->pose.orientation.y;
        double qz = msg->pose.orientation.z;
        double qw = msg->pose.orientation.w;
        double siny = 2.0 * (qw * qz + qx * qy);
        double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
        ego_yaw_ = std::atan2(siny, cosy);

        if (prev_pose_time_ > 0)
        {
            double dt = now_s - prev_pose_time_;
            if (dt > 0.01 && dt < 0.5)
            {
                double dist = get_dist(new_x, new_y, ego_x_, ego_y_);
                double raw_speed = dist / dt;
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
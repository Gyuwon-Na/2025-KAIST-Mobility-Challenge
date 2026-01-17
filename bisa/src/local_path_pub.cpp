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
    LocalPathPubCpp::LocalPathPubCpp() : Node("local_path_pub"), lap_start_time_(this->now())
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
        pub_lap_info_ = this->create_publisher<bisa::msg::LapInfo>("/lap_information", 10);

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

    bool LocalPathPubCpp::in_merge_gate(int idx)
    {
        return (idx >= FIXED_MERGE_IDX - MERGE_ZONE_THRESHOLD && idx <= FIXED_MERGE_IDX);
    }

    bool LocalPathPubCpp::in_split_gate(int idx)
    {
        return (idx >= FIXED_SPLIT_IDX && idx < FIXED_SPLIT_IDX + MERGE_ZONE_THRESHOLD);
    }

    bool LocalPathPubCpp::in_merge_zone(int idx)
    {
        return (idx > FIXED_MERGE_IDX && idx < FIXED_SPLIT_IDX);
    }

    MergeGapInfo LocalPathPubCpp::analyze_merge_gap()
    {
        MergeGapInfo info;
        info.gap_available = true; // 기본: 통과 허용
        info.recommended_vel = (env_slow_vel_ > 0.1) ? env_slow_vel_ : 1.5;

        if (processed_lanes_[2].empty() || processed_lanes_[1].empty())
            return info;

        // 1. 합류 지점 좌표
        double merge_x = processed_lanes_[2][FIXED_MERGE_IDX].x;
        double merge_y = processed_lanes_[2][FIXED_MERGE_IDX].y;

        // 2. Lane 2에서 합류점 매칭
        int lane2_merge_idx = 0;
        double min_d = 999.0;
        for (size_t i = 0; i < processed_lanes_[1].size(); i += 5)
        {
            double d = get_dist(merge_x, merge_y, processed_lanes_[1][i].x, processed_lanes_[1][i].y);
            if (d < min_d)
            {
                min_d = d;
                lane2_merge_idx = i;
            }
        }

        double ref_x = processed_lanes_[1][lane2_merge_idx].x;
        double ref_y = processed_lanes_[1][lane2_merge_idx].y;

        // Lane 2 방향 계산 (다음 점과의 차이로)
        int next_idx = std::min(lane2_merge_idx + 5, (int)processed_lanes_[1].size() - 1);
        double lane2_dx = processed_lanes_[1][next_idx].x - ref_x;
        double lane2_dy = processed_lanes_[1][next_idx].y - ref_y;
        double lane2_yaw = std::atan2(lane2_dy, lane2_dx);
        double l2_cos = std::cos(lane2_yaw);
        double l2_sin = std::sin(lane2_yaw);

        // 3. Ego가 합류 지점까지의 거리/시간
        double ego_to_merge = get_dist(ego_x_, ego_y_, merge_x, merge_y);
        double ego_speed_safe = std::max(ego_speed_, 0.5); // 최소 속도 가정
        double ego_tta = ego_to_merge / ego_speed_safe;    // Time To Arrival

        double closest_box_dist = 999.0;
        int danger_obs_id = -1;
        double danger_local_x = 0.0;
        double danger_local_y = 0.0;

        // 4. Lane 2 차량 탐색
        double closest_front_dist = 999.0;
        double closest_rear_dist = 999.0;
        double front_vel = 0.0;
        double rear_vel = 0.0;

        for (const auto &[id, obs] : obstacles_)
        {
            if (obs.lane != LaneID::LANE_2)
                continue;

            // ================================================================
            // ★ 직사각형 안전 영역 체크 (Ego 로컬 좌표 기준)
            // ================================================================
            double dx_ego = obs.x - ego_x_;
            double dy_ego = obs.y - ego_y_;

            // Ego 기준 로컬 좌표 변환
            double local_x = dx_ego * std::cos(ego_yaw_) + dy_ego * std::sin(ego_yaw_);
            double local_y = -dx_ego * std::sin(ego_yaw_) + dy_ego * std::cos(ego_yaw_);

            // 직사각형 영역 체크: 전방 SAFE_FRONT, 후방 SAFE_REAR, 측면 SAFE_LATERAL
            bool in_front_range = (local_x > -SAFE_REAR && local_x < SAFE_FRONT);
            bool in_lateral_range = (std::abs(local_y) < SAFE_LATERAL);

            if (in_front_range && in_lateral_range)
            {
                double dist = std::hypot(dx_ego, dy_ego);
                if (dist < closest_box_dist)
                {
                    closest_box_dist = dist;
                    danger_obs_id = id;
                    danger_local_x = local_x;
                    danger_local_y = local_y;
                }
            }

            // ================================================================
            // 합류점 기준 앞/뒤 체크 (기존 로직)
            // ================================================================
            double dx = obs.x - ref_x;
            double dy = obs.y - ref_y;
            double dist_long = dx * l2_cos + dy * l2_sin;

            if (std::abs(dist_long) > MERGE_LOOKAHEAD)
                continue;

            double current_obs_speed = std::hypot(obs.vx, obs.vy);

            if (dist_long > 0) // 앞차
            {
                if (dist_long < closest_front_dist)
                {
                    closest_front_dist = dist_long;
                    front_vel = current_obs_speed;
                }
            }
            else // 뒷차
            {
                double abs_dist = std::abs(dist_long);
                if (abs_dist < closest_rear_dist)
                {
                    closest_rear_dist = abs_dist;
                    rear_vel = current_obs_speed;
                }
            }
        }

        info.front_dist = closest_front_dist;
        info.rear_dist = closest_rear_dist;
        info.front_vel = front_vel;
        info.rear_vel = rear_vel;

        // =================================================================
        // [핵심] TTC 기반 판단 - "지금 속도로 가면 부딪히나?"
        // =================================================================

        double flow_speed = (env_slow_vel_ > 0.1) ? env_slow_vel_ : 1.5;

        // =================================================================
        // ★ [1단계] 직사각형 영역 안에 차가 있으면 무조건 대기
        // =================================================================
        if (closest_box_dist < 999.0)
        {
            info.gap_available = false;

            // 거리에 따른 속도 결정
            if (closest_box_dist < 0.35)
            {
                // 매우 가까움: 정지
                info.recommended_vel = 0.0;
                RCLCPP_ERROR(this->get_logger(),
                             "[MERGE] DANGER ZONE! obs_id=%d at (%.2f, %.2f), dist=%.2f -> STOP",
                             danger_obs_id, danger_local_x, danger_local_y, closest_box_dist);
            }
            else if (closest_box_dist < 0.5)
            {
                // 가까움: 매우 느리게
                info.recommended_vel = 0.3;
                RCLCPP_WARN(this->get_logger(),
                            "[MERGE] CAUTION! obs_id=%d at (%.2f, %.2f), dist=%.2f -> SLOW",
                            danger_obs_id, danger_local_x, danger_local_y, closest_box_dist);
            }
            else
            {
                // 영역 내지만 여유 있음: 감속하며 대기
                info.recommended_vel = 0.5;
                RCLCPP_WARN(this->get_logger(),
                            "[MERGE] WAITING! obs_id=%d at (%.2f, %.2f), dist=%.2f",
                            danger_obs_id, danger_local_x, danger_local_y, closest_box_dist);
            }

            return info; // 직사각형 체크가 최우선
        }

        // =================================================================
        // ★ [2단계] 직사각형 영역 밖이면 TTC 기반 판단
        // =================================================================
        bool front_ok = true;
        if (closest_front_dist < 999.0)
        {
            double front_moves = front_vel * ego_tta;
            double expected_front_gap = closest_front_dist + front_moves;
            front_ok = (expected_front_gap > 0.5);
        }

        bool rear_ok = true;
        if (closest_rear_dist < 999.0)
        {
            double rear_moves = rear_vel * ego_tta;
            double expected_rear_gap = closest_rear_dist - rear_moves + (flow_speed * ego_tta);
            rear_ok = (expected_rear_gap > 0.4);
        }

        // =================================================================
        // 최종 판단
        // =================================================================
        if (front_ok && rear_ok)
        {
            info.gap_available = true;
            info.recommended_vel = flow_speed;

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "[MERGE] CLEAR - front=%.2f, rear=%.2f, vel=%.2f",
                                 closest_front_dist, closest_rear_dist, info.recommended_vel);
        }
        else
        {
            info.gap_available = false;

            if (!front_ok)
            {
                // ★ 최소 안전 거리 이하면 무조건 정지
                if (info.front_dist <= MARGIN_FRONT)
                {
                    info.recommended_vel = 0.0;
                    RCLCPP_ERROR(this->get_logger(),
                                 "[MERGE] EMERGENCY STOP! front_dist=%.2f <= margin=%.2f",
                                 info.front_dist, MARGIN_FRONT);
                }
                else if (info.front_dist <= MARGIN_FRONT * 2)
                {
                    // 마진의 2배 이하면 매우 느리게
                    info.recommended_vel = 0.3;
                    RCLCPP_WARN(this->get_logger(),
                                "[MERGE] Front very close! dist=%.2f, vel=0.3", info.front_dist);
                }
                else
                {
                    // 그 외는 거리 비례 감속
                    double slow_factor = std::clamp(info.front_dist / 1.0, 0.3, 0.6);
                    info.recommended_vel = std::max(0.5, flow_speed * slow_factor);
                    RCLCPP_WARN(this->get_logger(),
                                "[MERGE] Front danger! dist=%.2f, vel=%.2f",
                                info.front_dist, info.recommended_vel);
                }
            }
            else if (!rear_ok)
            {
                info.recommended_vel = std::min(2.0, flow_speed * 1.3);
            }
        }

        return info;
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

        // --------------------------------------------------------------------
        // Determine Velocity (안전성 + 속도 결정)
        // 속도 발행하기 전 경우에 따라 나눠서 처리
        // --------------------------------------------------------------------
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
        bool is_in_merge_zone_ = in_merge_zone(ego_idx);
        bool is_in_merge_gate_ = in_merge_gate(ego_idx);
        bool is_in_split_gate_ = in_split_gate(ego_idx);

        for (auto const &[id, obs] : obstacles_)
        {
            if (obs.lane != current_lane_id)
                continue;

            double dx = obs.x - ego_x_;
            double dy = obs.y - ego_y_;
            // 로컬 좌표계 변환 (전방 거리)
            double local_x = dx * std::cos(ego_yaw_) + dy * std::sin(ego_yaw_);

            // 전방 0.0m ~ 2.0m 탐색
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
                if (!is_in_merge_zone_ && !is_in_merge_gate_)
                {
                    target_vel = 2.0;
                }

                // [Case 2-2] 합류 게이트 구간이면??  (= 합류할 시점 (merge index - 10 ~ merge index))
                if (is_in_merge_gate_ || is_in_split_gate_)
                {
                    MergeGapInfo gap_info = analyze_merge_gap();

                    if (gap_info.gap_available)
                    {
                        // Gap 확보됨 → 권장 속도로 합류 준비
                        target_vel = gap_info.recommended_vel;
                        RCLCPP_DEBUG(this->get_logger(),
                                     "[MERGE GATE] Gap OK: size=%.2f, front=%.2f, rear=%.2f, vel=%.2f",
                                     gap_info.gap_size, gap_info.front_dist, gap_info.rear_dist, target_vel);
                    }
                    else
                    {
                        // Gap 부족 → 속도 줄여서 다음 gap 대기
                        target_vel = gap_info.recommended_vel;
                        RCLCPP_DEBUG(this->get_logger(),
                                     "[MERGE GATE] Gap insufficient: size=%.2f, waiting...", gap_info.gap_size);
                    }
                }

                // [Case 2-3] 합류 구간이면??
                if (is_in_merge_zone_)
                {
                    // 합류했는데 장애물 있으면 상관 없지 않나? 이 때는 그냥 차선 그룹의 속도랑 유지하면 되지 않나?
                    // 앞차와의 거리가 너무 가까우면 감속 (대신 뒷차와의 safe 거리도 고려하여 감속해야함)
                    if (min_dist_ahead <= 0.2)
                    {
                        // 매우 가까움: 정지
                        target_vel = 0.0;
                        RCLCPP_ERROR(this->get_logger(),
                                     "[MERGE ZONE] Extremely close! dist=%.2f, STOP!", min_dist_ahead);
                    }
                    if (min_dist_ahead <= 0.3)
                    {
                        // 너무 가까움: 강하게 감속해서 간격 벌리기
                        target_vel = follow_speed * 0.5;
                        RCLCPP_WARN(this->get_logger(),
                                    "[MERGE ZONE] Too close! dist=%.2f, slowing to %.2f",
                                    min_dist_ahead, target_vel);
                    }
                    else if (min_dist_ahead <= 0.5)
                    {
                        // 가까움: 약간 감속
                        target_vel = follow_speed * 0.8;
                    }
                    else if (min_dist_ahead <= 0.8)
                    {
                        // 적정 거리: 그룹 속도 유지
                        target_vel = follow_speed;
                    }
                    else
                    {
                        // 여유 있음: 약간 가속해서 간격 좁히기 (분기 대비)
                        target_vel = std::min(follow_speed * 1.1, 2.0);
                    }
                }
            }
        }
        else
        {
            // [Case 3] 장애물 없음
            // [Case 3-1] 합류 구간이 아닐 때만 2.0으로 증속
            if (!is_in_merge_zone_ && (!is_in_merge_gate_ || !is_in_split_gate_))
            {
                target_vel = 2.0;
            }

            // [Case 3-2] 분기 게이트 구간이면?? (= 분기할 시점 (split index ~ split index + 10))
            if (is_in_split_gate_)
            {
                MergeGapInfo gap_info = analyze_merge_gap();

                if (gap_info.gap_available)
                {
                    // Gap 확보됨 → 권장 속도로 합류 준비
                    target_vel = gap_info.recommended_vel;
                    RCLCPP_DEBUG(this->get_logger(),
                                 "[SPLIT GATE] Gap OK: size=%.2f, front=%.2f, rear=%.2f, vel=%.2f",
                                 gap_info.gap_size, gap_info.front_dist, gap_info.rear_dist, target_vel);
                }
                else
                {
                    // Gap 부족 → 속도 줄여서 다음 gap 대기
                    target_vel = gap_info.recommended_vel;
                    RCLCPP_DEBUG(this->get_logger(),
                                 "[SPLIT GATE] Gap insufficient: size=%.2f, waiting...", gap_info.gap_size);
                }
            }
        }

        const auto &lane3 = processed_lanes_[2];
        int path_size = lane3.size();

        // --------------------------------------------------------------------
        // Path Generation (최소 길이 보장)
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

        publish_lap_info();
        // --------------------------------------------------------------------
        // Visualization (시각화)
        // 고정 포인트 표시 (제대로 작동하는지 눈으로 확인용)
        // --------------------------------------------------------------------
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
            double dx = new_x - ego_x_; // ego_x_는 갱신 전이므로 이전 위치임
            double dy = new_y - ego_y_;

            total_distance_ += std::hypot(dx, dy);

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

    int LocalPathPubCpp::count_lap_idx(int lane_idx, double x, double y)
    {
        if (processed_lanes_[lane_idx].empty())
            return 0;

        int n = processed_lanes_[lane_idx].size();
        double min_d = 1e9;
        int best_idx = 0;

        // 전체 포인트를 다 뒤져서 가장 가까운 점 찾기 (Global Search)
        for (int i = 0; i < n; ++i)
        {
            double d = get_dist(x, y, processed_lanes_[lane_idx][i].x, processed_lanes_[lane_idx][i].y);
            if (d < min_d)
            {
                min_d = d;
                best_idx = i;
            }
        }
        return best_idx;
    }

    void LocalPathPubCpp::publish_lap_info()
    {
        // 기준: Lane 2 (중앙 차선)
        if (processed_lanes_[1].empty() || !pose_received_)
            return;

        size_t total = processed_lanes_[1].size();
        if (total == 0)
            return;

        // Lane 2 기준 현재 위치에 가장 가까운 웨이포인트 찾기
        // (Lane 1이나 3에 있어도 Lane 2 기준으로 진척도 계산)
        int current_track_idx = count_lap_idx(1, ego_x_, ego_y_);

        // 한 바퀴 완료 감지 (인덱스 급감 & 90% 이상 진행 상태에서 발생)
        if (current_track_idx < prev_track_idx_ && prev_track_idx_ > total * 0.9)
        {
            lap_count_++;
            lap_start_time_ = this->now();
            RCLCPP_INFO(this->get_logger(),
                        "[LAP] Lap %d completed! Total dist: %.2fm",
                        lap_count_, total_distance_);
        }
        prev_track_idx_ = current_track_idx;

        double progress = (static_cast<double>(current_track_idx) / total) * 100.0;
        double elapsed_time = (this->now() - lap_start_time_).seconds();

        auto lap_info = bisa::msg::LapInfo();
        lap_info.lap_count = lap_count_;
        lap_info.progress = static_cast<float>(progress);
        lap_info.current_waypoint = static_cast<int32_t>(current_track_idx);
        lap_info.total_waypoints = static_cast<int32_t>(total);
        lap_info.elapsed_time = static_cast<float>(elapsed_time);
        lap_info.total_distance = static_cast<float>(total_distance_);

        pub_lap_info_->publish(lap_info);
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
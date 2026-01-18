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
        return (idx >= FIXED_SPLIT_IDX - SPLIT_THRESHOLD && idx < FIXED_SPLIT_IDX + SPLIT_THRESHOLD);
    }

    bool LocalPathPubCpp::in_merge_zone(int idx)
    {
        return (idx > FIXED_MERGE_IDX && idx < FIXED_SPLIT_IDX - SPLIT_THRESHOLD);
    }

    bool LocalPathPubCpp::is_fast_hv(int id)
    {
        return FAST_HV_IDS.find(id) != FAST_HV_IDS.end();
    }

    MergeGapInfo LocalPathPubCpp::analyze_gap(LaneID target_lane)
    {
        MergeGapInfo info;
        info.gap_available = true; // 기본: 통과 허용
        info.recommended_vel = (env_slow_vel_ > 0.1) ? env_slow_vel_ : 1.5;

        if (processed_lanes_[2].empty() || processed_lanes_[1].empty())
            return info;

        // ★ 목표 차선에 따라 게이트 인덱스 결정
        int gate_idx = (target_lane == LaneID::LANE_2) ? FIXED_MERGE_IDX : FIXED_SPLIT_IDX;
        int target_lane_idx = (target_lane == LaneID::LANE_2) ? 1 : 2; // processed_lanes 인덱스

        // 1. 게이트 지점 좌표 (Lane 3 기준)
        double gate_x = processed_lanes_[2][gate_idx].x;
        double gate_y = processed_lanes_[2][gate_idx].y;

        // 2. 목표 차선에서 게이트점 매칭
        int target_lane_gate_idx = 0;
        double min_d = 999.0;
        for (size_t i = 0; i < processed_lanes_[target_lane_idx].size(); i += 5)
        {
            double d = get_dist(gate_x, gate_y,
                                processed_lanes_[target_lane_idx][i].x,
                                processed_lanes_[target_lane_idx][i].y);
            if (d < min_d)
            {
                min_d = d;
                target_lane_gate_idx = i;
            }
        }

        double ref_x = processed_lanes_[target_lane_idx][target_lane_gate_idx].x;
        double ref_y = processed_lanes_[target_lane_idx][target_lane_gate_idx].y;

        // 목표 차선 방향 계산
        int next_idx = std::min(target_lane_gate_idx + 5, (int)processed_lanes_[target_lane_idx].size() - 1);
        double lane_dx = processed_lanes_[target_lane_idx][next_idx].x - ref_x;
        double lane_dy = processed_lanes_[target_lane_idx][next_idx].y - ref_y;
        double lane_yaw = std::atan2(lane_dy, lane_dx);
        double l_cos = std::cos(lane_yaw);
        double l_sin = std::sin(lane_yaw);

        // 3. Ego가 합류 지점까지의 거리/시간
        double ego_to_gate = get_dist(ego_x_, ego_y_, gate_x, gate_y);
        double ego_speed_safe = std::max(ego_speed_, 0.5);
        double ego_tta = ego_to_gate / ego_speed_safe;

        // 4. 목표 차선 차량 탐색
        double closest_front_dist = 999.0;
        double closest_rear_dist = 999.0;
        double closest_box_dist = 999.0;
        double front_vel = 0.0;
        double rear_vel = 0.0;
        int danger_obs_id = -1;
        double danger_local_x = 0.0;
        double danger_local_y = 0.0;

        for (const auto &[id, obs] : obstacles_)
        {
            if (obs.lane != target_lane)
                continue;

            // [핵심 수정] Ego 차량 기준의 로컬 좌표를 사용하여 앞/뒤 판별
            // 차량이 왼쪽(-x)으로 가고 있더라도 local_x > 0이면 내 진행 방향 앞입니다.
            double dx_ego = obs.x - ego_x_;
            double dy_ego = obs.y - ego_y_;
            double local_x = dx_ego * std::cos(ego_yaw_) + dy_ego * std::sin(ego_yaw_);
            double local_y = -dx_ego * std::sin(ego_yaw_) + dy_ego * std::cos(ego_yaw_);

            // 주변 장애물(박스) 체크 (유지)
            bool in_front_range = (local_x > -SAFE_REAR && local_x < SAFE_FRONT);
            bool in_lateral_range = (std::abs(local_y) < SAFE_LATERAL);

            if (in_front_range && in_lateral_range)
            {
                double dist = std::hypot(dx_ego, dy_ego);
                if (dist < closest_box_dist)
                {
                    closest_box_dist = dist;
                    // danger_local_x 등 변수 업데이트 (필요 시)
                }
            }

            // 탐색 범위 제한 (합류 지점 전후 탐색)
            if (std::abs(local_x) > MERGE_LOOKAHEAD)
                continue;

            double current_obs_speed = std::hypot(obs.vx, obs.vy);

            // [판단 기준 변경] l_cos를 곱한 dist_long 대신 local_x 직접 사용
            if (local_x > 0) // 내 정면 (진행 방향 앞)
            {
                if (local_x < closest_front_dist)
                {
                    closest_front_dist = local_x;
                    front_vel = current_obs_speed;
                }
            }
            else // 내 후방 (진행 방향 뒤)
            {
                double abs_dist = std::abs(local_x);
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

        double flow_speed = (target_lane == LaneID::LANE_2)
                                ? ((env_slow_vel_ > 0.1) ? env_slow_vel_ : 1.5)
                                : ((env_fast_vel_ > 0.1) ? env_fast_vel_ : 2.0);

        // [2단계] TTC 기반 판단
        bool front_ok = true;
        if (closest_front_dist < 999.0)
        {
            double front_moves = front_vel * ego_tta;
            double expected_front_gap = closest_front_dist + front_moves;
            front_ok = (expected_front_gap > 0.4);
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
                                 "[GAP-%s] CLEAR - front=%.2f, rear=%.2f, vel=%.2f",
                                 (target_lane == LaneID::LANE_2) ? "MERGE" : "SPLIT",
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
                }
                else if (info.front_dist <= MARGIN_FRONT * 2)
                {
                    // 마진의 2배 이하면 매우 느리게
                    info.recommended_vel = 0.3;
                }
                else
                {
                    // 그 외는 거리 비례 감속
                    double slow_factor = std::clamp(info.front_dist / 1.0, 0.3, 0.6);
                    info.recommended_vel = std::max(0.5, flow_speed * slow_factor);
                }
            }
            else if (!rear_ok)
            {
                if (info.rear_dist <= MARGIN_REAR)
                {
                    info.recommended_vel = 0.0;
                }
                else
                    info.recommended_vel = std::min(2.0, 1.5 + (flow_speed * (info.rear_dist / 1.0)));
            }
            RCLCPP_WARN(this->get_logger(),
                        "[GAP-%s] ADJUST - front_ok=%s(%.2f), rear_ok=%s(%.2f), vel=%.2f",
                        (target_lane == LaneID::LANE_2) ? "MERGE" : "SPLIT",
                        front_ok ? "Y" : "N", info.front_dist,
                        rear_ok ? "Y" : "N", info.rear_dist,
                        info.recommended_vel);
        }

        return info;
    }
    // ========================================================================
    // [NEW] Split Gate 전용 Gap 분석 - Fast HV (ID 0~3) 집중 추적
    // ========================================================================
    MergeGapInfo LocalPathPubCpp::analyze_split_gap()
    {
        MergeGapInfo info;
        info.gap_available = true;
        info.recommended_vel = (env_fast_vel_ > 0.1) ? env_fast_vel_ : 2.0;

        if (processed_lanes_[2].empty())
            return info;

        // Split 지점 좌표
        double split_x = processed_lanes_[2][FIXED_SPLIT_IDX].x;
        double split_y = processed_lanes_[2][FIXED_SPLIT_IDX].y;

        // Ego → Split 지점까지 거리/시간
        double ego_to_split = get_dist(ego_x_, ego_y_, split_x, split_y);
        double ego_speed_safe = std::max(ego_speed_, 0.3);
        double ego_tta = ego_to_split / ego_speed_safe;

        // =========================================================
        // [핵심] Fast HV (ID 0~3)만 추적 - 위에서 내려오는 차량
        // =========================================================
        double closest_fast_hv_dist = 999.0;
        double fast_hv_vel = 0.0;
        int threatening_id = -1;
        double threatening_tta = 999.0; // Fast HV가 합류 지점에 도착하는 시간

        for (const auto &[id, obs] : obstacles_)
        {
            // Fast HV만 체크 (ID 0~3)
            if (!is_fast_hv(id))
                continue;

            // Fast HV → Split 지점까지 거리
            double hv_to_split = get_dist(obs.x, obs.y, split_x, split_y);
            double hv_speed = std::hypot(obs.vx, obs.vy);
            double hv_speed_safe = std::max(hv_speed, 0.5); // 최소 0.5로 가정
            double hv_tta = hv_to_split / hv_speed_safe;

            // Ego 기준 로컬 좌표 (디버깅용)
            double dx = obs.x - ego_x_;
            double dy = obs.y - ego_y_;
            double local_x = dx * std::cos(ego_yaw_) + dy * std::sin(ego_yaw_);
            double local_y = -dx * std::sin(ego_yaw_) + dy * std::cos(ego_yaw_);
            double dist_to_ego = std::hypot(dx, dy);

            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                                  "[SPLIT] Fast HV %d: to_split=%.2f, speed=%.2f, tta=%.2f, ego_tta=%.2f",
                                  id, hv_to_split, hv_speed, hv_tta, ego_tta);

            // 가장 위협적인 Fast HV 찾기 (합류 지점에 가장 빨리 도착하는 차량)
            if (hv_to_split < 3.0) // 3m 이내에 있는 Fast HV만
            {
                if (hv_tta < threatening_tta)
                {
                    threatening_tta = hv_tta;
                    threatening_id = id;
                    closest_fast_hv_dist = dist_to_ego;
                    fast_hv_vel = hv_speed;
                }
            }
        }

        // =========================================================
        // [판단] TTA 비교 - 누가 먼저 도착하나?
        // =========================================================
        if (threatening_id >= 0)
        {
            // 시간 마진 (Fast HV가 먼저 도착해도 0.5초 이상 여유 필요)
            double time_margin = 0.5;

            // Case 1: 내가 훨씬 먼저 도착 (안전)
            if (ego_tta + time_margin < threatening_tta)
            {
                info.gap_available = true;
                info.recommended_vel = 2.0; // 빠르게 통과
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                                     "[SPLIT] CLEAR - I arrive first (ego=%.2f, hv%d=%.2f)",
                                     ego_tta, threatening_id, threatening_tta);
            }
            // Case 2: Fast HV가 먼저 또는 거의 동시 도착 (위험)
            else if (threatening_tta < ego_tta + time_margin)
            {
                info.gap_available = false;

                // 거리에 따른 속도 조절
                if (closest_fast_hv_dist < 0.5)
                {
                    // 매우 가까움 → 정지
                    info.recommended_vel = 0.0;
                    RCLCPP_WARN(this->get_logger(),
                                "[SPLIT] STOP! Fast HV %d too close (%.2fm)",
                                threatening_id, closest_fast_hv_dist);
                }
                else if (closest_fast_hv_dist < 1.0)
                {
                    // 가까움 → 매우 느리게
                    info.recommended_vel = 0.3;
                    RCLCPP_WARN(this->get_logger(),
                                "[SPLIT] SLOW! Fast HV %d nearby (%.2fm, tta=%.2f)",
                                threatening_id, closest_fast_hv_dist, threatening_tta);
                }
                else
                {
                    // 여유 있음 → Fast HV 지나갈 때까지 대기 (느리게)
                    // Fast HV 속도에 맞춰서 뒤따라가기
                    info.recommended_vel = std::min(fast_hv_vel * 0.8, 1.0);
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                                         "[SPLIT] YIELD to Fast HV %d (dist=%.2f, following at %.2f)",
                                         threatening_id, closest_fast_hv_dist, info.recommended_vel);
                }
            }
            // Case 3: 애매함 → 보수적으로
            else
            {
                info.gap_available = true;
                info.recommended_vel = 1.5; // 중간 속도
            }

            info.front_dist = closest_fast_hv_dist;
            info.front_vel = fast_hv_vel;
        }
        else
        {
            // Fast HV 없음 → 안전하게 통과
            info.gap_available = true;
            info.recommended_vel = 2.0;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "[SPLIT] No Fast HV nearby - GO!");
        }

        // =========================================================
        // [추가] Lane 2 차량도 체크 (Split 전까지는 Lane 2에 있으므로)
        // =========================================================
        for (const auto &[id, obs] : obstacles_)
        {
            if (obs.lane != LaneID::LANE_2)
                continue;
            if (is_fast_hv(id)) // Fast HV는 위에서 처리함
                continue;

            double dx = obs.x - ego_x_;
            double dy = obs.y - ego_y_;
            double local_x = dx * std::cos(ego_yaw_) + dy * std::sin(ego_yaw_);

            // 전방 0~1.5m에 Lane 2 차량 있으면 감속
            if (local_x > 0.0 && local_x < 1.5)
            {
                double dist = std::hypot(dx, dy);
                if (dist < 0.5)
                {
                    info.recommended_vel = std::min(info.recommended_vel, 0.5);
                }
                else if (dist < 1.0)
                {
                    info.recommended_vel = std::min(info.recommended_vel, 1.0);
                }
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
        // Publish Local Path
        LaneID current_lane_id = get_lane_at(ego_x_, ego_y_);
        const auto &current_lane = processed_lanes_[static_cast<int>(current_lane_id)];
        int path_size = current_lane.size();

        // --------------------------------------------------------------------
        // Determine Velocity (안전성 + 속도 결정)
        // 속도 발행하기 전 경우에 따라 나눠서 처리
        // --------------------------------------------------------------------
        // 1. 현재 위치 찾기 (사용자님 로직 사용)

        // if (current_lane_id == LaneID::NONE || current_lane_id == LaneID::LANE_1)
        // {
        //     current_lane_id = LaneID::LANE_3;
        // }

        int ego_idx = find_closest_idx_forward(static_cast<int>(current_lane_id), ego_x_, ego_y_);

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

        // --------------------------------------------------------------------
        // 4. 속도 결정 (구간별 분기)
        // --------------------------------------------------------------------
        double target_vel = follow_speed;

        // [구간 1] Merge Zone (합류 구간 내부)
        if (is_in_merge_zone_)
        {
            if (obj_found)
            {
                // 앞뒤 장애물 거리 계산
                double front_dist = 999.0;
                double rear_dist = 999.0;

                for (auto const &[id, obs] : obstacles_)
                {
                    if (obs.lane != current_lane_id)
                        continue;

                    double dx = obs.x - ego_x_;
                    double dy = obs.y - ego_y_;
                    double local_x = dx * std::cos(ego_yaw_) + dy * std::sin(ego_yaw_);

                    if (local_x > 0.0 && local_x < front_dist)
                        front_dist = local_x;
                    else if (local_x < 0.0 && std::abs(local_x) < rear_dist)
                        rear_dist = std::abs(local_x);
                }

                // 앞뒤 간격 기반 속도 결정
                double vel_for_front = follow_speed;
                double vel_for_rear = follow_speed;

                // 앞차 간격 부족 → 감속
                if (front_dist < MERGE_ZONE_MIN_FRONT_GAP)
                    vel_for_front = follow_speed * 0.8;

                // 뒷차 간격 부족 → 가속
                if (rear_dist < MERGE_ZONE_MIN_REAR_GAP)
                    vel_for_rear = std::min(follow_speed * 1.3, 2.0);
                else if (rear_dist < MERGE_ZONE_MIN_REAR_GAP * 1.5)
                    vel_for_rear = std::min(follow_speed * 1.1, 2.0);

                // 두 조건을 만족하는 최적 속도 (앞차 우선)
                target_vel = std::min(vel_for_front, vel_for_rear);

                RCLCPP_DEBUG(this->get_logger(),
                             "[MERGE ZONE] front=%.2f, rear=%.2f, vel=%.2f",
                             front_dist, rear_dist, target_vel);
            }
            else
            {
                target_vel = follow_speed;
            }
        }
        // [구간 2] Merge Gate 또는 Split Gate
        else if (is_in_merge_gate_)
        {
            LaneID target_lane = is_in_merge_gate_ ? LaneID::LANE_2 : LaneID::LANE_3;
            MergeGapInfo gap_info = analyze_gap(target_lane);
            target_vel = gap_info.recommended_vel;

            RCLCPP_DEBUG(this->get_logger(),
                         "[GATE] gap=%.2f, available=%d, vel=%.2f",
                         gap_info.gap_size, gap_info.gap_available, target_vel);
        }
        else if (is_in_split_gate_)
        {
            MergeGapInfo gap_info = analyze_split_gap();
            target_vel = gap_info.recommended_vel;

            RCLCPP_DEBUG(this->get_logger(),
                         "[SPLIT GATE] fast_hv_dist=%.2f, available=%d, vel=%.2f",
                         gap_info.front_dist, gap_info.gap_available, target_vel);
        }
        // [구간 3] 일반 구간
        else
        {
            if (obj_found && min_dist_ahead < BOOST_DIST)
            {
                target_vel = follow_speed;
            }
            else
            {
                target_vel = 2.0;
            }
        }

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
            pose.pose.position.x = current_lane[idx].x;
            pose.pose.position.y = current_lane[idx].y;
            pose.pose.orientation.w = 1.0;
            local_path_msg.poses.push_back(pose);
        }

        pub_local_path_->publish(local_path_msg);

        std_msgs::msg::Float32 v_msg;
        v_msg.data = target_vel;
        pub_target_vel_->publish(v_msg);

        publish_lap_info();
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
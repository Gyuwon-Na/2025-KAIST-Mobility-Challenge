/**
 * @file local_path_pub.cpp
 * @brief Safety-First Local Path: Consistent Zone-Based Safety (Optimized)
 *
 * @optimization_summary
 * 1. normalize_angle: while 루프 → fmod 사용 (정밀도/일관성 향상)
 * 2. 삼각함수 캐싱: ego_cos_, ego_sin_ 사용하여 중복 계산 제거
 * 3. get_dist_sq 추가: 거리 비교 시 불필요한 sqrt 제거
 * 4. inline 함수화: 자주 호출되는 작은 함수들
 * 5. reserve() 사용: vector 재할당 방지
 * 6. 지역 변수 재사용: control_loop 내 중복 계산 제거
 */

#include "bisa/local_path_pub.hpp"
#include <algorithm>
#include <cmath>

namespace bisa
{

    // ============================================================================
    // CONSTRUCTOR
    // ============================================================================
    LocalPathPubCpp::LocalPathPubCpp()
        : Node("local_path_pub"), lap_start_time_(this->now())
    {
        RCLCPP_INFO(this->get_logger(), "============================================");
        RCLCPP_INFO(this->get_logger(), "Local Path: FIXED INDICES + ZONE SAFETY (Optimized)");
        RCLCPP_INFO(this->get_logger(), " - Merge Index: %d (+/- %d)", FIXED_MERGE_IDX, MERGE_THRESHOLD);
        RCLCPP_INFO(this->get_logger(), " - Split Index: %d (+/- %d)", FIXED_SPLIT_IDX, SPLIT_THRESHOLD);
        RCLCPP_INFO(this->get_logger(), "============================================");

        auto qos = rclcpp::QoS(10).transient_local();

        // Subscribers - Lane paths
        sub_lane_[0] = this->create_subscription<nav_msgs::msg::Path>(
            "/hdmap/lane_one", qos,
            [this](nav_msgs::msg::Path::SharedPtr msg)
            {
                lane_paths_[0] = msg;
                process_lane_path(0);
            });
        sub_lane_[1] = this->create_subscription<nav_msgs::msg::Path>(
            "/hdmap/lane_two", qos,
            [this](nav_msgs::msg::Path::SharedPtr msg)
            {
                lane_paths_[1] = msg;
                process_lane_path(1);
            });
        sub_lane_[2] = this->create_subscription<nav_msgs::msg::Path>(
            "/hdmap/lane_three", qos,
            [this](nav_msgs::msg::Path::SharedPtr msg)
            {
                lane_paths_[2] = msg;
                process_lane_path(2);
            });

        // Subscribers - Others
        sub_obs_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/obstacles_markers", 10,
            std::bind(&LocalPathPubCpp::obstacle_callback, this, std::placeholders::_1));
        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose", rclcpp::SensorDataQoS(),
            std::bind(&LocalPathPubCpp::pose_callback, this, std::placeholders::_1));

        sub_env_slow_ = this->create_subscription<std_msgs::msg::Float32>(
            "/env/slow_vel", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            {
                env_slow_vel_ = msg->data;
            });
        sub_env_fast_ = this->create_subscription<std_msgs::msg::Float32>(
            "/env/fast_vel", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            {
                env_fast_vel_ = msg->data;
            });

        // Publishers
        pub_local_path_ = this->create_publisher<nav_msgs::msg::Path>("/local_path", 10);
        pub_target_vel_ = this->create_publisher<std_msgs::msg::Float32>("/planning/target_v", 10);
        pub_lap_info_ = this->create_publisher<bisa::msg::LapInfo>("/lap_information", 10);
        pub_safety_zone_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("/planning/safety_zone", 10);

        // Timer - 50Hz control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&LocalPathPubCpp::control_loop, this));
    }

    // ============================================================================
    // UTILITY FUNCTIONS (최적화됨)
    // ============================================================================

    /**
     * @brief 거리 제곱 계산 (sqrt 생략으로 비교 연산 최적화)
     * @note 거리 비교 시 사용 - sqrt 불필요
     */
    inline double LocalPathPubCpp::get_dist_sq(double x1, double y1, double x2, double y2) const
    {
        const double dx = x1 - x2;
        const double dy = y1 - y2;
        return dx * dx + dy * dy;
    }

    /**
     * @brief 유클리드 거리 계산
     */
    inline double LocalPathPubCpp::get_dist(double x1, double y1, double x2, double y2) const
    {
        return std::hypot(x1 - x2, y1 - y2);
    }

    /**
     * @brief 각도 정규화 [-π, π]
     * @optimization while 루프 대신 fmod 사용하여 일관된 결과 보장
     *               (부동소수점 누적 오차 제거)
     */
    inline double LocalPathPubCpp::normalize_angle(double angle) const
    {
        // fmod로 먼저 범위를 줄인 후 조정 (while 루프의 부동소수점 문제 해결)
        angle = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0.0)
        {
            angle += 2.0 * M_PI;
        }
        return angle - M_PI;
    }

    // ============================================================================
    // ZONE CHECK FUNCTIONS (인라인화)
    // ============================================================================

    inline bool LocalPathPubCpp::in_merge_gate(int idx) const
    {
        return (idx >= FIXED_MERGE_IDX - MERGE_THRESHOLD) &&
               (idx <= FIXED_MERGE_IDX + MERGE_THRESHOLD);
    }

    inline bool LocalPathPubCpp::in_split_gate(int idx) const
    {
        return (idx >= FIXED_SPLIT_IDX - SPLIT_THRESHOLD) &&
               (idx < FIXED_SPLIT_IDX + SPLIT_THRESHOLD);
    }

    inline bool LocalPathPubCpp::in_merge_zone(int idx) const
    {
        return (idx > FIXED_MERGE_IDX + MERGE_THRESHOLD) &&
               (idx < FIXED_SPLIT_IDX - SPLIT_THRESHOLD);
    }

    // ============================================================================
    // HV CLASSIFICATION (인라인화)
    // ============================================================================

    inline bool LocalPathPubCpp::is_fast_hv(int id) const
    {
        return FAST_HV_IDS.find(id) != FAST_HV_IDS.end();
    }

    inline bool LocalPathPubCpp::is_slow_hv(int id) const
    {
        return SLOW_HV_IDS.find(id) != SLOW_HV_IDS.end();
    }

    // ============================================================================
    // INDEX SEARCH FUNCTIONS
    // ============================================================================

    /**
     * @brief 전방 기준 가장 가까운 인덱스 탐색 (윈도우 검색)
     * @optimization 제곱 거리 비교로 sqrt 연산 감소
     */
    int LocalPathPubCpp::find_closest_idx_forward(int lane_idx, double x, double y)
    {
        const auto &lane = processed_lanes_[lane_idx];
        if (lane.empty())
            return 0;

        const int path_size = static_cast<int>(lane.size());

        // 최초 호출: 전체 탐색
        if (last_closest_idx_ < 0)
        {
            double min_d_sq = 1e18;
            int min_idx = 0;

            for (int i = 0; i < path_size; ++i)
            {
                const double d_sq = get_dist_sq(x, y, lane[i].x, lane[i].y);
                if (d_sq < min_d_sq)
                {
                    min_d_sq = d_sq;
                    min_idx = i;
                }
            }
            last_closest_idx_ = min_idx;
            return min_idx;
        }

        // 주행 중: 윈도우 검색
        int best_idx = last_closest_idx_;
        double min_d_sq = 1e18;

        for (int i = 0; i < SEARCH_WINDOW; ++i)
        {
            const int idx = (last_closest_idx_ + i) % path_size;
            const double d_sq = get_dist_sq(x, y, lane[idx].x, lane[idx].y);

            if (d_sq < min_d_sq)
            {
                min_d_sq = d_sq;
                best_idx = idx;
            }
        }

        last_closest_idx_ = best_idx;
        return best_idx;
    }

    // ============================================================================
    // LANE DETECTION
    // ============================================================================

    /**
     * @brief 좌표가 속한 차선 판별
     * @optimization 제곱 거리 비교 + 샘플링 유지
     */
    LaneID LocalPathPubCpp::get_lane_at(double x, double y)
    {
        double d2_sq = 1e18;
        double d3_sq = 1e18;
        const double threshold_sq = LANE_DIST_THRESHOLD * LANE_DIST_THRESHOLD;

        // Lane 2 거리
        if (!processed_lanes_[1].empty())
        {
            const auto &lane2 = processed_lanes_[1];
            for (size_t i = 0; i < lane2.size(); i += LANE_SAMPLE_STEP)
            {
                const double d_sq = get_dist_sq(x, y, lane2[i].x, lane2[i].y);
                if (d_sq < d2_sq)
                    d2_sq = d_sq;
            }
        }

        // Lane 3 거리
        if (!processed_lanes_[2].empty())
        {
            const auto &lane3 = processed_lanes_[2];
            for (size_t i = 0; i < lane3.size(); i += LANE_SAMPLE_STEP)
            {
                const double d_sq = get_dist_sq(x, y, lane3[i].x, lane3[i].y);
                if (d_sq < d3_sq)
                    d3_sq = d_sq;
            }
        }

        if (d2_sq < d3_sq && d2_sq < threshold_sq)
            return LaneID::LANE_2;
        if (d3_sq <= d2_sq && d3_sq < threshold_sq)
            return LaneID::LANE_3;
        return LaneID::NONE;
    }

    // ============================================================================
    // VISUALIZATION
    // ============================================================================

    void LocalPathPubCpp::publish_safety_zone_visual(bool is_safe)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "safety_zone";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 위치 계산 (캐싱된 삼각함수 사용)
        const double center_x_local = (LANE_CHANGE_LOOKAHEAD - LANE_CHANGE_LOOKBEHIND) / 2.0;
        const double center_y_local = -0.375;

        marker.pose.position.x = ego_x_ + (center_x_local * ego_cos_ - center_y_local * ego_sin_);
        marker.pose.position.y = ego_y_ + (center_x_local * ego_sin_ + center_y_local * ego_cos_);
        marker.pose.position.z = 0.05;

        // Quaternion (yaw only)
        const double half_yaw = ego_yaw_ * 0.5;
        marker.pose.orientation.z = std::sin(half_yaw);
        marker.pose.orientation.w = std::cos(half_yaw);

        // 크기
        marker.scale.x = LANE_CHANGE_LOOKAHEAD + LANE_CHANGE_LOOKBEHIND;
        marker.scale.y = 0.65;
        marker.scale.z = 0.1;

        // 색상
        if (is_safe)
        {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.4f;
        }
        else
        {
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.6f;
        }

        pub_safety_zone_marker_->publish(marker);
    }

    // ============================================================================
    // GAP ANALYSIS - MERGE GATE
    // ============================================================================

    MergeGapInfo LocalPathPubCpp::analyze_merge_gap()
    {
        MergeGapInfo info;
        info.gap_available = true;

        // 상수 정의 (0.33m 차량 스케일)
        constexpr double CAR_LEN = VEH_TOTAL_LENGTH; // 0.33m
        constexpr double LOOKAHEAD = 4.0;
        constexpr double SAFE_GAP = 0.55;
        constexpr double CRITICAL_GAP = 0.25;
        constexpr double PREDICT_DT = 1.0;

        const double flow_speed = (env_slow_vel_ > 0.1) ? env_slow_vel_ : 1.2;
        info.recommended_vel = flow_speed;

        if (processed_lanes_[2].empty() || processed_lanes_[1].empty())
            return info;

        // 합류 지점 계산
        const double gate_x = processed_lanes_[2][FIXED_MERGE_IDX].x;
        const double gate_y = processed_lanes_[2][FIXED_MERGE_IDX].y;
        const double ego_to_gate = get_dist(ego_x_, ego_y_, gate_x, gate_y);
        const double ego_speed_safe = std::max(ego_speed_, 0.3);
        const double ego_tta = ego_to_gate / ego_speed_safe;

        // 장애물 탐색
        double closest_front_dist = 99.0;
        double closest_rear_dist = 99.0;
        double front_vel = 0.0;
        double rear_vel = 0.0;

        for (const auto &[id, obs] : obstacles_)
        {
            // 차선 필터
            if (obs.lane != LaneID::LANE_2)
                continue;

            // 헤딩 필터
            const double obs_speed = std::hypot(obs.vx, obs.vy);
            if (obs_speed > 0.1)
            {
                const double obs_yaw = std::atan2(obs.vy, obs.vx);
                double yaw_diff = normalize_angle(obs_yaw - ego_yaw_);
                if (std::abs(yaw_diff) > (M_PI / 4.0))
                    continue;
            }

            // 로컬 좌표 변환 (캐싱된 삼각함수 사용)
            const double dx = obs.x - ego_x_;
            const double dy = obs.y - ego_y_;
            const double local_x = dx * ego_cos_ + dy * ego_sin_;
            const double local_y = -dx * ego_sin_ + dy * ego_cos_;

            // 거리 보정 (Bumper-to-Bumper)
            const double real_dist_x = std::max(0.0, std::abs(local_x) - CAR_LEN);

            // 탐색 범위 제한
            if (std::abs(local_x) > LOOKAHEAD)
                continue;

            // 미래 예측
            const double rel_vx = obs.vx - (ego_speed_ * ego_cos_);
            const double rel_vy = obs.vy - (ego_speed_ * ego_sin_);
            const double future_local_x = local_x + (rel_vx * PREDICT_DT);
            const double future_local_y = local_y + (rel_vy * PREDICT_DT);
            const double future_real_dist_x = std::max(0.0, std::abs(future_local_x) - CAR_LEN);

            // 측면 위험 판단
            const bool is_lateral_risk = (std::abs(local_y) < 1.0) || (std::abs(future_local_y) < 1.0);
            if (!is_lateral_risk)
                continue;

            // 전후방 판별
            const double check_x = (std::abs(future_local_x) < std::abs(local_x)) ? future_local_x : local_x;
            const double min_d = std::min(real_dist_x, future_real_dist_x);

            if (check_x > -0.1) // 전방
            {
                if (min_d < closest_front_dist)
                {
                    closest_front_dist = min_d;
                    front_vel = obs_speed;
                }
            }
            else // 후방
            {
                if (min_d < closest_rear_dist)
                {
                    closest_rear_dist = min_d;
                    rear_vel = obs_speed;
                }
            }
        }

        info.front_dist = closest_front_dist;
        info.rear_dist = closest_rear_dist;
        info.front_vel = front_vel;
        info.rear_vel = rear_vel;

        // 안전성 판단
        bool front_ok = true;
        if (closest_front_dist < 99.0)
        {
            const double expected_front_gap = closest_front_dist + (front_vel * ego_tta);
            front_ok = (expected_front_gap > SAFE_GAP);
        }

        bool rear_ok = true;
        if (closest_rear_dist < 99.0)
        {
            const double expected_rear_gap = closest_rear_dist - (rear_vel * ego_tta) + (flow_speed * ego_tta);
            rear_ok = (expected_rear_gap > SAFE_GAP);
        }

        // 속도 제어
        if (front_ok && rear_ok)
        {
            info.gap_available = true;
            info.recommended_vel = flow_speed;
        }
        else
        {
            info.gap_available = false;

            if (!front_ok)
            {
                if (closest_front_dist < CRITICAL_GAP)
                {
                    info.recommended_vel = 1.5;
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                                         "[MERGE] CRITICAL FRONT (%.2fm) -> CREEP", closest_front_dist);
                }
                else
                {
                    info.recommended_vel = std::max(MIN_CREEP_VELOCITY, front_vel * 0.8);
                }
            }
            else if (!rear_ok)
            {
                const double relative_speed = rear_vel - ego_speed_;
                if (relative_speed > 1.0 && closest_rear_dist < CRITICAL_GAP)
                {
                    info.recommended_vel = 0.0;
                    RCLCPP_WARN(this->get_logger(), "[MERGE] YIELD to fast rear car");
                }
                else
                {
                    info.recommended_vel = std::min(MAX_VELOCITY, flow_speed * 1.5);
                }
            }
            else
            {
                info.recommended_vel = flow_speed;
            }
        }

        return info;
    }

    // ============================================================================
    // GAP ANALYSIS - SPLIT GATE
    // ============================================================================

    MergeGapInfo LocalPathPubCpp::analyze_split_gap()
    {
        MergeGapInfo info;
        info.gap_available = true;
        info.recommended_vel = (env_fast_vel_ > 0.1) ? env_fast_vel_ : MAX_VELOCITY;

        if (processed_lanes_[2].empty())
            return info;

        // Split 지점 좌표
        const double split_x = processed_lanes_[2][FIXED_SPLIT_IDX].x;
        const double split_y = processed_lanes_[2][FIXED_SPLIT_IDX].y;
        const double ego_to_split = get_dist(ego_x_, ego_y_, split_x, split_y);
        const double ego_speed_safe = std::max(ego_speed_, 0.3);
        const double ego_tta = ego_to_split / ego_speed_safe;

        // Fast HV 추적
        double closest_fast_hv_dist = 999.0;
        double fast_hv_vel = 0.0;
        int threatening_id = -1;
        double threatening_tta = 999.0;

        for (const auto &[id, obs] : obstacles_)
        {
            if (!is_fast_hv(id))
                continue;

            const double hv_to_split = get_dist(obs.x, obs.y, split_x, split_y);
            const double hv_speed = std::hypot(obs.vx, obs.vy);
            const double hv_speed_safe = std::max(hv_speed, 0.5);
            const double hv_tta = hv_to_split / hv_speed_safe;

            const double dist_to_ego = get_dist(obs.x, obs.y, ego_x_, ego_y_);

            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                                  "[SPLIT] Fast HV %d: to_split=%.2f, speed=%.2f, tta=%.2f, ego_tta=%.2f",
                                  id, hv_to_split, hv_speed, hv_tta, ego_tta);

            // 3m 이내 Fast HV
            if (hv_to_split < 3.0 && hv_tta < threatening_tta)
            {
                threatening_tta = hv_tta;
                threatening_id = id;
                closest_fast_hv_dist = dist_to_ego;
                fast_hv_vel = hv_speed;
            }
        }

        // TTA 비교 판단
        constexpr double TIME_MARGIN = 0.5;

        if (threatening_id >= 0)
        {
            if (ego_tta + TIME_MARGIN < threatening_tta)
            {
                // 내가 먼저 도착
                info.gap_available = true;
                info.recommended_vel = MAX_VELOCITY;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                                     "[SPLIT] CLEAR - I arrive first (ego=%.2f, hv%d=%.2f)",
                                     ego_tta, threatening_id, threatening_tta);
            }
            else if (threatening_tta < ego_tta + TIME_MARGIN)
            {
                // Fast HV가 먼저 또는 동시 도착
                info.gap_available = false;

                if (closest_fast_hv_dist < 0.5)
                {
                    info.recommended_vel = 0.0;
                    RCLCPP_WARN(this->get_logger(),
                                "[SPLIT] STOP! Fast HV %d too close (%.2fm)",
                                threatening_id, closest_fast_hv_dist);
                }
                else if (closest_fast_hv_dist < 1.0)
                {
                    info.recommended_vel = 0.3;
                    RCLCPP_WARN(this->get_logger(),
                                "[SPLIT] SLOW! Fast HV %d nearby (%.2fm, tta=%.2f)",
                                threatening_id, closest_fast_hv_dist, threatening_tta);
                }
                else
                {
                    info.recommended_vel = std::min(fast_hv_vel * 0.8, 1.0);
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                                         "[SPLIT] YIELD to Fast HV %d (dist=%.2f, following at %.2f)",
                                         threatening_id, closest_fast_hv_dist, info.recommended_vel);
                }
            }
            else
            {
                // 애매함 - 보수적
                info.gap_available = true;
                info.recommended_vel = 1.5;
            }

            info.front_dist = closest_fast_hv_dist;
            info.front_vel = fast_hv_vel;
        }
        else
        {
            info.gap_available = true;
            info.recommended_vel = MAX_VELOCITY;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "[SPLIT] No Fast HV nearby - GO!");
        }

        // Lane 2 차량 추가 체크
        for (const auto &[id, obs] : obstacles_)
        {
            if (obs.lane != LaneID::LANE_2 || is_fast_hv(id))
                continue;

            const double dx = obs.x - ego_x_;
            const double dy = obs.y - ego_y_;
            const double local_x = dx * ego_cos_ + dy * ego_sin_;

            if (local_x > 0.0 && local_x < 1.5)
            {
                const double dist = std::hypot(dx, dy);
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

    // ============================================================================
    // LANE CHANGE SAFETY CHECK
    // ============================================================================

    bool LocalPathPubCpp::is_lane_change_safe(LaneID target_lane)
    {
        if (processed_lanes_[static_cast<int>(target_lane)].empty())
            return false;

        for (const auto &[id, obs] : obstacles_)
        {
            if (obs.lane != target_lane)
                continue;

            // 로컬 좌표 변환 (캐싱된 삼각함수 사용)
            const double dx = obs.x - ego_x_;
            const double dy = obs.y - ego_y_;
            const double local_x = dx * ego_cos_ + dy * ego_sin_;
            const double local_y = -dx * ego_sin_ + dy * ego_cos_;

            // Safety Box 검사
            const bool in_longitudinal = (local_x < LANE_CHANGE_LOOKAHEAD) &&
                                         (local_x > -LANE_CHANGE_LOOKBEHIND);
            const bool in_lateral = (local_y < -0.05) && (local_y > -0.7);

            if (in_longitudinal && in_lateral)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                     "[Safety Check] Unsafe! Obstacle ID %d at (x:%.2f, y:%.2f)",
                                     id, local_x, local_y);
                return false;
            }
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "[Safety Check] Clear to change lane.");
        return true;
    }

    // ============================================================================
    // MAIN CONTROL LOOP
    // ============================================================================

    void LocalPathPubCpp::control_loop()
    {
        if (!pose_received_)
            return;

        // [최적화] 삼각함수 캐싱 - 루프 시작 시 한 번만 계산
        ego_cos_ = std::cos(ego_yaw_);
        ego_sin_ = std::sin(ego_yaw_);

        // 현재 차선 판별
        const LaneID current_lane_id = get_lane_at(ego_x_, ego_y_);
        const auto &current_lane = processed_lanes_[static_cast<int>(current_lane_id)];
        const int path_size = static_cast<int>(current_lane.size());

        if (path_size == 0)
            return;

        // ========================================================================
        // 초기 차선 변경 로직 (Lane 2 → Lane 3)
        // ========================================================================
        if (!initial_lane_change_done_ && std::abs(ego_yaw_) < 0.5)
        {
            const bool safe = is_lane_change_safe(LaneID::LANE_3);
            publish_safety_zone_visual(safe);

            if (safe)
            {
                const int current_lane_idx = find_closest_idx_forward(1, ego_x_, ego_y_);
                const auto &lane3 = processed_lanes_[2];
                const int lane3_size = static_cast<int>(lane3.size());

                int best_idx = -1;
                double min_d_sq = 1e18;

                const int search_start = current_lane_idx - 10;
                const int search_end = current_lane_idx + 100;

                for (int i = search_start; i < search_end; ++i)
                {
                    const int idx = (i + lane3_size) % lane3_size;
                    const double dx = lane3[idx].x - ego_x_;
                    const double dy = lane3[idx].y - ego_y_;
                    const double local_x = dx * ego_cos_ + dy * ego_sin_;

                    if (local_x > 0.5)
                    {
                        const double d_sq = dx * dx + dy * dy;
                        if (d_sq < min_d_sq)
                        {
                            min_d_sq = d_sq;
                            best_idx = idx;
                        }
                    }
                }

                if (best_idx != -1)
                {
                    nav_msgs::msg::Path l3_path_msg;
                    l3_path_msg.header.frame_id = "world";
                    l3_path_msg.header.stamp = this->now();

                    constexpr int LOOKAHEAD_STEPS = 100;
                    l3_path_msg.poses.reserve(LOOKAHEAD_STEPS);

                    for (int i = 0; i < LOOKAHEAD_STEPS; ++i)
                    {
                        const int idx = (best_idx + i) % lane3_size;
                        geometry_msgs::msg::PoseStamped ps;
                        ps.pose.position.x = lane3[idx].x;
                        ps.pose.position.y = lane3[idx].y;
                        ps.pose.orientation.w = 1.0;
                        l3_path_msg.poses.push_back(ps);
                    }

                    pub_local_path_->publish(l3_path_msg);

                    std_msgs::msg::Float32 v_msg;
                    v_msg.data = static_cast<float>(env_slow_vel_);
                    pub_target_vel_->publish(v_msg);

                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                         "Lane Change Initiated: Heading=%.2f rad", ego_yaw_);

                    if (get_lane_at(ego_x_, ego_y_) == LaneID::LANE_3 && std::abs(ego_yaw_) < 0.2)
                    {
                        initial_lane_change_done_ = true;
                        RCLCPP_INFO(this->get_logger(), "Lane Change COMPLETE: yaw=%.2f", ego_yaw_);
                    }

                    return;
                }
            }
        }

        // ========================================================================
        // 일반 주행 로직
        // ========================================================================
        const int ego_idx = find_closest_idx_forward(static_cast<int>(current_lane_id), ego_x_, ego_y_);

        // 기본 추종 속도
        double follow_speed = MAX_VELOCITY;
        if (current_lane_id == LaneID::LANE_2)
        {
            follow_speed = (env_slow_vel_ > 0.1) ? env_slow_vel_ : 1.5;
        }
        else
        {
            follow_speed = (env_fast_vel_ > 0.1) ? env_fast_vel_ : MAX_VELOCITY;
        }

        // 전방 장애물 탐색
        double min_dist_ahead = 999.0;
        bool obj_found = false;

        const bool is_in_merge_zone = in_merge_zone(ego_idx);
        const bool is_in_merge_gate = in_merge_gate(ego_idx);
        const bool is_in_split_gate = in_split_gate(ego_idx);

        for (const auto &[id, obs] : obstacles_)
        {
            if (obs.lane != current_lane_id)
                continue;

            const double dx = obs.x - ego_x_;
            const double dy = obs.y - ego_y_;
            const double local_x = dx * ego_cos_ + dy * ego_sin_;

            if (local_x > 0.0 && local_x < 2.0)
            {
                const double dist = std::hypot(dx, dy);
                if (dist < min_dist_ahead)
                {
                    min_dist_ahead = dist;
                    obj_found = true;
                }
            }
        }

        // ========================================================================
        // 속도 결정 (구간별)
        // ========================================================================
        double target_vel = follow_speed;

        // [구간 1] Merge Zone
        if (is_in_merge_zone)
        {
            if (obj_found)
            {
                double front_dist = 999.0;
                double rear_dist = 999.0;

                for (const auto &[id, obs] : obstacles_)
                {
                    if (obs.lane != current_lane_id)
                        continue;

                    const double dx = obs.x - ego_x_;
                    const double dy = obs.y - ego_y_;
                    const double local_x = dx * ego_cos_ + dy * ego_sin_;

                    if (local_x > 0.0 && local_x < front_dist)
                        front_dist = local_x;
                    else if (local_x < 0.0 && std::abs(local_x) < rear_dist)
                        rear_dist = std::abs(local_x);
                }

                double vel_for_front = follow_speed;
                double vel_for_rear = follow_speed;

                if (front_dist < MERGE_ZONE_MIN_FRONT_GAP)
                    vel_for_front = follow_speed * 0.8;

                if (rear_dist < MERGE_ZONE_MIN_REAR_GAP)
                    vel_for_rear = std::min(follow_speed * 1.3, MAX_VELOCITY);
                else if (rear_dist < MERGE_ZONE_MIN_REAR_GAP * 1.5)
                    vel_for_rear = std::min(follow_speed * 1.1, MAX_VELOCITY);

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
        // [구간 2] Merge Gate
        else if (is_in_merge_gate)
        {
            const MergeGapInfo gap_info = analyze_merge_gap();
            target_vel = gap_info.recommended_vel;

            RCLCPP_DEBUG(this->get_logger(),
                         "[MERGE GATE] front=%.2f, rear=%.2f, available=%d, vel=%.2f",
                         gap_info.front_dist, gap_info.rear_dist, gap_info.gap_available, target_vel);
        }
        // [구간 2] Split Gate
        else if (is_in_split_gate)
        {
            const MergeGapInfo gap_info = analyze_split_gap();
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
                target_vel = MAX_VELOCITY;
            }
        }

        // ========================================================================
        // Path Generation (수정된 버전)
        // ========================================================================
        nav_msgs::msg::Path local_path_msg;
        local_path_msg.header.frame_id = "world";
        local_path_msg.header.stamp = this->now();

        const double current_speed = std::abs(ego_speed_);

        // [수정 3] Pre-merge zone 체크를 인라인으로 처리 (함수 없이)
        const bool is_pre_merge = (ego_idx >= FIXED_MERGE_IDX - 200) &&
                                  (ego_idx < FIXED_MERGE_IDX - 50);

        int lookahead_steps;
        if (is_pre_merge || is_in_merge_gate)
        {
            lookahead_steps = std::max(30, static_cast<int>((current_speed * 0.8) / 0.1));
            lookahead_steps = std::min(lookahead_steps, 50);
            target_vel = std::min(target_vel, 1.2);
        }
        else
        {
            lookahead_steps = std::max(50, static_cast<int>((current_speed * 1.5) / 0.1));
            lookahead_steps = std::min(lookahead_steps, 150);
        }

        // [수정 4] lookahead가 path_size를 넘지 않도록
        lookahead_steps = std::min(lookahead_steps, path_size);

        local_path_msg.poses.reserve(lookahead_steps);

        for (int i = 0; i < lookahead_steps; ++i)
        {
            // [수정 5] 안전한 modulo 연산
            const int idx = (ego_idx + i) % path_size; // 이제 path_size > 0 보장됨

            geometry_msgs::msg::PoseStamped pose;
            pose.header = local_path_msg.header;
            pose.pose.position.x = current_lane[idx].x;
            pose.pose.position.y = current_lane[idx].y;
            pose.pose.orientation.w = 1.0;
            local_path_msg.poses.push_back(pose);
        }

        pub_local_path_->publish(local_path_msg);

        std_msgs::msg::Float32 v_msg;
        v_msg.data = static_cast<float>(target_vel);
        pub_target_vel_->publish(v_msg);

        publish_lap_info();
    }

    // ============================================================================
    // PATH PROCESSING
    // ============================================================================

    void LocalPathPubCpp::process_lane_path(int lane_idx)
    {
        if (!lane_paths_[lane_idx] || lane_paths_[lane_idx]->poses.empty())
            return;

        auto &lane = processed_lanes_[lane_idx];
        const auto &poses = lane_paths_[lane_idx]->poses;

        lane.clear();
        lane.reserve(poses.size());

        for (const auto &p : poses)
        {
            PathPoint pt;
            pt.x = p.pose.position.x;
            pt.y = p.pose.position.y;
            lane.push_back(pt);
        }
    }

    // ============================================================================
    // OBSTACLE CALLBACK
    // ============================================================================

    void LocalPathPubCpp::obstacle_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        const double now_s = this->now().seconds();

        for (const auto &mk : msg->markers)
        {
            if (mk.type != visualization_msgs::msg::Marker::CUBE)
                continue;

            const int id = mk.id;
            ObstacleInfo &obs = obstacles_[id];

            // 속도 계산 (미분)
            if (obs.last_seen_sec > 0.0)
            {
                const double dt = now_s - obs.last_seen_sec;
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

    // ============================================================================
    // POSE CALLBACK
    // ============================================================================

    void LocalPathPubCpp::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        const double now_s = this->now().seconds();
        const double new_x = msg->pose.position.x;
        const double new_y = msg->pose.position.y;

        // Yaw 계산 (Quaternion → Euler)
        const double siny = 2.0 * (msg->pose.orientation.w * msg->pose.orientation.z +
                                   msg->pose.orientation.x * msg->pose.orientation.y);
        const double cosy = 1.0 - 2.0 * (msg->pose.orientation.y * msg->pose.orientation.y +
                                         msg->pose.orientation.z * msg->pose.orientation.z);
        ego_yaw_ = std::atan2(siny, cosy);

        // 속도 계산 (위치 미분 + 필터링)
        if (prev_pose_time_ > 0.0)
        {
            const double dx = new_x - ego_x_;
            const double dy = new_y - ego_y_;

            total_distance_ += std::hypot(dx, dy);

            const double dt = now_s - prev_pose_time_;
            if (dt > 0.01 && dt < 0.5)
            {
                const double dist = std::hypot(dx, dy);
                const double raw_speed = dist / dt;

                // [최적화] 상수로 정의된 필터 계수 사용
                ego_speed_ = ego_speed_ * (1.0 - SPEED_FILTER_ALPHA) + raw_speed * SPEED_FILTER_ALPHA;
            }
        }

        ego_x_ = new_x;
        ego_y_ = new_y;
        prev_pose_time_ = now_s;
        pose_received_ = true;
    }

    // ============================================================================
    // LAP INFO
    // ============================================================================

    int LocalPathPubCpp::count_lap_idx(int lane_idx, double x, double y)
    {
        const auto &lane = processed_lanes_[lane_idx];
        if (lane.empty())
            return 0;

        const int n = static_cast<int>(lane.size());
        double min_d_sq = 1e18;
        int best_idx = 0;

        // [최적화] 제곱 거리 사용
        for (int i = 0; i < n; ++i)
        {
            const double d_sq = get_dist_sq(x, y, lane[i].x, lane[i].y);
            if (d_sq < min_d_sq)
            {
                min_d_sq = d_sq;
                best_idx = i;
            }
        }
        return best_idx;
    }

    void LocalPathPubCpp::publish_lap_info()
    {
        if (processed_lanes_[1].empty() || !pose_received_)
            return;

        const size_t total = processed_lanes_[1].size();
        if (total == 0)
            return;

        const int current_track_idx = count_lap_idx(1, ego_x_, ego_y_);

        // 한 바퀴 완료 감지
        if (current_track_idx < prev_track_idx_ && prev_track_idx_ > static_cast<int>(total * 0.9))
        {
            lap_count_++;
            lap_start_time_ = this->now();
            RCLCPP_INFO(this->get_logger(),
                        "[LAP] Lap %d completed! Total dist: %.2fm",
                        lap_count_, total_distance_);
        }
        prev_track_idx_ = current_track_idx;

        const double progress = (static_cast<double>(current_track_idx) / total) * 100.0;
        const double elapsed_time = (this->now() - lap_start_time_).seconds();

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
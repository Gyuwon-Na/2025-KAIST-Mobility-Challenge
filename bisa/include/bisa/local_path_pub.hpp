/**
 * @file local_path_pub.hpp
 * @brief Safety-First Local Path Header (Optimized)
 *
 * @optimization_changes
 * 1. 미사용 상수 제거 (SAFE_FRONT, SAFE_REAR, SAFE_LATERAL 등)
 * 2. 상수들을 논리적 그룹으로 재배치
 * 3. 캐싱 변수 추가 (ego_cos_, ego_sin_)로 중복 계산 제거
 * 4. 중복 멤버 변수 제거 (current_lane_id 멤버 제거)
 * 5. Jitter 감소를 위한 필터 파라미터 추가
 */

#ifndef BISA_LOCAL_PATH_PUB_HPP_
#define BISA_LOCAL_PATH_PUB_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>
#include <map>
#include <cmath>
#include <string>
#include <set>

#include "bisa/msg/lap_info.hpp"

namespace bisa
{

    // ============================================================================
    // ENUMS & STRUCTS
    // ============================================================================
    enum class LaneID
    {
        LANE_1 = 0, // 왼쪽 차선 (Static HVs)
        LANE_2 = 1, // 가운데 차선 (Dynamic HVs - Slow)
        LANE_3 = 2, // 오른쪽 차선 (Dynamic HVs - Fast)
        NONE = -1
    };

    struct PathPoint
    {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
    };

    struct ObstacleInfo
    {
        double x = 0.0;
        double y = 0.0;
        double vx = 0.0;
        double vy = 0.0;
        LaneID lane = LaneID::NONE;
        double last_seen_sec = 0.0;
    };

    struct MergeGapInfo
    {
        bool gap_available = false;
        double front_dist = 999.0;
        double rear_dist = 999.0;
        double front_vel = 0.0;
        double rear_vel = 0.0;
        double gap_size = 0.0;
        double recommended_vel = 1.5;
    };

    // ============================================================================
    // MAIN CLASS
    // ============================================================================
    class LocalPathPubCpp : public rclcpp::Node
    {
    public:
        LocalPathPubCpp();
        virtual ~LocalPathPubCpp() = default;

    private:
        // ========================================================================
        // CONSTANTS - 논리적 그룹별 정리
        // ========================================================================

        // --- [1] 트랙 인덱스 상수 (ㅁ형 루프 맵 기준) ---
        static constexpr int FIXED_MERGE_IDX = 1500; // 합류 지점 인덱스
        static constexpr int FIXED_SPLIT_IDX = 2281; // 분기 지점 인덱스
        static constexpr int MERGE_THRESHOLD = 50;   // 합류 구간 판정 범위 (+/-)
        static constexpr int SPLIT_THRESHOLD = 50;   // 분기 구간 판정 범위 (+/-)

        // --- [2] 차량 제원 (시뮬레이터 기준, 단위: m) ---
        static constexpr double VEH_LENGTH_F = 0.17;                            // Ego 앞범퍼까지 거리
        static constexpr double VEH_LENGTH_R = 0.16;                            // Ego 뒷범퍼까지 거리
        static constexpr double VEH_WIDTH = 0.075;                              // Ego 측면까지 거리 (반폭)
        static constexpr double VEH_TOTAL_LENGTH = VEH_LENGTH_F + VEH_LENGTH_R; // 0.33m

        // --- [3] 장애물 제원 (Ego와 동일 가정) ---
        static constexpr double OBS_LENGTH_F = 0.17;
        static constexpr double OBS_LENGTH_R = 0.16;
        static constexpr double OBS_WIDTH = 0.075;

        // --- [4] 안전 마진 (충돌 회피용) ---
        static constexpr double MARGIN_FRONT = 0.5;    // 전방 안전 마진
        static constexpr double MARGIN_REAR = 0.3;     // 후방 안전 마진
        static constexpr double MARGIN_LATERAL = 0.17; // 측면 안전 마진

        // --- [5] Merge Zone 파라미터 ---
        static constexpr double MIN_MERGE_GAP = 0.5;   // 최소 합류 갭
        static constexpr double SAFE_MERGE_GAP = 0.8;  // 안전 합류 갭
        static constexpr double MERGE_LOOKAHEAD = 3.0; // 합류 전후 탐색 범위
        static constexpr double BOOST_DIST = 1.0;      // 앞차 따라잡기 구간
        static constexpr double MERGE_ZONE_MIN_FRONT_GAP = 0.5;
        static constexpr double MERGE_ZONE_MIN_REAR_GAP = 0.2;

        // --- [6] 차선 변경 파라미터 ---
        static constexpr double LANE_CHANGE_LOOKAHEAD = 0.7;  // 전방 여유 거리
        static constexpr double LANE_CHANGE_LOOKBEHIND = 0.7; // 후방 여유 거리
        static constexpr double LANE_CHANGE_COOLDOWN = 2.0;   // 변경 후 최소 유지 시간

        // --- [7] 탐색 및 필터 파라미터 ---
        static constexpr int SEARCH_WINDOW = 200;          // 인덱스 탐색 윈도우 크기
        static constexpr int LANE_SAMPLE_STEP = 10;        // 차선 판별 샘플링 간격
        static constexpr double LANE_DIST_THRESHOLD = 1.0; // 차선 소속 판정 거리
        static constexpr double SPEED_FILTER_ALPHA = 0.3;  // 속도 필터 계수 (새 값 비중)
        static constexpr int LANE3_FORWARD_OFFSET = 10;    // Lane3 전방 오프셋

        // --- [8] 속도 제한 ---
        static constexpr double MAX_VELOCITY = 2.0;       // 시스템 최대 속도
        static constexpr double MIN_CREEP_VELOCITY = 0.2; // 최소 크립 속도

        // ========================================================================
        // HV ID Sets (차량 분류)
        // ========================================================================
        const std::set<int> FAST_HV_IDS = {0, 1, 2, 3};
        const std::set<int> SLOW_HV_IDS = {4, 5, 6, 7, 8, 9, 10, 11};

        // ========================================================================
        // CORE FUNCTIONS
        // ========================================================================

        // --- 인덱스 탐색 ---
        int find_closest_idx_forward(int lane_idx, double x, double y);
        int find_closest_forward_idx_in_lane3();

        // --- 유틸리티 ---
        inline double get_dist_sq(double x1, double y1, double x2, double y2) const;
        inline double get_dist(double x1, double y1, double x2, double y2) const;
        inline double normalize_angle(double angle) const;

        // --- 차선 판별 ---
        LaneID get_lane_at(double x, double y);

        // --- 구간 판별 (인라인화) ---
        inline bool in_merge_gate(int idx) const;
        inline bool in_split_gate(int idx) const;
        inline bool in_merge_zone(int idx) const;

        // --- HV 분류 ---
        inline bool is_fast_hv(int id) const;
        inline bool is_slow_hv(int id) const;

        // --- Gap 분석 ---
        MergeGapInfo analyze_merge_gap();
        MergeGapInfo analyze_split_gap();

        // --- 차선 변경 ---
        bool is_lane_change_safe(LaneID target_lane);
        bool is_initial_merge_safe();
        double calculate_merge_zone_velocity(LaneID current_lane, double base_speed);

        // --- Lap Info ---
        int count_lap_idx(int lane_idx, double x, double y);
        void publish_lap_info();

        // --- 시각화 ---
        void publish_safety_zone_visual(bool is_safe);

        // ========================================================================
        // CALLBACKS
        // ========================================================================
        void control_loop();
        void process_lane_path(int lane_idx);
        void obstacle_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        // ========================================================================
        // ROS INTERFACES
        // ========================================================================

        // Subscribers
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_lane_[3];
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_obs_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_env_slow_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_env_fast_;

        // Publishers
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_local_path_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_target_vel_;
        rclcpp::Publisher<bisa::msg::LapInfo>::SharedPtr pub_lap_info_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_safety_zone_marker_;

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;

        // ========================================================================
        // DATA STORAGE
        // ========================================================================
        nav_msgs::msg::Path::SharedPtr lane_paths_[3];
        std::vector<PathPoint> processed_lanes_[3];
        std::map<int, ObstacleInfo> obstacles_;

        // ========================================================================
        // EGO STATE (캐싱 변수 포함)
        // ========================================================================
        bool pose_received_ = false;
        double ego_x_ = 0.0;
        double ego_y_ = 0.0;
        double ego_yaw_ = 0.0;
        double ego_speed_ = 0.0;

        // [최적화] 삼각함수 캐싱 - control_loop 시작 시 갱신
        double ego_cos_ = 1.0;
        double ego_sin_ = 0.0;

        double prev_pose_time_ = 0.0;
        double prev_target_vel_ = 0.0;

        // ========================================================================
        // INDEX TRACKING
        // ========================================================================
        int last_closest_idx_ = -1;       // Lane 2 전용
        int last_closest_idx_lane3_ = -1; // Lane 3 전용

        // ========================================================================
        // ENVIRONMENT STATE
        // ========================================================================
        double env_slow_vel_ = 0.0;
        double env_fast_vel_ = 0.0;

        // ========================================================================
        // LAP TRACKING
        // ========================================================================
        int lap_count_ = 0;
        double total_distance_ = 0.0;
        rclcpp::Time lap_start_time_;
        int prev_track_idx_ = 0;

        // ========================================================================
        // LANE CHANGE STATE
        // ========================================================================
        LaneID current_following_lane_ = LaneID::LANE_2; // 현재 추종 차선
        bool initial_merge_done_ = false;
        bool initial_lane_change_done_ = false;
        double lane_change_cooldown_ = 0.0;
    };

} // namespace bisa

#endif // BISA_LOCAL_PATH_PUB_HPP_
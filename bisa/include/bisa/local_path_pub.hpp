/**
 * @file local_path_pub.hpp
 * @brief Local Path Publisher Header - Integrated with Merge Detection & Priority Fix
 * @version 3.1
 */

#ifndef BISA_LOCAL_PATH_PUB_HPP_
#define BISA_LOCAL_PATH_PUB_HPP_

// ============================================================================
// INCLUDES
// ============================================================================
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <limits>
#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace bisa
{

    // ============================================================================
    // ENUMS
    // ============================================================================

    /**
     * @brief Lane identifier enum
     * @details LANE_1: Left, LANE_2: Center, LANE_3: Right
     */
    enum class LaneID
    {
        LANE_1 = 0, // Left lane (static HVs)
        LANE_2 = 1, // Center lane (slow dynamic HVs)
        LANE_3 = 2, // Right lane (fast dynamic HVs)
        NONE = -1
    };

    /**
     * @brief Driving state machine states
     */
    enum class DrivingState
    {
        CRUISE,           // Normal driving
        PREPARE_OVERTAKE, // Preparing for lane change
        LANE_CHANGING,    // Executing lane change
        EMERGENCY_ACCEL,  // Emergency acceleration due to rear threat
        RETURN_TO_CENTER  // Returning to center lane
    };

    // ============================================================================
    // STRUCTS
    // ============================================================================

    /**
     * @brief Obstacle information structure
     */
    struct ObstacleInfo
    {
        int id;
        double x, y;   // World coordinates
        double vx, vy; // World velocity
        double speed;  // Speed magnitude
        LaneID lane;   // Which lane the obstacle is in

        double rel_x;  // Body frame x (forward)
        double rel_y;  // Body frame y (left)
        double rel_vx; // Relative velocity in body frame

        double last_seen_sec;
        bool is_static;
    };

    /**
     * @brief Surrounding environment status
     */
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

        // Side zones
        bool left_clear = true;
        double left_dist = 999.0;
        bool right_clear = true;
        double right_dist = 999.0;

        // Path collision
        bool path_collision = false;
        double path_collision_dist = 999.0;
    };

    /**
     * @brief Path point structure with arc length
     */
    struct PathPoint
    {
        double x, y, yaw;
        double s; // Arc length from start
    };

    // ============================================================================
    // CLASS DEFINITION
    // ============================================================================

    class LocalPathPubCpp : public rclcpp::Node
    {
    public:
        LocalPathPubCpp();

    private:
        // ========================================================================
        // CONSTANTS
        // ========================================================================

        // Vehicle dimensions
        static constexpr double VEHICLE_LENGTH = 0.33;
        static constexpr double VEHICLE_WIDTH = 0.15;

        // Obstacle dimensions (half-extents)
        static constexpr double OBS_HALF_LENGTH = 0.17;
        static constexpr double OBS_HALF_WIDTH = 0.075;

        // Front zone parameters
        static constexpr double FRONT_ZONE_LENGTH = 0.8;
        static constexpr double FRONT_ZONE_WIDTH = 0.12;
        static constexpr double FRONT_SAFE_DIST = 0.70;
        static constexpr double FRONT_DANGER_DIST = 0.5;
        static constexpr double FRONT_CRITICAL_DIST = 0.35;

        // Rear zone parameters
        static constexpr double REAR_ZONE_LENGTH = 1.0;
        static constexpr double REAR_ZONE_WIDTH = 0.12;
        static constexpr double REAR_DANGER_DIST = 0.5;
        static constexpr double REAR_TTC_THRESHOLD = 2.0;

        // Side zone parameters
        static constexpr double SIDE_ZONE_START = -0.5;
        static constexpr double SIDE_ZONE_END = 0.6;
        static constexpr double SIDE_ZONE_INNER = 0.10;
        static constexpr double SIDE_ZONE_OUTER = 0.35;

        // Lane parameters
        static constexpr double LANE_WIDTH = 0.25;
        static constexpr double LANE_THRESHOLD = 0.20;
        static constexpr double MERGE_THRESHOLD = 0.15; // Merge zone detection

        // Speed parameters
        static constexpr double CRUISE_SPEED = 1.5;
        static constexpr double OVERTAKE_SPEED = 2.0;
        static constexpr double ACCEL_SPEED = 3.0;
        static constexpr double SLOW_SPEED = 0.8;
        static constexpr double MIN_SPEED = 0.0;

        // Lane change parameters
        static constexpr double LANE_CHANGE_FORWARD_DIST = 0.8;
        static constexpr double LANE_CHANGE_TIME = 1.5;

        // Path planning parameters
        static constexpr double CORNER_CURVATURE_THRESHOLD = 0.3;
        static constexpr double PATH_CHECK_HORIZON = 1.5;
        static constexpr double PATH_COLLISION_RADIUS = 0.22;
        static constexpr double STATIC_SPEED_THRESHOLD = 0.08;

        // Safety check count
        static constexpr int SAFE_CHECK_REQUIRED = 3;

        // ========================================================================
        // ROS INTERFACES
        // ========================================================================

        // Subscribers
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_lane_[3];
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obs_sub_;

        // Publishers
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_vel_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;

        // ========================================================================
        // STATE VARIABLES
        // ========================================================================

        // Lane data
        nav_msgs::msg::Path::SharedPtr lane_paths_[3];
        std::vector<PathPoint> processed_lanes_[3];

        // Ego vehicle state
        double ego_x_ = 0.0;
        double ego_y_ = 0.0;
        double ego_yaw_ = 0.0;
        double ego_speed_ = 0.0;
        bool pose_received_ = false;

        // Previous pose for speed calculation
        double prev_x_ = 0.0;
        double prev_y_ = 0.0;
        double prev_pose_sec_ = 0.0;
        bool prev_pose_valid_ = false;

        // Lane tracking
        LaneID current_lane_ = LaneID::LANE_2;
        LaneID target_lane_ = LaneID::LANE_2;
        DrivingState current_state_ = DrivingState::CRUISE;

        // Lane change state
        bool is_lane_changing_ = false;
        double lane_change_start_sec_ = 0.0;
        double lane_change_progress_ = 0.0;
        int safe_check_count_ = 0;

        // Corner state
        bool is_in_corner_ = false;
        bool corner_approaching_ = false;
        double current_curvature_ = 0.0;

        // Obstacle tracking
        std::map<int, ObstacleInfo> obstacles_;
        SurroundingStatus current_surrounding_;

        // Path tracking
        int last_closest_idx_[3] = {0, 0, 0};
        bool initial_search_done_ = false;
        double last_log_sec_ = 0.0;

        // ========================================================================
        // UTILITY FUNCTIONS
        // ========================================================================

        double now_sec();
        double normalize_angle(double angle);
        int lane_to_int(LaneID lane);
        LaneID int_to_lane(int i);
        std::string lane_str(LaneID lane);
        std::string state_str(DrivingState s);

        // ========================================================================
        // CALLBACKS
        // ========================================================================

        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void process_lane_path(int lane_idx);
        void obstacle_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);

        // ========================================================================
        // LANE UTILITIES & MERGE CHECK
        // ========================================================================

        double get_dist_to_lane(int lane_idx, double x, double y);
        LaneID get_lane_at(double x, double y);
        bool are_lanes_merged(LaneID lane_a, LaneID lane_b);
        int find_closest_idx(int lane_idx, double x, double y, bool global_search = false);
        int find_index_at_distance(int lane_idx, int start_idx, double target_dist);

        // ========================================================================
        // CURVATURE & CORNER DETECTION
        // ========================================================================

        double compute_curvature(int lane_idx, int idx, int window = 10);
        void update_corner_state(int lane_idx, int closest);

        // ========================================================================
        // OBSTACLE PROCESSING
        // ========================================================================

        void update_all_obstacles_body_frame();
        bool check_path_collision(int lane_idx, double &collision_dist);
        SurroundingStatus check_all_zones();

        // ========================================================================
        // SAFETY CHECK FUNCTIONS
        // ========================================================================

        bool is_target_left(LaneID target);
        bool is_lane_blocked_by_static(LaneID lane);
        bool is_overtake_safe(LaneID target);
        std::pair<double, double> check_lane_front(LaneID lane);

        // ========================================================================
        // LANE SELECTION
        // ========================================================================

        LaneID choose_overtake_lane();

        // ========================================================================
        // STATE MACHINE
        // ========================================================================

        void update_state_machine();

        // ========================================================================
        // VELOCITY COMPUTATION
        // ========================================================================

        double compute_velocity();

        // ========================================================================
        // PATH GENERATION
        // ========================================================================

        std::vector<PathPoint> generate_lane_change_path();

        // ========================================================================
        // MAIN CONTROL LOOP & VISUALIZATION
        // ========================================================================

        void control_loop();
        void publish_debug_markers();
    };

} // namespace bisa

#endif // BISA_LOCAL_PATH_PUB_HPP_
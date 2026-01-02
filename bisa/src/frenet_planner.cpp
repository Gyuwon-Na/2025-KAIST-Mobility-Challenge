#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2/utils.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>
#include <string>
#include <map>
#include <iomanip>

using namespace std::chrono_literals;

// ==========================================
// [파라미터] Spinning 방지 및 부드러운 합류
// ==========================================
const double MAX_SPEED = 1.3;
const double MAX_ACCEL = 1.0;
const double MAX_STEER = 0.52;
const double WHEELBASE = 0.33;
const double FORCE_MIN_SPEED = 0.8; // [상향] 너무 느리면 조향이 안 먹힘

// [핵심] 차선 변경 시 Lookahead 설정
// 기본 주행 시
const double BASE_LOOKAHEAD = 1.5;
// 차선 변경 시 (횡방향 오차 * 이 비율만큼 멀리 봄)
const double ERROR_LOOKAHEAD_RATIO = 4.0; // 2.5m 떨어져 있으면 10m 앞을 봄

const double ROAD_BOUNDARY = 0.35;
const double MAGNET_GAIN = 0.3; // 자석 힘 줄임 (부드럽게)

const double K_P_ACCEL = 2.0;
const double LANE_SEARCH_RADIUS = 5.0;

enum LaneID
{
    LANE_1 = 0,
    LANE_2 = 1,
    LANE_3 = 2,
    NONE = -1
};

std::string get_lane_name(LaneID id)
{
    if (id == LANE_1)
        return "ONE";
    if (id == LANE_2)
        return "TWO";
    if (id == LANE_3)
        return "THREE";
    return "NONE";
}

struct Obstacle
{
    int id;
    double x, y, dist_to_ego, velocity;
    LaneID lane;
};

struct LaneCandidate
{
    LaneID id;
    double cost;
    double target_vel;
    bool blocked;
};

class FrenetPlanner : public rclcpp::Node
{
public:
    FrenetPlanner() : Node("frenet_planner")
    {
        auto map_qos = rclcpp::QoS(10).transient_local();

        sub_lane_1_ = this->create_subscription<nav_msgs::msg::Path>("/hdmap/lane_one", map_qos,
                                                                     [this](const nav_msgs::msg::Path::SharedPtr msg)
                                                                     { path_lane_1_ = msg; check_map_ready(); });
        sub_lane_2_ = this->create_subscription<nav_msgs::msg::Path>("/hdmap/lane_two", map_qos,
                                                                     [this](const nav_msgs::msg::Path::SharedPtr msg)
                                                                     { path_lane_2_ = msg; check_map_ready(); });
        sub_lane_3_ = this->create_subscription<nav_msgs::msg::Path>("/hdmap/lane_three", map_qos,
                                                                     [this](const nav_msgs::msg::Path::SharedPtr msg)
                                                                     { path_lane_3_ = msg; check_map_ready(); });

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose", rclcpp::SensorDataQoS(),
            std::bind(&FrenetPlanner::pose_callback, this, std::placeholders::_1));

        obs_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("/obstacles_markers", 10,
                                                                                   std::bind(&FrenetPlanner::obs_callback, this, std::placeholders::_1));

        accel_pub_ = this->create_publisher<geometry_msgs::msg::Accel>("/Accel", 10);
        local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/local_path", 10);
        target_lane_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planning/target_info", 10);
        car_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/car_marker", 10);
        debug_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planning/debug", 10);

        timer_ = this->create_wall_timer(20ms, std::bind(&FrenetPlanner::control_loop, this));

        current_target_lane_ = LANE_2;
        map_received_ = false;
        current_speed_ = 0.0;
        last_closest_idx_ = 0;
        last_steer_cmd_ = 0.0;

        is_initialized_ = false;
        lateral_offset_ = 0.0;

        RCLCPP_INFO(this->get_logger(), ">>> Smooth Lane Change Planner Started <<<");
    }

private:
    void check_map_ready()
    {
        if (path_lane_1_ && path_lane_2_ && path_lane_3_)
        {
            if (!map_received_)
            {
                map_received_ = true;
                RCLCPP_INFO(this->get_logger(), "✅ All Lanes Received!");
            }
        }
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = msg->pose;
        static rclcpp::Time last_time = this->now();
        static geometry_msgs::msg::Point last_pos = current_pose_.position;
        rclcpp::Time now = this->now();
        double dt = (now - last_time).seconds();
        if (dt > 0.02)
        {
            double dist = std::hypot(current_pose_.position.x - last_pos.x, current_pose_.position.y - last_pos.y);
            double raw_speed = dist / dt;
            if (raw_speed < 10.0)
                current_speed_ = 0.7 * raw_speed + 0.3 * current_speed_;
            last_time = now;
            last_pos = current_pose_.position;
        }
        publish_car_marker();
    }

    void obs_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        detected_obstacles_.clear();
        for (const auto &marker : msg->markers)
        {
            if (marker.ns == "surrounding_cars" && marker.type == visualization_msgs::msg::Marker::CUBE)
            {
                double raw_dist = std::hypot(marker.pose.position.x - current_pose_.position.x,
                                             marker.pose.position.y - current_pose_.position.y);
                if (raw_dist < 0.5)
                    continue;
                Obstacle obs;
                obs.id = marker.id;
                obs.x = marker.pose.position.x;
                obs.y = marker.pose.position.y;
                obs.dist_to_ego = raw_dist;
                obs.lane = get_nearest_lane_id(obs.x, obs.y);
                if (obs.dist_to_ego < 15.0)
                    detected_obstacles_.push_back(obs);
            }
        }
    }

    LaneID get_nearest_lane_id(double x, double y)
    {
        double d1 = get_dist_to_path(path_lane_1_, x, y);
        double d2 = get_dist_to_path(path_lane_2_, x, y);
        double d3 = get_dist_to_path(path_lane_3_, x, y);

        double min_d = std::min({d1, d2, d3});
        if (min_d > 10.0)
            return NONE;

        if (min_d == d1)
            return LANE_1;
        if (min_d == d2)
            return LANE_2;
        return LANE_3;
    }

    double get_dist_to_path(const nav_msgs::msg::Path::SharedPtr &path, double x, double y)
    {
        if (!path || path->poses.empty())
            return 9999.0;
        double min_dist = 9999.0;
        for (const auto &pose : path->poses)
        {
            double d = std::hypot(pose.pose.position.x - x, pose.pose.position.y - y);
            if (d < min_dist)
                min_dist = d;
        }
        return min_dist;
    }

    double get_signed_lateral_error(const nav_msgs::msg::Path::SharedPtr &path, int closest_idx)
    {
        if (!path || path->poses.empty() || closest_idx >= path->poses.size() - 1)
            return 0.0;

        double cx = current_pose_.position.x;
        double cy = current_pose_.position.y;
        double px = path->poses[closest_idx].pose.position.x;
        double py = path->poses[closest_idx].pose.position.y;
        double npx = path->poses[closest_idx + 1].pose.position.x;
        double npy = path->poses[closest_idx + 1].pose.position.y;

        double path_vec_x = npx - px;
        double path_vec_y = npy - py;
        double ego_vec_x = cx - px;
        double ego_vec_y = cy - py;

        double cross_prod = path_vec_x * ego_vec_y - path_vec_y * ego_vec_x;
        double path_len = std::hypot(path_vec_x, path_vec_y);

        if (path_len < 0.001)
            return 0.0;
        return cross_prod / path_len;
    }

    void control_loop()
    {
        if (!map_received_)
            return;

        LaneID my_lane_id = get_nearest_lane_id(current_pose_.position.x, current_pose_.position.y);

        // --- 초기화 (Start Zeroing) ---
        if (!is_initialized_)
        {
            if (my_lane_id == NONE)
                return;

            current_target_lane_ = my_lane_id;
            nav_msgs::msg::Path::SharedPtr init_path = get_path_ptr(my_lane_id);
            int idx = get_closest_idx(init_path);
            double raw_error = get_signed_lateral_error(init_path, idx);
            lateral_offset_ = raw_error;
            is_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "🔵 Zeroing Done. Offset: %.2fm", lateral_offset_);
        }

        if (my_lane_id == NONE)
            my_lane_id = current_target_lane_;

        // 차선 선택
        std::vector<LaneCandidate> candidates;
        candidates.push_back(analyze_lane(LANE_1, my_lane_id));
        candidates.push_back(analyze_lane(LANE_2, my_lane_id));
        candidates.push_back(analyze_lane(LANE_3, my_lane_id));

        LaneCandidate best_lane = candidates[current_target_lane_];
        double min_cost = 99999.0;
        for (const auto &cand : candidates)
        {
            if (std::abs((int)cand.id - (int)my_lane_id) > 1)
                continue;
            if (cand.cost < min_cost)
            {
                min_cost = cand.cost;
                best_lane = cand;
            }
        }

        if (best_lane.id != current_target_lane_)
        {
            if (min_cost < get_current_lane_cost(candidates, current_target_lane_) - 5.0)
            {
                current_target_lane_ = best_lane.id;
                last_closest_idx_ = 0;
            }
            else
            {
                best_lane = candidates[current_target_lane_];
            }
        }
        else
        {
            best_lane = candidates[current_target_lane_];
        }

        // 장애물 감지
        double final_speed = best_lane.target_vel;
        bool lateral_danger = false;
        double my_yaw = tf2::getYaw(current_pose_.orientation);

        for (const auto &obs : detected_obstacles_)
        {
            double dx = obs.x - current_pose_.position.x;
            double dy = obs.y - current_pose_.position.y;
            double local_x = dx * cos(my_yaw) + dy * sin(my_yaw);
            double local_y = -dx * sin(my_yaw) + dy * cos(my_yaw);

            // 전방 2m, 측면 0.35m 내
            if (local_x > -0.5 && local_x < 2.0 && std::abs(local_y) < 0.35)
            {
                lateral_danger = true;
                break;
            }
        }

        if (lateral_danger)
        {
            final_speed = 0.0;
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500, "🛑 EMERGENCY STOP!");
        }

        execute_lane_following(current_target_lane_, final_speed, my_lane_id);
    }

    int get_closest_idx(const nav_msgs::msg::Path::SharedPtr &path)
    {
        if (!path || path->poses.empty())
            return 0;
        double min_dist = 1e9;
        int closest_idx = 0;
        for (size_t i = 0; i < path->poses.size(); ++i)
        {
            double d = std::hypot(path->poses[i].pose.position.x - current_pose_.position.x,
                                  path->poses[i].pose.position.y - current_pose_.position.y);
            if (d < min_dist)
            {
                min_dist = d;
                closest_idx = i;
            }
        }
        return closest_idx;
    }

    nav_msgs::msg::Path::SharedPtr get_path_ptr(LaneID id)
    {
        if (id == LANE_1)
            return path_lane_1_;
        if (id == LANE_2)
            return path_lane_2_;
        return path_lane_3_;
    }

    double get_current_lane_cost(const std::vector<LaneCandidate> &cands, LaneID id)
    {
        for (const auto &c : cands)
            if (c.id == id)
                return c.cost;
        return 9999.0;
    }

    LaneCandidate analyze_lane(LaneID lane_id, LaneID my_lane)
    {
        LaneCandidate cand;
        cand.id = lane_id;
        cand.blocked = false;
        cand.cost = 0.0;
        cand.target_vel = MAX_SPEED;
        nav_msgs::msg::Path::SharedPtr path = get_path_ptr(lane_id);
        if (!path)
        {
            cand.blocked = true;
            cand.cost = 999.0;
            return cand;
        }

        double min_dist = 999.0;
        double my_yaw = tf2::getYaw(current_pose_.orientation);
        for (const auto &obs : detected_obstacles_)
        {
            if (obs.lane == lane_id)
            {
                double dx = obs.x - current_pose_.position.x;
                double dy = obs.y - current_pose_.position.y;
                double local_x = dx * cos(my_yaw) + dy * sin(my_yaw);
                if (local_x > -0.5 && local_x < 15.0)
                {
                    double d = std::hypot(dx, dy);
                    if (d < min_dist)
                        min_dist = d;
                }
            }
        }
        if (min_dist < 15.0)
        {
            cand.cost += (20.0 - min_dist);
            cand.target_vel = MAX_SPEED * (min_dist / 20.0);
        }
        if (lane_id != my_lane)
            cand.cost += 2.0;
        return cand;
    }

    void execute_lane_following(LaneID target_id, double target_speed, LaneID my_lane)
    {
        nav_msgs::msg::Path::SharedPtr target_path = get_path_ptr(target_id);
        if (!target_path || target_path->poses.empty())
            return;

        // Closest Index
        int path_len = target_path->poses.size();
        int search_start = last_closest_idx_;
        int search_end = std::min(path_len, last_closest_idx_ + 50);
        if (last_closest_idx_ == 0 || last_closest_idx_ >= path_len - 5)
        {
            search_start = 0;
            search_end = path_len;
        }

        double min_dist = 1e9;
        int closest_idx = -1;
        for (int i = search_start; i < search_end; ++i)
        {
            double d = std::hypot(target_path->poses[i].pose.position.x - current_pose_.position.x,
                                  target_path->poses[i].pose.position.y - current_pose_.position.y);
            if (d < min_dist)
            {
                min_dist = d;
                closest_idx = i;
            }
        }
        if (closest_idx == -1)
            closest_idx = 0;
        last_closest_idx_ = closest_idx;

        // 오차 계산 (Zeroing 적용)
        double raw_error = get_signed_lateral_error(target_path, closest_idx);
        double corrected_error = raw_error - lateral_offset_;

        // Magnet Steer
        double magnet_steer = -corrected_error * MAGNET_GAIN;

        // =========================================================
        // [핵심] Lookahead Boosting (횡방향 오차 비례)
        // 오차가 크면(멀리 떨어져 있으면) 훨씬 멀리 보게 하여 진입 각도를 완화함
        // 예: 오차 2.5m -> Lookahead 10.0m -> 조향각 줄어듦
        // =========================================================
        double required_lookahead = std::abs(corrected_error) * ERROR_LOOKAHEAD_RATIO;
        double current_ld = std::max(BASE_LOOKAHEAD, required_lookahead);

        // 최대 시야 제한 (너무 멀면 코너 못 돔, 적절히 제한)
        current_ld = std::min(current_ld, 15.0);

        // Lookahead Point 찾기
        int lookahead_idx = closest_idx;
        for (int i = closest_idx; i < path_len; ++i)
        {
            double d = std::hypot(target_path->poses[i].pose.position.x - current_pose_.position.x,
                                  target_path->poses[i].pose.position.y - current_pose_.position.y);
            if (d > current_ld)
            {
                lookahead_idx = i;
                break;
            }
        }

        double tx = target_path->poses[lookahead_idx].pose.position.x;
        double ty = target_path->poses[lookahead_idx].pose.position.y;
        double dx = tx - current_pose_.position.x;
        double dy = ty - current_pose_.position.y;
        double my_yaw = tf2::getYaw(current_pose_.orientation);
        double alpha = atan2(dy, dx) - my_yaw;

        double pp_steer = atan2(2.0 * WHEELBASE * sin(alpha), current_ld);

        // Final Combine
        double total_steer = pp_steer + magnet_steer;

        // 조향 제한
        total_steer = std::max(-MAX_STEER, std::min(MAX_STEER, total_steer));

        // Accel
        double accel_cmd = (target_speed - current_speed_) * K_P_ACCEL;
        accel_cmd = std::max(-MAX_ACCEL, std::min(MAX_ACCEL, accel_cmd));

        geometry_msgs::msg::Accel cmd_msg;
        cmd_msg.linear.x = accel_cmd;
        cmd_msg.angular.z = total_steer;
        accel_pub_->publish(cmd_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "📡 Err:%.2fm | Ld:%.2f | Str:%.2f", corrected_error, current_ld, total_steer);

        // Visuals
        nav_msgs::msg::Path local_path;
        local_path.header.frame_id = "world";
        local_path.header.stamp = this->now();
        for (int i = closest_idx; i < std::min((int)target_path->poses.size(), lookahead_idx + 20); ++i)
        {
            local_path.poses.push_back(target_path->poses[i]);
        }
        local_path_pub_->publish(local_path);

        visualization_msgs::msg::Marker text;
        text.header.frame_id = "world";
        text.id = 999;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose = current_pose_;
        text.pose.position.z += 0.5;
        text.scale.z = 0.3;
        text.color.a = 1.0;
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 0.0;
        std::stringstream ss;
        ss << "Err: " << std::fixed << std::setprecision(2) << corrected_error << "m\nLd: " << current_ld;
        text.text = ss.str();
        target_lane_pub_->publish(text);
    }

    void publish_car_marker()
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "world";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose = current_pose_;
        m.scale.x = WHEELBASE;
        m.scale.y = 0.15;
        m.scale.z = 0.1;
        m.color.a = 0.9;
        m.color.r = 1.0;
        m.color.g = 1.0;
        car_marker_pub_->publish(m);
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_lane_1_, sub_lane_2_, sub_lane_3_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obs_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr accel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_lane_pub_, car_marker_pub_, debug_marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path::SharedPtr path_lane_1_, path_lane_2_, path_lane_3_;
    geometry_msgs::msg::Pose current_pose_;
    double current_speed_;
    LaneID current_target_lane_;
    std::vector<Obstacle> detected_obstacles_;
    bool map_received_;
    int last_closest_idx_;
    double last_steer_cmd_;

    bool is_initialized_;
    double lateral_offset_;

    struct ObsState
    {
        double x, y;
        rclcpp::Time time;
    };
    std::map<int, ObsState> obs_tracker_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrenetPlanner>());
    rclcpp::shutdown();
    return 0;
}
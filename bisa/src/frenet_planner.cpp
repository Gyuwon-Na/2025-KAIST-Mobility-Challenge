#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>
#include <string>
#include <map>

using namespace std::chrono_literals;

// [환경 설정]
const double DEFAULT_LANE_WIDTH = 0.8;
const double MAX_SPEED = 2.0;
const double PREDICT_DIST = 6.0;
const double CAR_LENGTH = 0.33;

enum LaneID
{
    LEFT_LANE = 0,   // Static
    CENTER_LANE = 1, // Slow
    RIGHT_LANE = 2   // Fast (Preferred)
};

struct Obstacle
{
    int id;
    double s;
    double d;
    double dist;
};

struct MapPoint
{
    double x, y, s, d;
};

// [NEW] 후보 차선 정보 구조체
struct LaneCandidate
{
    LaneID id;
    double d;
    double cost;
    double target_vel;
    bool blocked;
};

class FrenetPlanner : public rclcpp::Node
{
public:
    FrenetPlanner() : Node("frenet_planner")
    {
        auto qos_profile = rclcpp::QoS(10).transient_local();

        global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/user_global_path", qos_profile,
            std::bind(&FrenetPlanner::global_path_callback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose", rclcpp::SensorDataQoS(),
            std::bind(&FrenetPlanner::pose_callback, this, std::placeholders::_1));

        obs_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/obstacles_markers", 10,
            std::bind(&FrenetPlanner::obs_callback, this, std::placeholders::_1));

        hdmap_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/hdmap_data/link_markers", qos_profile,
            std::bind(&FrenetPlanner::hdmap_callback, this, std::placeholders::_1));

        local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/local_path", 10);
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        target_lane_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planning/target_lane", 10);
        car_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/car_marker", 10);

        timer_ = this->create_wall_timer(
            50ms, std::bind(&FrenetPlanner::control_loop, this));

        target_lane_ = CENTER_LANE;
        current_speed_ = 0.0;
        is_global_path_received_ = false;

        RCLCPP_INFO(this->get_logger(), ">>> Tesla-Style Planner Started (Cost Function Mode) <<<");
    }

private:
    void global_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        global_path_ = msg;
        if (!is_global_path_received_ && !global_path_->poses.empty())
        {
            RCLCPP_INFO(this->get_logger(), ">>> Global Path Loaded (%zu pts) <<<", global_path_->poses.size());
            is_global_path_received_ = true;
        }
    }

    void hdmap_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        map_points_.clear();
        for (const auto &marker : msg->markers)
        {
            for (const auto &p : marker.points)
            {
                MapPoint mp;
                mp.x = p.x;
                mp.y = p.y;
                map_points_.push_back(mp);
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
        if (dt > 0.0)
        {
            double dist = std::hypot(current_pose_.position.x - last_pos.x, current_pose_.position.y - last_pos.y);
            current_speed_ = dist / dt;
        }
        last_time = now;
        last_pos = current_pose_.position;
        publish_car_marker();
    }

    void obs_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        if (!is_global_path_received_)
            return;

        detected_obstacles_.clear();
        double my_s, my_d;
        get_frenet(current_pose_.position.x, current_pose_.position.y, my_s, my_d);

        for (const auto &marker : msg->markers)
        {
            if (marker.ns == "surrounding_cars" && marker.type == visualization_msgs::msg::Marker::CUBE)
            {
                double obs_s, obs_d;
                get_frenet(marker.pose.position.x, marker.pose.position.y, obs_s, obs_d);
                double dist = obs_s - my_s;

                // 맵 Loop 처리 없이 단순 거리 체크 (-5m ~ 20m)
                if (dist > -5.0 && dist < 20.0)
                {
                    Obstacle obs;
                    obs.id = marker.id;
                    obs.s = obs_s;
                    obs.d = obs_d;
                    obs.dist = dist;
                    detected_obstacles_.push_back(obs);
                }
            }
        }
    }

    double get_real_lane_offset(LaneID lane_id, double current_s)
    {
        if (map_points_.empty())
        {
            if (lane_id == LEFT_LANE)
                return DEFAULT_LANE_WIDTH;
            if (lane_id == RIGHT_LANE)
                return -DEFAULT_LANE_WIDTH;
            return 0.0;
        }

        double sum_d = 0.0;
        int count = 0;

        for (const auto &p : map_points_)
        {
            double pt_s, pt_d;
            get_frenet(p.x, p.y, pt_s, pt_d);

            if (std::abs(pt_s - current_s) < 2.0)
            { // 탐색 범위 2.0m
                if (lane_id == LEFT_LANE && pt_d > 0.3)
                {
                    sum_d += pt_d;
                    count++;
                }
                else if (lane_id == RIGHT_LANE && pt_d < -0.3)
                {
                    sum_d += pt_d;
                    count++;
                }
                else if (lane_id == CENTER_LANE && std::abs(pt_d) <= 0.3)
                {
                    sum_d += pt_d;
                    count++;
                }
            }
        }

        if (count > 0)
            return sum_d / count;

        if (lane_id == LEFT_LANE)
            return DEFAULT_LANE_WIDTH;
        if (lane_id == RIGHT_LANE)
            return -DEFAULT_LANE_WIDTH;
        return 0.0;
    }

    int get_closest_waypoint(double x, double y)
    {
        double min_dist = std::numeric_limits<double>::max();
        int closest_idx = -1;
        for (size_t i = 0; i < global_path_->poses.size(); ++i)
        {
            double dx = global_path_->poses[i].pose.position.x - x;
            double dy = global_path_->poses[i].pose.position.y - y;
            double dist = dx * dx + dy * dy;
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_idx = i;
            }
        }
        return closest_idx;
    }

    void get_frenet(double x, double y, double &s, double &d)
    {
        int idx = get_closest_waypoint(x, y);
        if (idx == -1)
        {
            s = 0;
            d = 0;
            return;
        }

        double wx = global_path_->poses[idx].pose.position.x;
        double wy = global_path_->poses[idx].pose.position.y;

        // [수정 전] 쿼터니언 값 사용 (데이터가 부정확할 위험 있음)
        // double wyaw = tf2::getYaw(global_path_->poses[idx].pose.orientation);

        // [수정 후] 웨이포인트 좌표 간의 각도를 직접 계산 (가장 확실함)
        double wyaw = 0.0;

        if (idx + 1 < (int)global_path_->poses.size())
        {
            double next_x = global_path_->poses[idx + 1].pose.position.x;
            double next_y = global_path_->poses[idx + 1].pose.position.y;
            wyaw = atan2(next_y - wy, next_x - wx);
        }
        else if (idx - 1 >= 0)
        {
            // 마지막 점이라면 이전 점과의 각도 사용
            double prev_x = global_path_->poses[idx - 1].pose.position.x;
            double prev_y = global_path_->poses[idx - 1].pose.position.y;
            wyaw = atan2(wy - prev_y, wx - prev_x);
        }
        // else: 점이 1개뿐이면 0.0 유지

        double dx = x - wx;
        double dy = y - wy;

        // Frenet d 계산
        d = -dx * sin(wyaw) + dy * cos(wyaw);

        // s 계산 (단순화된 버전)
        double s_local = dx * cos(wyaw) + dy * sin(wyaw);
        s = (double)idx * 0.1 + s_local;
    }

    void get_cartesian(double s, double d, double &x, double &y)
    {
        int idx = std::min((int)(s / 0.1), (int)global_path_->poses.size() - 1);
        if (idx < 0)
            idx = 0;

        double way_x = global_path_->poses[idx].pose.position.x;
        double way_y = global_path_->poses[idx].pose.position.y;
        double way_yaw = tf2::getYaw(global_path_->poses[idx].pose.orientation);

        x = way_x + d * -sin(way_yaw);
        y = way_y + d * cos(way_yaw);
    }

    void publish_car_marker()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "my_car";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = current_pose_;
        marker.scale.x = CAR_LENGTH;
        marker.scale.y = 0.15;
        marker.scale.z = 0.1;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.9;
        car_marker_pub_->publish(marker);
    }

    // ==========================================
    // [핵심] Cost Function 기반 최적 차선 선택
    // ==========================================
    void control_loop()
    {
        if (!is_global_path_received_)
            return;

        double my_s, my_d;
        get_frenet(current_pose_.position.x, current_pose_.position.y, my_s, my_d);

        // 1. 후보 차선들의 실제 d값(Offset) 계산
        double d_left = get_real_lane_offset(LEFT_LANE, my_s);
        double d_center = get_real_lane_offset(CENTER_LANE, my_s);
        double d_right = get_real_lane_offset(RIGHT_LANE, my_s);

        // 현재 내 차선 판단
        LaneID current_lane = CENTER_LANE;
        if (std::abs(my_d - d_left) < 0.4)
            current_lane = LEFT_LANE;
        else if (std::abs(my_d - d_right) < 0.4)
            current_lane = RIGHT_LANE;

        // 2. 각 차선별 비용(Cost) 계산
        std::vector<LaneCandidate> candidates;
        candidates.push_back(calculate_lane_cost(LEFT_LANE, d_left, my_s, current_lane));
        candidates.push_back(calculate_lane_cost(CENTER_LANE, d_center, my_s, current_lane));
        candidates.push_back(calculate_lane_cost(RIGHT_LANE, d_right, my_s, current_lane));

        // 3. 최적의 차선 선택 (Cost가 가장 낮은 것)
        LaneCandidate best_lane = candidates[0]; // 초기값
        double min_cost = 99999.0;

        for (const auto &cand : candidates)
        {
            if (cand.cost < min_cost)
            {
                min_cost = cand.cost;
                best_lane = cand;
            }
        }

        // [Smooth Transition] 너무 잦은 차선 변경 방지
        // 현재 타겟과 베스트가 다르고, 코스트 차이가 크지 않으면 유지할 수도 있음 (여기선 생략)
        target_lane_ = best_lane.id;
        double target_d = best_lane.d;
        double target_vel = best_lane.target_vel;

        // 로그 출력
        std::string lane_str = (target_lane_ == RIGHT_LANE) ? "RIGHT" : (target_lane_ == LEFT_LANE ? "LEFT" : "CENTER");
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        //     "Best: %s | Cost(L/C/R): %.0f / %.0f / %.0f",
        //     lane_str.c_str(), candidates[0].cost, candidates[1].cost, candidates[2].cost);

        publish_local_path_and_control(my_s, target_d, target_lane_, target_vel);
    }

    // [Cost Function] 각 차선의 점수를 매기는 함수
    LaneCandidate calculate_lane_cost(LaneID lane, double lane_d, double my_s, LaneID current_lane)
    {
        LaneCandidate cand;
        cand.id = lane;
        cand.d = lane_d;
        cand.blocked = false;
        cand.cost = 0.0;
        cand.target_vel = MAX_SPEED;

        // A. 장애물 확인
        double min_dist = 999.0;
        bool obstacle_exist = false;

        for (const auto &obs : detected_obstacles_)
        {
            // 해당 차선에 있는 장애물인지 확인
            if (std::abs(obs.d - lane_d) < 0.4)
            {
                // 내 앞쪽 장애물만 고려
                if (obs.dist > -CAR_LENGTH && obs.dist < 20.0)
                {
                    if (obs.dist < min_dist)
                        min_dist = obs.dist;
                    obstacle_exist = true;
                }
            }
        }

        // B. Cost 계산 로직

        // 1. 충돌 위험 (Collision Cost)
        // 내 바로 앞/옆(2m 이내)에 장애물이 있으면 그 차선은 절대 가면 안 됨
        if (min_dist < 2.0)
        {
            cand.cost += 10000.0; // 폭발적인 비용
            cand.blocked = true;
            cand.target_vel = 0.0; // 멈춤
            return cand;
        }

        // 2. 속도 효율성 (Efficiency Cost)
        // 장애물이 멀리 있을수록 좋음 (Cost 낮음)
        // 장애물이 가까우면 속도를 줄여야 하므로 Cost 높음
        if (obstacle_exist)
        {
            // 여유 공간이 좁을수록 비용 증가 (2m ~ 20m)
            // 예: 3m 앞이면 cost 높음, 15m 앞이면 cost 낮음
            double space_score = (20.0 - min_dist);
            cand.cost += space_score * 10.0;

            // 목표 속도도 거리에 비례해서 줄임
            cand.target_vel = MAX_SPEED * (min_dist / 10.0);
            if (cand.target_vel > MAX_SPEED)
                cand.target_vel = MAX_SPEED;
        }

        // 3. 차선 변경 페널티 (Change Cost)
        if (lane != current_lane)
        {
            cand.cost += 20.0; // 웬만하면 차선 유지하려고 함
        }

        // 4. 차선 선호도 (Lane Bias)
        // Fast Lane(Right)은 가산점, Static Lane(Left)은 감점
        if (lane == RIGHT_LANE)
            cand.cost -= 5.0; // Fast Lane 선호
        if (lane == LEFT_LANE)
            cand.cost += 50.0; // Static Lane은 정말 급할 때만 사용

        return cand;
    }

    void publish_local_path_and_control(double current_s, double target_d, LaneID lane_id, double target_vel)
    {
        nav_msgs::msg::Path local_path;
        local_path.header.frame_id = "world";
        local_path.header.stamp = this->now();

        // 경로 생성 (6m)
        for (double s_inc = 0; s_inc < PREDICT_DIST; s_inc += 0.1)
        {
            double next_s = current_s + s_inc;
            double progress = s_inc / PREDICT_DIST;
            double factor = (1.0 - cos(progress * M_PI)) / 2.0;

            double my_current_d_pos;
            double dummy;
            get_frenet(current_pose_.position.x, current_pose_.position.y, dummy, my_current_d_pos);

            double next_d = my_current_d_pos + (target_d - my_current_d_pos) * factor;

            double x, y;
            get_cartesian(next_s, next_d, x, y);

            geometry_msgs::msg::PoseStamped p;
            p.pose.position.x = x;
            p.pose.position.y = y;
            local_path.poses.push_back(p);
        }

        local_path_pub_->publish(local_path);

        // Marker (Thick Line)
        visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = "world";
        path_marker.header.stamp = this->now();
        path_marker.ns = "path_thick";
        path_marker.id = 777;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.scale.x = 0.1;
        path_marker.color.a = 0.8;
        path_marker.color.g = 1.0;
        for (const auto &pose : local_path.poses)
            path_marker.points.push_back(pose.pose.position);
        car_marker_pub_->publish(path_marker);

        // Control
        size_t lookahead_idx = 8;
        if (local_path.poses.size() > lookahead_idx)
        {
            double target_x = local_path.poses[lookahead_idx].pose.position.x;
            double target_y = local_path.poses[lookahead_idx].pose.position.y;
            double dx = target_x - current_pose_.position.x;
            double dy = target_y - current_pose_.position.y;
            double my_yaw = tf2::getYaw(current_pose_.orientation);
            double alpha = atan2(dy, dx) - my_yaw;
            double Ld = std::sqrt(dx * dx + dy * dy);
            double steering = atan2(2.0 * CAR_LENGTH * sin(alpha), Ld);
            steering = std::max(-0.5, std::min(0.5, steering));

            ackermann_msgs::msg::AckermannDriveStamped drive_msg;
            drive_msg.drive.speed = target_vel;
            drive_msg.drive.steering_angle = steering;
            drive_pub_->publish(drive_msg);
        }

        // Text Info
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "world";
        text_marker.id = 9999;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position = current_pose_.position;
        text_marker.pose.position.z += 0.5;
        text_marker.scale.z = 0.2;
        text_marker.color.a = 1.0;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;

        std::string lane_str = (lane_id == RIGHT_LANE) ? "RIGHT" : (lane_id == LEFT_LANE ? "LEFT" : "CENTER");
        std::stringstream ss;
        ss << "Target: " << lane_str << "\nObs: " << detected_obstacles_.size();
        text_marker.text = ss.str();
        target_lane_pub_->publish(text_marker);
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obs_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr hdmap_sub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_lane_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr car_marker_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path::SharedPtr global_path_;
    geometry_msgs::msg::Pose current_pose_;
    double current_speed_;
    LaneID target_lane_;
    std::vector<Obstacle> detected_obstacles_;
    std::vector<MapPoint> map_points_;
    bool is_global_path_received_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrenetPlanner>());
    rclcpp::shutdown();
    return 0;
}
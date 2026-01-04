#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <tf2/utils.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>
#include <map>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

enum LaneID
{
    LANE_1 = 0,
    LANE_2 = 1,
    LANE_3 = 2,
    NONE = -1
};

struct ObstacleState
{
    int id;
    double x, y, vx, vy;
    double risk_score;
    rclcpp::Time last_seen;
};

struct Point
{
    double x, y;
};

class FrenetPlanner : public rclcpp::Node
{
public:
    FrenetPlanner() : Node("frenet_planner")
    {
        // 1. 파라미터 선언
        this->declare_parameter("wheelbase", 0.33);
        this->declare_parameter("max_steer", 0.52);
        this->declare_parameter("target_speed", 1.0);
        this->declare_parameter("boost_speed", 1.5);
        this->declare_parameter("base_lookahead", 0.6);
        this->declare_parameter("max_lookahead", 1.2);
        this->declare_parameter("error_gain", 0.2);
        this->declare_parameter("weight_heading", 0.7);
        this->declare_parameter("weight_path", 0.3);
        this->declare_parameter("steer_alpha", 0.1);
        this->declare_parameter("static_margin_front", 0.7);
        this->declare_parameter("static_margin_rear", 0.3);
        this->declare_parameter("predict_time", 1.0);
        this->declare_parameter("predict_step", 0.2);

        // 리셋 트리거
        this->declare_parameter("reset_trigger", false);

        get_current_parameters();

        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&FrenetPlanner::param_callback, this, std::placeholders::_1));

        auto map_qos = rclcpp::QoS(10).transient_local();
        // [사용자 수정 반영 확인됨] Lane 1 -> path_lane_1_
        sub_lane_1_ = this->create_subscription<nav_msgs::msg::Path>("/hdmap/lane_one", map_qos, [this](const nav_msgs::msg::Path::SharedPtr msg)
                                                                     { path_lane_1_ = msg; });
        sub_lane_2_ = this->create_subscription<nav_msgs::msg::Path>("/hdmap/lane_two", map_qos, [this](const nav_msgs::msg::Path::SharedPtr msg)
                                                                     { path_lane_2_ = msg; });
        sub_lane_3_ = this->create_subscription<nav_msgs::msg::Path>("/hdmap/lane_three", map_qos, [this](const nav_msgs::msg::Path::SharedPtr msg)
                                                                     { path_lane_3_ = msg; });

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose", rclcpp::SensorDataQoS(), std::bind(&FrenetPlanner::pose_callback, this, std::placeholders::_1));

        obs_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/obstacles_markers", 10, std::bind(&FrenetPlanner::obs_callback, this, std::placeholders::_1));

        accel_pub_ = this->create_publisher<geometry_msgs::msg::Accel>("/Accel", 10);
        local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/target_path_viz", 10);
        debug_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planning/lookahead_point", 10);
        risk_text_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/planning/risk_scores", 10);

        timer_ = this->create_wall_timer(20ms, std::bind(&FrenetPlanner::control_loop, this));

        // 망설임 방지용 상태 변수 초기화
        is_waiting_for_gap_ = false;

        reset_state();

        RCLCPP_INFO(this->get_logger(), ">>> Smart Lane Changer Started (No Reverse Mode) <<<");
    }

private:
    double wheelbase_;
    double max_steer_;
    double target_speed_;
    double boost_speed_;
    double base_lookahead_;
    double max_lookahead_;
    double error_gain_;
    double weight_heading_;
    double weight_path_;
    double steer_alpha_;
    double static_margin_front_;
    double static_margin_rear_;
    double predict_time_;
    double predict_step_;

    // [추가] 망설임 방지용 변수
    bool is_waiting_for_gap_;

    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    void reset_state()
    {
        current_target_lane_ = LANE_2;
        last_closest_idx_ = -1;
        current_speed_ = 0.0;
        last_steer_cmd_ = 0.0;
        is_changing_lane_ = false;
        tracked_obstacles_.clear();
        is_waiting_for_gap_ = false; // 리셋 시 대기 상태 해제
        RCLCPP_WARN(this->get_logger(), "!!! STATE RESET (Cool Start) !!!");
    }

    void get_current_parameters()
    {
        this->get_parameter("wheelbase", wheelbase_);
        this->get_parameter("max_steer", max_steer_);
        this->get_parameter("target_speed", target_speed_);
        this->get_parameter("boost_speed", boost_speed_);
        this->get_parameter("base_lookahead", base_lookahead_);
        this->get_parameter("max_lookahead", max_lookahead_);
        this->get_parameter("error_gain", error_gain_);
        this->get_parameter("weight_heading", weight_heading_);
        this->get_parameter("weight_path", weight_path_);
        this->get_parameter("steer_alpha", steer_alpha_);
        this->get_parameter("static_margin_front", static_margin_front_);
        this->get_parameter("static_margin_rear", static_margin_rear_);
        this->get_parameter("predict_time", predict_time_);
        this->get_parameter("predict_step", predict_step_);
    }

    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        bool need_update = false;
        for (const auto &param : parameters)
        {
            if (param.get_name() == "reset_trigger" && param.as_bool() == true)
            {
                reset_state();
            }
            else
            {
                need_update = true;
            }
        }

        if (need_update)
        {
            get_current_parameters();
        }
        return result;
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = msg->pose;
        static rclcpp::Time last_time = this->now();
        static double last_x = current_pose_.position.x;
        static double last_y = current_pose_.position.y;
        rclcpp::Time now = this->now();
        double dt = (now - last_time).seconds();
        if (dt > 0.02)
        {
            double dist = std::hypot(current_pose_.position.x - last_x, current_pose_.position.y - last_y);
            double raw_speed = dist / dt;

            // 후진 중일 경우 속도를 음수로 표현 (간단한 방향 체크)
            // (실제로는 쿼터니언 방향과 이동 벡터 내적을 해야 정확하지만, 여기서는 raw_speed 스칼라만 사용 중)
            // 여기서는 raw_speed가 항상 양수이므로, 뒤로 밀리는 건 control_loop에서 처리

            current_speed_ = (current_speed_ * 0.7) + (raw_speed * 0.3);
            last_time = now;
            last_x = current_pose_.position.x;
            last_y = current_pose_.position.y;
        }
    }

    void obs_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        rclcpp::Time now = this->now();
        for (const auto &marker : msg->markers)
        {
            if (marker.ns == "surrounding_cars" && marker.type == visualization_msgs::msg::Marker::CUBE)
            {
                int id = marker.id;
                if (tracked_obstacles_.find(id) != tracked_obstacles_.end())
                {
                    double dt = (now - tracked_obstacles_[id].last_seen).seconds();
                    if (dt > 0.001)
                    {
                        double vx = (marker.pose.position.x - tracked_obstacles_[id].x) / dt;
                        double vy = (marker.pose.position.y - tracked_obstacles_[id].y) / dt;
                        if (std::abs(vx) < 5.0 && std::abs(vy) < 5.0)
                        {
                            tracked_obstacles_[id].vx = tracked_obstacles_[id].vx * 0.6 + vx * 0.4;
                            tracked_obstacles_[id].vy = tracked_obstacles_[id].vy * 0.6 + vy * 0.4;
                        }
                    }
                    tracked_obstacles_[id].x = marker.pose.position.x;
                    tracked_obstacles_[id].y = marker.pose.position.y;
                    tracked_obstacles_[id].last_seen = now;
                }
                else
                {
                    tracked_obstacles_[id] = {id, marker.pose.position.x, marker.pose.position.y, 0.0, 0.0, 0.0, now};
                }
            }
        }
    }

    Point transform_to_body(double gx, double gy)
    {
        double dx = gx - current_pose_.position.x;
        double dy = gy - current_pose_.position.y;
        double yaw = tf2::getYaw(current_pose_.orientation);
        return {dx * cos(yaw) + dy * sin(yaw), -dx * sin(yaw) + dy * cos(yaw)};
    }

    int get_closest_idx(const nav_msgs::msg::Path::SharedPtr &path, int last_idx)
    {
        if (!path || path->poses.empty())
            return 0;
        int sz = path->poses.size();
        int best_idx = 0;
        if (last_idx != -1)
            best_idx = last_idx;

        double min_d = 1e9;
        int search_start = (last_idx == -1) ? 0 : -10;
        int search_end = (last_idx == -1) ? sz : 50;

        for (int i = search_start; i < search_end; ++i)
        {
            int idx = (best_idx + i + sz) % sz;
            double d = std::hypot(path->poses[idx].pose.position.x - current_pose_.position.x,
                                  path->poses[idx].pose.position.y - current_pose_.position.y);
            if (d < min_d)
            {
                min_d = d;
                best_idx = idx;
            }
        }
        return best_idx;
    }

    double get_dist_to_path_simple(const nav_msgs::msg::Path::SharedPtr &path, double x, double y)
    {
        if (!path || path->poses.empty())
            return 999.0;
        double min_d = 999.0;
        for (size_t i = 0; i < path->poses.size(); i += 5)
        {
            double d = std::hypot(path->poses[i].pose.position.x - x, path->poses[i].pose.position.y - y);
            if (d < min_d)
                min_d = d;
        }
        return min_d;
    }

    LaneID get_lane_id_of_point(double x, double y)
    {
        double d1 = get_dist_to_path_simple(path_lane_1_, x, y);
        double d2 = get_dist_to_path_simple(path_lane_2_, x, y);
        double d3 = get_dist_to_path_simple(path_lane_3_, x, y);
        if (d1 < d2 && d1 < d3 && d1 < 0.45)
            return LANE_1;
        if (d2 < d3 && d2 < 0.45)
            return LANE_2;
        if (d3 < 0.45)
            return LANE_3;
        return NONE;
    }

    nav_msgs::msg::Path::SharedPtr get_path_by_id(LaneID id)
    {
        if (id == LANE_1)
            return path_lane_1_;
        if (id == LANE_2)
            return path_lane_2_;
        if (id == LANE_3)
            return path_lane_3_;
        return nullptr;
    }

    double get_lane_max_risk(LaneID lane)
    {
        nav_msgs::msg::Path::SharedPtr path = get_path_by_id(lane);
        if (!path || path->poses.empty())
            return 1.0;
        double max_risk = 0.0;
        int idx = get_closest_idx(path, last_closest_idx_);
        double lane_x = path->poses[idx].pose.position.x;
        double lane_y = path->poses[idx].pose.position.y;

        for (auto &[id, obs] : tracked_obstacles_)
        {
            if (std::hypot(obs.x - lane_x, obs.y - lane_y) < 0.7)
            {
                double obs_spd = std::hypot(obs.vx, obs.vy);
                double r = (obs_spd > current_speed_ - 0.2) ? 0.2 : 0.8;
                Point p = transform_to_body(obs.x, obs.y);
                if (std::abs(p.x) < 0.6)
                    r = 1.0;

                // 현재 타겟 차선인 경우에만 장애물 객체에 점수 기록 (시각화용)
                if (lane == current_target_lane_)
                {
                    obs.risk_score = r;
                }

                if (r > max_risk)
                    max_risk = r;
            }
        }
        return max_risk;
    }

    int check_safe_join_trajectory(LaneID target_lane)
    {
        double my_yaw = tf2::getYaw(current_pose_.orientation);
        double cos_yaw = cos(my_yaw);
        double sin_yaw = sin(my_yaw);

        bool need_boost = false;

        std::vector<ObstacleState> target_obs;
        for (const auto &[id, obs] : tracked_obstacles_)
        {
            if (get_lane_id_of_point(obs.x, obs.y) == target_lane)
            {
                target_obs.push_back(obs);
            }
        }

        if (target_obs.empty())
            return 1;

        for (double t = 0.0; t <= predict_time_; t += predict_step_)
        {
            for (const auto &obs : target_obs)
            {
                double dx = obs.x - current_pose_.position.x;
                double dy = obs.y - current_pose_.position.y;
                double body_x_0 = dx * cos_yaw + dy * sin_yaw;
                double obs_vx_proj = obs.vx * cos_yaw + obs.vy * sin_yaw;
                double future_body_x = body_x_0 + (obs_vx_proj * t) - (current_speed_ * t);

                if (future_body_x > -static_margin_rear_ && future_body_x < static_margin_front_)
                {
                    if (t < 0.5 && body_x_0 < 0 && obs_vx_proj > current_speed_)
                    {
                        double future_boost_x = body_x_0 + (obs_vx_proj * t) - (boost_speed_ * t);
                        if (future_boost_x < -static_margin_rear_)
                        {
                            need_boost = true;
                            continue;
                        }
                    }
                    return 0;
                }
            }
        }
        return need_boost ? 2 : 1;
    }

    void control_loop()
    {
        double current_lane_risk = get_lane_max_risk(current_target_lane_);
        LaneID best_lane = current_target_lane_;
        bool boost_needed = false;

        if (current_lane_risk > 0.6)
        {
            double min_risk = current_lane_risk;
            std::vector<LaneID> candidates;
            if (current_target_lane_ == LANE_2)
                candidates = {LANE_1, LANE_3};
            else
                candidates = {LANE_2};

            for (LaneID cand : candidates)
            {
                int safety = check_safe_join_trajectory(cand);

                if (safety == 0)
                    continue;

                double cand_risk = get_lane_max_risk(cand);
                if (safety == 2)
                    cand_risk *= 0.5;

                if (cand_risk < min_risk - 0.2)
                {
                    min_risk = cand_risk;
                    best_lane = cand;
                    boost_needed = (safety == 2);
                }
            }
        }

        if (best_lane != current_target_lane_)
        {
            current_target_lane_ = best_lane;
            is_changing_lane_ = true;
            if (boost_needed)
                RCLCPP_INFO(this->get_logger(), "🚀 AGGRESSIVE BOOST -> Lane %d", best_lane);
            else
                RCLCPP_INFO(this->get_logger(), "Lane Change -> %d", best_lane);
        }

        nav_msgs::msg::Path::SharedPtr target_path = get_path_by_id(current_target_lane_);
        if (!target_path || target_path->poses.empty())
            return;

        int closest_idx = get_closest_idx(target_path, last_closest_idx_);
        last_closest_idx_ = closest_idx;

        double err = std::hypot(target_path->poses[closest_idx].pose.position.x - current_pose_.position.x,
                                target_path->poses[closest_idx].pose.position.y - current_pose_.position.y);

        if (is_changing_lane_ && err < 0.3)
            is_changing_lane_ = false;

        double l_dist = std::min(max_lookahead_, base_lookahead_ + err * error_gain_);
        int l_idx = closest_idx;
        for (int i = 0; i < 100; ++i)
        {
            int idx = (closest_idx + i) % target_path->poses.size();
            double d = std::hypot(target_path->poses[idx].pose.position.x - current_pose_.position.x,
                                  target_path->poses[idx].pose.position.y - current_pose_.position.y);
            if (d > l_dist)
            {
                l_idx = idx;
                break;
            }
        }

        int next_idx = (l_idx + 2) % target_path->poses.size();
        double road_dx = target_path->poses[next_idx].pose.position.x - target_path->poses[l_idx].pose.position.x;
        double road_dy = target_path->poses[next_idx].pose.position.y - target_path->poses[l_idx].pose.position.y;
        double road_yaw = atan2(road_dy, road_dx);

        double tx = target_path->poses[l_idx].pose.position.x;
        double ty = target_path->poses[l_idx].pose.position.y;
        double pp_yaw = atan2(ty - current_pose_.position.y, tx - current_pose_.position.x);
        double my_yaw = tf2::getYaw(current_pose_.orientation);

        double diff_road = road_yaw - my_yaw;
        while (diff_road > M_PI)
            diff_road -= 2 * M_PI;
        while (diff_road < -M_PI)
            diff_road += 2 * M_PI;
        double diff_pp = pp_yaw - my_yaw;
        while (diff_pp > M_PI)
            diff_pp -= 2 * M_PI;
        while (diff_pp < -M_PI)
            diff_pp += 2 * M_PI;

        double steer_pp = atan2(2.0 * wheelbase_ * sin(diff_pp), l_dist);
        double final_steer = (steer_pp * weight_path_) + (diff_road * weight_heading_);
        double s_cmd = final_steer * steer_alpha_ + last_steer_cmd_ * (1.0 - steer_alpha_);
        last_steer_cmd_ = s_cmd;
        s_cmd = std::max(-max_steer_, std::min(max_steer_, s_cmd));

        bool is_emergency_braking = false;
        double target_vel = target_speed_;

        if (is_changing_lane_ || boost_needed)
        {
            target_vel = boost_speed_;
        }

        for (const auto &[id, obs] : tracked_obstacles_)
        {
            Point p = transform_to_body(obs.x, obs.y);
            if (std::abs(p.y) < 0.5)
            {
                double stop_threshold = 0.7;
                double resume_threshold = 1.5; // 다시 출발할 때는 더 확실하게 멉니다

                if (boost_needed || is_changing_lane_)
                {
                    stop_threshold = 0.3;
                    resume_threshold = 0.6;
                }

                // ==========================================
                // [망설임 방지] Hysteresis Logic 적용
                // ==========================================
                if (p.x > 0)
                {
                    if (is_waiting_for_gap_)
                    {
                        // 이미 멈춰있는 상태라면, 장애물이 resume_threshold보다 멀어져야 출발
                        if (p.x < resume_threshold)
                        {
                            target_vel = 0.0;
                            is_emergency_braking = true;
                        }
                        else
                        {
                            // 충분히 멀어짐 -> 출발!
                            is_waiting_for_gap_ = false;
                        }
                    }
                    else
                    {
                        // 달리는 중이라면, stop_threshold 안으로 들어오면 정지
                        if (p.x < stop_threshold)
                        {
                            target_vel = 0.0;
                            is_emergency_braking = true;
                            is_waiting_for_gap_ = true; // 대기 모드 진입
                        }
                    }
                }

                if (!is_emergency_braking && p.x > 0 && p.x < 2.0 && current_speed_ > obs.vx)
                {
                    if (!boost_needed && !is_changing_lane_)
                    {
                        target_vel = obs.vx;
                    }
                }
            }
        }

        // [망설임 방지 2] 너무 느린 속도는 0으로 하거나 최소 속도 보장
        if (!is_emergency_braking && target_vel > 0.01 && target_vel < 0.3)
            target_vel = 0.5;

        double accel = (target_vel - current_speed_) * 1.0;

        // ==========================================
        // [후진 방지] Anti-Reverse Logic
        // ==========================================
        // 1. 이미 차가 멈춰있거나 아주 느릴 때, 목표 속도가 0이면
        //    강한 음수 가속도(후진 명령이 될 수 있음)를 보내지 않고 브레이크만 유지 (-1.0은 보통 브레이크)
        if (current_speed_ < 0.1 && target_vel == 0.0)
        {
            accel = -1.0; // 정지 유지 (Simulator에 따라 -1이 후진이 될 수도 있으니 주의)
        }

        // 2. [강력한 방지] 만약 차가 뒤로 밀리고 있다면(음수 속도),
        //    즉시 양수 가속도를 주어 멈추게 함 (Hill Start Assist 처럼)
        //    단, 주행 방향(시계방향)과 로컬 좌표계가 정렬되어 있다는 가정 하에 x 속도가 음수면 후진임.
        //    여기서는 current_speed_가 절대값이 아니라 부호가 있다고 가정하지 않았으나(pose_callback에서 hypot사용),
        //    만약 simulator feedback이 음수 속도를 준다면 아래 로직이 유효함.
        //    pose_callback에서 hypot만 썼으므로 current_speed_는 항상 양수임.
        //    따라서 "뒤로 가는 것"을 감지하려면 이전 pose와 현재 pose 벡터 내적을 해야 함.
        //    하지만 간단히, '가속도가 계속 음수인데 차가 멈췄다'면 Accel을 0으로 컷 해주는게 안전함.

        if (current_speed_ < 0.05 && accel < -0.5)
        {
            // 이미 멈췄는데 계속 뒤로 당기면 -> 0으로 중립
            // (경사로가 없다면 0이 안전, 경사로라면 -0.1 정도가 안전)
            accel = 0.0;
        }

        geometry_msgs::msg::Accel cmd;
        cmd.linear.x = std::max(-1.0, std::min(1.0, accel));
        cmd.angular.z = s_cmd;
        accel_pub_->publish(cmd);

        visualization_msgs::msg::Marker m;
        m.header.frame_id = "world";
        m.header.stamp = this->now();
        m.id = 999;
        m.type = 2;
        m.action = 0;
        m.pose.position.x = tx;
        m.pose.position.y = ty;
        m.pose.position.z = 0.5;
        m.scale.x = 0.3;
        m.scale.y = 0.3;
        m.scale.z = 0.3;
        m.color.a = 1.0;
        m.color.r = 1.0;
        m.color.g = 0.0;
        m.color.b = 1.0;
        debug_marker_pub_->publish(m);

        publish_risk_visualization();
    }

    void publish_risk_visualization()
    {
        visualization_msgs::msg::MarkerArray markers;
        for (auto &[id, obs] : tracked_obstacles_)
        {
            // 오래된 장애물은 표시하지 않음
            if ((this->now() - obs.last_seen).seconds() > 1.0)
                continue;

            visualization_msgs::msg::Marker text;
            text.header.frame_id = "world";
            text.header.stamp = this->now();
            text.ns = "risk_val";
            text.id = id;
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::msg::Marker::ADD;
            text.pose.position.x = obs.x;
            text.pose.position.y = obs.y;
            text.pose.position.z = 1.5; // 차 위로 잘 보이게 띄움
            text.scale.z = 0.5;

            text.color.a = 1.0;
            // 리스크가 높으면 빨간색, 낮으면 초록색
            if (obs.risk_score > 0.5)
            {
                text.color.r = 1.0;
                text.color.g = 0.0;
            }
            else
            {
                text.color.r = 0.0;
                text.color.g = 1.0;
            }
            text.color.b = 0.0;

            // [수정] ID 제거하고 Risk Score만 소수점 2자리로 표시
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << obs.risk_score;
            text.text = ss.str();

            markers.markers.push_back(text);
        }
        risk_text_pub_->publish(markers);
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_lane_1_, sub_lane_2_, sub_lane_3_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obs_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr accel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr risk_text_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path::SharedPtr path_lane_1_, path_lane_2_, path_lane_3_;
    geometry_msgs::msg::Pose current_pose_;
    std::map<int, ObstacleState> tracked_obstacles_;

    LaneID current_target_lane_;
    double current_speed_;
    int last_closest_idx_;
    double last_steer_cmd_;
    bool is_changing_lane_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrenetPlanner>());
    rclcpp::shutdown();
    return 0;
}
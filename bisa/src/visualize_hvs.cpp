#include "bisa/visualize_hvs.hpp"

using namespace std::chrono_literals;

ObstacleRelay::ObstacleRelay() : Node("obstacle_relay")
{
    // [수정 1] Static 기준 대폭 하향 (RC카 스케일 반영)
    // 0.5m/s 차량을 감지해야 하므로 0.1로 설정
    vel_static_thres_ = 0.1;

    // [수정 2] 초기 기준값도 RC카 속도 대역(0.5 ~ 0.75)의 사이값으로 설정
    vel_slow_thres_ = 0.6;

    roi_front_ = 30.0;

    rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
    qos_profile.best_effort();
    qos_profile.durability_volatile();

    ego_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/Ego_pose", qos_profile,
        std::bind(&ObstacleRelay::ego_callback, this, std::placeholders::_1));

    for (int i = 19; i <= 36; ++i)
    {
        std::string topic_name = "/HV_" + std::to_string(i);
        std::function<void(const geometry_msgs::msg::PoseStamped::SharedPtr)> cb =
            [this, topic_name](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            this->hv_callback(msg, topic_name);
        };
        hv_subs_.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>(
            topic_name, qos_profile, cb));
    }

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/obstacles_markers", 10);
    pub_slow_vel_ = this->create_publisher<std_msgs::msg::Float32>("/env/slow_vel", 10);
    pub_fast_vel_ = this->create_publisher<std_msgs::msg::Float32>("/env/fast_vel", 10);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&ObstacleRelay::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "RC-Scale Visualizer Started");
}

void ObstacleRelay::update_dynamic_threshold()
{
    std::vector<double> moving_speeds;

    // 1. 데이터 수집
    for (auto const &[topic, data] : obstacles_)
    {
        if ((this->now().seconds() - data.time) > 1.0)
            continue;
        // 0.1(최소) ~ 3.0(최대) 사이의 유효한 속도만 수집
        if (data.vel > vel_static_thres_ && data.vel < 3.0)
        {
            moving_speeds.push_back(data.vel);
        }
    }

    if (moving_speeds.size() < 2)
        return;

    std::sort(moving_speeds.begin(), moving_speeds.end());
    double min_v = moving_speeds.front();
    double max_v = moving_speeds.back();

    // [범용성 핵심]
    // 특정 숫자가 아니라 "두 그룹으로 나눌 수 있는가?"만 봅니다.
    // 0.3 vs 0.5 상황에서도 차이는 0.2이므로 0.15보다 큽니다. (작동 O)
    // 0.7 vs 1.0 상황에서도 차이는 0.3이므로 0.15보다 큽니다. (작동 O)
    if ((max_v - min_v) >= 0.15)
    {
        double mid_ref = (min_v + max_v) / 2.0;
        double sum_slow = 0.0, cnt_slow = 0.0;
        double sum_fast = 0.0, cnt_fast = 0.0;

        for (double v : moving_speeds)
        {
            if (v < mid_ref)
            {
                sum_slow += v;
                cnt_slow++;
            }
            else
            {
                sum_fast += v;
                cnt_fast++;
            }
        }

        // 두 그룹이 모두 존재할 때만 기준값을 업데이트합니다.
        // (한 그룹만 있을 때는 이전 기준값을 유지하는 것이 가장 안전함)
        if (cnt_slow > 0 && cnt_fast > 0)
        {
            double avg_slow = sum_slow / cnt_slow;
            double avg_fast = sum_fast / cnt_fast;

            this->slow_vel = avg_slow;
            this->fast_vel = avg_fast;

            double new_threshold = (avg_slow + avg_fast) / 2.0;

            // [안전장치] 기준값이 너무 터무니없이 튀지 않도록 Clamp
            // RC카 범위를 고려해 0.2 ~ 1.5 사이로 제한
            if (new_threshold < 0.2)
                new_threshold = 0.2;
            if (new_threshold > 1.5)
                new_threshold = 1.5;

            // Soft Update (천천히 반영)
            vel_slow_thres_ = (vel_slow_thres_ * 0.9) + (new_threshold * 0.1);

            // 디버그 출력 (1초에 1번 제한)
            // RCLCPP_WARN_THROTTLE(
            //     this->get_logger(),
            //     *this->get_clock(),
            //     1000,
            //     "FAST/SLOW SPLIT → slow_avg=%.2f m/s, fast_avg=%.2f m/s, thres=%.2f",
            //     avg_slow, avg_fast, new_threshold);
        }
    }
    // else 블록 제거: 한 그룹만 있을 때는 섣불리 판단하지 않고 기존 기준 유지
}

void ObstacleRelay::ego_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    ego_pose_ = msg;
}

void ObstacleRelay::hv_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const std::string &topic_name)
{
    double now_sec = this->now().seconds();
    double calc_vel = 0.0;
    bool has_prev = false;
    double prev_vel = 0.0;

    if (obstacles_.find(topic_name) != obstacles_.end())
    {
        has_prev = true;
        prev_vel = obstacles_[topic_name].vel;
        double dt = now_sec - obstacles_[topic_name].time;

        if (dt < 0.02)
            return;

        double dist = std::sqrt(std::pow(msg->pose.position.x - obstacles_[topic_name].x, 2) +
                                std::pow(msg->pose.position.y - obstacles_[topic_name].y, 2));

        // [수정 6] 이동 거리 노이즈 기준도 스케일에 맞게 축소 (0.5cm 미만 무시)
        if (dist < 0.005)
        {
            calc_vel = prev_vel * 0.9;
        }
        else
        {
            double raw_vel = dist / dt;
            // 튀는 값 기준 5.0m/s로 하향
            if (raw_vel < 5.0)
            {
                if (prev_vel == 0.0)
                    calc_vel = raw_vel;
                else
                    calc_vel = (prev_vel * 0.4) + (raw_vel * 0.6);
            }
            else
            {
                calc_vel = prev_vel;
            }
        }
    }

    obstacles_[topic_name] = {
        msg->pose.position.x, msg->pose.position.y, msg->pose.orientation.z,
        now_sec, calc_vel, "UNKNOWN"};
}

void ObstacleRelay::timer_callback()
{
    update_dynamic_threshold();

    std_msgs::msg::Float32 slow_msg;
    slow_msg.data = this->slow_vel; // 또는 실제 평균 속도 변수
    pub_slow_vel_->publish(slow_msg);

    std_msgs::msg::Float32 fast_msg;
    fast_msg.data = this->fast_vel; // 클래스 내 계산된 fast 속도
    pub_fast_vel_->publish(fast_msg);

    double ego_x = 0.0, ego_y = 0.0;
    if (ego_pose_)
    {
        ego_x = ego_pose_->pose.position.x;
        ego_y = ego_pose_->pose.position.y;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    int id_cnt = 0;
    rclcpp::Time now = this->now();

    for (auto &[topic, data] : obstacles_)
    {
        if ((now.seconds() - data.time) > 1.5)
            continue;
        if (std::sqrt(std::pow(data.x - ego_x, 2) + std::pow(data.y - ego_y, 2)) > roi_front_)
            continue;

        // [분류]
        if (data.vel < vel_static_thres_)
            data.type = "STATIC";
        else if (data.vel < vel_slow_thres_)
            data.type = "SLOW";
        else
            data.type = "FAST";

        // 마커 (차량)
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = now;
        marker.ns = "surrounding_cars";
        marker.id = id_cnt++;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = data.x;
        marker.pose.position.y = data.y;
        marker.pose.position.z = 0.15;

        marker.pose.orientation.w = cos(data.yaw * 0.5);
        marker.pose.orientation.z = sin(data.yaw * 0.5);

        marker.scale.x = 0.33;
        marker.scale.y = 0.15;
        marker.scale.z = 0.2;
        marker.color.a = 0.9;

        if (data.type == "STATIC")
        {
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.5;
        }
        else if (data.type == "SLOW")
        {
            marker.color.r = 1.0;
            marker.color.g = 0.8;
            marker.color.b = 0.0;
        }
        else
        {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }

        marker.lifetime = rclcpp::Duration::from_seconds(0.2);
        marker_array.markers.push_back(marker);

        // [디버깅용 텍스트] 속도와 기준값 표시
        visualization_msgs::msg::Marker text;
        text.header.frame_id = "world";
        text.header.stamp = now;
        text.ns = "car_info";
        text.id = id_cnt + 1000;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position.x = data.x;
        text.pose.position.y = data.y;
        text.pose.position.z = 0.5;
        text.scale.z = 0.25;
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 1.0;
        text.color.a = 1.0;

        std::stringstream ss;
        ss << data.type << "\n"
           << std::fixed << std::setprecision(2) << data.vel // 소수점 2자리로 변경
           << " | T:" << vel_slow_thres_;

        text.text = ss.str();
        text.lifetime = rclcpp::Duration::from_seconds(0.2);
        marker_array.markers.push_back(text);
    }
    marker_pub_->publish(marker_array);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleRelay>());
    rclcpp::shutdown();
    return 0;
}
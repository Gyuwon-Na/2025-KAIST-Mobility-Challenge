#include "bisa/local_path_pub_cpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

namespace bisa
{

    LocalPathPubCpp::LocalPathPubCpp()
        : Node("local_path_pub_cpp"),
          lap_start_time_(this->now())
    {

        // RCLCPP_INFO(this->get_logger(), "===========================================");
        // RCLCPP_INFO(this->get_logger(), "Local Path Publisher C++ - 1kHz");
        // RCLCPP_INFO(this->get_logger(), "Infinite Loop Mode with LapInfo");
        // RCLCPP_INFO(this->get_logger(), "===========================================");

        // [추가] 내 차량 ID 파라미터
        this->declare_parameter("target_cav_id", 1);
        int target_id = this->get_parameter("target_cav_id").as_int();
        std::string id_str = (target_id < 10) ? "0" + std::to_string(target_id) : std::to_string(target_id);

        // ★ [추가] RViz 슬롯 파라미터
        this->declare_parameter("rviz_slot", -1);
        rviz_slot_ = this->get_parameter("rviz_slot").as_int();

        // RCLCPP_INFO(this->get_logger(), "Local Path Pub for CAV_%s (RViz Slot: %d)", id_str.c_str(), rviz_slot_);

        // 기존 토픽 설정
        std::string global_path_topic = "/user_global_path_cav" + id_str;
        std::string pose_topic = "/CAV_" + id_str;
        std::string local_pub_topic = "/local_path_cav" + id_str;
        std::string lap_info_topic = "/lap_info_cav" + id_str;

        global_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            global_path_topic,
            rclcpp::QoS(10).transient_local(),
            std::bind(&LocalPathPubCpp::global_path_callback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&LocalPathPubCpp::pose_callback, this, std::placeholders::_1));

        local_pub_ = this->create_publisher<nav_msgs::msg::Path>(local_pub_topic, 10);
        lap_info_pub_ = this->create_publisher<bisa::msg::LapInfo>(lap_info_topic, 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/car_marker_" + id_str, 10);

        // ★ [추가] RViz용 퍼블리셔 (슬롯이 유효할 때만)
        if (rviz_slot_ >= 0)
        {
            std::string rviz_local_topic = "/viz/slot" + std::to_string(rviz_slot_) + "/local_path";
            std::string rviz_marker_topic = "/viz/slot" + std::to_string(rviz_slot_) + "/car_marker";

            rviz_local_pub_ = this->create_publisher<nav_msgs::msg::Path>(rviz_local_topic, 10);
            rviz_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(rviz_marker_topic, 10);

            // RCLCPP_INFO(this->get_logger(), "RViz topics: %s, %s", rviz_local_topic.c_str(), rviz_marker_topic.c_str());
        }

        // 1kHz timer
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(1000),
            std::bind(&LocalPathPubCpp::publish_local_path, this));

        // RCLCPP_INFO(this->get_logger(), "Ready!");
    }

    void LocalPathPubCpp::global_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        // if (!global_path_)
        // {
        //     RCLCPP_INFO(this->get_logger(), "Global Path: %zu pts", msg->poses.size());
        // }
        global_path_ = msg;
    }

    void LocalPathPubCpp::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = msg->pose;

        // 총 주행 거리 계산
        if (prev_pose_)
        {
            double dx = current_pose_->position.x - prev_pose_->position.x;
            double dy = current_pose_->position.y - prev_pose_->position.y;
            total_distance_ += std::sqrt(dx * dx + dy * dy);
        }
        prev_pose_ = current_pose_;

        publish_car_marker();
    }

    void LocalPathPubCpp::publish_car_marker()
    {
        if (!current_pose_)
            return;

        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "my_car";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position = current_pose_->position;

        // Euler to Quaternion
        double roll = current_pose_->orientation.x;
        double pitch = current_pose_->orientation.y;
        double yaw = current_pose_->orientation.z;

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        marker.pose.orientation = tf2::toMsg(q);

        marker.scale.x = 0.33;
        marker.scale.y = 0.15;
        marker.scale.z = 0.2;

        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;

        marker_pub_->publish(marker);

        if (rviz_marker_pub_)
        {
            rviz_marker_pub_->publish(marker);
        }
    }

    void LocalPathPubCpp::publish_local_path()
    {
        if (!global_path_ || !current_pose_)
            return;

        size_t total = global_path_->poses.size();
        if (total == 0)
            return;

        double x = current_pose_->position.x;
        double y = current_pose_->position.y;

        // Find closest waypoint
        double min_d = 1e9;
        size_t closest = current_waypoint_;

        // 만약 초기화 전이라면 전체 탐색 (지난번 답변의 is_initialized_ 활용 가정)
        // 혹은 별도 플래그가 없다면, 그냥 아래 로직이 자동으로 커버합니다.
        bool need_global_search = !is_initialized_;

        if (!need_global_search)
        {
            for (int i = -search_window; i <= search_window; ++i)
            {
                long idx_temp = static_cast<long>(current_waypoint_) + i;

                // 인덱스 순환 처리 (음수 및 초과 방지)
                if (idx_temp < 0)
                    idx_temp += total;
                else if (idx_temp >= static_cast<long>(total))
                    idx_temp %= total;

                size_t idx = static_cast<size_t>(idx_temp);

                double dx = global_path_->poses[idx].pose.position.x - x;
                double dy = global_path_->poses[idx].pose.position.y - y;
                double d = std::sqrt(dx * dx + dy * dy);

                if (d < min_d)
                {
                    min_d = d;
                    closest = idx;
                }
            }
        }

        // 2. [핵심] "차를 들어서 옮긴 경우" 감지 (Global Re-initialization)
        // Window 탐색 결과 가장 가까운 점이 2.0m 이상 떨어져 있다면,
        // 차를 멀리 옮겼다고 판단하고 전체 경로에서 다시 찾습니다.
        if (need_global_search || min_d > 2.0)
        {
            min_d = 1e9; // 거리 초기화
            for (size_t i = 0; i < total; ++i)
            {
                double dx = global_path_->poses[i].pose.position.x - x;
                double dy = global_path_->poses[i].pose.position.y - y;
                double d = std::sqrt(dx * dx + dy * dy);

                if (d < min_d)
                {
                    min_d = d;
                    closest = i;
                }
            }
            // 전역 탐색을 수행했다면 초기화 완료 처리
            is_initialized_ = true;
            // RCLCPP_WARN(this->get_logger(), "Relocated! Reset path index to %zu", closest);
        }

        // ==========================================

        // 한 바퀴 완료 감지
        // (갑자기 뒤로 옮겼을 때 랩 카운트가 오작동하지 않도록 방어 코드 추가 권장)
        // 단순히 인덱스가 줄어든 경우는 랩 변경이 아님.
        if (closest < current_waypoint_ && current_waypoint_ > total * 0.9 && closest < total * 0.1)
        {
            lap_count_++;
            lap_start_time_ = this->now();
            // RCLCPP_INFO(this->get_logger(),
            //             "🏁 Lap %d completed! Total distance: %.2fm",
            //             lap_count_, total_distance_);
        }

        current_waypoint_ = closest;

        // 진행률 계산 (0.0 ~ 100.0)
        double progress = (static_cast<double>(current_waypoint_) / total) * 100.0;

        // 현재 바퀴 경과 시간
        double elapsed_time = (this->now() - lap_start_time_).seconds();

        // LapInfo 메시지 생성 및 발행
        auto lap_info = bisa::msg::LapInfo();
        lap_info.lap_count = lap_count_;
        lap_info.progress = static_cast<float>(progress);
        lap_info.current_waypoint = static_cast<int32_t>(current_waypoint_);
        lap_info.total_waypoints = static_cast<int32_t>(total);
        lap_info.elapsed_time = static_cast<float>(elapsed_time);
        lap_info.total_distance = static_cast<float>(total_distance_);

        lap_info_pub_->publish(lap_info);

        // 10초마다 로그
        auto now = this->now();
        if ((now - last_log_time_).seconds() > 10.0)
        {
            // RCLCPP_INFO(this->get_logger(),
            //             "Lap %d | Progress: %.1f%% | Time: %.1fs | Distance: %.2fm",
            //             lap_count_, progress, elapsed_time, total_distance_);
            last_log_time_ = now;
        }

        // Local path (순환 경로)
        auto local = nav_msgs::msg::Path();
        local.header.frame_id = "world";
        local.header.stamp = this->now();

        for (size_t i = 0; i < local_path_size_; ++i)
        {
            size_t idx = (current_waypoint_ + i) % total; // 순환!
            local.poses.push_back(global_path_->poses[idx]);
        }

        local_pub_->publish(local);

        // ★ [추가] RViz용 토픽에도 발행
        if (rviz_local_pub_)
        {
            rviz_local_pub_->publish(local);
        }
    }

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bisa::LocalPathPubCpp>());
    rclcpp::shutdown();
    return 0;
}
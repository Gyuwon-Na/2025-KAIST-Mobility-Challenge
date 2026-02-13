#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <thread>
#include <atomic>

class SynchronizedStartNode : public rclcpp::Node
{
public:
    SynchronizedStartNode()
        : Node("synchronized_start_node"), is_started_(false), all_ready_(false)
    {
        this->declare_parameter("cav_ids", std::vector<int64_t>{1, 2, 3, 4});
        std::vector<int64_t> ids = this->get_parameter("cav_ids").as_integer_array();

        for (int64_t id_val : ids)
        {
            int id = static_cast<int>(id_val);
            std::string id_str = (id < 10) ? "0" + std::to_string(id) : std::to_string(id);

            std::string sub_topic = "/CAV_" + id_str + "_accel_raw";
            std::string pub_topic = "/CAV_" + id_str + "_accel_sync";

            auto callback = [this, id](const geometry_msgs::msg::Accel::SharedPtr msg)
            {
                this->accel_callback(id, msg);
            };
            subscribers_[id] = this->create_subscription<geometry_msgs::msg::Accel>(sub_topic, 10, callback);
            publishers_[id] = this->create_publisher<geometry_msgs::msg::Accel>(pub_topic, 10);
            target_ids_.push_back(id);
        }

        // ROS2 서비스 생성 - 시작 신호를 받기 위한 서비스
        start_service_ = this->create_service<std_srvs::srv::Trigger>(
            "start_race",
            std::bind(&SynchronizedStartNode::start_race_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Waiting for %zu CAVs to be ready...", target_ids_.size());
        RCLCPP_INFO(this->get_logger(), "Start service '/start_race' is ready.");
        RCLCPP_INFO(this->get_logger(), "Call 'ros2 service call /start_race std_srvs/srv/Trigger' to start!");
    }

    ~SynchronizedStartNode()
    {
    }

private:
    void start_race_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request; // unused parameter

        if (!all_ready_)
        {
            response->success = false;
            response->message = "Not all CAVs are ready yet!";
            RCLCPP_WARN(this->get_logger(), "Start request rejected: Not all CAVs ready (%zu/%zu)",
                        initial_buffer_.size(), target_ids_.size());
            return;
        }

        if (is_started_)
        {
            response->success = false;
            response->message = "Race already started!";
            RCLCPP_WARN(this->get_logger(), "Start request rejected: Already started");
            return;
        }

        // 레이스 시작!
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "  RELEASE THE KRAKEN! (Simultaneous Start)");
        RCLCPP_INFO(this->get_logger(), "========================================");

        is_started_ = true;
        publish_buffered_data();

        response->success = true;
        response->message = "Race started successfully!";
    }

    void accel_callback(int id, const geometry_msgs::msg::Accel::SharedPtr msg)
    {
        if (is_started_)
        {
            // 출발 이후에는 드레이 없이 즉시 통과
            publishers_[id]->publish(*msg);
            return;
        }

        // 출발 전: 버퍼에 지속적으로 최신 계산값 업데이트 (데이터 소실/왜곡 방지)
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        if (initial_buffer_.find(id) == initial_buffer_.end())
        {
            RCLCPP_INFO(this->get_logger(), "CAV_%02d is Ready. (%zu/%zu)",
                        id, initial_buffer_.size() + 1, target_ids_.size());
        }
        initial_buffer_[id] = *msg;

        if (initial_buffer_.size() == target_ids_.size() && !all_ready_)
        {
            all_ready_ = true;
            RCLCPP_INFO(this->get_logger(), "");
            RCLCPP_INFO(this->get_logger(), "========================================");
            RCLCPP_INFO(this->get_logger(), "  ALL CAVs READY!");
            RCLCPP_INFO(this->get_logger(), "  Call service to start:");
            RCLCPP_INFO(this->get_logger(), "  ros2 service call /start_race std_srvs/srv/Trigger");
            RCLCPP_INFO(this->get_logger(), "========================================");
            RCLCPP_INFO(this->get_logger(), "");
        }
    }

    void publish_buffered_data()
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        for (int id : target_ids_)
        {
            publishers_[id]->publish(initial_buffer_[id]);
        }
        initial_buffer_.clear();
    }

    std::atomic<bool> is_started_;
    std::atomic<bool> all_ready_;
    std::vector<int> target_ids_;
    std::map<int, geometry_msgs::msg::Accel> initial_buffer_;
    std::map<int, rclcpp::Subscription<geometry_msgs::msg::Accel>::SharedPtr> subscribers_;
    std::map<int, rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr> publishers_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;

    std::mutex buffer_mutex_; // 버퍼 접근 시 충돌 방지
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SynchronizedStartNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
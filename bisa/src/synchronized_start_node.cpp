#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/accel.hpp>
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

        RCLCPP_INFO(this->get_logger(), "Waiting for %zu CAVs to be ready...", target_ids_.size());

        // 키 입력을 대기할 별도 쓰레드 생성
        input_thread_ = std::thread(&SynchronizedStartNode::wait_for_enter, this);
    }

    ~SynchronizedStartNode()
    {
        if (input_thread_.joinable())
        {
            input_thread_.detach(); // 노드 종료 시 쓰레드 분리
        }
    }

private:
    void wait_for_enter()
    {
        while (rclcpp::ok())
        {
            if (all_ready_ && !is_started_)
            {
                std::cout << "\n========================================" << std::endl;
                std::cout << "  ALL CAVs READY! PRESS [ENTER] TO START!" << std::endl;
                std::cout << "========================================\n"
                          << std::endl;

                std::cin.get(); // 엔터 키 입력 대기 (블로킹)

                RCLCPP_INFO(this->get_logger(), "RELEASE THE KRAKEN! (Simultaneous Start)");
                is_started_ = true;

                // 엔터 누르는 즉시 버퍼에 저장된 최신 값 발행
                publish_buffered_data();
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void accel_callback(int id, const geometry_msgs::msg::Accel::SharedPtr msg)
    {
        if (is_started_)
        {
            // 출발 이후에는 딜레이 없이 즉시 통과
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

        if (initial_buffer_.size() == target_ids_.size())
        {
            all_ready_ = true;
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

    std::thread input_thread_;
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
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <map>
#include <vector>
#include <string>

class SynchronizedStartNode : public rclcpp::Node
{
public:
    SynchronizedStartNode()
        : Node("synchronized_start_node"), is_started_(false)
    {
        // 1. 파라미터: 동기화할 차량 ID 목록 (기본값 1,2,3,4)
        this->declare_parameter("cav_ids", std::vector<int64_t>{1, 2, 3, 4});
        std::vector<int64_t> ids = this->get_parameter("cav_ids").as_integer_array();

        for (int64_t id_val : ids)
        {
            int id = static_cast<int>(id_val);
            std::string id_str = (id < 10) ? "0" + std::to_string(id) : std::to_string(id);

            // 입력: MPC가 보내는 Raw 토픽
            std::string sub_topic = "/CAV_" + id_str + "_accel_raw";
            // 출력: 실제 차량/시뮬레이터가 받는 토픽
            std::string pub_topic = "/CAV_" + id_str + "_accel_sync";

            // Subscriber (Lambda로 ID 캡처)
            auto callback = [this, id](const geometry_msgs::msg::Accel::SharedPtr msg)
            {
                this->accel_callback(id, msg);
            };
            subscribers_[id] = this->create_subscription<geometry_msgs::msg::Accel>(
                sub_topic, 10, callback);

            // Publisher
            publishers_[id] = this->create_publisher<geometry_msgs::msg::Accel>(pub_topic, 10);

            // 관리용 리스트에 추가
            target_ids_.push_back(id);
        }

        RCLCPP_INFO(this->get_logger(), "Waiting for %zu CAVs to be ready...", target_ids_.size());
    }

private:
    void accel_callback(int id, const geometry_msgs::msg::Accel::SharedPtr msg)
    {
        // [CASE 1] 이미 출발했다면? -> 지연 없이 즉시 통과 (Passthrough)
        if (is_started_)
        {
            publishers_[id]->publish(*msg);
            return;
        }

        // [CASE 2] 아직 출발 전이라면? -> 버퍼에 저장하고 대기
        if (initial_buffer_.find(id) == initial_buffer_.end())
        {
            initial_buffer_[id] = *msg;
            RCLCPP_INFO(this->get_logger(), "CAV_%02d is Ready. (%zu/%zu)",
                        id, initial_buffer_.size(), target_ids_.size());
        }
        else
        {
            // 이미 준비된 차량이 또 데이터를 보내면 최신 값으로 갱신만 해둠
            initial_buffer_[id] = *msg;
        }

        // 모든 차량이 준비되었는지 확인
        check_and_start();
    }

    void check_and_start()
    {
        // 등록된 모든 ID가 버퍼에 들어왔는지 확인
        if (initial_buffer_.size() == target_ids_.size())
        {
            RCLCPP_INFO(this->get_logger(), "All CAVs are ready! RELEASE THE KRAKEN! (Simultaneous Start)");

            // 1. 버퍼에 모아둔 첫 메시지들을 일괄 전송
            for (int id : target_ids_)
            {
                publishers_[id]->publish(initial_buffer_[id]);
            }

            // 2. 상태 플래그 변경 (이제부터는 대기 안 함)
            is_started_ = true;

            // 3. 메모리 정리를 위해 버퍼 비움 (선택 사항)
            initial_buffer_.clear();
        }
    }

    // 멤버 변수
    bool is_started_;
    std::vector<int> target_ids_;
    std::map<int, geometry_msgs::msg::Accel> initial_buffer_; // 초기 데이터 저장소
    std::map<int, rclcpp::Subscription<geometry_msgs::msg::Accel>::SharedPtr> subscribers_;
    std::map<int, rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr> publishers_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SynchronizedStartNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
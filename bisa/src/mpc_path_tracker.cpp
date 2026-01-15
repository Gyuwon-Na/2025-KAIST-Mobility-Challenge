#include "bisa/mpc_path_tracker.hpp"

namespace bisa
{

    MPCPathTrackerCpp::MPCPathTrackerCpp()
        : Node("mpc_path_tracker"), last_log_time_(this->now())
    {

        RCLCPP_INFO(this->get_logger(), "===========================================");
        RCLCPP_INFO(this->get_logger(), "MPC Path Tracker C++ - 1kHz (YAML Config)");
        RCLCPP_INFO(this->get_logger(), "===========================================");

        // [수정] 모든 파라미터 선언 (기본값은 YAML이 없을 때 사용됨)

        // 1. 가중치 (Weights)
        this->declare_parameter("Q_pos", 15.0);
        this->declare_parameter("Q_heading", 8.0);
        this->declare_parameter("R_v", 0.5);
        this->declare_parameter("R_w", 0.5);

        // 2. 차량 제약 조건 (Constraints)
        this->declare_parameter("max_velocity", 3.0);
        this->declare_parameter("max_accel", 2.0);
        this->declare_parameter("max_angular_vel", 2.0);
        this->declare_parameter("min_velocity", 0.0);

        // 3. MPC 설정 & 차량 물리 정보
        this->declare_parameter("horizon", 20);
        this->declare_parameter("dt", 0.05);            // 제어 주기
        this->declare_parameter("wheelbase", 0.33);     // 축거
        this->declare_parameter("vehicle_width", 0.15); // 차량 폭
        this->declare_parameter("lookahead_base", 0.3);
        this->declare_parameter("lookahead_per_speed", 0.2);

        // 파라미터 변경 콜백 등록 (런타임 튜닝용)
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&MPCPathTrackerCpp::parameter_callback, this, std::placeholders::_1));

        // 컨트롤러 초기화 및 파라미터 적용
        controller_ = std::make_unique<MPCControllerCpp>();
        update_controller_params();

        // Subscribers & Publishers
        local_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/local_path", 10,
            std::bind(&MPCPathTrackerCpp::local_path_callback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose",
            rclcpp::SensorDataQoS(),
            std::bind(&MPCPathTrackerCpp::pose_callback, this, std::placeholders::_1));

        target_vel_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/planning/target_v", 10,
            std::bind(&MPCPathTrackerCpp::target_vel_callback, this, std::placeholders::_1));

        accel_pub_ = this->create_publisher<geometry_msgs::msg::Accel>("/Accel", 10);
        pred_pub_ = this->create_publisher<nav_msgs::msg::Path>("/mpc_predicted_path", 10);

        // 1kHz timer (YAML의 dt와 별개로 루프 주기는 고정하거나 파라미터화 가능)
        // 여기서는 안전하게 1ms(1kHz) 루프 유지
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(1000),
            std::bind(&MPCPathTrackerCpp::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Ready (1kHz)");
    }

    void MPCPathTrackerCpp::update_controller_params()
    {
        MPCParams params;

        // [수정] ROS 파라미터 서버(YAML)에서 값 읽어오기
        params.Q_pos = this->get_parameter("Q_pos").as_double();
        params.Q_heading = this->get_parameter("Q_heading").as_double();
        params.R_v = this->get_parameter("R_v").as_double();
        params.R_w = this->get_parameter("R_w").as_double();

        params.max_velocity = this->get_parameter("max_velocity").as_double();
        params.max_accel = this->get_parameter("max_accel").as_double();
        params.max_angular_vel = this->get_parameter("max_angular_vel").as_double();
        params.min_velocity = this->get_parameter("min_velocity").as_double();

        params.horizon = this->get_parameter("horizon").as_int();
        params.dt = this->get_parameter("dt").as_double();
        params.wheelbase = this->get_parameter("wheelbase").as_double();
        params.vehicle_width = this->get_parameter("vehicle_width").as_double();
        params.lookahead_base = this->get_parameter("lookahead_base").as_double();
        params.lookahead_per_speed = this->get_parameter("lookahead_per_speed").as_double();

        // 컨트롤러에 업데이트된 파라미터 전달
        controller_->update_parameters(params);
    }

    rcl_interfaces::msg::SetParametersResult MPCPathTrackerCpp::parameter_callback(
        const std::vector<rclcpp::Parameter> &params)
    {
        for (const auto &param : params)
        {
            RCLCPP_INFO(this->get_logger(), "Param updated: %s", param.get_name().c_str());
        }

        // 파라미터가 변경되면 컨트롤러에 즉시 반영
        update_controller_params();

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    void MPCPathTrackerCpp::local_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        local_path_ = msg->poses;
    }

    void MPCPathTrackerCpp::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = msg->pose;
    }
    void MPCPathTrackerCpp::target_vel_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        current_target_vel_ = msg->data;
    }

    void MPCPathTrackerCpp::control_loop()
    {
        if (local_path_.empty() || !current_pose_)
            return;

        // [핵심 수정] 수신된 목표 속도를 MPC 파라미터에 반영
        // Planner가 0을 보내면 MPC도 즉시 0을 목표로 함 (단, 급정거 방지는 MPC 내부 로직에 맡김)
        MPCParams current_params = controller_->get_parameters(); // getter 필요 (없으면 params_ 접근)

        // YAML 설정값(상한선)과 Planner 명령값 중 작은 것을 선택
        double yaml_max_vel = this->get_parameter("max_velocity").as_double();
        current_params.max_velocity = std::min(yaml_max_vel, current_target_vel_);

        // 컨트롤러에 업데이트된 파라미터 적용 (매 루프마다 적용)
        controller_->update_parameters(current_params);

        // 경로가 너무 짧으면 정지
        if (local_path_.size() < 2)
        {
            publish_control(0.0, 0.0);
            return;
        }

        auto output = controller_->compute_control(*current_pose_, local_path_);
        publish_control(output.velocity, output.angular_velocity);
        publish_predicted_path(output.predicted_trajectory);

        auto now = this->now();
        if ((now - last_log_time_).seconds() > 2.0)
        {
            RCLCPP_INFO(this->get_logger(), "MPC | v: %.2f m/s, w: %.3f rad/s",
                        output.velocity, output.angular_velocity);
            last_log_time_ = now;
        }
    }

    void MPCPathTrackerCpp::publish_control(double v, double w)
    {
        auto msg = geometry_msgs::msg::Accel();
        msg.linear.x = v;
        msg.angular.z = w;
        accel_pub_->publish(msg);
    }

    void MPCPathTrackerCpp::publish_predicted_path(
        const std::vector<std::array<double, 3>> &traj)
    {
        auto msg = nav_msgs::msg::Path();
        msg.header.frame_id = "world";
        msg.header.stamp = this->now();

        for (const auto &pt : traj)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = msg.header;
            pose.pose.position.x = pt[0];
            pose.pose.position.y = pt[1];
            pose.pose.position.z = pt[2];
            pose.pose.orientation.w = 1.0;
            msg.poses.push_back(pose);
        }

        pred_pub_->publish(msg);
    }

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bisa::MPCPathTrackerCpp>());
    rclcpp::shutdown();
    return 0;
}

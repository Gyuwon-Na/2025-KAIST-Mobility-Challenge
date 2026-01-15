#include "bisa/mpc_path_tracker.hpp"
#include "std_msgs/msg/float32.hpp"

namespace bisa
{

    MPCPathTrackerCpp::MPCPathTrackerCpp()
        : Node("mpc_path_tracker"), last_log_time_(this->now())
    {

        RCLCPP_INFO(this->get_logger(), "===========================================");
        RCLCPP_INFO(this->get_logger(), "MPC Path Tracker C++ - 1kHz (Hybrid Speed Control)");
        RCLCPP_INFO(this->get_logger(), "===========================================");

        // 1. 媛�以묒튂 (Weights)
        this->declare_parameter("Q_pos", 15.0);
        this->declare_parameter("Q_heading", 8.0);
        this->declare_parameter("R_v", 0.5);
        this->declare_parameter("R_w", 0.5);

        // 2. 李⑤웾 �쒖빟 議곌굔 (Constraints)
        this->declare_parameter("max_velocity", 3.0);
        this->declare_parameter("max_accel", 2.0);
        this->declare_parameter("max_angular_vel", 2.0);
        this->declare_parameter("min_velocity", 0.0);

        // 3. MPC �ㅼ젙 & 李⑤웾 臾쇰━ �뺣낫
        this->declare_parameter("horizon", 20);
        this->declare_parameter("dt", 0.05);            // �쒖뼱 二쇨린
        this->declare_parameter("wheelbase", 0.33);     // 異뺢굅
        this->declare_parameter("vehicle_width", 0.15); // 李⑤웾 ��
        this->declare_parameter("lookahead_base", 0.3);
        this->declare_parameter("lookahead_per_speed", 0.2);

        // �뚮씪誘명꽣 蹂�寃� 肄쒕갚 �깅줉
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&MPCPathTrackerCpp::parameter_callback, this, std::placeholders::_1));

        // 而⑦듃濡ㅻ윭 珥덇린�� 諛� 湲곕낯 �뚮씪誘명꽣 �곸슜
        controller_ = std::make_unique<MPCControllerCpp>();
        update_controller_params(); // 珥덇린�� �쒖뿉�� 湲곕낯 max_velocity �ъ슜

        // Subscribers & Publishers
        local_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/local_path", 10,
            std::bind(&MPCPathTrackerCpp::local_path_callback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose",
            rclcpp::SensorDataQoS(),
            std::bind(&MPCPathTrackerCpp::pose_callback, this, std::placeholders::_1));

        accel_pub_ = this->create_publisher<geometry_msgs::msg::Accel>("/Accel", 10);
        pred_pub_ = this->create_publisher<nav_msgs::msg::Path>("/mpc_predicted_path", 10);

        // [異붽�] Planner�먯꽌 �ㅻ뒗 Smart Velocity 援щ룆
        target_vel_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/planning/target_v", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            {
                current_target_vel_ = msg->data;
            });

        // 1ms(1kHz) 猷⑦봽
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(1000),
            std::bind(&MPCPathTrackerCpp::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Ready (1kHz)");
    }

    // [�섏젙] �띾룄 �쒗븳�� �몄옄濡� 諛쏆븘 �숈쟻�쇰줈 �뚮씪誘명꽣瑜� �낅뜲�댄듃�섎뒗 �⑥닔濡� 蹂�寃�
    // override_limit媛� �뚯닔�대㈃ YAML �ㅼ젙媛� �ъ슜
    void MPCPathTrackerCpp::update_controller_params(double override_limit)
    {
        MPCParams params;

        params.Q_pos = this->get_parameter("Q_pos").as_double();
        params.Q_heading = this->get_parameter("Q_heading").as_double();
        params.R_v = this->get_parameter("R_v").as_double();
        params.R_w = this->get_parameter("R_w").as_double();

        // [�듭떖] Planner 媛쒖엯 �щ��� �곕Ⅸ �띾룄 �쒗븳 �ㅼ젙
        double default_max_v = this->get_parameter("max_velocity").as_double();

        if (override_limit > 0.0 && override_limit < 10.0)
        {
            // Planner媛� 媛쒖엯(10.0 誘몃쭔)�� 寃쎌슦: �대떦 �띾룄瑜� �곹븳�좎쑝濡� �ㅼ젙
            params.max_velocity = override_limit;
        }
        else
        {
            // Planner媛� 999.0(媛쒖엯 �놁쓬)�� 蹂대궦 寃쎌슦: 湲곕낯 �ㅼ젙媛� �ъ슜
            params.max_velocity = default_max_v;
        }

        params.max_accel = this->get_parameter("max_accel").as_double();
        params.max_angular_vel = this->get_parameter("max_angular_vel").as_double();
        params.min_velocity = this->get_parameter("min_velocity").as_double();

        params.horizon = this->get_parameter("horizon").as_int();
        params.dt = this->get_parameter("dt").as_double();
        params.wheelbase = this->get_parameter("wheelbase").as_double();
        params.vehicle_width = this->get_parameter("vehicle_width").as_double();
        params.lookahead_base = this->get_parameter("lookahead_base").as_double();
        params.lookahead_per_speed = this->get_parameter("lookahead_per_speed").as_double();

        controller_->update_parameters(params);
    }

    rcl_interfaces::msg::SetParametersResult MPCPathTrackerCpp::parameter_callback(
        const std::vector<rclcpp::Parameter> &params)
    {
        for (const auto &param : params)
        {
            RCLCPP_INFO(this->get_logger(), "Param updated: %s", param.get_name().c_str());
        }
        // �뚮씪誘명꽣 蹂�寃� �� �꾩옱 ��寃� �띾룄瑜� 諛섏쁺�섏뿬 �낅뜲�댄듃
        update_controller_params(current_target_vel_);

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

    void MPCPathTrackerCpp::control_loop()
    {
        if (local_path_.empty() || !current_pose_)
            return;

        if (local_path_.size() < 2)
        {
            publish_control(0.0, 0.0);
            return;
        }

        // [�듭떖 �섏젙] �쒖뼱 猷⑦봽留덈떎 Planner�� �띾룄 紐낅졊 諛섏쁺
        // current_target_vel_�� 999.0�대㈃ 湲곕낯媛�, �꾨땲硫� Planner 媛� �곸슜
        update_controller_params(current_target_vel_);

        auto output = controller_->compute_control(*current_pose_, local_path_);

        publish_control(output.velocity, output.angular_velocity);
        publish_predicted_path(output.predicted_trajectory);

        auto now = this->now();
        if ((now - last_log_time_).seconds() > 2.0)
        {
            // 濡쒓렇�� �꾩옱 �곸슜�� �쒗븳 �띾룄 �쒖떆
            double active_limit = (current_target_vel_ < 10.0) ? current_target_vel_ : this->get_parameter("max_velocity").as_double();
            RCLCPP_INFO(this->get_logger(), "MPC | Limit: %.2f | Out: %.2f m/s",
                        active_limit, output.velocity);
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
#include "bisa/mpc_path_tracker_cpp.hpp"

namespace bisa
{

    MPCPathTrackerCpp::MPCPathTrackerCpp()
        : Node("mpc_path_tracker_cpp"), last_log_time_(this->now())
    {

        // RCLCPP_INFO(this->get_logger(), "===========================================");
        // RCLCPP_INFO(this->get_logger(), "MPC Path Tracker C++ - 1kHz");
        // RCLCPP_INFO(this->get_logger(), "===========================================");

        this->declare_parameter("target_cav_id", 1);
        int target_id = this->get_parameter("target_cav_id").as_int();
        // ID 포맷팅 (예: 1 -> "01", 24 -> "24")
        std::string id_str = (target_id < 10) ? "0" + std::to_string(target_id) : std::to_string(target_id);

        // RCLCPP_INFO(this->get_logger(), "MPC Tracker for CAV_%s Started", id_str.c_str());

        // Parameters
        this->declare_parameter("Q_pos", 15.0);
        this->declare_parameter("Q_heading", 8.0);
        this->declare_parameter("R_v", 0.5);
        this->declare_parameter("R_w", 0.5);
        this->declare_parameter("max_velocity", 3.0);
        this->declare_parameter("max_accel", 2.0);
        this->declare_parameter("max_angular_vel", 2.0);
        this->declare_parameter("horizon", 20);

        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&MPCPathTrackerCpp::parameter_callback, this, std::placeholders::_1));

        controller_ = std::make_unique<MPCControllerCpp>();
        update_controller_params();
        std::string local_path_topic = "/local_path_cav" + id_str;
        std::string pose_topic = "/CAV_" + id_str;                 // 기존 /Ego_pose 대체
        std::string accel_topic = "/CAV_" + id_str + "_accel_raw"; // 기존 /Accel 대체
        // Subscriber 생성
        local_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            local_path_topic, 10,
            std::bind(&MPCPathTrackerCpp::local_path_callback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&MPCPathTrackerCpp::pose_callback, this, std::placeholders::_1));

        // Publisher 생성
        accel_pub_ = this->create_publisher<geometry_msgs::msg::Accel>(accel_topic, 10);
        pred_pub_ = this->create_publisher<nav_msgs::msg::Path>("/mpc_predicted_path", 10);
        cte_pub_ = this->create_publisher<std_msgs::msg::Float32>("/CAV_" + id_str + "_debug/cte", 10);
        he_pub_ = this->create_publisher<std_msgs::msg::Float32>("/CAV_" + id_str + "_debug/heading_error", 10);
        // 1kHz timer
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(1000),
            std::bind(&MPCPathTrackerCpp::control_loop, this));

        // RCLCPP_INFO(this->get_logger(), "Ready (1kHz)");
    }

    void MPCPathTrackerCpp::update_controller_params()
    {
        MPCParams params;
        params.Q_pos = this->get_parameter("Q_pos").as_double();
        params.Q_heading = this->get_parameter("Q_heading").as_double();
        params.R_v = this->get_parameter("R_v").as_double();
        params.R_w = this->get_parameter("R_w").as_double();
        params.max_velocity = this->get_parameter("max_velocity").as_double();
        params.max_accel = this->get_parameter("max_accel").as_double();
        params.max_angular_vel = this->get_parameter("max_angular_vel").as_double();
        params.horizon = this->get_parameter("horizon").as_int();

        controller_->update_parameters(params);
    }

    rcl_interfaces::msg::SetParametersResult MPCPathTrackerCpp::parameter_callback(
        const std::vector<rclcpp::Parameter> &params)
    {

        // for (const auto &param : params)
        // {
        //     RCLCPP_INFO(this->get_logger(), "Param updated: %s", param.get_name().c_str());
        // }

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

    void MPCPathTrackerCpp::control_loop()
    {
        if (local_path_.empty() || !current_pose_)
            return;
        if (local_path_.size() < 2)
        {
            publish_control(0.0, 0.0);
            return;
        }

        auto output = controller_->compute_control(*current_pose_, local_path_);

        publish_control(output.velocity, output.angular_velocity);
        publish_predicted_path(output.predicted_trajectory);

        std_msgs::msg::Float32 cte_msg;
        cte_msg.data = output.cross_track_error;
        cte_pub_->publish(cte_msg);

        std_msgs::msg::Float32 he_msg;
        he_msg.data = output.heading_error;
        he_pub_->publish(he_msg);

        auto now = this->now();
        if ((now - last_log_time_).seconds() > 2.0)
        {
            // RCLCPP_INFO(this->get_logger(), "MPC | v: %.2f m/s, w: %.3f rad/s",
            //             output.velocity, output.angular_velocity);
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

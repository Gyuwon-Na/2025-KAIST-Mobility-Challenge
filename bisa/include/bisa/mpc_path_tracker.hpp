#ifndef BISA_MPC_PATH_TRACKER_HPP_
#define BISA_MPC_PATH_TRACKER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include "std_msgs/msg/float32.hpp"

#include "bisa/mpc_controller.hpp"
#include <memory>
#include <optional>

namespace bisa
{

    class MPCPathTrackerCpp : public rclcpp::Node
    {
    public:
        MPCPathTrackerCpp();

    private:
        void local_path_callback(const nav_msgs::msg::Path::SharedPtr msg);
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void target_vel_callback(const std_msgs::msg::Float32::SharedPtr msg);

        void control_loop();
        void publish_control(double v, double w);
        void publish_predicted_path(const std::vector<std::array<double, 3>> &traj);
        void update_controller_params();
        rcl_interfaces::msg::SetParametersResult parameter_callback(
            const std::vector<rclcpp::Parameter> &params);

        std::unique_ptr<MPCControllerCpp> controller_;
        std::vector<geometry_msgs::msg::PoseStamped> local_path_;
        std::optional<geometry_msgs::msg::Pose> current_pose_;

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_vel_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr accel_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pred_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Time last_log_time_;
        OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        double current_target_vel_ = 0.0; // 수신된 목표 속도 저장
    };

}

#endif

#ifndef BISA_MPC_CONTROLLER_CPP_HPP_
#define BISA_MPC_CONTROLLER_CPP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <array>
#include <cmath>

namespace bisa {

struct MPCParams {
    double wheelbase = 0.33;
    double vehicle_width = 0.15;
    double dt = 0.001;
    int horizon = 20;
    
    double Q_pos = 15.0;
    double Q_heading = 8.0;
    double R_v = 0.5;
    double R_w = 0.5;
    
    double max_velocity = 3.0;
    double min_velocity = 0.0;
    double max_accel = 2.0;
    double max_angular_vel = 2.0;
    
    double lookahead_base = 0.3;
    double lookahead_per_speed = 0.2;
};

struct ControlOutput {
    double velocity;
    double angular_velocity;
    std::vector<std::array<double, 3>> predicted_trajectory;
};

class MPCControllerCpp {
public:
    MPCControllerCpp();
    
    void update_parameters(const MPCParams& params);
    
    ControlOutput compute_control(
        const geometry_msgs::msg::Pose& current_pose,
        const std::vector<geometry_msgs::msg::PoseStamped>& local_path
    );

private:
    MPCParams params_;
    double prev_v_ = 0.0;
    double prev_w_ = 0.0;
    
    double quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q);
    
    std::vector<std::array<double, 2>> get_target_points(
        const std::vector<geometry_msgs::msg::PoseStamped>& local_path,
        const geometry_msgs::msg::Pose& current_pose
    );
    
    double calculate_path_curvature(
        const std::vector<std::array<double, 2>>& targets, 
        int idx
    );
};

}

#endif

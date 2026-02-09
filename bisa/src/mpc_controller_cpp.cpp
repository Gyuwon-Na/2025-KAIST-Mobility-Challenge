#include "bisa/mpc_controller_cpp.hpp"
#include <algorithm>

namespace bisa {

MPCControllerCpp::MPCControllerCpp() {}

void MPCControllerCpp::update_parameters(const MPCParams& params) {
    params_ = params;
}

double MPCControllerCpp::quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q) {
    return q.z;
}

std::vector<std::array<double, 2>> MPCControllerCpp::get_target_points(
    const std::vector<geometry_msgs::msg::PoseStamped>& local_path,
    const geometry_msgs::msg::Pose& current_pose) {
    
    std::vector<std::array<double, 2>> targets;
    if (local_path.empty()) return targets;
    
    double current_v = std::abs(prev_v_);
    double lookahead = params_.lookahead_base + params_.lookahead_per_speed * current_v;
    
    int step = std::max(1, static_cast<int>(lookahead / 0.01 / params_.horizon));
    
    for (int i = 0; i < params_.horizon; ++i) {
        int idx = std::min(i * step, static_cast<int>(local_path.size()) - 1);
        targets.push_back({
            local_path[idx].pose.position.x,
            local_path[idx].pose.position.y
        });
    }
    
    while (static_cast<int>(targets.size()) < params_.horizon) {
        targets.push_back(targets.back());
    }
    
    return targets;
}

double MPCControllerCpp::calculate_path_curvature(
    const std::vector<std::array<double, 2>>& targets, int idx) {
    
    if (idx < 1 || idx >= static_cast<int>(targets.size()) - 1) return 0.0;
    
    double x1 = targets[idx - 1][0];
    double y1 = targets[idx - 1][1];
    double x2 = targets[idx][0];
    double y2 = targets[idx][1];
    double x3 = targets[idx + 1][0];
    double y3 = targets[idx + 1][1];
    
    double dx1 = x2 - x1;
    double dy1 = y2 - y1;
    double dx2 = x3 - x2;
    double dy2 = y3 - y2;
    
    double cross = dx1 * dy2 - dy1 * dx2;
    double d1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    double d2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
    
    if (d1 < 1e-6 || d2 < 1e-6) return 0.0;
    
    return cross / (d1 * d2);
}

ControlOutput MPCControllerCpp::compute_control(
    const geometry_msgs::msg::Pose& current_pose,
    const std::vector<geometry_msgs::msg::PoseStamped>& local_path) {
    
    ControlOutput output;
    output.velocity = 0.0;
    output.angular_velocity = 0.0;
    
    if (local_path.size() < 2) return output;
    
    double x0 = current_pose.position.x;
    double y0 = current_pose.position.y;
    double theta0 = quaternion_to_yaw(current_pose.orientation);
    
    auto targets = get_target_points(local_path, current_pose);
    if (targets.empty()) return output;
    
    // ==========================================
    // Cost-based MPC (Improved velocity control)
    // ==========================================
    
    int lookahead_idx = std::clamp(params_.horizon / 2, 5, static_cast<int>(targets.size()) - 1);
    
    double dx = targets[lookahead_idx][0] - x0;
    double dy = targets[lookahead_idx][1] - y0;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    if (distance < 0.01) {
        return output;
    }
    
    double target_heading = std::atan2(dy, dx);
    double heading_error = target_heading - theta0;
    
    while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
    while (heading_error < -M_PI) heading_error += 2.0 * M_PI;
    
    double path_curvature = calculate_path_curvature(targets, lookahead_idx);
    double curvature_magnitude = std::abs(path_curvature);
    
    // Q_pos effect (경로 곡률에 따른 감속)
    double position_weight_factor = std::clamp(params_.Q_pos / 20.0, 0.5, 2.0);
    // 계수를 10.0 → 5.0으로 낮춤 (감속 완화)
    double curvature_penalty = 1.0 / (1.0 + curvature_magnitude * 5.0 * position_weight_factor);
    
    // Q_heading effect (방향 오차에 따른 감속)
    double heading_weight_factor = std::clamp(params_.Q_heading / 12.0, 0.5, 2.0);
    double heading_penalty = 1.0 / (1.0 + std::abs(heading_error) * 3.0 * heading_weight_factor);
    
    // 최소값 선택 (곱셈 → 최소값)
    // 두 penalty 중 더 큰 제약을 선택 (곱하면 너무 낮아짐)
    double combined_penalty = std::max(curvature_penalty, heading_penalty);
    
    // 최소 속도 보장 (max_velocity의 40%)
    double min_speed_ratio = 0.4;
    double v_target = params_.max_velocity * std::max(combined_penalty, min_speed_ratio);
    
    // R_v effect (velocity smoothness)
    double v_smoothing = std::clamp(params_.R_v * 2.0, 1.0, 10.0);
    double v_cmd = prev_v_ + (v_target - prev_v_) / v_smoothing;
    
    double alpha = target_heading - theta0;
    double pure_pursuit_w = (2.0 * std::sin(alpha)) / distance;
    
    double heading_correction = heading_error * heading_weight_factor * 0.5;
    double w_target = pure_pursuit_w + heading_correction;
    
    // R_w effect (angular smoothness)
    double w_smoothing = std::clamp(params_.R_w * 2.0, 1.0, 10.0);
    double w_cmd = prev_w_ + (w_target - prev_w_) / w_smoothing;
    
    w_cmd = std::clamp(w_cmd, -params_.max_angular_vel, params_.max_angular_vel);
    
    double max_dv = params_.max_accel * params_.dt;
    v_cmd = std::clamp(v_cmd, prev_v_ - max_dv, prev_v_ + max_dv);
    v_cmd = std::clamp(v_cmd, params_.min_velocity, params_.max_velocity);
    
    prev_v_ = v_cmd;
    prev_w_ = w_cmd;
    
    output.velocity = v_cmd;
    output.angular_velocity = w_cmd;
    
    double x = x0, y = y0, theta = theta0;
    for (int i = 0; i < params_.horizon; ++i) {
        x += v_cmd * std::cos(theta) * params_.dt;
        y += v_cmd * std::sin(theta) * params_.dt;
        theta += w_cmd * params_.dt;
        output.predicted_trajectory.push_back({x, y, 0.0});
    }
    
    return output;
}

}

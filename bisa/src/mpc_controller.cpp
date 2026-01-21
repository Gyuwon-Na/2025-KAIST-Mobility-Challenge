#include "bisa/mpc_controller.hpp"
#include <algorithm>

namespace bisa
{

    MPCControllerCpp::MPCControllerCpp() {}

    void MPCControllerCpp::update_parameters(const MPCParams &params)
    {
        params_ = params;
    }

    double MPCControllerCpp::quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q)
    {
        return q.z;
    }

    std::vector<std::array<double, 2>> MPCControllerCpp::get_target_points(
        const std::vector<geometry_msgs::msg::PoseStamped> &local_path,
        const geometry_msgs::msg::Pose &current_pose)
    {

        std::vector<std::array<double, 2>> targets;
        if (local_path.empty())
            return targets;

        double current_v = std::abs(prev_v_);
        double lookahead = params_.lookahead_base + params_.lookahead_per_speed * current_v;

        int step = std::max(1, static_cast<int>(lookahead / 0.01 / params_.horizon));

        for (int i = 0; i < params_.horizon; ++i)
        {
            int idx = std::min(i * step, static_cast<int>(local_path.size()) - 1);
            targets.push_back({local_path[idx].pose.position.x,
                               local_path[idx].pose.position.y});
        }

        while (static_cast<int>(targets.size()) < params_.horizon)
        {
            targets.push_back(targets.back());
        }

        return targets;
    }

    double MPCControllerCpp::calculate_path_curvature(
        const std::vector<std::array<double, 2>> &targets, int idx)
    {

        if (idx < 1 || idx >= static_cast<int>(targets.size()) - 1)
            return 0.0;

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

        if (d1 < 1e-6 || d2 < 1e-6)
            return 0.0;

        return cross / (d1 * d2);
    }

    ControlOutput MPCControllerCpp::compute_control(
        const geometry_msgs::msg::Pose &current_pose,
        const std::vector<geometry_msgs::msg::PoseStamped> &local_path)
    {

        ControlOutput output;
        output.velocity = 0.0;
        output.angular_velocity = 0.0;

        if (local_path.size() < 2)
            return output;

        double x0 = current_pose.position.x;
        double y0 = current_pose.position.y;
        double theta0 = quaternion_to_yaw(current_pose.orientation);

        auto targets = get_target_points(local_path, current_pose);
        if (targets.empty())
            return output;

        // ==========================================
        // Cost-based MPC (Logic Modified)
        // ==========================================

        int lookahead_idx = std::clamp(params_.horizon / 2, 5, static_cast<int>(targets.size()) - 1);

        double dx = targets[lookahead_idx][0] - x0;
        double dy = targets[lookahead_idx][1] - y0;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance < 0.01)
        {
            return output;
        }

        double target_heading = std::atan2(dy, dx);
        double heading_error = target_heading - theta0;

        while (heading_error > M_PI)
            heading_error -= 2.0 * M_PI;
        while (heading_error < -M_PI)
            heading_error += 2.0 * M_PI;

        /* ================================================================================== */
        // [수정 완료] 코너링 감속 로직 완전 제거
        /* ================================================================================== */

        // 기존 로직: 곡률(Curvature)과 헤딩 에러가 크면 v_target을 줄임
        // double path_curvature = calculate_path_curvature(targets, lookahead_idx);
        // double curvature_penalty = ...
        // double heading_penalty = ...
        // double v_target = params_.max_velocity * curvature_penalty * heading_penalty;

        // [변경 로직] 무조건 입력받은 Max Velocity(즉, env_slow_vel) 유지
        double v_target = params_.max_velocity;

        /* ================================================================================== */

        // R_v effect (Velocity Smoothness)
        // 목표 속도까지 도달하는 가속도 제한은 유지 (안전상 급발진 방지)
        double v_smoothing = std::clamp(params_.R_v * 2.0, 1.0, 10.0);
        double v_cmd = prev_v_ + (v_target - prev_v_) / v_smoothing;

        // Pure Pursuit-like Angular Velocity Calculation
        // 코너 감속은 뺐지만, 조향(W)은 정확히 해야 하므로 아래 로직 유지
        double heading_weight_factor = std::clamp(params_.Q_heading / 12.0, 0.5, 2.0);
        double alpha = target_heading - theta0;
        double pure_pursuit_w = (2.0 * std::sin(alpha)) / distance;

        double heading_correction = heading_error * heading_weight_factor * 0.5;
        double w_target = pure_pursuit_w + heading_correction;

        // R_w effect (Angular Smoothness)
        double w_smoothing = std::clamp(params_.R_w * 2.0, 1.0, 10.0);
        double w_cmd = prev_w_ + (w_target - prev_w_) / w_smoothing;

        w_cmd = std::clamp(w_cmd, -params_.max_angular_vel, params_.max_angular_vel);

        // 가속도 제한 (Jerk 방지)
        double max_dv = params_.max_accel * params_.dt;
        v_cmd = std::clamp(v_cmd, prev_v_ - max_dv, prev_v_ + max_dv);

        // 최종 속도 범위 제한
        v_cmd = std::clamp(v_cmd, params_.min_velocity, params_.max_velocity);

        prev_v_ = v_cmd;
        prev_w_ = w_cmd;

        output.velocity = v_cmd;
        output.angular_velocity = w_cmd;

        // 예측 경로 생성 (시각화용)
        double x = x0, y = y0, theta = theta0;
        for (int i = 0; i < params_.horizon; ++i)
        {
            x += v_cmd * std::cos(theta) * params_.dt;
            y += v_cmd * std::sin(theta) * params_.dt;
            theta += w_cmd * params_.dt;
            output.predicted_trajectory.push_back({x, y, 0.0});
        }

        return output;
    }

}

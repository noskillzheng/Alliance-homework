#include "core/constraint_checker.hpp"
#include <iostream>
#include <iomanip>
#include <algorithm>

namespace trajectory {

ConstraintChecker::ConstraintChecker(double max_velocity,
                                    double max_acceleration,
                                    int sampling_points)
    : max_velocity_(max_velocity),
      max_acceleration_(max_acceleration),
      sampling_points_(sampling_points) {}

bool ConstraintChecker::checkTrajectory(const Trajectory2D& trajectory) const {
    double duration = trajectory.x_traj.getDuration();
    double dt = duration / sampling_points_;
    
    for (int i = 0; i <= sampling_points_; ++i) {
        double t = i * dt;
        
        // 检查速度
        double speed = trajectory.speed(t);
        if (speed > max_velocity_ + 1e-6) {
            return false;
        }
        
        // 检查加速度
        double acc = trajectory.accelerationMagnitude(t);
        if (acc > max_acceleration_ + 1e-6) {
            return false;
        }
    }
    
    return true;
}

bool ConstraintChecker::checkAllTrajectories(
    const std::vector<Trajectory2D>& trajectories) const {
    
    for (const auto& traj : trajectories) {
        if (!checkTrajectory(traj)) {
            return false;
        }
    }
    return true;
}

double ConstraintChecker::getMaxVelocity(const Trajectory2D& trajectory) const {
    double duration = trajectory.x_traj.getDuration();
    double dt = duration / sampling_points_;
    double max_vel = 0.0;
    
    for (int i = 0; i <= sampling_points_; ++i) {
        double t = i * dt;
        double speed = trajectory.speed(t);
        max_vel = std::max(max_vel, speed);
    }
    
    return max_vel;
}

double ConstraintChecker::getMaxAcceleration(const Trajectory2D& trajectory) const {
    double duration = trajectory.x_traj.getDuration();
    double dt = duration / sampling_points_;
    double max_acc = 0.0;
    
    for (int i = 0; i <= sampling_points_; ++i) {
        double t = i * dt;
        double acc = trajectory.accelerationMagnitude(t);
        max_acc = std::max(max_acc, acc);
    }
    
    return max_acc;
}

void ConstraintChecker::printReport(
    const std::vector<Trajectory2D>& trajectories) const {
    
    std::cout << "\n========== 约束检查报告 ==========\n";
    std::cout << std::fixed << std::setprecision(4);
    
    std::cout << "速度限制: " << max_velocity_ << " m/s\n";
    std::cout << "加速度限制: " << max_acceleration_ << " m/s²\n";
    std::cout << "--------------------------------\n";
    
    bool all_satisfied = true;
    double max_v_all = 0.0;
    double max_a_all = 0.0;
    
    for (size_t i = 0; i < trajectories.size(); ++i) {
        double max_vel = getMaxVelocity(trajectories[i]);
        double max_acc = getMaxAcceleration(trajectories[i]);
        bool vel_ok = max_vel <= max_velocity_ + 1e-6;
        bool acc_ok = max_acc <= max_acceleration_ + 1e-6;
        
        max_v_all = std::max(max_v_all, max_vel);
        max_a_all = std::max(max_a_all, max_acc);
        
        std::cout << "段 " << i << ":\n";
        std::cout << "  最大速度: " << max_vel << " m/s ";
        std::cout << (vel_ok ? "[✓]" : "[✗]");
        std::cout << " (利用率: " << (max_vel / max_velocity_ * 100) << "%)\n";
        std::cout << "  最大加速度: " << max_acc << " m/s² ";
        std::cout << (acc_ok ? "[✓]" : "[✗]");
        std::cout << " (利用率: " << (max_acc / max_acceleration_ * 100) << "%)\n";
        
        if (!vel_ok || !acc_ok) {
            all_satisfied = false;
        }
    }
    
    std::cout << "--------------------------------\n";
    std::cout << "全局最大速度: " << max_v_all << " m/s (" 
              << (max_v_all / max_velocity_ * 100) << "% 利用率)\n";
    std::cout << "全局最大加速度: " << max_a_all << " m/s² (" 
              << (max_a_all / max_acceleration_ * 100) << "% 利用率)\n";
    std::cout << "总体结果: " << (all_satisfied ? "所有约束满足 ✓" : "存在违反约束 ✗") << "\n";
    std::cout << "==================================\n\n";
}

void ConstraintChecker::printContinuityReport(
    const std::vector<Trajectory2D>& trajectories) const {
    
    std::cout << "\n========== C^2连续性验证 ==========\n";
    std::cout << std::fixed << std::setprecision(6);
    
    for (size_t i = 0; i < trajectories.size() - 1; ++i) {
        const auto& curr = trajectories[i];
        const auto& next = trajectories[i + 1];
        double T = curr.x_traj.getDuration();
        
        // 检查段间连续性
        auto p_curr_end = curr.position(T);
        auto p_next_start = next.position(0.0);
        auto v_curr_end = curr.velocity(T);
        auto v_next_start = next.velocity(0.0);
        auto a_curr_end = curr.acceleration(T);
        auto a_next_start = next.acceleration(0.0);
        
        double pos_diff = (p_curr_end - p_next_start).norm();
        double vel_diff = (v_curr_end - v_next_start).norm();
        double acc_diff = (a_curr_end - a_next_start).norm();
        
        std::cout << "段 " << i << " → 段 " << (i+1) << ":\n";
        std::cout << "  位置跳变: " << pos_diff << " m " 
                  << (pos_diff < 1e-6 ? "[✓]" : "[✗]") << "\n";
        std::cout << "  速度跳变: " << vel_diff << " m/s " 
                  << (vel_diff < 1e-6 ? "[✓]" : "[✗]") << "\n";
        std::cout << "  加速度跳变: " << acc_diff << " m/s² " 
                  << (acc_diff < 1e-6 ? "[✓]" : "[✗]") << "\n";
    }
    
    std::cout << "==================================\n\n";
}

double ConstraintChecker::computeCurvature(const Trajectory2D& trajectory, double t) {
    auto vel = trajectory.velocity(t);
    auto acc = trajectory.acceleration(t);
    
    double vx = vel.x();
    double vy = vel.y();
    double ax = acc.x();
    double ay = acc.y();
    
    double cross = vx * ay - vy * ax;
    double speed = vel.norm();
    
    if (speed < 1e-6) {
        return 0.0;
    }
    
    // κ = |v × a| / |v|^3
    return std::abs(cross) / (speed * speed * speed);
}

} // namespace trajectory


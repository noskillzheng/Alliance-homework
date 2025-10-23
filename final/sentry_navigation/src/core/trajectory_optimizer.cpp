#include "core/trajectory_optimizer.hpp"
#include <iostream>
#include <cmath>

namespace trajectory {

TrajectoryOptimizer::TrajectoryOptimizer(const std::vector<Waypoint>& waypoints,
                                        const OptimizerConfig& config)
    : waypoints_(waypoints), config_(config) {
    if (waypoints.size() < 2) {
        throw std::invalid_argument("Need at least 2 waypoints");
    }
}

bool TrajectoryOptimizer::optimize() {
    // 1. 初始化时间分配
    initializeTimeAllocation();
    
    // 2. 提取x和y坐标
    std::vector<double> x_positions, y_positions;
    for (const auto& wp : waypoints_) {
        x_positions.push_back(wp.position.x());
        y_positions.push_back(wp.position.y());
    }
    
    // 3. 分别优化x和y方向的轨迹
    auto x_trajs = optimizeSingleAxis(
        x_positions,
        config_.initial_velocity.x(),
        config_.initial_acceleration.x(),
        config_.final_velocity.x(),
        config_.final_acceleration.x()
    );
    
    auto y_trajs = optimizeSingleAxis(
        y_positions,
        config_.initial_velocity.y(),
        config_.initial_acceleration.y(),
        config_.final_velocity.y(),
        config_.final_acceleration.y()
    );
    
    // 4. 组合成2D轨迹
    trajectories_.clear();
    for (size_t i = 0; i < x_trajs.size(); ++i) {
        Trajectory2D traj;
        traj.x_traj = x_trajs[i];
        traj.y_traj = y_trajs[i];
        trajectories_.push_back(traj);
    }
    
    // 5. 时间缩放优化（逼近约束限制）
    refineTimeAllocation();
    
    return true;
}

bool TrajectoryOptimizer::refineTimeAllocation(int max_iterations) {
    std::cout << "\n开始时间缩放优化...\n";
    
    double scale_factor = 1.0;
    double best_scale = 1.0;
    int iteration = 0;
    
    // 二分搜索找到最小的时间缩放因子
    double lower_bound = 0.5;   // 最小缩放50%
    double upper_bound = 1.0;   // 初始不缩放
    
    while (iteration < max_iterations && (upper_bound - lower_bound) > 0.001) {
        scale_factor = (lower_bound + upper_bound) / 2.0;
        
        // 临时缩放时间
        std::vector<double> scaled_times = time_allocation_;
        for (auto& t : scaled_times) {
            t *= scale_factor;
        }
        
        // 用缩放后的时间重新生成轨迹
        auto original_time = time_allocation_;
        time_allocation_ = scaled_times;
        
        std::vector<double> x_positions, y_positions;
        for (const auto& wp : waypoints_) {
            x_positions.push_back(wp.position.x());
            y_positions.push_back(wp.position.y());
        }
        
        auto x_trajs = optimizeSingleAxis(
            x_positions,
            config_.initial_velocity.x(),
            config_.initial_acceleration.x(),
            config_.final_velocity.x(),
            config_.final_acceleration.x()
        );
        
        auto y_trajs = optimizeSingleAxis(
            y_positions,
            config_.initial_velocity.y(),
            config_.initial_acceleration.y(),
            config_.final_velocity.y(),
            config_.final_acceleration.y()
        );
        
        trajectories_.clear();
        for (size_t i = 0; i < x_trajs.size(); ++i) {
            Trajectory2D traj;
            traj.x_traj = x_trajs[i];
            traj.y_traj = y_trajs[i];
            trajectories_.push_back(traj);
        }
        
        // 检查约束（加入安全裕度：98%）
        bool constraints_ok = true;
        double safety_margin = 0.98;  // 保留2%裕度避免数值误差
        for (const auto& traj : trajectories_) {
            double max_v = std::sqrt(traj.x_traj.getVelocityExtremum() * traj.x_traj.getVelocityExtremum() +
                                    traj.y_traj.getVelocityExtremum() * traj.y_traj.getVelocityExtremum());
            double max_a = std::sqrt(traj.x_traj.getAccelerationExtremum() * traj.x_traj.getAccelerationExtremum() +
                                    traj.y_traj.getAccelerationExtremum() * traj.y_traj.getAccelerationExtremum());
            
            if (max_v > config_.max_velocity * safety_margin ||
                max_a > config_.max_acceleration * safety_margin) {
                constraints_ok = false;
                break;
            }
        }
        
        if (constraints_ok) {
            // 约束满足，尝试更小的缩放因子（更快）
            best_scale = scale_factor;
            upper_bound = scale_factor;
        } else {
            // 约束违反，需要更大的时间
            lower_bound = scale_factor;
        }
        
        time_allocation_ = original_time;
        iteration++;
    }
    
    // 应用最佳缩放因子
    if (best_scale < 0.999) {
        for (auto& t : time_allocation_) {
            t *= best_scale;
        }
        
        // 重新生成最终轨迹
        std::vector<double> x_positions, y_positions;
        for (const auto& wp : waypoints_) {
            x_positions.push_back(wp.position.x());
            y_positions.push_back(wp.position.y());
        }
        
        auto x_trajs = optimizeSingleAxis(
            x_positions,
            config_.initial_velocity.x(),
            config_.initial_acceleration.x(),
            config_.final_velocity.x(),
            config_.final_acceleration.x()
        );
        
        auto y_trajs = optimizeSingleAxis(
            y_positions,
            config_.initial_velocity.y(),
            config_.initial_acceleration.y(),
            config_.final_velocity.y(),
            config_.final_acceleration.y()
        );
        
        trajectories_.clear();
        for (size_t i = 0; i < x_trajs.size(); ++i) {
            Trajectory2D traj;
            traj.x_traj = x_trajs[i];
            traj.y_traj = y_trajs[i];
            trajectories_.push_back(traj);
        }
        
        std::cout << "时间缩放完成: 缩放因子=" << best_scale 
                  << ", 新总时间=" << getTotalTime() << "s\n";
    } else {
        std::cout << "无需时间缩放，当前配置已接近最优\n";
    }
    
    return true;
}

void TrajectoryOptimizer::initializeTimeAllocation() {
    time_allocation_.clear();
    int num_segments = waypoints_.size() - 1;
    
    // 改进的启发式：考虑加速度约束的时间分配
    // 对于4阶多项式，峰值加速度约为 a_max ≈ 12*distance/T^2
    // 因此 T ≥ sqrt(12*distance/a_max)
    
    for (size_t i = 0; i < waypoints_.size() - 1; ++i) {
        double dist = (waypoints_[i + 1].position - waypoints_[i].position).norm();
        
        // 基于速度约束的最小时间（使用配置的利用率）
        double t_vel = dist / (config_.max_velocity * config_.velocity_utilization);
        
        // 基于加速度约束的最小时间（精确计算）
        // 对于多项式轨迹，峰值加速度约为 12*dist/T^2
        double t_acc = std::sqrt(12.0 * dist / config_.max_acceleration);
        
        // 取较大值，使用配置的安全系数
        double t = std::max(t_vel, t_acc) * config_.time_safety_factor;
        
        // 限制在最大时间范围内
        t = std::min(t, config_.max_segment_time);
        // 使用配置的最小段时间
        t = std::max(t, config_.min_segment_time);
        
        time_allocation_.push_back(t);
    }
}

std::vector<PolynomialTrajectory> TrajectoryOptimizer::optimizeSingleAxis(
    const std::vector<double>& axis_positions,
    double initial_vel,
    double initial_acc,
    double final_vel,
    double final_acc) {
    
    std::vector<PolynomialTrajectory> trajectories;
    int num_segments = axis_positions.size() - 1;
    
    // 计算段间速度和加速度（确保严格C^2连续）
    std::vector<double> waypoint_velocities(axis_positions.size());
    std::vector<double> waypoint_accelerations(axis_positions.size());
    
    waypoint_velocities[0] = initial_vel;
    waypoint_accelerations[0] = initial_acc;
    waypoint_velocities[num_segments] = final_vel;
    waypoint_accelerations[num_segments] = final_acc;
    
    // 对中间路径点，使用相邻段的平均速度
    for (int i = 1; i < num_segments; ++i) {
        // 前一段的平均速度
        double v_prev = (axis_positions[i] - axis_positions[i-1]) / time_allocation_[i-1];
        // 后一段的平均速度
        double v_next = (axis_positions[i+1] - axis_positions[i]) / time_allocation_[i];
        // 取平均，限制在合理范围内
        waypoint_velocities[i] = (v_prev + v_next) / 2.0;
        // 使用配置的段间速度保持率
        double v_limit = config_.max_velocity * config_.waypoint_velocity_ratio;
        waypoint_velocities[i] = std::max(-v_limit, std::min(v_limit, waypoint_velocities[i]));
        
        // 段间加速度设为0（简化处理，确保平滑过渡）
        waypoint_accelerations[i] = 0.0;
    }
    
    // 对每一段求解
    for (int seg = 0; seg < num_segments; ++seg) {
        double p0 = axis_positions[seg];
        double pf = axis_positions[seg + 1];
        double t = time_allocation_[seg];
        
        // 边界条件：使用计算的段间速度和加速度（严格C^2连续）
        double v0 = waypoint_velocities[seg];
        double a0 = waypoint_accelerations[seg];
        double vf = waypoint_velocities[seg + 1];
        double af = waypoint_accelerations[seg + 1];
        
        // 构建约束矩阵 A (6x6 for quintic)
        Eigen::MatrixXd A = buildCoefficientMatrix(t);
        
        // 构建约束向量 b = [p0, v0, a0, pf, vf, af]^T
        Eigen::VectorXd b(6);
        b << p0, v0, a0, pf, vf, af;
        
        // 求解 A * coeffs = b
        Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(b);
        
        trajectories.emplace_back(coeffs, t);
    }
    
    return trajectories;
}

Eigen::MatrixXd TrajectoryOptimizer::buildCoefficientMatrix(double t) const {
    // 构建系数矩阵（5阶多项式），使得 A * [a5, a4, a3, a2, a1, a0]^T = [p0, v0, a0, pf, vf, af]^T
    // 其中：
    // p(0) = a0 = p0
    // v(0) = a1 = v0
    // a(0) = 2*a2 = a0
    // p(T) = a5*T^5 + a4*T^4 + a3*T^3 + a2*T^2 + a1*T + a0 = pf
    // v(T) = 5*a5*T^4 + 4*a4*T^3 + 3*a3*T^2 + 2*a2*T + a1 = vf
    // a(T) = 20*a5*T^3 + 12*a4*T^2 + 6*a3*T + 2*a2 = af
    
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;
    
    Eigen::MatrixXd A(6, 6);
    
    // p(0) = a0
    A.row(0) << 0, 0, 0, 0, 0, 1;
    
    // v(0) = a1
    A.row(1) << 0, 0, 0, 0, 1, 0;
    
    // a(0) = 2*a2
    A.row(2) << 0, 0, 0, 2, 0, 0;
    
    // p(T) = a5*T^5 + a4*T^4 + a3*T^3 + a2*T^2 + a1*T + a0
    A.row(3) << t5, t4, t3, t2, t, 1;
    
    // v(T) = 5*a5*T^4 + 4*a4*T^3 + 3*a3*T^2 + 2*a2*T + a1
    A.row(4) << 5*t4, 4*t3, 3*t2, 2*t, 1, 0;
    
    // a(T) = 20*a5*T^3 + 12*a4*T^2 + 6*a3*T + 2*a2
    A.row(5) << 20*t3, 12*t2, 6*t, 2, 0, 0;
    
    return A;
}

Eigen::MatrixXd TrajectoryOptimizer::buildQMatrix(double t) const {
    // Q矩阵是jerk的平方积分的Hessian矩阵（5阶多项式）
    // ∫[0,T] (60*a5*t^2 + 24*a4*t + 6*a3)^2 dt 对系数的二阶导数
    // = 720*a5^2*T^5 + 720*a5*a4*T^4 + 192*a4^2*T^3 + 240*a5*a3*T^3 + 144*a4*a3*T^2 + 36*a3^2*T
    
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;
    
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);
    
    // 只有a5, a4, a3相关的项非零
    Q(0, 0) = 720.0 * t5;      // ∂²/∂a5²
    Q(0, 1) = 360.0 * t4;      // ∂²/∂a5∂a4
    Q(1, 0) = 360.0 * t4;      // ∂²/∂a4∂a5
    Q(0, 2) = 120.0 * t3;      // ∂²/∂a5∂a3
    Q(2, 0) = 120.0 * t3;      // ∂²/∂a3∂a5
    Q(1, 1) = 192.0 * t3;      // ∂²/∂a4²
    Q(1, 2) = 72.0 * t2;       // ∂²/∂a4∂a3
    Q(2, 1) = 72.0 * t2;       // ∂²/∂a3∂a4
    Q(2, 2) = 36.0 * t;        // ∂²/∂a3²
    
    return Q;
}

double TrajectoryOptimizer::getTotalTime() const {
    double total = 0.0;
    for (double t : time_allocation_) {
        total += t;
    }
    return total;
}

double TrajectoryOptimizer::getObjectiveValue() const {
    double total_cost = 0.0;
    for (const auto& traj : trajectories_) {
        total_cost += traj.x_traj.computeJerkSquareIntegral();
        total_cost += traj.y_traj.computeJerkSquareIntegral();
    }
    return total_cost;
}

} // namespace trajectory


#ifndef TRAJECTORY_OPTIMIZER_HPP
#define TRAJECTORY_OPTIMIZER_HPP

#include "core/polynomial_trajectory.hpp"
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace trajectory {

/**
 * @brief 路径点结构
 */
struct Waypoint {
    Eigen::Vector2d position;  // 位置 (x, y)
    
    Waypoint(double x, double y) : position(x, y) {}
};

/**
 * @brief 轨迹优化器配置
 */
struct OptimizerConfig {
    // 约束参数
    double max_velocity = 1.0;       // 最大速度约束
    double max_acceleration = 1.0;   // 最大加速度约束
    double max_segment_time = 10.0;  // 每段轨迹最大时间
    int sampling_points = 100;       // 采样点数（用于验证约束）
    
    // 性能调优参数
    double time_safety_factor = 1.05;      // 时间分配安全系数
    double velocity_utilization = 0.85;    // 速度利用率
    double waypoint_velocity_ratio = 0.85; // 段间速度保持率
    double min_segment_time = 0.5;         // 最小段时间
    
    // 边界条件
    Eigen::Vector2d initial_velocity = Eigen::Vector2d::Zero();
    Eigen::Vector2d initial_acceleration = Eigen::Vector2d::Zero();
    Eigen::Vector2d final_velocity = Eigen::Vector2d::Zero();
    Eigen::Vector2d final_acceleration = Eigen::Vector2d::Zero();
};

/**
 * @brief 轨迹优化器
 * 
 * 实现最小jerk轨迹优化
 * 目标：min ∫(jerk^2)dt
 * 约束：
 *   - 连续性（位置、速度、加速度）
 *   - 时间约束
 *   - 速度和加速度限制
 */
class TrajectoryOptimizer {
public:
    /**
     * @brief 构造函数
     * @param waypoints 路径点集合
     * @param config 优化器配置
     */
    TrajectoryOptimizer(const std::vector<Waypoint>& waypoints, 
                       const OptimizerConfig& config);
    
    /**
     * @brief 执行优化
     * @return 是否优化成功
     */
    bool optimize();
    
    /**
     * @brief 时间缩放优化（在保证约束的前提下缩短总时间）
     * @param max_iterations 最大迭代次数
     * @return 是否成功缩放
     */
    bool refineTimeAllocation(int max_iterations = 20);
    
    /**
     * @brief 获取优化后的轨迹
     */
    const std::vector<Trajectory2D>& getTrajectories() const { 
        return trajectories_; 
    }
    
    /**
     * @brief 获取每段轨迹的时间分配
     */
    const std::vector<double>& getTimeAllocation() const { 
        return time_allocation_; 
    }
    
    /**
     * @brief 获取总时间
     */
    double getTotalTime() const;
    
    /**
     * @brief 获取优化目标值（总jerk的平方积分）
     */
    double getObjectiveValue() const;

private:
    /**
     * @brief 初始化时间分配（启发式方法）
     */
    void initializeTimeAllocation();
    
    /**
     * @brief 为单个轴优化轨迹
     * @param axis_positions 该轴的所有路径点位置
     * @param initial_vel 初始速度
     * @param initial_acc 初始加速度
     * @param final_vel 终止速度
     * @param final_acc 终止加速度
     * @return 优化后的轨迹段
     */
    std::vector<PolynomialTrajectory> optimizeSingleAxis(
        const std::vector<double>& axis_positions,
        double initial_vel,
        double initial_acc,
        double final_vel,
        double final_acc);
    
    /**
     * @brief 构建单段轨迹的系数矩阵
     * @param t 时间duration
     * @return 5x5矩阵
     */
    Eigen::MatrixXd buildCoefficientMatrix(double t) const;
    
    /**
     * @brief 计算单段轨迹的Q矩阵（jerk积分的二次型）
     * @param t 时间duration
     * @return 5x5矩阵
     */
    Eigen::MatrixXd buildQMatrix(double t) const;

private:
    std::vector<Waypoint> waypoints_;
    OptimizerConfig config_;
    
    std::vector<Trajectory2D> trajectories_;  // 优化后的轨迹段
    std::vector<double> time_allocation_;     // 每段的时间分配
};

} // namespace trajectory

#endif // TRAJECTORY_OPTIMIZER_HPP


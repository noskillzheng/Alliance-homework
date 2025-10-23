#ifndef CONSTRAINT_CHECKER_HPP
#define CONSTRAINT_CHECKER_HPP

#include "core/polynomial_trajectory.hpp"
#include <vector>

namespace trajectory {

/**
 * @brief 约束检查器
 * 
 * 检查轨迹是否满足速度和加速度约束
 */
class ConstraintChecker {
public:
    /**
     * @brief 构造函数
     * @param max_velocity 最大速度
     * @param max_acceleration 最大加速度
     * @param sampling_points 采样点数
     */
    ConstraintChecker(double max_velocity, 
                     double max_acceleration,
                     int sampling_points = 100);
    
    /**
     * @brief 检查单段轨迹是否满足约束
     * @param trajectory 待检查的轨迹
     * @return 是否满足约束
     */
    bool checkTrajectory(const Trajectory2D& trajectory) const;
    
    /**
     * @brief 检查多段轨迹是否都满足约束
     * @param trajectories 轨迹段集合
     * @return 是否所有段都满足约束
     */
    bool checkAllTrajectories(const std::vector<Trajectory2D>& trajectories) const;
    
    /**
     * @brief 获取轨迹的最大速度
     */
    double getMaxVelocity(const Trajectory2D& trajectory) const;
    
    /**
     * @brief 获取轨迹的最大加速度
     */
    double getMaxAcceleration(const Trajectory2D& trajectory) const;
    
    /**
     * @brief 打印约束检查报告
     */
    void printReport(const std::vector<Trajectory2D>& trajectories) const;
    
    /**
     * @brief 打印详细的连续性检查报告（验证C^2连续）
     */
    void printContinuityReport(const std::vector<Trajectory2D>& trajectories) const;
    
    /**
     * @brief 计算轨迹曲率
     * @param trajectory 2D轨迹
     * @param t 时间点
     * @return 曲率值 κ
     */
    static double computeCurvature(const Trajectory2D& trajectory, double t);

private:
    double max_velocity_;
    double max_acceleration_;
    int sampling_points_;
};

} // namespace trajectory

#endif // CONSTRAINT_CHECKER_HPP


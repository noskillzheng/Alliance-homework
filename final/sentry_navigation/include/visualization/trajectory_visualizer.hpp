#ifndef TRAJECTORY_VISUALIZER_HPP
#define TRAJECTORY_VISUALIZER_HPP

#include "core/polynomial_trajectory.hpp"
#include <vector>
#include <string>

 

namespace trajectory {

/**
 * @brief 轨迹可视化器
 * 
 * 支持以下可视化方式：
 * 1. 控制台输出
 * 2. CSV文件导出
 * 3. ROS2消息发布（用于Foxglove）
 */
class TrajectoryVisualizer {
public:
    /**
     * @brief 构造函数
     */
    TrajectoryVisualizer();
    
    /**
     * @brief 可视化轨迹
     * @param trajectories 轨迹段集合
     * @param time_allocation 时间分配
     * @param waypoints 原始路径点
     */
    void visualize(
        const std::vector<Trajectory2D>& trajectories,
        const std::vector<double>& time_allocation,
        const std::vector<Eigen::Vector2d>& waypoints);
    
    /**
     * @brief 打印轨迹信息到控制台
     */
    void printToConsole(
        const std::vector<Trajectory2D>& trajectories,
        const std::vector<double>& time_allocation,
        const std::vector<Eigen::Vector2d>& waypoints);
    
    /**
     * @brief 导出到CSV文件
     */
    void exportToCSV(
        const std::vector<Trajectory2D>& trajectories,
        const std::vector<double>& time_allocation,
        const std::string& trajectory_file = "trajectory.csv",
        const std::string& waypoints_file = "waypoints.csv");

 
};

} // namespace trajectory

#endif // TRAJECTORY_VISUALIZER_HPP


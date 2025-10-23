#ifndef CSV_WRITER_HPP
#define CSV_WRITER_HPP

#include "core/polynomial_trajectory.hpp"
#include <string>
#include <vector>

namespace trajectory {

/**
 * @brief CSV文件写入器
 * 
 * 用于将轨迹数据导出到CSV文件，便于可视化和分析
 */
class CSVWriter {
public:
    /**
     * @brief 将轨迹数据写入CSV文件
     * @param filename 文件名
     * @param trajectories 轨迹段集合
     * @param time_allocation 时间分配
     * @param sampling_rate 采样率（Hz）
     * @return 是否写入成功
     */
    static bool writeTrajectory(
        const std::string& filename,
        const std::vector<Trajectory2D>& trajectories,
        const std::vector<double>& time_allocation,
        double sampling_rate = 100.0);
    
    /**
     * @brief 将路径点写入CSV文件
     * @param filename 文件名
     * @param waypoints 路径点
     * @return 是否写入成功
     */
    static bool writeWaypoints(
        const std::string& filename,
        const std::vector<Eigen::Vector2d>& waypoints);
};

} // namespace trajectory

#endif // CSV_WRITER_HPP


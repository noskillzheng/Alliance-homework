#include "visualization/trajectory_visualizer.hpp"
#include "utils/csv_writer.hpp"
#include <iostream>
#include <iomanip>

namespace trajectory {

TrajectoryVisualizer::TrajectoryVisualizer() {
    std::cout << "ROS2未启用，仅支持控制台和CSV导出" << std::endl;
}

void TrajectoryVisualizer::visualize(
    const std::vector<Trajectory2D>& trajectories,
    const std::vector<double>& time_allocation,
    const std::vector<Eigen::Vector2d>& waypoints) {
    
    // 1. 打印到控制台
    printToConsole(trajectories, time_allocation, waypoints);
    
    // 2. 导出CSV
    exportToCSV(trajectories, time_allocation);
    
 
}

void TrajectoryVisualizer::printToConsole(
    const std::vector<Trajectory2D>& trajectories,
    const std::vector<double>& time_allocation,
    const std::vector<Eigen::Vector2d>& waypoints) {
    
    std::cout << "\n========== 轨迹优化结果 ==========\n";
    std::cout << std::fixed << std::setprecision(4);
    
    std::cout << "\n路径点:\n";
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "  P" << i << ": (" 
                  << waypoints[i].x() << ", " 
                  << waypoints[i].y() << ")\n";
    }
    
    std::cout << "\n轨迹段信息:\n";
    double total_time = 0.0;
    for (size_t i = 0; i < trajectories.size(); ++i) {
        std::cout << "  段 " << i << ":\n";
        std::cout << "    持续时间: " << time_allocation[i] << " s\n";
        std::cout << "    起点: (" 
                  << trajectories[i].position(0.0).x() << ", "
                  << trajectories[i].position(0.0).y() << ")\n";
        std::cout << "    终点: (" 
                  << trajectories[i].position(time_allocation[i]).x() << ", "
                  << trajectories[i].position(time_allocation[i]).y() << ")\n";
        total_time += time_allocation[i];
    }
    
    std::cout << "\n总时间: " << total_time << " s\n";
    std::cout << "==================================\n\n";
}

void TrajectoryVisualizer::exportToCSV(
    const std::vector<Trajectory2D>& trajectories,
    const std::vector<double>& time_allocation,
    const std::string& trajectory_file,
    const std::string& waypoints_file) {
    
    CSVWriter::writeTrajectory(trajectory_file, trajectories, time_allocation);
    
    // 提取路径点
    std::vector<Eigen::Vector2d> waypoints;
    waypoints.push_back(trajectories[0].position(0.0));
    for (size_t i = 0; i < trajectories.size(); ++i) {
        waypoints.push_back(trajectories[i].position(time_allocation[i]));
    }
    
    CSVWriter::writeWaypoints(waypoints_file, waypoints);
}

 

} // namespace trajectory


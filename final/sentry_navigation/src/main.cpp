#include "core/trajectory_optimizer.hpp"
#include "core/constraint_checker.hpp"
#include "visualization/trajectory_visualizer.hpp"
#include "utils/config_loader.hpp"
#include <iostream>
#include <vector>

 

using namespace trajectory;

int main() {
    std::cout << "========================================\n";
    std::cout << "  哨兵导航 - 离散点平滑轨迹规划器\n";
    std::cout << "========================================\n\n";
    
    std::cout << "独立模式: 使用CSV导出可视化\n\n";
    
    // ========== 1. 定义路径点 ==========
    std::vector<Waypoint> waypoints = {
        Waypoint(0.0, 0.0),
        Waypoint(1.0, 0.0),
        Waypoint(1.0, 1.0),
        Waypoint(1.0, 2.0),
        Waypoint(2.0, 3.0)
    };
    
    std::cout << "输入的路径点:\n";
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "  P" << i << ": (" 
                  << waypoints[i].position.x() << ", " 
                  << waypoints[i].position.y() << ")\n";
    }
    std::cout << "\n";
    
    // ========== 2. 加载配置文件 ==========
    std::string config_file = "config/optimizer_config.yaml";
    if (!ConfigLoader::loadFromFile(config_file)) {
        std::cerr << "警告: 无法加载配置文件，使用默认值\n";
    } else {
        std::cout << "✓ 已加载配置文件: " << config_file << "\n";
    }
    
    // ========== 3. 配置优化器 ==========
    OptimizerConfig config;
    
    // 从配置文件读取约束参数
    config.max_velocity = ConfigLoader::getDouble("constraints.max_velocity", 1.0);
    config.max_acceleration = ConfigLoader::getDouble("constraints.max_acceleration", 1.0);
    config.max_segment_time = ConfigLoader::getDouble("constraints.max_segment_time", 10.0);
    config.sampling_points = ConfigLoader::getInt("optimization.sampling_points", 100);
    
    // 从配置文件读取性能调优参数
    config.time_safety_factor = ConfigLoader::getDouble("performance.time_safety_factor", 1.05);
    config.velocity_utilization = ConfigLoader::getDouble("performance.velocity_utilization", 0.85);
    config.waypoint_velocity_ratio = ConfigLoader::getDouble("performance.waypoint_velocity_ratio", 0.85);
    config.min_segment_time = ConfigLoader::getDouble("performance.min_segment_time", 0.5);
    
    // 读取边界条件
    auto init_vel = ConfigLoader::getDoubleArray("boundary_conditions.initial.velocity");
    auto init_acc = ConfigLoader::getDoubleArray("boundary_conditions.initial.acceleration");
    auto final_vel = ConfigLoader::getDoubleArray("boundary_conditions.final.velocity");
    auto final_acc = ConfigLoader::getDoubleArray("boundary_conditions.final.acceleration");
    
    if (init_vel.size() == 2) config.initial_velocity = Eigen::Vector2d(init_vel[0], init_vel[1]);
    if (init_acc.size() == 2) config.initial_acceleration = Eigen::Vector2d(init_acc[0], init_acc[1]);
    if (final_vel.size() == 2) config.final_velocity = Eigen::Vector2d(final_vel[0], final_vel[1]);
    if (final_acc.size() == 2) config.final_acceleration = Eigen::Vector2d(final_acc[0], final_acc[1]);
    
    std::cout << "\n优化配置:\n";
    std::cout << "  【约束】\n";
    std::cout << "    最大速度: " << config.max_velocity << " m/s\n";
    std::cout << "    最大加速度: " << config.max_acceleration << " m/s²\n";
    std::cout << "    每段最大时间: " << config.max_segment_time << " s\n";
    std::cout << "  【性能调优】\n";
    std::cout << "    时间安全系数: " << config.time_safety_factor << "\n";
    std::cout << "    速度利用率: " << (config.velocity_utilization * 100) << "%\n";
    std::cout << "    段间速度保持率: " << (config.waypoint_velocity_ratio * 100) << "%\n";
    std::cout << "\n";
    
    // ========== 4. 执行优化 ==========
    std::cout << "开始优化...\n";
    TrajectoryOptimizer optimizer(waypoints, config);
    
    if (!optimizer.optimize()) {
        std::cerr << "优化失败！\n";
        return -1;
    }
    
    std::cout << "优化完成！\n";
    std::cout << "  总时间: " << optimizer.getTotalTime() << " s\n";
    std::cout << "  目标值 (∫jerk²dt): " << optimizer.getObjectiveValue() << "\n";
    std::cout << "\n";
    
    // ========== 5. 检查约束 ==========
    std::cout << "检查约束...\n";
    ConstraintChecker checker(
        config.max_velocity, 
        config.max_acceleration, 
        config.sampling_points
    );
    
    bool constraints_satisfied = checker.checkAllTrajectories(
        optimizer.getTrajectories()
    );
    
    checker.printReport(optimizer.getTrajectories());
    
    // 验证C^2连续性
    checker.printContinuityReport(optimizer.getTrajectories());
    
    if (!constraints_satisfied) {
        std::cout << "警告: 存在约束违反，可能需要调整时间分配\n\n";
    }
    
    // ========== 6. 可视化 ==========
    std::cout << "生成可视化...\n";
    
    // 提取路径点位置
    std::vector<Eigen::Vector2d> waypoint_positions;
    for (const auto& wp : waypoints) {
        waypoint_positions.push_back(wp.position);
    }
    
    TrajectoryVisualizer visualizer;
    
    visualizer.visualize(
        optimizer.getTrajectories(),
        optimizer.getTimeAllocation(),
        waypoint_positions
    );
    
    std::cout << "\n========================================\n";
    std::cout << "程序执行完成！\n";
    std::cout << "\n可视化方式:\n";
    std::cout << "1. 查看 trajectory.csv 和 waypoints.csv\n";
    std::cout << "   - 使用Python/Matplotlib绘图\n";
    std::cout << "   - 使用Excel或其他工具\n";
 
    std::cout << "========================================\n";
    
    return 0;
}


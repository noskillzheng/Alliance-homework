#include "utils/csv_writer.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>

namespace trajectory {

bool CSVWriter::writeTrajectory(
    const std::string& filename,
    const std::vector<Trajectory2D>& trajectories,
    const std::vector<double>& time_allocation,
    double sampling_rate) {
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return false;
    }
    
    // 写入表头（增强版：添加曲率、段编号、速度/加速度利用率）
    file << "time,x,y,vx,vy,ax,ay,speed,acceleration,curvature,segment,v_util,a_util\n";
    file << std::fixed << std::setprecision(6);
    
    double global_time = 0.0;
    
    for (size_t seg = 0; seg < trajectories.size(); ++seg) {
        const auto& traj = trajectories[seg];
        double duration = time_allocation[seg];
        int num_samples = static_cast<int>(duration * sampling_rate);
        double dt = duration / num_samples;
        
        for (int i = 0; i <= num_samples; ++i) {
            double local_t = i * dt;
            double t = global_time + local_t;
            
            auto pos = traj.position(local_t);
            auto vel = traj.velocity(local_t);
            auto acc = traj.acceleration(local_t);
            double speed = vel.norm();
            double acc_mag = acc.norm();
            
            // 计算曲率 κ = |v × a| / |v|^3
            double vx = vel.x(), vy = vel.y();
            double ax = acc.x(), ay = acc.y();
            double cross = vx * ay - vy * ax;
            double curvature = (speed > 1e-6) ? std::abs(cross) / (speed * speed * speed) : 0.0;
            
            // 速度和加速度利用率（用于热力图着色）
            double v_util = speed;        // 实际值，可视化时与max比较
            double a_util = acc_mag;
            
            file << t << ","
                 << pos.x() << "," << pos.y() << ","
                 << vel.x() << "," << vel.y() << ","
                 << acc.x() << "," << acc.y() << ","
                 << speed << "," << acc_mag << ","
                 << curvature << "," << seg << ","
                 << v_util << "," << a_util << "\n";
        }
        
        global_time += duration;
    }
    
    file.close();
    std::cout << "轨迹数据已保存到: " << filename << std::endl;
    return true;
}

bool CSVWriter::writeWaypoints(
    const std::string& filename,
    const std::vector<Eigen::Vector2d>& waypoints) {
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return false;
    }
    
    // 写入表头
    file << "x,y\n";
    file << std::fixed << std::setprecision(6);
    
    for (const auto& wp : waypoints) {
        file << wp.x() << "," << wp.y() << "\n";
    }
    
    file.close();
    std::cout << "路径点数据已保存到: " << filename << std::endl;
    return true;
}

} // namespace trajectory


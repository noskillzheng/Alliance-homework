#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include "common/armor.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <deque>

namespace armor_detector {

/**
 * @brief 可视化工具类
 * 用于绘制装甲板检测结果、预测结果和轨迹
 */
class Visualizer {
public:
    /**
     * @brief 构造函数
     * @param max_trajectory_length 最大轨迹长度（用于显示历史轨迹）
     */
    explicit Visualizer(size_t max_trajectory_length = 50);
    
    /**
     * @brief 绘制装甲板（角点和连线）
     * @param img 图像
     * @param armor 装甲板
     * @param color 绘制颜色（默认根据装甲板颜色自动选择）
     */
    void drawArmor(cv::Mat& img, const Armor& armor, const cv::Scalar& color = cv::Scalar(-1, -1, -1));
    
    /**
     * @brief 绘制多个装甲板
     * @param img 图像
     * @param armors 装甲板列表
     */
    void drawArmors(cv::Mat& img, const std::vector<Armor>& armors);
    
    /**
     * @brief 绘制预测位置
     * @param img 图像
     * @param predicted_pos 预测位置
     * @param current_pos 当前位置（可选，用于绘制预测向量）
     */
    void drawPrediction(cv::Mat& img, const cv::Point2f& predicted_pos, 
                       const cv::Point2f& current_pos = cv::Point2f(-1, -1));
    
    /**
     * @brief 绘制运动轨迹
     * @param img 图像
     * @param trajectory 轨迹点列表
     */
    void drawTrajectory(cv::Mat& img, const std::vector<cv::Point2f>& trajectory);
    
    /**
     * @brief 添加轨迹点（自动维护轨迹历史）
     * @param point 新的轨迹点
     */
    void addTrajectoryPoint(const cv::Point2f& point);
    
    /**
     * @brief 绘制内部维护的轨迹
     * @param img 图像
     */
    void drawInternalTrajectory(cv::Mat& img);
    
    /**
     * @brief 绘制FPS信息
     * @param img 图像
     * @param fps FPS值
     */
    void drawFPS(cv::Mat& img, float fps);
    
    /**
     * @brief 绘制时间延迟信息
     * @param img 图像
     * @param latency 延迟（毫秒）
     */
    void drawLatency(cv::Mat& img, float latency);
    
    /**
     * @brief 绘制信息面板（包含FPS、延迟、状态等）
     * @param img 图像
     * @param fps FPS
     * @param latency 延迟（毫秒）
     * @param armor_count 检测到的装甲板数量
     * @param has_prediction 是否有预测
     */
    void drawInfoPanel(cv::Mat& img, float fps, float latency, 
                      int armor_count, bool has_prediction);
    
    /**
     * @brief 清空轨迹历史
     */
    void clearTrajectory();
    
    /**
     * @brief 设置最大轨迹长度
     */
    void setMaxTrajectoryLength(size_t length) { max_trajectory_length_ = length; }

private:
    std::deque<cv::Point2f> trajectory_;    // 轨迹点队列
    size_t max_trajectory_length_;          // 最大轨迹长度
    
    /**
     * @brief 根据装甲板颜色获取绘制颜色
     */
    cv::Scalar getArmorColor(int armor_color);
};

} // namespace armor_detector

#endif // VISUALIZER_HPP


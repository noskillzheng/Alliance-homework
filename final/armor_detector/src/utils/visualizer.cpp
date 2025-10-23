#include "utils/visualizer.hpp"
#include <iostream>

namespace armor_detector {

Visualizer::Visualizer(size_t max_trajectory_length)
    : max_trajectory_length_(max_trajectory_length) {
    std::cout << "[Visualizer] 可视化工具初始化完成" << std::endl;
}

cv::Scalar Visualizer::getArmorColor(int armor_color) {
    // 0: 蓝色, 1: 红色
    if (armor_color == 0) {
        return cv::Scalar(255, 0, 0);  // BGR格式：蓝色
    } else {
        return cv::Scalar(0, 0, 255);  // BGR格式：红色
    }
}

void Visualizer::drawArmor(cv::Mat& img, const Armor& armor, const cv::Scalar& color) {
    if (!armor.isValid()) {
        return;
    }
    
    // 确定绘制颜色
    cv::Scalar draw_color = color;
    if (color[0] < 0) {  // 如果没有指定颜色，使用装甲板颜色
        draw_color = getArmorColor(armor.color);
    }
    
    std::vector<cv::Point2f> corners = armor.corners.toVector();
    
    // 1. 绘制四个角点（大圆点，突出显示）
    cv::circle(img, armor.corners.top_left, 6, draw_color, -1);
    cv::circle(img, armor.corners.top_right, 6, draw_color, -1);
    cv::circle(img, armor.corners.bottom_right, 6, draw_color, -1);
    cv::circle(img, armor.corners.bottom_left, 6, draw_color, -1);
    
    // 2. 绘制边框连线（粗线）
    for (size_t i = 0; i < 4; i++) {
        cv::line(img, corners[i], corners[(i+1)%4], draw_color, 3);
    }
    
    // 3. 绘制对角线（参考样例图）
    cv::line(img, corners[0], corners[2], draw_color, 2);
    cv::line(img, corners[1], corners[3], draw_color, 2);
    
    // 4. 绘制中心点（绿色）
    cv::circle(img, armor.center, 8, cv::Scalar(0, 255, 0), 2);  // 绿色空心圆
    cv::circle(img, armor.center, 3, cv::Scalar(0, 255, 0), -1); // 绿色实心圆
    
    // 5. 绘制置信度文本
    std::string text = cv::format("%.2f", armor.confidence);
    cv::putText(img, text, 
                cv::Point(armor.center.x - 20, armor.center.y - 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, draw_color, 2);
}

void Visualizer::drawArmors(cv::Mat& img, const std::vector<Armor>& armors) {
    for (const auto& armor : armors) {
        drawArmor(img, armor);
    }
}

void Visualizer::drawPrediction(cv::Mat& img, const cv::Point2f& predicted_pos, 
                                const cv::Point2f& current_pos) {
    // 绘制预测位置（黄色大圆圈）
    cv::circle(img, predicted_pos, 10, cv::Scalar(0, 255, 255), 2);  // 黄色空心圆
    cv::circle(img, predicted_pos, 3, cv::Scalar(0, 255, 255), -1);  // 黄色实心圆
    
    // 如果提供了当前位置，绘制预测向量（箭头）
    if (current_pos.x >= 0 && current_pos.y >= 0) {
        cv::arrowedLine(img, current_pos, predicted_pos, 
                       cv::Scalar(0, 255, 255), 2, cv::LINE_AA, 0, 0.3);
    }
    
    // 添加标签
    cv::putText(img, "Predicted (0.5s)", 
                cv::Point(predicted_pos.x + 15, predicted_pos.y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1.5);
}

void Visualizer::drawTrajectory(cv::Mat& img, const std::vector<cv::Point2f>& trajectory) {
    if (trajectory.size() < 2) {
        return;
    }
    
    // 绘制轨迹线，颜色从淡到深表示时间从旧到新
    for (size_t i = 1; i < trajectory.size(); i++) {
        // 计算颜色渐变（从灰色到青色）
        float alpha = static_cast<float>(i) / trajectory.size();
        cv::Scalar color(255 * (1 - alpha), 255 * alpha, 255 * alpha);
        
        cv::line(img, trajectory[i - 1], trajectory[i], color, 2, cv::LINE_AA);
    }
    
    // 绘制轨迹点
    for (const auto& point : trajectory) {
        cv::circle(img, point, 2, cv::Scalar(255, 255, 0), -1);
    }
}

void Visualizer::addTrajectoryPoint(const cv::Point2f& point) {
    // 添加新点到轨迹
    trajectory_.push_back(point);
    
    // 如果超过最大长度，移除最旧的点
    while (trajectory_.size() > max_trajectory_length_) {
        trajectory_.pop_front();
    }
}

void Visualizer::drawInternalTrajectory(cv::Mat& img) {
    if (trajectory_.empty()) {
        return;
    }
    
    // 将deque转换为vector
    std::vector<cv::Point2f> trajectory_vec(trajectory_.begin(), trajectory_.end());
    drawTrajectory(img, trajectory_vec);
}

void Visualizer::drawFPS(cv::Mat& img, float fps) {
    std::string fps_text = cv::format("FPS: %.1f", fps);
    
    // 绘制背景矩形
    cv::rectangle(img, cv::Point(10, 10), cv::Point(150, 40), 
                 cv::Scalar(0, 0, 0), -1);
    
    // 绘制文本
    cv::putText(img, fps_text, cv::Point(20, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
}

void Visualizer::drawLatency(cv::Mat& img, float latency) {
    std::string latency_text = cv::format("Latency: %.1f ms", latency);
    
    // 绘制背景矩形
    cv::rectangle(img, cv::Point(10, 50), cv::Point(200, 80), 
                 cv::Scalar(0, 0, 0), -1);
    
    // 绘制文本
    cv::putText(img, latency_text, cv::Point(20, 70),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
}

void Visualizer::drawInfoPanel(cv::Mat& img, float fps, float latency, 
                               int armor_count, bool has_prediction) {
    int panel_width = 280;
    int panel_height = 150;
    int margin = 10;
    
    // 绘制半透明背景
    cv::Mat overlay = img.clone();
    cv::rectangle(overlay, 
                 cv::Point(margin, margin), 
                 cv::Point(margin + panel_width, margin + panel_height),
                 cv::Scalar(0, 0, 0), -1);
    cv::addWeighted(overlay, 0.6, img, 0.4, 0, img);
    
    // 绘制边框
    cv::rectangle(img, 
                 cv::Point(margin, margin), 
                 cv::Point(margin + panel_width, margin + panel_height),
                 cv::Scalar(255, 255, 255), 2);
    
    int text_x = margin + 15;
    int text_y = margin + 30;
    int line_height = 30;
    
    // 绘制信息
    cv::putText(img, cv::format("FPS: %.1f", fps),
                cv::Point(text_x, text_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1.5);
    
    text_y += line_height;
    cv::putText(img, cv::format("Latency: %.1f ms", latency),
                cv::Point(text_x, text_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 1.5);
    
    text_y += line_height;
    cv::Scalar armor_color = armor_count > 0 ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::putText(img, cv::format("Armors: %d", armor_count),
                cv::Point(text_x, text_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, armor_color, 1.5);
    
    text_y += line_height;
    cv::Scalar pred_color = has_prediction ? cv::Scalar(0, 255, 255) : cv::Scalar(128, 128, 128);
    std::string pred_text = has_prediction ? "Prediction: ON" : "Prediction: OFF";
    cv::putText(img, pred_text,
                cv::Point(text_x, text_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, pred_color, 1.5);
}

void Visualizer::clearTrajectory() {
    trajectory_.clear();
    std::cout << "[Visualizer] 轨迹已清空" << std::endl;
}

} // namespace armor_detector


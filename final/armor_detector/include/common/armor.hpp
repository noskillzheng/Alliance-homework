#ifndef ARMOR_HPP
#define ARMOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>

namespace armor_detector {

/**
 * @brief 装甲板四个角点结构体
 * 用于存储装甲板的四个角点坐标
 */
struct ArmorCorners {
    cv::Point2f top_left;       // 左上角点
    cv::Point2f top_right;      // 右上角点
    cv::Point2f bottom_right;   // 右下角点
    cv::Point2f bottom_left;    // 左下角点
    
    // 获取所有角点的向量表示（用于绘制多边形）
    std::vector<cv::Point2f> toVector() const {
        return {top_left, top_right, bottom_right, bottom_left};
    }
};

/**
 * @brief 装甲板完整信息结构体
 * 包含装甲板的所有必要信息：位置、大小、颜色、置信度等
 */
struct Armor {
    ArmorCorners corners;       // 四个角点
    cv::Point2f center;         // 中心点坐标
    float width;                // 装甲板宽度（像素）
    float height;               // 装甲板高度（像素）
    int color;                  // 装甲板颜色：0=蓝色, 1=红色
    float confidence;           // 检测置信度 [0, 1]
    int64_t timestamp;          // 时间戳（毫秒）
    
    // 默认构造函数
    Armor() : width(0), height(0), color(0), confidence(0), timestamp(0) {}
    
    // 判断装甲板是否有效
    bool isValid() const {
        return confidence > 0.0f && width > 0 && height > 0;
    }
};

/**
 * @brief 灯条结构体
 * 用于传统检测方法中的灯条检测和匹配
 */
struct LightBar {
    cv::RotatedRect rect;       // 旋转矩形
    cv::Point2f center;         // 中心点
    float angle;                // 旋转角度
    float length;               // 长度
    float width;                // 宽度
    float area;                 // 面积
    
    // 默认构造函数
    LightBar() : angle(0), length(0), width(0), area(0) {}
    
    // 从旋转矩形构造
    explicit LightBar(const cv::RotatedRect& r) : rect(r) {
        center = r.center;
        angle = r.angle;
        length = std::max(r.size.width, r.size.height);
        width = std::min(r.size.width, r.size.height);
        area = r.size.area();
    }
    
    // 获取长宽比
    float aspectRatio() const {
        return width > 0 ? length / width : 0;
    }
};

/**
 * @brief 预测结果结构体
 * 用于存储卡尔曼滤波的预测结果（支持8维增强模式）
 */
struct PredictionResult {
    cv::Point2f position;       // 预测位置
    cv::Size2f size;            // 预测尺寸（8维模式）
    cv::Point2f velocity;       // 预测速度
    cv::Point2f size_velocity;  // 尺寸变化速度（8维模式）
    float confidence;           // 预测置信度
    int64_t timestamp;          // 预测时间戳
    
    PredictionResult() : size(0, 0), confidence(0), timestamp(0) {}
};

} // namespace armor_detector

#endif // ARMOR_HPP


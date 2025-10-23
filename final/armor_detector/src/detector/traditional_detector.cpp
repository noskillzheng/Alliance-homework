#include "detector/traditional_detector.hpp"
#include <iostream>
#include <algorithm>

namespace armor_detector {

TraditionalDetector::TraditionalDetector(const std::string& target_color)
    : target_color_(target_color),
      color_method_("channel_separation"),
      binary_threshold_(80),
      use_otsu_(false),
      otsu_mode_("roi"),
      otsu_roi_scale_(1.8),
      otsu_min_threshold_(40),
      otsu_max_threshold_(150),
      otsu_temporal_smooth_(0.7),
      last_otsu_threshold_(80),
      has_prediction_roi_(false),
      min_light_area_(50.0f),
      max_light_ratio_(4.0f),
      min_armor_ratio_(1.5f),
      max_armor_ratio_(5.0f),
      max_angle_diff_(15.0f),
      max_height_diff_ratio_(0.3f),
      max_y_diff_ratio_(0.5f),
      debug_mode_(false) {
    
    // 设置默认HSV阈值（实战优化参数）
    if (target_color_ == "red") {
        // 红色第一段：[0-10]
        hsv_low_ = cv::Scalar(0, 120, 150);
        hsv_high_ = cv::Scalar(10, 255, 255);
        // 红色第二段：[170-180]
        hsv_low2_ = cv::Scalar(170, 120, 150);
        hsv_high2_ = cv::Scalar(180, 255, 255);
    } else if (target_color_ == "blue") {
        hsv_low_ = cv::Scalar(100, 120, 100);
        hsv_high_ = cv::Scalar(130, 255, 255);
        // 蓝色不需要第二段
        hsv_low2_ = cv::Scalar(0, 0, 0);
        hsv_high2_ = cv::Scalar(0, 0, 0);
    }
    
    std::cout << "[TraditionalDetector] 初始化完成，目标颜色: " << target_color_ << std::endl;
    std::cout << "[TraditionalDetector] 颜色提取方法: " << color_method_ << std::endl;
}

std::vector<Armor> TraditionalDetector::detect(const cv::Mat& frame) {
    if (frame.empty()) {
        return {};
    }
    
    // 1. 颜色分割
    cv::Mat binary = colorSegmentation(frame);
    
    // 2. 查找灯条
    std::vector<LightBar> light_bars = findLightBars(binary);
    
    // 3. 匹配装甲板
    std::vector<Armor> armors = matchArmor(light_bars);
    
    // 调试模式：绘制中间结果
    if (debug_mode_) {
        debug_image_ = frame.clone();
        
        // 绘制灯条
        for (const auto& bar : light_bars) {
            cv::Point2f vertices[4];
            bar.rect.points(vertices);
            for (int i = 0; i < 4; i++) {
                cv::line(debug_image_, vertices[i], vertices[(i+1)%4], cv::Scalar(0, 255, 0), 2);
            }
        }
    }
    
    return armors;
}

void TraditionalDetector::setHSVThreshold(const cv::Scalar& hsv_low, const cv::Scalar& hsv_high) {
    hsv_low_ = hsv_low;
    hsv_high_ = hsv_high;
    std::cout << "[TraditionalDetector] 更新HSV阈值: [" 
              << hsv_low[0] << "," << hsv_low[1] << "," << hsv_low[2] << "] - ["
              << hsv_high[0] << "," << hsv_high[1] << "," << hsv_high[2] << "]" << std::endl;
}

void TraditionalDetector::setTargetColor(const std::string& color) {
    target_color_ = color;
    
    // 更新HSV阈值（实战优化参数）
    if (target_color_ == "red") {
        hsv_low_ = cv::Scalar(0, 120, 150);
        hsv_high_ = cv::Scalar(10, 255, 255);
        hsv_low2_ = cv::Scalar(170, 120, 150);
        hsv_high2_ = cv::Scalar(180, 255, 255);
    } else if (target_color_ == "blue") {
        hsv_low_ = cv::Scalar(100, 120, 100);
        hsv_high_ = cv::Scalar(130, 255, 255);
        hsv_low2_ = cv::Scalar(0, 0, 0);
        hsv_high2_ = cv::Scalar(0, 0, 0);
    }
    
    std::cout << "[TraditionalDetector] 切换目标颜色: " << target_color_ << std::endl;
}

void TraditionalDetector::setColorMethod(const std::string& method) {
    color_method_ = method;
    std::cout << "[TraditionalDetector] 颜色提取方法: " << color_method_ << std::endl;
}

void TraditionalDetector::setOtsuParams(bool use_otsu, const std::string& mode, double roi_scale,
                                        int min_thresh, int max_thresh, double temporal_smooth) {
    use_otsu_ = use_otsu;
    otsu_mode_ = mode;
    otsu_roi_scale_ = roi_scale;
    otsu_min_threshold_ = min_thresh;
    otsu_max_threshold_ = max_thresh;
    otsu_temporal_smooth_ = temporal_smooth;
    
    std::cout << "[TraditionalDetector] Otsu自适应阈值: " << (use_otsu ? "启用" : "禁用") << std::endl;
    if (use_otsu) {
        std::cout << "  模式: " << mode << ", ROI缩放: " << roi_scale 
                  << ", 阈值范围: [" << min_thresh << ", " << max_thresh << "]" << std::endl;
    }
}

int TraditionalDetector::calculateAdaptiveThreshold(const cv::Mat& gray) {
    int otsu_thresh = binary_threshold_;
    cv::Mat temp;
    
    if (otsu_mode_ == "global") {
        // 全局Otsu
        otsu_thresh = static_cast<int>(cv::threshold(gray, temp, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU));
        
    } else if (otsu_mode_ == "roi" && has_prediction_roi_) {
        // ROI内Otsu
        cv::Rect safe_roi = prediction_roi_ & cv::Rect(0, 0, gray.cols, gray.rows);
        
        if (safe_roi.area() > 100) {
            cv::Mat roi_gray = gray(safe_roi).clone();
            otsu_thresh = static_cast<int>(cv::threshold(roi_gray, temp, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU));
        } else {
            // ROI太小，使用全局
            otsu_thresh = static_cast<int>(cv::threshold(gray, temp, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU));
        }
        
    } else {
        // 未初始化或无ROI，使用全局
        otsu_thresh = static_cast<int>(cv::threshold(gray, temp, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU));
    }
    
    // 限制阈值范围
    otsu_thresh = std::max(otsu_min_threshold_, std::min(otsu_max_threshold_, otsu_thresh));
    
    // 时序平滑
    if (last_otsu_threshold_ > 0) {
        otsu_thresh = static_cast<int>(otsu_temporal_smooth_ * last_otsu_threshold_ + 
                                       (1.0 - otsu_temporal_smooth_) * otsu_thresh);
    }
    
    last_otsu_threshold_ = otsu_thresh;
    
    return otsu_thresh;
}

cv::Mat TraditionalDetector::colorSegmentation(const cv::Mat& frame) {
    cv::Mat result;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    
    if (color_method_ == "channel_separation") {
        // 方法1：通道分离法（推荐 - 光照鲁棒性更好）
        std::vector<cv::Mat> channels;
        cv::split(frame, channels);  // BGR通道分离
        
        if (target_color_ == "red") {
            // 红色：红通道 - 蓝通道
            cv::subtract(channels[2], channels[0], result);
        } else {
            // 蓝色：蓝通道 - 红通道  
            cv::subtract(channels[0], channels[2], result);
        }
        
        // 二值化（使用自适应阈值或固定阈值）
        int adaptive_threshold = binary_threshold_;
        if (use_otsu_) {
            adaptive_threshold = calculateAdaptiveThreshold(result);
        }
        cv::threshold(result, result, adaptive_threshold, 255, cv::THRESH_BINARY);
        
        // 形态学处理：去噪和填充
        cv::morphologyEx(result, result, cv::MORPH_CLOSE, kernel);
        
    } else if (color_method_ == "hsv") {
        // 方法2：HSV方法（备用）
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, hsv_low_, hsv_high_, result);
        
        // 如果是红色，需要处理HSV环绕问题（红色在0度和180度附近）
        if (target_color_ == "red") {
            cv::Mat binary2;
            cv::inRange(hsv, hsv_low2_, hsv_high2_, binary2);
            cv::bitwise_or(result, binary2, result);
        }
        
        // 形态学处理：去噪
        cv::morphologyEx(result, result, cv::MORPH_OPEN, kernel);  // 开运算去噪
        cv::morphologyEx(result, result, cv::MORPH_CLOSE, kernel); // 闭运算填充
    }
    
    return result;
}

std::vector<LightBar> TraditionalDetector::findLightBars(const cv::Mat& binary) {
    std::vector<LightBar> light_bars;
    
    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // 遍历轮廓，筛选灯条
    for (const auto& contour : contours) {
        // 面积过滤
        float area = cv::contourArea(contour);
        if (area < min_light_area_) {
            continue;
        }
        
        // 拟合旋转矩形
        cv::RotatedRect rect = cv::minAreaRect(contour);
        LightBar bar(rect);
        
        // 灯条有效性判断
        if (isValidLightBar(bar)) {
            light_bars.push_back(bar);
        }
    }
    
    std::cout << "[TraditionalDetector] 找到 " << light_bars.size() << " 个有效灯条" << std::endl;
    return light_bars;
}

bool TraditionalDetector::isValidLightBar(const LightBar& bar) {
    // 1. 长宽比检查（灯条应该是细长的，但允许一定范围）
    float ratio = bar.aspectRatio();
    if (ratio < 1.5f || ratio > 8.0f) {  // 放宽到1.5-8.0
        return false;
    }
    
    // 2. 面积检查（根据实际装甲板大小调整）
    if (bar.area < 30.0f) return false;    // 太小的噪点
    if (bar.area > 5000.0f) return false;  // 太大，可能不是灯条
    
    // 3. 角度检查（接近竖直，但允许较大倾斜以适应透视）
    float angle = std::abs(bar.angle);
    if (angle > 45) {
        angle = 90 - angle;  // 旋转矩形的角度在[-90, 0]之间
    }
    if (angle > 40) {  // 允许40度倾斜
        return false;
    }
    
    return true;
}

std::vector<Armor> TraditionalDetector::matchArmor(const std::vector<LightBar>& light_bars) {
    std::vector<Armor> armors;
    
    // 至少需要两个灯条才能配对
    if (light_bars.size() < 2) {
        return armors;
    }
    
    // 两两配对
    for (size_t i = 0; i < light_bars.size(); i++) {
        for (size_t j = i + 1; j < light_bars.size(); j++) {
            const LightBar& left = light_bars[i].center.x < light_bars[j].center.x ? 
                                   light_bars[i] : light_bars[j];
            const LightBar& right = light_bars[i].center.x < light_bars[j].center.x ? 
                                    light_bars[j] : light_bars[i];
            
            // 判断是否能配对
            if (canMatch(left, right)) {
                Armor armor = buildArmor(left, right);
                armors.push_back(armor);
            }
        }
    }
    
    std::cout << "[TraditionalDetector] 匹配到 " << armors.size() << " 个装甲板" << std::endl;
    return armors;
}

bool TraditionalDetector::canMatch(const LightBar& left, const LightBar& right) {
    float avg_height = (left.length + right.length) / 2.0f;
    
    // 1. 角度差检查（允许透视导致的差异）
    float angle_diff = std::abs(left.angle - right.angle);
    if (angle_diff > 20.0f) return false;  // 放宽到20度
    
    // 2. 高度差检查（透视会导致近大远小）
    float height_diff = std::abs(left.length - right.length);
    if (height_diff / avg_height > 0.5f) return false;  // 允许50%差异
    
    // 3. Y坐标差检查（允许一定错位）
    float y_diff = std::abs(left.center.y - right.center.y);
    if (y_diff / avg_height > 1.5f) return false;  // 放宽限制
    
    // 4. 装甲板宽高比检查（关键！）
    float armor_width = std::abs(right.center.x - left.center.x);
    float armor_ratio = armor_width / avg_height;
    if (armor_ratio < 1.0f) return false;   // 太窄
    if (armor_ratio > 5.0f) return false;   // 太宽
    
    // 5. 倾斜方向一致性检查（新增！防止误匹配）
    bool same_direction = (left.angle * right.angle >= 0);
    if (!same_direction && std::abs(left.angle) > 5 && std::abs(right.angle) > 5) {
        return false;  // 倾斜方向相反，不匹配
    }
    
    // 6. 面积相似性检查（透视下也不应该差太多）
    float area_ratio = std::min(left.area, right.area) / 
                      std::max(left.area, right.area);
    if (area_ratio < 0.3f) return false;  // 面积差异太大
    
    return true;
}

Armor TraditionalDetector::buildArmor(const LightBar& left, const LightBar& right) {
    Armor armor;
    
    // 计算中心点
    armor.center.x = (left.center.x + right.center.x) / 2.0f;
    armor.center.y = (left.center.y + right.center.y) / 2.0f;
    
    // 计算宽度和高度
    armor.width = std::abs(right.center.x - left.center.x);
    armor.height = (left.length + right.length) / 2.0f;
    
    // 设置颜色
    armor.color = (target_color_ == "red") ? 1 : 0;
    
    // 计算置信度（基于匹配质量）
    float angle_diff = std::abs(left.angle - right.angle);
    float height_diff = std::abs(left.length - right.length) / armor.height;
    armor.confidence = 1.0f - (angle_diff / max_angle_diff_ + height_diff) / 2.0f;
    armor.confidence = std::max(0.0f, std::min(1.0f, armor.confidence));
    
    // 计算四个角点
    armor.corners = calculateCorners(left, right);
    
    // 时间戳
    armor.timestamp = cv::getTickCount() * 1000 / cv::getTickFrequency();
    
    return armor;
}

ArmorCorners TraditionalDetector::calculateCorners(const LightBar& left, const LightBar& right) {
    ArmorCorners corners;
    
    // 获取左右灯条的四个顶点
    cv::Point2f left_points[4], right_points[4];
    left.rect.points(left_points);
    right.rect.points(right_points);
    
    // 找到左灯条的左侧两个点（X坐标较小的）
    std::vector<cv::Point2f> left_side_points(left_points, left_points + 4);
    std::sort(left_side_points.begin(), left_side_points.end(), 
              [](const cv::Point2f& a, const cv::Point2f& b) { return a.x < b.x; });
    
    // 找到右灯条的右侧两个点（X坐标较大的）
    std::vector<cv::Point2f> right_side_points(right_points, right_points + 4);
    std::sort(right_side_points.begin(), right_side_points.end(), 
              [](const cv::Point2f& a, const cv::Point2f& b) { return a.x > b.x; });
    
    // 确定上下点
    if (left_side_points[0].y < left_side_points[1].y) {
        corners.top_left = left_side_points[0];
        corners.bottom_left = left_side_points[1];
    } else {
        corners.top_left = left_side_points[1];
        corners.bottom_left = left_side_points[0];
    }
    
    if (right_side_points[0].y < right_side_points[1].y) {
        corners.top_right = right_side_points[0];
        corners.bottom_right = right_side_points[1];
    } else {
        corners.top_right = right_side_points[1];
        corners.bottom_right = right_side_points[0];
    }
    
    return corners;
}

} // namespace armor_detector


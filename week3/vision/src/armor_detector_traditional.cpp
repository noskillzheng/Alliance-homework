#include "armor_detector_traditional.h"
#include <iostream>
#include <algorithm>
#include <fstream>

TraditionalArmorDetector::TraditionalArmorDetector(const std::string& color_mode)
    : color_mode(color_mode) {

    setupColorRanges();
    std::cout << "Traditional detector initialized" << std::endl;
    std::cout << "Color mode: " << color_mode << std::endl;
}

void TraditionalArmorDetector::setupColorRanges() {
    if (color_mode == "blue") {
        // 基于test.PNG分析的蓝色灯条特征调整
        // 灯条在黑暗环境中发出高亮度蓝光，需要更宽的范围
        lower_color1 = cv::Scalar(85, 60, 40);   // 降低饱和度下限，适应更多蓝光
        upper_color1 = cv::Scalar(135, 255, 255); // 扩大色调范围

        // 添加更宽的备用颜色范围，覆盖不同亮度的蓝光
        lower_color2 = cv::Scalar(95, 40, 30);   // 更低饱和度和亮度
        upper_color2 = cv::Scalar(125, 255, 255); // 保持高亮度上限
    } else if (color_mode == "red") {
        // 红色装甲板的HSV范围（需要两个范围）- 优化参数
        lower_color1 = cv::Scalar(0, 50, 50);    // 提高饱和度和亮度下限
        upper_color1 = cv::Scalar(10, 255, 255);
        lower_color2 = cv::Scalar(160, 50, 50);  // 调整色调范围
        upper_color2 = cv::Scalar(180, 255, 255);
    } else {
        std::cerr << "Unsupported color mode: " << color_mode << std::endl;
        std::cerr << "Supported color modes: blue, red" << std::endl;
    }
}

std::vector<ArmorDetection> TraditionalArmorDetector::detectArmors(const cv::Mat& image) {
    std::vector<ArmorDetection> detections;

    if (image.empty()) {
        std::cerr << "Input image is empty" << std::endl;
        return detections;
    }

    // 记录开始时间
    auto start_time = std::chrono::high_resolution_clock::now();

    // 预处理
    cv::Mat hsv = preprocessImage(image);

    // 1. 检测灯条（使用改进的方法）
    cv::Mat light_mask = detectLightBars(image);
    std::vector<cv::RotatedRect> light_bars = findLightBarContours(light_mask);

    std::cout << "Detected " << light_bars.size() << " light bars" << std::endl;

    // 2. 基于灯条匹配检测装甲板
    std::vector<ArmorDetection> light_bar_armors = detectArmorsFromLightBars(light_bars, image.size());
    
    // 3. 颜色分割
    cv::Mat color_mask = colorSegmentation(hsv);

    // 4. 查找颜色轮廓
    std::vector<std::vector<cv::Point>> contours = findContours(color_mask);

    // 5. 筛选装甲板候选
    std::vector<ArmorDetection> armor_candidates = filterArmorContours(contours, image.size());

    // 6. 使用灯条验证装甲板
    for (auto& candidate : armor_candidates) {
        if (validateArmorWithLightBars(candidate.bbox, light_bars)) {
            candidate.confidence = 0.9; // 有灯条验证的装甲板置信度更高
            detections.push_back(candidate);
        }
    }

    // 7. 合并基于灯条匹配的检测结果
    for (const auto& armor : light_bar_armors) {
        // 检查是否与现有检测结果重叠
        bool is_duplicate = false;
        for (const auto& existing : detections) {
            cv::Rect intersection = existing.bbox & armor.bbox;
            if (intersection.area() > existing.bbox.area() * 0.5) {
                is_duplicate = true;
                break;
            }
        }
        
        if (!is_duplicate) {
            detections.push_back(armor);
        }
    }

    // 8. 应用非极大值抑制，去除重叠的检测结果
    detections = applyNonMaxSuppression(detections);

    // 9. 如果没有检测到装甲板，使用传统的颜色分割结果
    if (detections.empty() && !armor_candidates.empty()) {
        std::cout << "No armor plates with light bars detected, using traditional method" << std::endl;
        detections = armor_candidates;
        for (auto& detection : detections) {
            detection.confidence = 0.7; // 传统方法置信度稍低
        }
    }

    // 记录结束时间
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    detection_time = duration.count();

    std::cout << "Detection completed in " << detection_time << "ms" << std::endl;
    std::cout << "Found " << detections.size() << " armor plates" << std::endl;

    return detections;
}

cv::Mat TraditionalArmorDetector::preprocessImage(const cv::Mat& image) {
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    return hsv;
}

cv::Mat TraditionalArmorDetector::detectLightBars(const cv::Mat& image) {
    // 转换为灰度图
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // 自适应直方图均衡化，比全局均衡化更适合装甲板检测
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(3.0);
    clahe->setTilesGridSize(cv::Size(8, 8));
    clahe->apply(gray, gray);

    // 自适应阈值分割 + 大津法组合，提高在不同光照下的性能
    cv::Mat adaptive_binary, otsu_binary;
    cv::adaptiveThreshold(gray, adaptive_binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                         cv::THRESH_BINARY, 11, 2);
    cv::threshold(gray, otsu_binary, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

    // 融合两种二值化结果
    cv::Mat binary;
    cv::bitwise_and(adaptive_binary, otsu_binary, binary);

    // 改进的多尺度形态学操作增强灯条
    std::vector<cv::Mat> kernels;
    kernels.push_back(cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 10)));
    kernels.push_back(cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 20)));
    kernels.push_back(cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 15)));

    cv::Mat light_mask = cv::Mat::zeros(binary.size(), CV_8UC1);
    for (const auto& kernel : kernels) {
        cv::Mat morph;
        cv::morphologyEx(binary, morph, cv::MORPH_CLOSE, kernel);
        cv::bitwise_or(light_mask, morph, light_mask);
    }

    // 椭圆滤波器 - 基于最新研究的椭圆拟合技术
    cv::Mat kernel_ellipse = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 15));
    cv::Mat ellipse_morph;
    cv::morphologyEx(binary, ellipse_morph, cv::MORPH_CLOSE, kernel_ellipse);
    cv::bitwise_or(light_mask, ellipse_morph, light_mask);

    // 去除小噪声
    cv::Mat kernel_noise = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(light_mask, light_mask, cv::MORPH_OPEN, kernel_noise);

    return light_mask;
}

cv::Mat TraditionalArmorDetector::colorSegmentation(const cv::Mat& hsv) {
    cv::Mat color_mask;

    if (color_mode == "blue") {
        // 双颜色范围合并，提高在不同光照条件下的鲁棒性
        cv::Mat mask1, mask2;
        cv::inRange(hsv, lower_color1, upper_color1, mask1);
        cv::inRange(hsv, lower_color2, upper_color2, mask2);
        cv::add(mask1, mask2, color_mask);

        // 添加亮度过滤，避免过曝区域被误识别
        cv::Mat brightness_mask;
        cv::inRange(hsv, cv::Scalar(0, 0, 30), cv::Scalar(180, 255, 220), brightness_mask);
        cv::bitwise_and(color_mask, brightness_mask, color_mask);
    } else if (color_mode == "red") {
        cv::Mat mask1, mask2;
        cv::inRange(hsv, lower_color1, upper_color1, mask1);
        cv::inRange(hsv, lower_color2, upper_color2, mask2);
        cv::add(mask1, mask2, color_mask);

        // 红色通道亮度过滤
        cv::Mat brightness_mask;
        cv::inRange(hsv, cv::Scalar(0, 0, 40), cv::Scalar(180, 255, 240), brightness_mask);
        cv::bitwise_and(color_mask, brightness_mask, color_mask);
    }

    // 改进的多尺度形态学操作
    cv::Mat kernel_small = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::Mat kernel_medium = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::Mat kernel_large = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));

    // 开运算去除小噪声
    cv::morphologyEx(color_mask, color_mask, cv::MORPH_OPEN, kernel_small);

    // 闭运算填充小孔洞
    cv::morphologyEx(color_mask, color_mask, cv::MORPH_CLOSE, kernel_medium);

    // 再次使用大结构元素增强连通性
    cv::morphologyEx(color_mask, color_mask, cv::MORPH_CLOSE, kernel_large);

    return color_mask;
}

std::vector<cv::RotatedRect> TraditionalArmorDetector::findLightBarContours(const cv::Mat& light_mask) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(light_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::RotatedRect> light_bars;

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        // 放宽面积范围，检测更多可能的灯条
        if (area < 10 || area > 5000) continue;

        cv::RotatedRect rect = cv::minAreaRect(contour);
        float width = std::min(rect.size.width, rect.size.height);
        float height = std::max(rect.size.width, rect.size.height);
        float aspect_ratio = height / width;

        // 保持之前的宽高比条件，确保能检测到足够的灯条
        if (aspect_ratio >= 1.2 && aspect_ratio <= 15.0 && height >= 8 && height <= 150) {
            // 添加基本的形状检查 - 轮廓不能太分散
            double perimeter = cv::arcLength(contour, true);
            if (perimeter > 0) {
                double compactness = 4.0 * CV_PI * area / (perimeter * perimeter);
                // 允许更不规则的形状，但避免完全散乱的噪声
                if (compactness > 0.1) {
                    light_bars.push_back(rect);
                }
            }
        }
    }

    // 按照灯条Y坐标排序，优先选择上方的灯条（真实灯条永远在投影上方）
    std::sort(light_bars.begin(), light_bars.end(),
              [](const cv::RotatedRect& a, const cv::RotatedRect& b) {
                  return a.center.y < b.center.y;  // Y坐标小的在前（上方）
              });

    // 基于真实灯条特征进行筛选：
    // 1. 真实灯条应该在相对较高的位置（Y坐标较小）
    // 2. 真实灯条应该成对出现，左右分布
    // 3. 真实灯条对应该有合理的距离和相似的高度

    if (light_bars.size() > 8) {
        // 如果检测到太多灯条，只保留前8个最上方的候选
        light_bars.resize(8);
    }

    // 进一步筛选：优先保留上方的灯条对
    std::vector<cv::RotatedRect> filtered_bars;

    // 更严格的策略：只选择最上方的真实灯条对
    // 真实装甲板灯条应该在最上方，投影都在下方

    std::vector<cv::RotatedRect> top_candidates;

    // 选择最上方的候选灯条，包括一些可能的真实灯条
    for (size_t i = 0; i < std::min(size_t(4), light_bars.size()); ++i) {
        top_candidates.push_back(light_bars[i]);
    }

    // 在上方候选中寻找最佳灯条对
    if (top_candidates.size() >= 2) {
        cv::RotatedRect best_pair[2] = {top_candidates[0], top_candidates[0]};
        float best_score = -1.0f;

        for (size_t i = 0; i < top_candidates.size(); ++i) {
            for (size_t j = i + 1; j < top_candidates.size(); ++j) {
                const auto& bar1 = top_candidates[i];
                const auto& bar2 = top_candidates[j];

                float height_diff = std::abs(bar1.center.y - bar2.center.y);
                float horizontal_distance = std::abs(bar1.center.x - bar2.center.x);
                float avg_length = (std::max(bar1.size.width, bar1.size.height) +
                                   std::max(bar2.size.width, bar2.size.height)) / 2.0f;

                // 真实灯条对的评分标准：
                // 1. 高度差越小越好
                // 2. 水平距离适中
                // 3. 长度相似
                float height_score = 1.0f / (1.0f + height_diff / avg_length);
                float distance_score = 1.0f;
                if (horizontal_distance < avg_length * 2.0f || horizontal_distance > avg_length * 6.0f) {
                    distance_score = 0.1f; // 距离不合适，严重扣分
                }

                float len1 = std::max(bar1.size.width, bar1.size.height);
                float len2 = std::max(bar2.size.width, bar2.size.height);
                float length_ratio = std::min(len1, len2) / std::max(len1, len2);
                float length_score = length_ratio;

                float total_score = height_score * distance_score * length_score;

                if (total_score > best_score) {
                    best_score = total_score;
                    best_pair[0] = bar1;
                    best_pair[1] = bar2;
                }
            }
        }

        if (best_score > 0.05f) { // 降低阈值，更容易找到灯条对
            filtered_bars.push_back(best_pair[0]);
            filtered_bars.push_back(best_pair[1]);
        }
    }

    // 如果没有找到，只使用最上方的单个灯条
    if (filtered_bars.empty() && !light_bars.empty()) {
        filtered_bars.push_back(light_bars[0]);
        if (light_bars.size() > 1) {
            // 寻找与第一个灯条水平距离合理的第二个灯条
            for (size_t i = 1; i < std::min(size_t(4), light_bars.size()); ++i) {
                float horizontal_distance = std::abs(light_bars[0].center.x - light_bars[i].center.x);
                float avg_length = std::max(light_bars[0].size.width, light_bars[0].size.height);

                if (horizontal_distance > avg_length * 1.5f &&
                    horizontal_distance < avg_length * 5.0f) {
                    filtered_bars.push_back(light_bars[i]);
                    break;
                }
            }
        }
    }

    return filtered_bars;
}

bool TraditionalArmorDetector::validateArmorWithLightBars(const cv::Rect& armor_bbox,
                                                        const std::vector<cv::RotatedRect>& light_bars) {
    if (light_bars.size() < 2) return false;

    cv::Point armor_center(armor_bbox.x + armor_bbox.width/2, armor_bbox.y + armor_bbox.height/2);

    // 放宽灯条位置验证条件
    std::vector<cv::RotatedRect> left_lights, right_lights;

    for (const auto& light : light_bars) {
        cv::Point light_center = light.center;

        // 灯条应该在装甲板的左右两侧 - 放宽条件
        if (light_center.y >= armor_bbox.y - 10 && light_center.y <= armor_bbox.y + armor_bbox.height + 10) {
            if (light_center.x < armor_center.x - armor_bbox.width/6) {  // 左侧 - 减小距离要求
                left_lights.push_back(light);
            } else if (light_center.x > armor_center.x + armor_bbox.width/6) {  // 右侧 - 减小距离要求
                right_lights.push_back(light);
            }
        }
    }

    // 如果两侧都有灯条，或者只有一侧但有多个灯条，可能是装甲板
    bool has_lights = (!left_lights.empty() && !right_lights.empty()) || 
                     (left_lights.size() >= 2) || (right_lights.size() >= 2);
    
    if (!has_lights) return false;
    
    // 检查装甲板中间是否有数字区域（较暗的区域）
    // 这是基于"两个灯条中间有数字"这一关键特征
    return hasDigitRegion(armor_bbox);
}

bool TraditionalArmorDetector::hasDigitRegion(const cv::Rect& armor_bbox) {
    // 这个方法需要访问原始图像，所以我们暂时返回true
    // 在实际应用中，可以检查装甲板中间区域是否有较暗的数字区域
    // 数字区域通常比周围的装甲板区域更暗
    
    // 简单验证：装甲板应该有一定的宽度和高度
    return armor_bbox.width > 40 && armor_bbox.height > 20;
}

std::vector<std::vector<cv::Point>> TraditionalArmorDetector::findContours(const cv::Mat& mask) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    return contours;
}

std::vector<ArmorDetection> TraditionalArmorDetector::filterArmorContours(
    const std::vector<std::vector<cv::Point>>& contours,
    const cv::Size& image_size) {

    std::vector<ArmorDetection> armor_candidates;

    // 大幅放宽参数设置，提高检测率
    const double min_area = 100.0;  // 降低最小面积
    const double max_area = (image_size.width * image_size.height) / 3.0;  // 增大最大面积
    const double min_aspect_ratio = 0.8;  // 降低最小宽高比，允许更接近正方形
    const double max_aspect_ratio = 10.0;  // 增大最大宽高比
    const double min_rectity = 0.2;        // 大幅降低最小矩形度

    // 计算所有轮廓的平均面积，用于动态调整参数
    double total_area = 0.0;
    int valid_contours = 0;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > 20) {
            total_area += area;
            valid_contours++;
        }
    }
    double avg_area = valid_contours > 0 ? total_area / valid_contours : 0.0;

    for (size_t i = 0; i < contours.size(); ++i) {
        // 计算轮廓面积
        double area = cv::contourArea(contours[i]);

        if (area < min_area || area > max_area) {
            continue;
        }

        // 获取边界框
        cv::Rect bbox = cv::boundingRect(contours[i]);
        double aspect_ratio = static_cast<double>(bbox.width) / bbox.height;

        // 筛选宽高比 - 大幅放宽条件
        if (aspect_ratio < min_aspect_ratio || aspect_ratio > max_aspect_ratio) {
            continue;
        }

        // 计算矩形度
        double rect_area = bbox.width * bbox.height;
        double rectity = area / rect_area;

        if (rectity < min_rectity) {
            continue;
        }

        // 简化紧凑度检查 - 放宽要求
        double perimeter = cv::arcLength(contours[i], true);
        double compactness = 4.0 * CV_PI * area / (perimeter * perimeter);
        if (compactness < 0.1) { // 从0.3降低到0.1
            continue;
        }

        // 更宽松的置信度计算
        double confidence = 0.5; // 基础置信度
        if (avg_area > 0) {
            double area_ratio = area / avg_area;
            if (area_ratio >= 0.2 && area_ratio <= 5.0) { // 大幅放宽面积比范围
                confidence += 0.3;
            }
        }

        // 形状质量评分 - 降低要求
        if (rectity > 0.4) confidence += 0.1; // 从0.7降低到0.4
        if (compactness > 0.2) confidence += 0.1; // 从0.5降低到0.2

        // 额外的面积奖励
        if (area > 500 && area < image_size.width * image_size.height / 10) {
            confidence += 0.1;
        }

        // 创建装甲板检测结果
        ArmorDetection detection;
        detection.bbox = refineBoundingBox(bbox, contours[i], image_size);
        detection.area = detection.bbox.width * detection.bbox.height;
        detection.aspect_ratio = static_cast<double>(detection.bbox.width) / detection.bbox.height;
        detection.rectity = rectity;
        detection.confidence = std::min(confidence, 1.0);

        armor_candidates.push_back(detection);
    }

    return armor_candidates;
}

std::vector<ArmorDetection> TraditionalArmorDetector::detectArmorsFromLightBars(
    const std::vector<cv::RotatedRect>& light_bars, const cv::Size& image_size) {

    std::vector<ArmorDetection> detections;

    if (light_bars.size() < 2) return detections;

    // 对所有灯条对进行匹配
    for (size_t i = 0; i < light_bars.size(); ++i) {
        for (size_t j = i + 1; j < light_bars.size(); ++j) {
            const auto& bar1 = light_bars[i];
            const auto& bar2 = light_bars[j];

            // 检查是否为有效的灯条对
            if (isValidLightBarPair(bar1, bar2)) {
                // 计算装甲板边界框（包含旋转矩形）
                cv::Rect armor_bbox = calculateArmorBoundingBox(bar1, bar2);

                // 验证装甲板区域
                if (isValidArmorRegion(armor_bbox, image_size)) {
                    ArmorDetection detection;
                    detection.bbox = armor_bbox;

                    // 计算并设置旋转矩形
                    cv::Point2f center1 = bar1.center;
                    cv::Point2f center2 = bar2.center;
                    cv::Point2f armor_center((center1.x + center2.x) / 2.0f, (center1.y + center2.y) / 2.0f);

                    float len1 = std::max(bar1.size.width, bar1.size.height);
                    float len2 = std::max(bar2.size.width, bar2.size.height);
                    float avg_length = (len1 + len2) / 2.0f;

                    float dx = center2.x - center1.x;
                    float dy = center2.y - center1.y;
                    float armor_angle = std::atan2(dy, dx) * 180.0f / CV_PI;

                    float armor_width = std::sqrt(dx * dx + dy * dy) + avg_length * 0.3f;
                    float armor_height = avg_length * 1.8f;

                    detection.rotated_bbox = cv::RotatedRect(armor_center, cv::Size2f(armor_width, armor_height), armor_angle);

                    detection.area = armor_bbox.width * armor_bbox.height;
                    detection.aspect_ratio = armor_width / armor_height; // 使用旋转矩形的宽高比
                    detection.rectity = 0.8; // 基于灯条匹配的矩形度较高
                    detection.confidence = 0.85; // 基于灯条匹配的置信度

                    detections.push_back(detection);
                }
            }
        }
    }

    return detections;
}

bool TraditionalArmorDetector::isValidLightBarPair(const cv::RotatedRect& bar1, const cv::RotatedRect& bar2) {
    // 计算两个灯条的中心点
    cv::Point2f center1 = bar1.center;
    cv::Point2f center2 = bar2.center;

    // 计算距离 - 放宽范围，适应不同尺寸的装甲板
    float distance = cv::norm(center1 - center2);
    if (distance < 30.0f || distance > 400.0f) {
        return false;
    }

    // 计算两个灯条的角度
    float angle1 = bar1.angle;
    float angle2 = bar2.angle;

    // 调整角度范围到[-90, 90]
    if (angle1 > 90) angle1 -= 180;
    if (angle2 > 90) angle2 -= 180;

    // 两个灯条应该大致平行 - 放宽角度差要求
    float angle_diff = std::abs(angle1 - angle2);
    if (angle_diff > 45.0f) {
        return false;
    }

    // 计算两个灯条的长度
    float len1 = std::max(bar1.size.width, bar1.size.height);
    float len2 = std::max(bar2.size.width, bar2.size.height);

    // 长度应该相近 - 放宽长度比要求
    float length_ratio = std::min(len1, len2) / std::max(len1, len2);
    if (length_ratio < 0.3f) {
        return false;
    }

    // 添加水平对齐检查 - 放宽要求
    float height_diff = std::abs(center1.y - center2.y);
    float avg_length = (len1 + len2) / 2.0f;
    if (height_diff > avg_length * 1.0f) {
        return false;
    }

    // 检查灯条垂直性
    float width1 = std::min(bar1.size.width, bar1.size.height);
    float width2 = std::min(bar2.size.width, bar2.size.height);

    bool bar1_vertical = (len1 / width1) > 2.0f;
    bool bar2_vertical = (len2 / width2) > 2.0f;

    if (!bar1_vertical || !bar2_vertical) {
        return false;
    }

    // 添加面积一致性检查 - 放宽要求
    float area1 = bar1.size.width * bar1.size.height;
    float area2 = bar2.size.width * bar2.size.height;
    float area_ratio = std::min(area1, area2) / std::max(area1, area2);
    if (area_ratio < 0.2f) {
        return false;
    }

    return true;
}

cv::Rect TraditionalArmorDetector::calculateArmorBoundingBox(const cv::RotatedRect& bar1, const cv::RotatedRect& bar2) {
    // 使用旋转矩形计算装甲板边界框，更贴合装甲板实际方向

    // 计算两个灯条的中心点和长度
    cv::Point2f center1 = bar1.center;
    cv::Point2f center2 = bar2.center;

    float len1 = std::max(bar1.size.width, bar1.size.height);
    float len2 = std::max(bar2.size.width, bar2.size.height);
    float avg_length = (len1 + len2) / 2.0f;

    // 计算装甲板的中心点
    cv::Point2f armor_center((center1.x + center2.x) / 2.0f, (center1.y + center2.y) / 2.0f);

    // 计算装甲板的角度（基于两个灯条连线的角度）
    float dx = center2.x - center1.x;
    float dy = center2.y - center1.y;
    float armor_angle = std::atan2(dy, dx) * 180.0f / CV_PI;

    // 计算装甲板的宽度和高度
    float armor_width = std::sqrt(dx * dx + dy * dy) + avg_length * 0.3f;  // 稍微扩展宽度
    float armor_height = avg_length * 1.8f;  // 向上扩展到数字区域，向下最小扩展

    // 创建旋转矩形
    cv::RotatedRect rotated_armor(armor_center, cv::Size2f(armor_width, armor_height), armor_angle);

    // 将旋转矩形转换为轴对齐矩形（用于显示）
    cv::Point2f vertices[4];
    rotated_armor.points(vertices);

    // 找到旋转矩形的边界
    float min_x = vertices[0].x, max_x = vertices[0].x;
    float min_y = vertices[0].y, max_y = vertices[0].y;

    for (int i = 1; i < 4; ++i) {
        min_x = std::min(min_x, vertices[i].x);
        max_x = std::max(max_x, vertices[i].x);
        min_y = std::min(min_y, vertices[i].y);
        max_y = std::max(max_y, vertices[i].y);
    }

    // 确保最小尺寸
    armor_width = std::max(armor_width, avg_length * 1.5f);
    armor_height = std::max(armor_height, avg_length * 1.5f);

    // 边界框安全检查
    min_x = std::max(0.0f, min_x);
    min_y = std::max(0.0f, min_y);

    // 重新创建旋转矩形以确保尺寸正确
    cv::RotatedRect final_rotated_armor(armor_center, cv::Size2f(armor_width, armor_height), armor_angle);

    // 存储旋转矩形信息供绘图使用
    // 注意：这里需要在调用处设置rotated_bbox字段

    return cv::Rect(static_cast<int>(min_x), static_cast<int>(min_y),
                   static_cast<int>(max_x - min_x), static_cast<int>(max_y - min_y));
}

bool TraditionalArmorDetector::isValidArmorRegion(const cv::Rect& bbox, const cv::Size& image_size) {
    // 检查边界框是否在图像范围内
    if (bbox.x < 0 || bbox.y < 0 || 
        bbox.x + bbox.width > image_size.width || 
        bbox.y + bbox.height > image_size.height) {
        return false;
    }
    
    // 检查宽高比是否合理
    float aspect_ratio = static_cast<float>(bbox.width) / bbox.height;
    if (aspect_ratio < 0.8f || aspect_ratio > 5.0f) {
        return false;
    }
    
    // 检查面积是否合理
    int area = bbox.width * bbox.height;
    if (area < 500 || area > image_size.width * image_size.height / 10) {
        return false;
    }
    
    return true;
}

std::vector<ArmorDetection> TraditionalArmorDetector::applyNonMaxSuppression(
    const std::vector<ArmorDetection>& detections) {

    if (detections.empty()) return detections;

    // 按置信度降序排序
    std::vector<ArmorDetection> sorted_detections = detections;
    std::sort(sorted_detections.begin(), sorted_detections.end(),
              [](const ArmorDetection& a, const ArmorDetection& b) {
                  return a.confidence > b.confidence;
              });

    std::vector<ArmorDetection> nms_detections;
    std::vector<bool> suppressed(sorted_detections.size(), false);

    for (size_t i = 0; i < sorted_detections.size(); ++i) {
        if (suppressed[i]) continue;

        nms_detections.push_back(sorted_detections[i]);

        // 更激进的NMS策略 - 收紧IoU阈值
        for (size_t j = i + 1; j < sorted_detections.size(); ++j) {
            if (suppressed[j]) continue;

            // 计算IoU（交并比）
            cv::Rect intersection = sorted_detections[i].bbox & sorted_detections[j].bbox;
            float intersection_area = intersection.area();
            float union_area = sorted_detections[i].bbox.area() +
                              sorted_detections[j].bbox.area() - intersection_area;

            float iou = intersection_area / union_area;

            // 添加多级抑制策略
            float iou_threshold = 0.1f; // 从0.3收紧到0.1

            // 如果置信度差异很大，使用更严格的阈值
            float confidence_diff = sorted_detections[i].confidence - sorted_detections[j].confidence;
            if (confidence_diff > 0.2f) {
                iou_threshold = 0.05f; // 更严格的阈值
            }

            // 添加面积比例检查
            float area_ratio_i = static_cast<float>(sorted_detections[i].area) /
                                (sorted_detections[i].area + sorted_detections[j].area);
            float area_ratio_j = static_cast<float>(sorted_detections[j].area) /
                                (sorted_detections[i].area + sorted_detections[j].area);

            // 如果一个框明显比另一个小，且重叠度高，抑制小框
            if (area_ratio_j < 0.3f && iou > 0.15f) {
                suppressed[j] = true;
            }
            // 标准IoU抑制
            else if (iou > iou_threshold) {
                suppressed[j] = true;
            }
            // 中心距离检查 - 如果中心点很接近且重叠度高
            else {
                cv::Point center_i(sorted_detections[i].bbox.x + sorted_detections[i].bbox.width/2,
                                  sorted_detections[i].bbox.y + sorted_detections[i].bbox.height/2);
                cv::Point center_j(sorted_detections[j].bbox.x + sorted_detections[j].bbox.width/2,
                                  sorted_detections[j].bbox.y + sorted_detections[j].bbox.height/2);
                float center_distance = cv::norm(center_i - center_j);
                float avg_diagonal = (std::sqrt(sorted_detections[i].bbox.width*sorted_detections[i].bbox.width +
                                              sorted_detections[i].bbox.height*sorted_detections[i].bbox.height) +
                                    std::sqrt(sorted_detections[j].bbox.width*sorted_detections[j].bbox.width +
                                              sorted_detections[j].bbox.height*sorted_detections[j].bbox.height)) / 2.0f;

                if (center_distance < avg_diagonal * 0.3f && iou > 0.05f) {
                    suppressed[j] = true;
                }
            }
        }
    }

    return nms_detections;
}

cv::Rect TraditionalArmorDetector::refineBoundingBox(const cv::Rect& bbox, const std::vector<cv::Point>& contour, const cv::Size& image_size) {
    if (contour.empty()) {
        return bbox;
    }

    // 计算轮廓的质心
    cv::Moments moments = cv::moments(contour);
    if (moments.m00 == 0) {
        return bbox;
    }
    cv::Point2f centroid(moments.m10 / moments.m00, moments.m01 / moments.m00);

    // 基于轮廓的实际分布调整边界框
    std::vector<float> x_coords, y_coords;
    for (const auto& point : contour) {
        x_coords.push_back(point.x);
        y_coords.push_back(point.y);
    }

    auto [min_x_it, max_x_it] = std::minmax_element(x_coords.begin(), x_coords.end());
    auto [min_y_it, max_y_it] = std::minmax_element(y_coords.begin(), y_coords.end());

    int min_x = static_cast<int>(*min_x_it);
    int max_x = static_cast<int>(*max_x_it);
    int min_y = static_cast<int>(*min_y_it);
    int max_y = static_cast<int>(*max_y_it);

    // 计算轮廓的实际宽度和高度
    int contour_width = max_x - min_x;
    int contour_height = max_y - min_y;

    // 基于轮廓几何特征调整边界框大小
    float aspect_ratio = static_cast<float>(contour_width) / contour_height;

    // 如果轮廓过于细长，可能是噪声，使用更保守的边界框
    if (aspect_ratio > 8.0f || aspect_ratio < 0.125f) {
        return bbox;
    }

    // 添加小的边距，但不超过图像边界
    int margin_x = std::max(2, static_cast<int>(contour_width * 0.05));
    int margin_y = std::max(2, static_cast<int>(contour_height * 0.05));

    int refined_x = std::max(0, min_x - margin_x);
    int refined_y = std::max(0, min_y - margin_y);
    int refined_width = std::min(image_size.width - refined_x,
                                contour_width + 2 * margin_x);
    int refined_height = std::min(image_size.height - refined_y,
                                 contour_height + 2 * margin_y);

    // 确保边界框有合理的最小尺寸
    refined_width = std::max(refined_width, 20);
    refined_height = std::max(refined_height, 15);

    // 确保宽高比在合理范围内
    float refined_aspect_ratio = static_cast<float>(refined_width) / refined_height;
    if (refined_aspect_ratio > 8.0f) {
        refined_width = static_cast<int>(refined_height * 8.0f);
    } else if (refined_aspect_ratio < 0.125f) {
        refined_height = static_cast<int>(refined_width * 8.0f);
    }

    return cv::Rect(refined_x, refined_y, refined_width, refined_height);
}

cv::Mat TraditionalArmorDetector::drawResults(const cv::Mat& image,
                                           const std::vector<ArmorDetection>& detections) {
    cv::Mat result_image = image.clone();

    for (size_t i = 0; i < detections.size(); ++i) {
        const auto& detection = detections[i];
        const cv::Rect& bbox = detection.bbox;

        // 绘制旋转矩形（主要的检测结果）
        cv::Point2f vertices[4];
        detection.rotated_bbox.points(vertices);

        // 绘制旋转矩形的四条边
        for (int j = 0; j < 4; ++j) {
            cv::line(result_image, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 255, 0), 3);
        }

        // 绘制轴对齐边界框（虚线，用于对比）
        // cv::rectangle(result_image, bbox, cv::Scalar(255, 255, 0), 1);

        // 添加标签（使用旋转矩形的中心）
        cv::Point2f center = detection.rotated_bbox.center;
        std::string label = "Armor " + std::to_string(i + 1) + " (" +
                           std::to_string(detection.confidence).substr(0, 3) + ")";
        cv::putText(result_image, label, cv::Point(center.x - 50, center.y - detection.rotated_bbox.size.height/2 - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

        // 显示详细信息
        std::string info = "A:" + std::to_string(static_cast<int>(detection.area)) +
                          " R:" + std::to_string(detection.aspect_ratio).substr(0, 3) +
                          " Ang:" + std::to_string(static_cast<int>(detection.rotated_bbox.angle));
        cv::putText(result_image, info, cv::Point(center.x - 50, center.y + detection.rotated_bbox.size.height/2 + 20),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);
    }

    // 添加检测信息
    std::string info_text1 = "RotatedRect Detection | Armors: " + std::to_string(detections.size()) +
                           " | Time: " + std::to_string(detection_time) + "ms";
    std::string info_text2 = "Color mode: " + color_mode +
                           " | Image size: " + std::to_string(image.cols) + "x" + std::to_string(image.rows);

    cv::putText(result_image, info_text1, cv::Point(10, 30),
               cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
    cv::putText(result_image, info_text2, cv::Point(10, 60),
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

    return result_image;
}

bool TraditionalArmorDetector::saveResults(const cv::Mat& result_image,
                                         const std::string& output_path) {
    try {
        cv::imwrite(output_path, result_image);
        std::cout << "Results saved: " << output_path << std::endl;
        return true;
    } catch (const cv::Exception& e) {
        std::cerr << "Failed to save results: " << e.what() << std::endl;
        return false;
    }
}

void TraditionalArmorDetector::printDetectionDetails(const std::vector<ArmorDetection>& detections) {
    for (size_t i = 0; i < detections.size(); ++i) {
        const auto& detection = detections[i];
        const cv::Rect& bbox = detection.bbox;

        std::cout << "  " << (i + 1) << ". 位置: (" << bbox.x << ", " << bbox.y
                  << ")-(" << (bbox.x + bbox.width) << ", " << (bbox.y + bbox.height) << ")" << std::endl;
        std::cout << "      面积: " << static_cast<int>(detection.area)
                  << ", 宽高比: " << detection.aspect_ratio << std::endl;
    }
}
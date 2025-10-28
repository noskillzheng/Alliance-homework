#include "detector/traditional_detector.hpp"
#include <iostream>
#include <iomanip>
#include <algorithm>

namespace armor_detector {

TraditionalDetector::TraditionalDetector(const std::string& target_color)
    : target_color_(target_color),
      color_method_("channel_separation"),
      binary_threshold_(80),
      brightness_threshold_(150),
      use_brightness_filter_(true),
      use_morphology_(true),
      morph_kernel_size_(3),
      use_otsu_(false),
      otsu_mode_("roi"),
      otsu_roi_scale_(1.8),
      otsu_min_threshold_(40),
      otsu_max_threshold_(150),
      otsu_temporal_smooth_(0.7),
      last_otsu_threshold_(80),
      has_prediction_roi_(false),
      min_light_area_(50.0f),
      max_light_area_(2000.0f),
      min_light_ratio_(2.0f),
      max_light_ratio_(8.0f),
      max_light_angle_(15.0f),
      min_armor_ratio_(2.0f),
      max_armor_ratio_(4.5f),
      max_angle_diff_(15.0f),
      max_height_diff_ratio_(0.3f),
      max_y_diff_ratio_(0.8f),
      min_area_ratio_(0.3f),
      max_tilt_for_direction_(5.0f),
      min_fill_ratio_(0.25f),
      max_fill_ratio_(0.95f),
      max_fill_area_(800.0f),
      min_compactness_(0.2f),
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
    
    // 调试模式：保存二值化图像
    if (debug_mode_) {
        static bool first_call = true;
        if (first_call) {
            std::cout << "[调试] debug_mode_=" << debug_mode_ 
                      << ", binary.empty()=" << binary.empty() 
                      << ", binary.size=" << binary.cols << "x" << binary.rows << std::endl;
            first_call = false;
        }
        
        // 将二值化图像转为彩色以便标注
        if (!binary.empty()) {
            cv::cvtColor(binary, debug_image_, cv::COLOR_GRAY2BGR);
            
            // 在二值化图像上绘制灯条框（绿色）
            for (const auto& bar : light_bars) {
                cv::Point2f vertices[4];
                bar.rect.points(vertices);
                for (int i = 0; i < 4; i++) {
                    cv::line(debug_image_, vertices[i], vertices[(i+1)%4], cv::Scalar(0, 255, 0), 2);
                }
                // 显示灯条参数
                std::string info = cv::format("A:%.0f R:%.1f", bar.area, bar.aspectRatio());
                cv::putText(debug_image_, info, bar.center, 
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
            }
            
            // 添加标题和统计信息
            std::string title = cv::format("Binary Image - %d Light Bars", (int)light_bars.size());
            cv::putText(debug_image_, title, cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
            
            static bool first_debug = true;
            if (first_debug) {
                std::cout << "[调试] Debug窗口已创建，显示二值化图像" << std::endl;
                first_debug = false;
            }
        } else {
            std::cout << "[警告] 二值化图像为空！" << std::endl;
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
    static bool first_call = true;
    if (first_call && debug_mode_) {
        std::cout << "[调试] colorSegmentation: frame.size=" << frame.cols << "x" << frame.rows 
                  << ", use_brightness_filter_=" << use_brightness_filter_ 
                  << ", binary_threshold_=" << binary_threshold_
                  << ", brightness_threshold_=" << brightness_threshold_ << std::endl;
        first_call = false;
    }
    
    cv::Mat result;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, 
                                               cv::Size(morph_kernel_size_, morph_kernel_size_));
    
    if (color_method_ == "channel_separation") {
        // 方法1：通道分离法
        std::vector<cv::Mat> channels;
        cv::split(frame, channels);  // BGR通道分离
        
        // 通道相减
        cv::Mat color_diff;
        if (target_color_ == "red") {
            cv::subtract(channels[2], channels[0], color_diff);
        } else {
            cv::subtract(channels[0], channels[2], color_diff);
        }
        
        // 二值化颜色差异
        int adaptive_threshold = binary_threshold_;
        if (use_otsu_) {
            adaptive_threshold = calculateAdaptiveThreshold(color_diff);
        }
        cv::Mat color_mask;
        cv::threshold(color_diff, color_mask, adaptive_threshold, 255, cv::THRESH_BINARY);
        
        // 可选：亮度辅助过滤（针对高亮LED灯条）
        if (use_brightness_filter_) {
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            cv::Mat brightness_mask;
            cv::threshold(gray, brightness_mask, brightness_threshold_, 255, cv::THRESH_BINARY);
            
            // 组合：颜色正确 OR 亮度高
            cv::bitwise_or(color_mask, brightness_mask, result);
            
            if (debug_mode_) {
                std::cout << "[Debug] 颜色阈值=" << adaptive_threshold 
                          << ", 亮度阈值=" << brightness_threshold_
                          << ", 非零像素=" << cv::countNonZero(result) << std::endl;
            }
        } else {
            // 纯通道相减
            result = color_mask;
            if (debug_mode_) {
                std::cout << "[Debug] 颜色阈值=" << adaptive_threshold 
                          << ", 非零像素=" << cv::countNonZero(result) << std::endl;
            }
        }
        
        // 形态学处理：去噪和填充（可配置）
        if (use_morphology_ && morph_kernel_size_ > 0) {
            cv::morphologyEx(result, result, cv::MORPH_CLOSE, kernel);
            
            if (debug_mode_) {
                static bool check_morph = true;
                if (check_morph) {
                    std::cout << "[调试] 形态学: 核大小=" << morph_kernel_size_ 
                              << "x" << morph_kernel_size_ << std::endl;
                    check_morph = false;
                }
            }
        }
        
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
    
    // 统计信息
    int total_contours = static_cast<int>(contours.size());
    int rejected_area = 0;
    int rejected_other = 0;
    
    // 遍历轮廓，筛选灯条
    for (const auto& contour : contours) {
        // 面积过滤
        float area = cv::contourArea(contour);
        if (area < min_light_area_) {
            rejected_area++;
            continue;
        }
        
        // 拟合旋转矩形
        cv::RotatedRect rect = cv::minAreaRect(contour);
        LightBar bar(rect);
        
        // 灯条有效性判断（传入二值图和轮廓用于占比检查）
        if (isValidLightBar(bar, binary, contour)) {
            light_bars.push_back(bar);
        } else {
            rejected_other++;
        }
    }
    
    // 输出第一层检查统计
    std::cout << "[TraditionalDetector] 第一层检查：总轮廓" << total_contours << "个 → ";
    std::cout << "通过" << light_bars.size() << "个灯条";
    std::cout << " (拒绝" << rejected_area << "个面积过小, " << rejected_other << "个其他不合格)" << std::endl;
    
    // 输出每个通过的灯条详细信息
    if (!light_bars.empty()) {
        for (size_t i = 0; i < light_bars.size(); i++) {
            const auto& bar = light_bars[i];
            
            // 计算角度偏差（与竖直方向）
            float angle = bar.rect.angle;
            float vertical_deviation;
            if (bar.rect.size.width < bar.rect.size.height) {
                vertical_deviation = std::abs(angle + 90);
            } else {
                vertical_deviation = 90.0f;
            }
            
            std::cout << "  ├─ 灯条#" << (i + 1) << ": ";
            std::cout << "面积=" << static_cast<int>(bar.area) << ", ";
            std::cout << "长宽比=" << std::fixed << std::setprecision(1) << bar.aspectRatio() << ", ";
            std::cout << "角度偏差=" << static_cast<int>(vertical_deviation) << "°  ✓" << std::endl;
        }
    }
    
    return light_bars;
}

bool TraditionalDetector::isValidLightBar(const LightBar& bar, const cv::Mat& binary, 
                                          const std::vector<cv::Point>& contour) {
    // 注：contour 参数保留用于未来可能的精确填充率计算（如轮廓内像素统计）
    (void)contour;  // 暂时未使用，消除警告
    
    // 1. 长宽比检查（灯条应该是细长的）
    float ratio = bar.aspectRatio();
    if (ratio < min_light_ratio_ || ratio > max_light_ratio_) {
        if (debug_mode_) {
            std::cout << "  [灯条拒绝] 长宽比=" << ratio 
                      << " (要求" << min_light_ratio_ << "-" << max_light_ratio_ << ")" << std::endl;
        }
        return false;
    }
    
    // 2. 面积检查（根据实际装甲板大小调整）
    if (bar.area < min_light_area_) {
        if (debug_mode_) {
            std::cout << "  [灯条拒绝] 面积太小=" << bar.area 
                      << " (下限" << min_light_area_ << ")" << std::endl;
        }
        return false;
    }
    if (bar.area > max_light_area_) {
        if (debug_mode_) {
            std::cout << "  [灯条拒绝] 面积太大=" << bar.area 
                      << " (上限" << max_light_area_ << ")" << std::endl;
        }
        return false;
    }
    
    // 3. 角度检查（严格要求接近竖直）
    // OpenCV的RotatedRect角度范围：[-90, 0]
    // 灯条应该是细长的竖直条，需要检查角度偏差
    float angle = bar.rect.angle;
    
    // 计算与竖直方向的偏差角度
    // angle=-90表示竖直，angle=0表示水平
    float vertical_deviation = std::abs(angle + 90);
    
    // 如果偏差>45度，说明更接近水平方向，需要换算
    // 例如angle=-10时，vertical_deviation=80，但实际是接近水平（偏差应该是10度）
    if (vertical_deviation > 45) {
        vertical_deviation = 90 - vertical_deviation;
    }
    
    if (vertical_deviation > max_light_angle_) {
        if (debug_mode_) {
            std::cout << "  [灯条拒绝] 角度偏差=" << vertical_deviation 
                      << "° (上限" << max_light_angle_ << "°) [原始角度=" << angle << "°]" << std::endl;
        }
        return false;
    }
    
    // 4. 【新增】二值化占比过滤（优化2：减少误识别大块白光）
    cv::Rect bbox = bar.rect.boundingRect();
    
    // 确保边界框在图像范围内
    bbox.x = std::max(0, bbox.x);
    bbox.y = std::max(0, bbox.y);
    bbox.width = std::min(bbox.width, binary.cols - bbox.x);
    bbox.height = std::min(bbox.height, binary.rows - bbox.y);
    
    if (bbox.width <= 0 || bbox.height <= 0) {
        return false;  // 无效的边界框
    }
    
    // 提取ROI区域
    cv::Mat roi = binary(bbox);
    
    // 计算白色像素占比（填充率）
    int white_pixels = cv::countNonZero(roi);
    float fill_ratio = static_cast<float>(white_pixels) / (bbox.width * bbox.height);
    
    // 灯条应该是实心的，填充率应该较高
    // 但也不能是整个大块白光（那样填充率会非常高且面积很大）
    if (fill_ratio < min_fill_ratio_) {
        // 填充率太低，可能是稀疏噪点或边缘残留
        return false;
    }
    
    if (fill_ratio > max_fill_ratio_ && bbox.area() > max_fill_area_) {
        // 填充率很高且面积较大，可能是大块反光
        return false;
    }
    
    // 5. 【新增】轮廓面积与外接矩形面积比（紧凑性检查）
    float compactness = bar.area / bbox.area();
    if (compactness < min_compactness_) {
        // 轮廓面积相对外接矩形太小，可能是不规则噪点
        return false;
    }
    
    return true;
}

std::vector<Armor> TraditionalDetector::matchArmor(const std::vector<LightBar>& light_bars) {
    std::vector<Armor> armors;
    
    // 至少需要两个灯条才能配对
    if (light_bars.size() < 2) {
        std::cout << "[TraditionalDetector] 第二层检查：灯条数量不足(" << light_bars.size() 
                  << "<2)，无法配对" << std::endl;
        return armors;
    }
    
    std::cout << "\n[TraditionalDetector] 第二层检查：尝试配对 " << light_bars.size() << " 个灯条..." << std::endl;
    
    // 两两配对
    int pair_count = 0;
    int success_count = 0;
    for (size_t i = 0; i < light_bars.size(); i++) {
        for (size_t j = i + 1; j < light_bars.size(); j++) {
            pair_count++;
            const LightBar& left = light_bars[i].center.x < light_bars[j].center.x ? 
                                   light_bars[i] : light_bars[j];
            const LightBar& right = light_bars[i].center.x < light_bars[j].center.x ? 
                                    light_bars[j] : light_bars[i];
            
            // 找到原始索引
            size_t left_idx = (light_bars[i].center.x < light_bars[j].center.x) ? i : j;
            size_t right_idx = (light_bars[i].center.x < light_bars[j].center.x) ? j : i;
            
            std::cout << "  ├─ 尝试 [灯条#" << (left_idx + 1) << " - 灯条#" << (right_idx + 1) << "]: ";
            
            // 判断是否能配对
            if (canMatch(left, right)) {
                Armor armor = buildArmor(left, right);
                armors.push_back(armor);
                success_count++;
                std::cout << "✓ 匹配成功 (置信度=" << std::fixed << std::setprecision(2) 
                          << armor.confidence << ")" << std::endl;
            }
            // canMatch函数内部已经输出了拒绝原因
        }
    }
    
    std::cout << "\n[TraditionalDetector] 最终结果：尝试配对" << pair_count << "次 → 成功" 
              << success_count << "个装甲板" << std::endl;
    std::cout << std::string(60, '=') << std::endl;  // 分隔线
    
    return armors;
}

bool TraditionalDetector::canMatch(const LightBar& left, const LightBar& right) {
    float avg_height = (left.length + right.length) / 2.0f;
    
    // 1. 角度差检查（两灯条应平行）
    float angle_diff = std::abs(left.angle - right.angle);
    if (angle_diff > max_angle_diff_) {
        std::cout << "✗ 拒绝 (角度差=" << std::fixed << std::setprecision(1) 
                  << angle_diff << "° > " << max_angle_diff_ << "°)" << std::endl;
        return false;
    }
    
    // 2. 高度差检查（透视会导致近大远小，但不应差太多）
    float height_diff = std::abs(left.length - right.length);
    float height_ratio = height_diff / avg_height;
    if (height_ratio > max_height_diff_ratio_) {
        std::cout << "✗ 拒绝 (高度差比=" << std::fixed << std::setprecision(2) 
                  << height_ratio << " > " << max_height_diff_ratio_ << ")" << std::endl;
        return false;
    }
    
    // 3. Y坐标差检查（灯条应水平对齐）
    float y_diff = std::abs(left.center.y - right.center.y);
    float y_ratio = y_diff / avg_height;
    if (y_ratio > max_y_diff_ratio_) {
        std::cout << "✗ 拒绝 (Y坐标差比=" << std::fixed << std::setprecision(2) 
                  << y_ratio << " > " << max_y_diff_ratio_ << ")" << std::endl;
        return false;
    }
    
    // 4. 装甲板宽高比检查（关键！正常装甲板比例）
    float armor_width = std::abs(right.center.x - left.center.x);
    float armor_ratio = armor_width / avg_height;
    if (armor_ratio < min_armor_ratio_) {
        std::cout << "✗ 拒绝 (宽高比=" << std::fixed << std::setprecision(2) 
                  << armor_ratio << " < " << min_armor_ratio_ << ", 太窄)" << std::endl;
        return false;
    }
    if (armor_ratio > max_armor_ratio_) {
        std::cout << "✗ 拒绝 (宽高比=" << std::fixed << std::setprecision(2) 
                  << armor_ratio << " > " << max_armor_ratio_ << ", 太宽)" << std::endl;
        return false;
    }
    
    // 5. 倾斜方向一致性检查（新增！防止误匹配）
    bool same_direction = (left.angle * right.angle >= 0);
    if (!same_direction && std::abs(left.angle) > max_tilt_for_direction_ 
        && std::abs(right.angle) > max_tilt_for_direction_) {
        std::cout << "✗ 拒绝 (倾斜方向相反: 左=" << std::fixed << std::setprecision(1) 
                  << left.angle << "° 右=" << right.angle << "°)" << std::endl;
        return false;
    }
    
    // 6. 面积相似性检查（透视下也不应该差太多）
    float area_ratio = std::min(left.area, right.area) / 
                      std::max(left.area, right.area);
    if (area_ratio < min_area_ratio_) {
        std::cout << "✗ 拒绝 (面积相似度=" << std::fixed << std::setprecision(2) 
                  << area_ratio << " < " << min_area_ratio_ << ")" << std::endl;
        return false;
    }
    
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


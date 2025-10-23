#ifndef TRADITIONAL_DETECTOR_HPP
#define TRADITIONAL_DETECTOR_HPP

#include "common/armor.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

namespace armor_detector {

/**
 * @brief 传统图像处理装甲板检测器
 * 使用颜色分割+轮廓检测+灯条匹配的方法检测装甲板
 */
class TraditionalDetector {
public:
    /**
     * @brief 构造函数
     * @param target_color 目标颜色："red" 或 "blue"
     */
    explicit TraditionalDetector(const std::string& target_color = "red");
    
    /**
     * @brief 检测装甲板
     * @param frame 输入图像
     * @return 检测到的装甲板列表
     */
    std::vector<Armor> detect(const cv::Mat& frame);
    
    /**
     * @brief 设置HSV颜色阈值
     * @param hsv_low HSV下限 [H, S, V]
     * @param hsv_high HSV上限 [H, S, V]
     */
    void setHSVThreshold(const cv::Scalar& hsv_low, const cv::Scalar& hsv_high);
    
    /**
     * @brief 设置目标颜色
     * @param color "red" 或 "blue"
     */
    void setTargetColor(const std::string& color);
    
    /**
     * @brief 设置颜色提取方法
     * @param method "channel_separation" 或 "hsv"
     */
    void setColorMethod(const std::string& method);
    
    /**
     * @brief 设置二值化阈值（通道分离法）
     */
    void setBinaryThreshold(int threshold) { binary_threshold_ = threshold; }
    
    /**
     * @brief 设置Otsu参数
     */
    void setOtsuParams(bool use_otsu, const std::string& mode, double roi_scale,
                       int min_thresh, int max_thresh, double temporal_smooth);
    
    /**
     * @brief 设置预测ROI（用于自适应阈值）
     */
    void setPredictionROI(const cv::Rect& roi) { prediction_roi_ = roi; has_prediction_roi_ = true; }
    
    /**
     * @brief 清除预测ROI
     */
    void clearPredictionROI() { has_prediction_roi_ = false; }
    
    /**
     * @brief 设置灯条面积阈值
     */
    void setMinLightArea(float min_area) { min_light_area_ = min_area; }
    
    /**
     * @brief 设置灯条长宽比阈值
     */
    void setMaxLightRatio(float max_ratio) { max_light_ratio_ = max_ratio; }
    
    /**
     * @brief 获取调试图像（用于可视化中间步骤）
     */
    cv::Mat getDebugImage() const { return debug_image_; }
    
    /**
     * @brief 启用/禁用调试模式
     */
    void setDebugMode(bool enable) { debug_mode_ = enable; }

private:
    /**
     * @brief 颜色分割，提取目标颜色区域
     * @param frame 输入图像
     * @return 二值化图像
     */
    cv::Mat colorSegmentation(const cv::Mat& frame);
    
    /**
     * @brief 查找灯条
     * @param binary 二值化图像
     * @return 灯条列表
     */
    std::vector<LightBar> findLightBars(const cv::Mat& binary);
    
    /**
     * @brief 匹配装甲板
     * @param light_bars 灯条列表
     * @return 装甲板列表
     */
    std::vector<Armor> matchArmor(const std::vector<LightBar>& light_bars);
    
    /**
     * @brief 计算自适应阈值（Otsu）
     */
    int calculateAdaptiveThreshold(const cv::Mat& gray);
    
    /**
     * @brief 判断灯条是否有效
     */
    bool isValidLightBar(const LightBar& bar);
    
    /**
     * @brief 判断两个灯条是否能配对成装甲板
     */
    bool canMatch(const LightBar& left, const LightBar& right);
    
    /**
     * @brief 从两个灯条构建装甲板
     */
    Armor buildArmor(const LightBar& left, const LightBar& right);
    
    /**
     * @brief 计算装甲板的四个角点
     */
    ArmorCorners calculateCorners(const LightBar& left, const LightBar& right);

private:
    std::string target_color_;      // 目标颜色："red" 或 "blue"
    std::string color_method_;      // 颜色提取方法："channel_separation" 或 "hsv"
    cv::Scalar hsv_low_;            // HSV下限
    cv::Scalar hsv_high_;           // HSV上限
    cv::Scalar hsv_low2_;           // HSV下限2（红色环绕）
    cv::Scalar hsv_high2_;          // HSV上限2（红色环绕）
    int binary_threshold_;          // 通道分离法的二值化阈值
    
    // Otsu自适应阈值参数
    bool use_otsu_;                 // 是否使用Otsu
    std::string otsu_mode_;         // Otsu模式: global/roi/hybrid
    double otsu_roi_scale_;         // ROI放大倍数
    int otsu_min_threshold_;        // 最小阈值保护
    int otsu_max_threshold_;        // 最大阈值保护
    double otsu_temporal_smooth_;   // 时序平滑系数
    int last_otsu_threshold_;       // 上一帧Otsu阈值
    cv::Rect prediction_roi_;       // 预测ROI
    bool has_prediction_roi_;       // 是否有预测ROI
    
    // 检测参数
    float min_light_area_;          // 灯条最小面积
    float max_light_ratio_;         // 灯条最大长宽比
    float min_armor_ratio_;         // 装甲板最小宽高比
    float max_armor_ratio_;         // 装甲板最大宽高比
    float max_angle_diff_;          // 两灯条最大角度差
    float max_height_diff_ratio_;   // 两灯条高度差比例
    float max_y_diff_ratio_;        // 两灯条Y坐标差比例
    
    // 调试
    bool debug_mode_;               // 调试模式
    cv::Mat debug_image_;           // 调试图像
};

} // namespace armor_detector

#endif // TRADITIONAL_DETECTOR_HPP


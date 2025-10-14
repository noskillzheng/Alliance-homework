#ifndef ARMOR_DETECTOR_TRADITIONAL_H
#define ARMOR_DETECTOR_TRADITIONAL_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <chrono>

struct ArmorDetection {
    cv::Rect bbox;          // 边界框
    cv::RotatedRect rotated_bbox;  // 旋转边界框
    double area;            // 面积
    double aspect_ratio;    // 宽高比
    double rectity;         // 矩形度
    double confidence;      // 置信度
};

class TraditionalArmorDetector {
private:
    std::string color_mode;
    cv::Scalar lower_color1, upper_color1;
    cv::Scalar lower_color2, upper_color2;
    int detection_time;

    void setupColorRanges();

    // 图像处理方法
    cv::Mat preprocessImage(const cv::Mat& image);
    cv::Mat detectLightBars(const cv::Mat& image);  // 大津法检测灯条
    cv::Mat colorSegmentation(const cv::Mat& hsv);
    std::vector<std::vector<cv::Point>> findContours(const cv::Mat& mask);
    std::vector<cv::RotatedRect> findLightBarContours(const cv::Mat& light_mask);
    std::vector<ArmorDetection> filterArmorContours(
        const std::vector<std::vector<cv::Point>>& contours,
        const cv::Size& image_size);
    bool validateArmorWithLightBars(const cv::Rect& armor_bbox,
                                   const std::vector<cv::RotatedRect>& light_bars);
    
    // 基于灯条匹配的装甲板检测方法
    std::vector<ArmorDetection> detectArmorsFromLightBars(
        const std::vector<cv::RotatedRect>& light_bars, const cv::Size& image_size);
    bool isValidLightBarPair(const cv::RotatedRect& bar1, const cv::RotatedRect& bar2);
    cv::Rect calculateArmorBoundingBox(const cv::RotatedRect& bar1, const cv::RotatedRect& bar2);
    bool isValidArmorRegion(const cv::Rect& bbox, const cv::Size& image_size);
    bool hasDigitRegion(const cv::Rect& armor_bbox);
    std::vector<ArmorDetection> applyNonMaxSuppression(const std::vector<ArmorDetection>& detections);

    // 边界框精修机制
    cv::Rect refineBoundingBox(const cv::Rect& bbox, const std::vector<cv::Point>& contour, const cv::Size& image_size);

public:
    // 构造函数
    TraditionalArmorDetector(const std::string& color_mode = "blue");

    // 析构函数
    ~TraditionalArmorDetector() = default;

    // 主要检测方法
    std::vector<ArmorDetection> detectArmors(const cv::Mat& image);

    // 结果处理方法
    cv::Mat drawResults(const cv::Mat& image, const std::vector<ArmorDetection>& detections);
    bool saveResults(const cv::Mat& result_image, const std::string& output_path);
    void printDetectionDetails(const std::vector<ArmorDetection>& detections);

    // 获取检测时间
    int getDetectionTime() const { return detection_time; }
};

#endif // ARMOR_DETECTOR_TRADITIONAL_H
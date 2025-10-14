#ifndef ARMOR_DETECTOR_CMD_H
#define ARMOR_DETECTOR_CMD_H

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <string>

struct DetectionResult {
    cv::Rect box;           // 边界框
    float confidence;       // 置信度
    int class_id;          // 类别ID
    std::string class_name; // 类别名称
};

class ArmorDetector {
private:
    cv::dnn::Net net;
    float conf_threshold;
    std::vector<std::string> class_names;
    cv::Size input_size;

public:
    // 构造函数
    ArmorDetector(const std::string& model_path, float conf_threshold = 0.3f);

    // 析构函数
    ~ArmorDetector() = default;

    // 检测装甲板
    std::vector<DetectionResult> detect(const cv::Mat& image);

    // 绘制检测结果
    cv::Mat drawResults(const cv::Mat& image, const std::vector<DetectionResult>& results);

    // 预处理图像
    cv::Mat preprocess(const cv::Mat& image);

    // 后处理检测结果
    std::vector<DetectionResult> postprocess(const cv::Mat& output,
                                           const cv::Size& original_size);

private:
    // 加载类别名称
    void loadClassNames();

    // 计算IOU
    float calculateIOU(const cv::Rect& box1, const cv::Rect& box2);

    // 非极大值抑制
    std::vector<int> nms(const std::vector<cv::Rect>& boxes,
                        const std::vector<float>& scores,
                        float nms_threshold = 0.4f);
};

#endif // ARMOR_DETECTOR_CMD_H
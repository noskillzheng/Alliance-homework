#include "armor_detector_cmd.h"
#include <iostream>
#include <algorithm>
#include <fstream>

ArmorDetector::ArmorDetector(const std::string& model_path, float conf_threshold)
    : conf_threshold(conf_threshold), input_size(416, 416) {

    // 加载类别名称
    loadClassNames();

    // 加载模型
    try {
        // 检查文件扩展名决定加载方式
        if (model_path.find(".onnx") != std::string::npos) {
            net = cv::dnn::readNetFromONNX(model_path);
            std::cout << "✅ ONNX模型加载成功" << std::endl;
        } else {
            // 尝试加载为ONNX格式（即使扩展名是.pt）
            std::string onnx_path = model_path.substr(0, model_path.find_last_of('.')) + ".onnx";
            if (std::ifstream(onnx_path).good()) {
                net = cv::dnn::readNetFromONNX(onnx_path);
                std::cout << "✅ ONNX模型加载成功: " << onnx_path << std::endl;
            } else {
                std::cerr << "❌ 未找到ONNX模型文件: " << onnx_path << std::endl;
                std::cerr << "💡 提示: 请先将PyTorch模型转换为ONNX格式" << std::endl;
                throw std::runtime_error("无法找到可用的模型文件");
            }
        }
    } catch (const cv::Exception& e) {
        std::cerr << "❌ 模型加载失败: " << e.what() << std::endl;
        throw std::runtime_error("无法加载模型文件");
    }

    // 设置计算后端
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    
    // 打印模型信息
    std::cout << "🔍 模型信息:" << std::endl;
    std::cout << "  输入层名称: ";
    auto input_names = net.getLayerNames();
    for (const auto& name : input_names) {
        std::cout << name << " ";
    }
    std::cout << std::endl;
    
    std::cout << "  输出层名称: ";
    auto out_names = net.getUnconnectedOutLayersNames();
    for (const auto& name : out_names) {
        std::cout << name << " ";
    }
    std::cout << std::endl;

    std::cout << "🔧 置信度阈值: " << conf_threshold << std::endl;
}

std::vector<DetectionResult> ArmorDetector::detect(const cv::Mat& image) {
    std::vector<DetectionResult> results;

    if (image.empty()) {
        return results;
    }

    // 预处理
    cv::Mat processed_image = preprocess(image);

    // 前向推理
    cv::Mat output;
    net.setInput(processed_image);
    
    try {
        // 直接获取输出
        output = net.forward();
        
        // 打印输出形状
        std::cout << "🔍 模型输出信息:" << std::endl;
        std::cout << "  输出形状: [";
        for (int i = 0; i < output.dims; i++) {
            std::cout << output.size[i];
            if (i < output.dims - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    } catch (const cv::Exception& e) {
        std::cerr << "❌ 推理失败: " << e.what() << std::endl;
        throw std::runtime_error("推理过程中发生错误");
    }

    // 后处理
    results = postprocess(output, image.size());

    return results;
}

cv::Mat ArmorDetector::preprocess(const cv::Mat& image) {
    cv::Mat blob;
    
    std::cout << "🔍 预处理信息:" << std::endl;
    std::cout << "  输入图像尺寸: " << image.cols << "x" << image.rows << std::endl;
    std::cout << "  目标尺寸: " << input_size.width << "x" << input_size.height << std::endl;

    // 将图像转换为blob格式
    cv::dnn::blobFromImage(image, blob, 1.0/255.0, input_size,
                          cv::Scalar(0, 0, 0), true, false);
    
    std::cout << "  Blob形状: [";
    for (int i = 0; i < blob.dims; i++) {
        std::cout << blob.size[i];
        if (i < blob.dims - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    return blob;
}

std::vector<DetectionResult> ArmorDetector::postprocess(const cv::Mat& output,
                                                       const cv::Size& original_size) {
    std::vector<DetectionResult> results;
    
    // 打印输出形状
    std::cout << "🔍 输出形状: [";
    for (int i = 0; i < output.dims; i++) {
        std::cout << output.size[i];
        if (i < output.dims - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // 模型输出格式: (1, 5, 3549)
    // 5 = [x_center, y_center, width, height, confidence]
    // 3549 = 检测框数量
    
    // 转置输出以方便处理
    cv::Mat reshaped = output.reshape(0, {5, (int)output.total() / 5}).t();
    
    // 获取检测框数量
    int num_detections = reshaped.rows;
    
    std::cout << "🔍 检测框数量: " << num_detections << std::endl;
    
    // 解析输出
    float* data = (float*)reshaped.data;
    for (int i = 0; i < num_detections; ++i) {
        float confidence = data[i * 5 + 4];
        if (confidence >= conf_threshold) {
            // 获取边界框坐标 (相对于输入尺寸)
            float x_center = data[i * 5 + 0];
            float y_center = data[i * 5 + 1];
            float width = data[i * 5 + 2];
            float height = data[i * 5 + 3];
            
            // 转换为绝对坐标 (相对于输入尺寸)
            float x1 = x_center - width / 2;
            float y1 = y_center - height / 2;
            float x2 = x_center + width / 2;
            float y2 = y_center + height / 2;
            
            // 缩放到原始图像尺寸
            float scale_x = (float)original_size.width / input_size.width;
            float scale_y = (float)original_size.height / input_size.height;
            
            int orig_x1 = (int)(x1 * scale_x);
            int orig_y1 = (int)(y1 * scale_y);
            int orig_x2 = (int)(x2 * scale_x);
            int orig_y2 = (int)(y2 * scale_y);
            
            // 确保坐标在图像范围内
            orig_x1 = std::max(0, std::min(orig_x1, original_size.width - 1));
            orig_y1 = std::max(0, std::min(orig_y1, original_size.height - 1));
            orig_x2 = std::max(0, std::min(orig_x2, original_size.width - 1));
            orig_y2 = std::max(0, std::min(orig_y2, original_size.height - 1));
            
            // 创建检测结果
            DetectionResult result;
            result.box = cv::Rect(orig_x1, orig_y1, orig_x2 - orig_x1, orig_y2 - orig_y1);
            result.confidence = confidence;
            result.class_id = 0; // 只有一个类别: 装甲板
            result.class_name = "armor";
            
            results.push_back(result);
        }
    }
    
    std::cout << "🔍 检测到 " << results.size() << " 个装甲板" << std::endl;
    
    return results;
}

cv::Mat ArmorDetector::drawResults(const cv::Mat& image,
                                  const std::vector<DetectionResult>& results) {
    cv::Mat result_image = image.clone();

    for (const auto& detection : results) {
        // 绘制边界框
        cv::rectangle(result_image, detection.box, cv::Scalar(0, 255, 0), 2);

        // 绘制标签背景
        std::string label = detection.class_name + " " +
                           std::to_string(detection.confidence).substr(0, 4);

        int baseline;
        cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
                                            0.6, 1, &baseline);

        cv::Point label_pos(detection.box.x, detection.box.y - 10);
        cv::rectangle(result_image,
                     cv::Point(detection.box.x, detection.box.y - label_size.height - 10),
                     cv::Point(detection.box.x + label_size.width, detection.box.y),
                     cv::Scalar(0, 255, 0), -1);

        // 绘制标签文本
        cv::putText(result_image, label, label_pos,
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);
    }

    return result_image;
}

void ArmorDetector::loadClassNames() {
    // 装甲板检测只有一个类别
    class_names.push_back("armor");
}

float ArmorDetector::calculateIOU(const cv::Rect& box1, const cv::Rect& box2) {
    int x1 = std::max(box1.x, box2.x);
    int y1 = std::max(box1.y, box2.y);
    int x2 = std::min(box1.x + box1.width, box2.x + box2.width);
    int y2 = std::min(box1.y + box1.height, box2.y + box2.height);

    if (x2 <= x1 || y2 <= y1) {
        return 0.0f;
    }

    int intersection = (x2 - x1) * (y2 - y1);
    int area1 = box1.width * box1.height;
    int area2 = box2.width * box2.height;
    int union_area = area1 + area2 - intersection;

    return static_cast<float>(intersection) / union_area;
}

std::vector<int> ArmorDetector::nms(const std::vector<cv::Rect>& boxes,
                                   const std::vector<float>& scores,
                                   float nms_threshold) {
    std::vector<int> indices;
    std::vector<std::pair<float, int>> score_index_pairs;

    // 创建分数-索引对并排序
    for (size_t i = 0; i < scores.size(); ++i) {
        score_index_pairs.push_back({scores[i], static_cast<int>(i)});
    }

    std::sort(score_index_pairs.begin(), score_index_pairs.end(),
              [](const std::pair<float, int>& a, const std::pair<float, int>& b) {
                  return a.first > b.first;
              });

    std::vector<bool> suppressed(boxes.size(), false);

    for (size_t i = 0; i < score_index_pairs.size(); ++i) {
        int idx = score_index_pairs[i].second;

        if (suppressed[idx]) {
            continue;
        }

        indices.push_back(idx);

        // 抑制与当前框重叠度高的框
        for (size_t j = i + 1; j < score_index_pairs.size(); ++j) {
            int other_idx = score_index_pairs[j].second;

            if (!suppressed[other_idx]) {
                float iou = calculateIOU(boxes[idx], boxes[other_idx]);
                if (iou > nms_threshold) {
                    suppressed[other_idx] = true;
                }
            }
        }
    }

    return indices;
}
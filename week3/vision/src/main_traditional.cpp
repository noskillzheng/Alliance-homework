#include <iostream>
#include <string>
#include <unistd.h>
#include <getopt.h>
#include <opencv2/opencv.hpp>
#include "armor_detector_traditional.h"

void print_usage(const char* program_name) {
    std::cout << "🎯 RoboMaster 装甲板检测系统 - C++传统视觉版本" << std::endl;
    std::cout << "用法: " << program_name << " [选项]" << std::endl;
    std::cout << std::endl;
    std::cout << "选项:" << std::endl;
    std::cout << "  -i, --image <路径>     检测指定图片并保存结果" << std::endl;
    std::cout << "  -o, --output <路径>    指定输出路径 (默认: ./results/)" << std::endl;
    std::cout << "  -c, --color <模式>     检测颜色模式 (默认: blue)" << std::endl;
    std::cout << "  -h, --help             显示帮助信息" << std::endl;
    std::cout << std::endl;
    std::cout << "颜色模式:" << std::endl;
    std::cout << "  blue  - 检测蓝色装甲板" << std::endl;
    std::cout << "  red   - 检测红色装甲板" << std::endl;
    std::cout << std::endl;
    std::cout << "示例:" << std::endl;
    std::cout << "  " << program_name << " --image test.png" << std::endl;
    std::cout << "  " << program_name << " --image test.png --color red --output ./results/" << std::endl;
}

void showImage(const std::string& window_name, const cv::Mat& image) {
    try {
        cv::namedWindow(window_name, cv::WINDOW_NORMAL);

        // 根据图片大小设置合适的窗口尺寸
        int max_width = 1200;
        int max_height = 800;

        if (image.cols > max_width || image.rows > max_height) {
            double scale = std::min(static_cast<double>(max_width) / image.cols,
                                  static_cast<double>(max_height) / image.rows);
            int new_width = static_cast<int>(image.cols * scale);
            int new_height = static_cast<int>(image.rows * scale);
            cv::resizeWindow(window_name, new_width, new_height);
        } else {
            cv::resizeWindow(window_name, image.cols, image.rows);
        }

        cv::imshow(window_name, image);
        std::cout << "🔍 按任意键关闭窗口..." << std::endl;
        cv::waitKey(0);
        cv::destroyAllWindows();
        std::cout << "✅ 窗口已关闭" << std::endl;
    } catch (const cv::Exception& e) {
        std::cerr << "⚠️  显示窗口失败: " << e.what() << std::endl;
        std::cerr << "💡 您可以手动查看保存的结果文件" << std::endl;
    }
}

int main(int argc, char** argv) {
    std::string image_path;
    std::string output_path = "./results/";
    std::string color_mode = "blue";
    bool show_help = false;

    // 解析命令行参数
    int opt;
    static struct option long_options[] = {
        {"image", required_argument, 0, 'i'},
        {"output", required_argument, 0, 'o'},
        {"color", required_argument, 0, 'c'},
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}
    };

    while ((opt = getopt_long(argc, argv, "i:o:c:h", long_options, nullptr)) != -1) {
        switch (opt) {
            case 'i':
                image_path = optarg;
                break;
            case 'o':
                output_path = optarg;
                break;
            case 'c':
                color_mode = optarg;
                break;
            case 'h':
                show_help = true;
                break;
            default:
                std::cerr << "❌ 未知选项: " << char(opt) << std::endl;
                print_usage(argv[0]);
                return 1;
        }
    }

    // 显示帮助信息
    if (show_help) {
        print_usage(argv[0]);
        return 0;
    }

    // 检查参数
    if (image_path.empty()) {
        std::cerr << "❌ 错误: 请指定 --image 参数" << std::endl;
        print_usage(argv[0]);
        return 1;
    }

    // 检查颜色模式
    if (color_mode != "blue" && color_mode != "red") {
        std::cerr << "❌ 错误: 不支持的颜色模式 '" << color_mode << "'" << std::endl;
        std::cerr << "💡 支持的颜色模式: blue, red" << std::endl;
        return 1;
    }

    // 创建输出目录
    std::string mkdir_cmd = "mkdir -p " + output_path;
    system(mkdir_cmd.c_str());

    // 初始化传统视觉装甲板检测器
    std::cout << "🎯 RoboMaster 装甲板检测系统 - C++传统视觉版本" << std::endl;
    std::cout << "===============================================" << std::endl;
    std::cout << "🔧 正在初始化传统视觉检测器..." << std::endl;

    try {
        TraditionalArmorDetector detector(color_mode);

        // 读取图片
        std::cout << "📸 开始检测图片: " << image_path << std::endl;
        cv::Mat image = cv::imread(image_path);
        if (image.empty()) {
            std::cerr << "❌ 无法读取图片: " << image_path << std::endl;
            return 1;
        }

        // 显示图片信息
        std::cout << "📐 图片尺寸: " << image.cols << "x" << image.rows << std::endl;

        // 检测装甲板
        std::vector<ArmorDetection> detections = detector.detectArmors(image);

        // 打印检测结果详情
        if (!detections.empty()) {
            detector.printDetectionDetails(detections);
        } else {
            std::cout << "❌ 未检测到装甲板" << std::endl;
        }

        // 绘制检测结果
        cv::Mat result_image = detector.drawResults(image, detections);

        // 保存结果
        std::string filename = image_path.substr(image_path.find_last_of("/\\") + 1);
        std::string output_file = output_path + "traditional_" + filename;

        if (detector.saveResults(result_image, output_file)) {
            std::cout << std::endl;
            std::cout << "🎉 传统视觉检测完成！" << std::endl;
            std::cout << "📊 检测统计:" << std::endl;
            std::cout << "   - 检测到装甲板: " << detections.size() << " 个" << std::endl;
            std::cout << "   - 检测时间: " << detector.getDetectionTime() << "ms" << std::endl;
            std::cout << "   - 检测颜色: " << color_mode << std::endl;
            std::cout << "💾 结果已保存: " << output_file << std::endl;

            // 显示结果
            showImage("传统视觉装甲板检测结果", result_image);
        } else {
            std::cerr << "❌ 保存结果失败" << std::endl;
            return 1;
        }

    } catch (const std::exception& e) {
        std::cerr << "❌ 检测过程中发生错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
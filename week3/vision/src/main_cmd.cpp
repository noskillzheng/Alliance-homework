#include <iostream>
#include <string>
#include <unistd.h>
#include <getopt.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include "armor_detector_cmd.h"

void print_usage(const char* program_name) {
    std::cout << "🎯 RoboMaster 装甲板检测系统 - C++版本" << std::endl;
    std::cout << "用法: " << program_name << " [选项]" << std::endl;
    std::cout << std::endl;
    std::cout << "选项:" << std::endl;
    std::cout << "  -i, --image <路径>     检测指定图片并保存结果" << std::endl;
    std::cout << "  -v, --video <路径>     检测指定视频并实时显示结果" << std::endl;
    std::cout << "  -m, --model <路径>     指定模型文件路径 (默认: models/trained/best.pt)" << std::endl;
    std::cout << "  -o, --output <路径>    指定输出路径 (默认: ./results/)" << std::endl;
    std::cout << "  -c, --confidence <值>  置信度阈值 (默认: 0.3)" << std::endl;
    std::cout << "  -h, --help             显示帮助信息" << std::endl;
    std::cout << std::endl;
    std::cout << "示例:" << std::endl;
    std::cout << "  " << program_name << " --image test.png" << std::endl;
    std::cout << "  " << program_name << " --video test.avi --output ./results/" << std::endl;
    std::cout << "  " << program_name << " --image test.png --confidence 0.5" << std::endl;
}

int main(int argc, char** argv) {
    std::string image_path;
    std::string video_path;
    std::string model_path = "models/trained/best.pt";
    std::string output_path = "./output/";
    float confidence = 0.3f;
    bool show_help = false;

    // 解析命令行参数
    int opt;
    static struct option long_options[] = {
        {"image", required_argument, 0, 'i'},
        {"video", required_argument, 0, 'v'},
        {"model", required_argument, 0, 'm'},
        {"output", required_argument, 0, 'o'},
        {"confidence", required_argument, 0, 'c'},
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}
    };

    while ((opt = getopt_long(argc, argv, "i:v:m:o:c:h", long_options, nullptr)) != -1) {
        switch (opt) {
            case 'i':
                image_path = optarg;
                break;
            case 'v':
                video_path = optarg;
                break;
            case 'm':
                model_path = optarg;
                break;
            case 'o':
                output_path = optarg;
                break;
            case 'c':
                confidence = std::stof(optarg);
                break;
            case 'h':
                show_help = true;
                break;
            default:
                print_usage(argv[0]);
                return 1;
        }
    }

    if (show_help) {
        print_usage(argv[0]);
        return 0;
    }

    if (image_path.empty() && video_path.empty()) {
        std::cerr << "❌ 错误: 请指定图片或视频路径" << std::endl;
        print_usage(argv[0]);
        return 1;
    }

    std::cout << "🎯 RoboMaster 装甲板检测系统 - C++版本" << std::endl;
    std::cout << "=======================================" << std::endl;

    // 构建Python命令
    std::string python_cmd = "cd " + std::string(getcwd(nullptr, 0)) + " && bash -c 'source yolo_env/bin/activate && python ";
    
    if (!image_path.empty()) {
        python_cmd += "scripts/armor_detection_image.py";
        python_cmd += " --image " + image_path;
        std::cout << "📸 开始检测图片: " << image_path << std::endl;
    } else {
        python_cmd += "scripts/armor_detection_display.py";
        python_cmd += " --video " + video_path;
        std::cout << "🎥 开始检测视频: " << video_path << std::endl;
    }
    
    python_cmd += " --model " + model_path;
    python_cmd += " --confidence " + std::to_string(confidence);
    python_cmd += " --output " + output_path;
    python_cmd += "'";

    std::cout << "🔄 正在调用Python脚本进行检测..." << std::endl;
    
    // 执行Python命令
    int result = system(python_cmd.c_str());
    
    if (result == 0) {
        std::cout << "✅ 检测完成！结果已保存到: " << output_path << std::endl;
    } else {
        std::cerr << "❌ 检测过程中发生错误" << std::endl;
        return 1;
    }

    return 0;
}
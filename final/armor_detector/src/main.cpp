/**
 * @file main.cpp
 * @brief 装甲板识别与预测系统主程序
 * 
 * 功能说明：
 * 1. 使用海康工业相机采集图像（或使用VideoCapture备用方案）
 * 2. 使用传统图像处理方法检测装甲板
 * 3. 使用卡尔曼滤波器预测装甲板0.5秒后的位置
 * 4. 实时可视化检测和预测结果
 */

#include "camera/hik_camera.hpp"
#include "detector/traditional_detector.hpp"
#include "predictor/kalman_filter.hpp"
#include "utils/visualizer.hpp"
#include "utils/config_reader.hpp"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <chrono>

using namespace armor_detector;

// 辅助函数：计算两个矩形的IoU
static float calculateIoU(const cv::Rect2f& a, const cv::Rect2f& b) {
    float intersection_area = (a & b).area();
    float union_area = a.area() + b.area() - intersection_area;
    return union_area > 0 ? intersection_area / union_area : 0.0F;
}

// 辅助函数：从装甲板生成矩形
static cv::Rect2f armorToRect(const Armor& armor) {
    return cv::Rect2f(armor.center.x - armor.width / 2, 
                      armor.center.y - armor.height / 2,
                      armor.width, armor.height);
}

// 辅助函数：从预测结果生成矩形
static cv::Rect2f predictionToRect(const PredictionResult& pred) {
    return cv::Rect2f(pred.position.x - pred.size.width / 2,
                      pred.position.y - pred.size.height / 2,
                      pred.size.width, pred.size.height);
}

// 辅助函数：计算两点距离
static float pointDistance(const cv::Point2f& a, const cv::Point2f& b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

int main(int /* argc */, char** /* argv */) {
    std::cout << "========================================" << std::endl;
    std::cout << "  装甲板识别与预测系统" << std::endl;
    std::cout << "  RoboMaster 视觉训练项目" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    
    // ========== 1. 加载配置 ==========
    std::cout << "[主程序] 正在加载配置文件..." << std::endl;
    
    // 智能查找配置文件目录（支持从项目根目录或build目录运行）
    std::string config_dir = "../config";
    if (!std::ifstream(config_dir + "/camera_params.yaml").good()) {
        config_dir = "config";  // 尝试相对于项目根目录
    }
    if (!std::ifstream(config_dir + "/camera_params.yaml").good()) {
        std::cerr << "[主程序] 错误：无法找到配置文件目录！" << std::endl;
        std::cerr << "[主程序] 请确保从项目根目录或build目录运行程序" << std::endl;
        return -1;
    }
    std::cout << "[主程序] 使用配置目录: " << config_dir << std::endl;
    
    ConfigReader config(config_dir);
    ConfigReader detector_config(config_dir);
    ConfigReader kalman_config(config_dir);
    
    // 加载所有配置文件
    if (!config.loadConfig("camera_params.yaml")) {
        std::cerr << "[主程序] 警告：无法加载相机配置，使用默认值" << std::endl;
    }
    if (!detector_config.loadConfig("detector_params.yaml")) {
        std::cerr << "[主程序] 警告：无法加载检测器配置，使用默认值" << std::endl;
    }
    if (!kalman_config.loadConfig("kalman_params.yaml")) {
        std::cerr << "[主程序] 警告：无法加载卡尔曼滤波器配置，使用默认值" << std::endl;
    }
    
    std::cout << std::endl;
    
    // ========== 2. 初始化相机 ==========
    std::cout << "[主程序] 正在初始化相机..." << std::endl;
    HikCamera camera;
    
    // 读取相机参数
    int exposure_time = config.getInt("camera.exposure_time", 5000);
    float gain = config.getFloat("camera.gain", 10.0F);
    float gamma = config.getFloat("camera.gamma", 0.7F);
    int width = config.getInt("camera.resolution.width", 640);
    int height = config.getInt("camera.resolution.height", 480);
    
    camera.init(exposure_time, gain, gamma);
    camera.setResolution(width, height);
    
    // 尝试打开海康相机
    bool camera_opened = camera.open(0);
    
    // 如果海康相机打开失败，使用VideoCapture备用方案
    if (!camera_opened) {
        std::cout << "[主程序] 海康相机未连接，切换到VideoCapture备用方案" << std::endl;
        
        bool use_video = config.getBool("fallback.use_video", true);
        std::string video_source = config.getString("fallback.video_source", "0");
        
        std::cout << "[主程序] 配置读取: use_video=" << use_video 
                  << ", video_source=" << video_source << std::endl;
        
        if (use_video) {
            if (!camera.openVideoCapture(video_source)) {
                std::cerr << "[主程序] 错误：无法打开视频源: " << video_source << std::endl;
                return -1;
            }
        } else {
            std::cerr << "[主程序] 错误：无可用的图像源" << std::endl;
            return -1;
        }
    }
    
    camera.startGrabbing();
    std::cout << std::endl;
    
    // ========== 3. 初始化检测器 ==========
    std::cout << "[主程序] 正在初始化装甲板检测器..." << std::endl;
    std::string target_color = detector_config.getString("detector.target_color", "red");
    TraditionalDetector detector(target_color);
    
    // 设置颜色提取方法
    std::string color_method = detector_config.getString("detector.color_method", "channel_separation");
    detector.setColorMethod(color_method);
    
    // 设置二值化阈值（通道分离法）
    int binary_threshold = detector_config.getInt("detector.binary_threshold", 80);
    detector.setBinaryThreshold(binary_threshold);
    
    // 设置检测参数
    detector.setMinLightArea(detector_config.getFloat("detector.light.min_area", 30.0F));
    detector.setMaxLightRatio(detector_config.getFloat("detector.light.max_aspect_ratio", 8.0F));
    detector.setDebugMode(detector_config.getBool("detector.debug_mode", false));
    
    // 设置Otsu自适应阈值参数
    bool use_otsu = detector_config.getBool("detector.threshold.use_otsu", true);
    std::string otsu_mode = detector_config.getString("detector.threshold.mode", "roi");
    double otsu_roi_scale = detector_config.getDouble("detector.threshold.roi_scale", 1.8);
    int otsu_min = detector_config.getInt("detector.threshold.min_threshold", 40);
    int otsu_max = detector_config.getInt("detector.threshold.max_threshold", 150);
    double otsu_smooth = detector_config.getDouble("detector.threshold.temporal_smooth", 0.7);
    detector.setOtsuParams(use_otsu, otsu_mode, otsu_roi_scale, otsu_min, otsu_max, otsu_smooth);
    
    // 如果使用HSV方法，从配置文件加载HSV阈值
    if (color_method == "hsv") {
        if (target_color == "red") {
            cv::Scalar hsv_low(detector_config.getInt("detector.hsv.red.h_low", 0),
                              detector_config.getInt("detector.hsv.red.s_low", 120),
                              detector_config.getInt("detector.hsv.red.v_low", 150));
            cv::Scalar hsv_high(detector_config.getInt("detector.hsv.red.h_high", 10),
                               detector_config.getInt("detector.hsv.red.s_high", 255),
                               detector_config.getInt("detector.hsv.red.v_high", 255));
            detector.setHSVThreshold(hsv_low, hsv_high);
        } else if (target_color == "blue") {
            cv::Scalar hsv_low(detector_config.getInt("detector.hsv.blue.h_low", 100),
                              detector_config.getInt("detector.hsv.blue.s_low", 120),
                              detector_config.getInt("detector.hsv.blue.v_low", 100));
            cv::Scalar hsv_high(detector_config.getInt("detector.hsv.blue.h_high", 130),
                               detector_config.getInt("detector.hsv.blue.s_high", 255),
                               detector_config.getInt("detector.hsv.blue.v_high", 255));
            detector.setHSVThreshold(hsv_low, hsv_high);
        }
    }
    
    std::cout << std::endl;
    
    // ========== 4. 初始化卡尔曼滤波器 ==========
    std::cout << "[主程序] 正在初始化卡尔曼滤波器..." << std::endl;
    double dt = kalman_config.getDouble("kalman.dt", 0.033);
    bool use_enhanced = kalman_config.getBool("kalman.use_enhanced_2d_filter", true);
    double prediction_time = kalman_config.getDouble("kalman.prediction_time", 0.5);
    double fading_factor = kalman_config.getDouble("kalman.adaptive.fading_factor", 1.05);
    
    // 创建增强8维卡尔曼滤波器
    KalmanFilter kf(dt, use_enhanced);
    
    // 设置渐消因子（应对非匀速运动）
    kf.setFadingFactor(fading_factor);
    
    // 启用log域尺寸跟踪
    bool use_log_scale = kalman_config.getBool("kalman.use_log_scale", true);
    kf.setLogScale(use_log_scale);
    
    // 设置自适应R/Q
    bool adaptive_enabled = kalman_config.getBool("kalman.adaptive.enabled", true);
    double k_innov_low = kalman_config.getDouble("kalman.adaptive.k_innovation_low", 2.0);
    double k_innov_high = kalman_config.getDouble("kalman.adaptive.k_innovation_high", 8.0);
    double r_scale_low = kalman_config.getDouble("kalman.adaptive.r_scale_low", 0.7);
    double r_scale_high = kalman_config.getDouble("kalman.adaptive.r_scale_high", 1.4);
    kf.setAdaptiveNoise(adaptive_enabled, k_innov_low, k_innov_high, r_scale_low, r_scale_high);
    
    // 设置噪声参数（从配置文件读取）
    if (use_enhanced) {
        Eigen::VectorXd process_noise(8);
        process_noise << kalman_config.getDouble("kalman.process_noise.x", 0.5),
                        kalman_config.getDouble("kalman.process_noise.y", 0.5),
                        kalman_config.getDouble("kalman.process_noise.vx", 2.0),
                        kalman_config.getDouble("kalman.process_noise.vy", 2.0),
                        kalman_config.getDouble("kalman.process_noise.w", 5.0),
                        kalman_config.getDouble("kalman.process_noise.h", 5.0),
                        kalman_config.getDouble("kalman.process_noise.vw", 10.0),
                        kalman_config.getDouble("kalman.process_noise.vh", 10.0);
        kf.setProcessNoise(process_noise);
        
        Eigen::VectorXd measurement_noise(4);
        measurement_noise << kalman_config.getDouble("kalman.measurement_noise.x", 3.0),
                            kalman_config.getDouble("kalman.measurement_noise.y", 3.0),
                            kalman_config.getDouble("kalman.measurement_noise.w", 5.0),
                            kalman_config.getDouble("kalman.measurement_noise.h", 5.0);
        kf.setMeasurementNoise(measurement_noise);
    }
    
    std::cout << std::endl;
    
    // ========== 5. 初始化可视化 ==========
    std::cout << "[主程序] 正在初始化可视化工具..." << std::endl;
    Visualizer visualizer(50);  // 保留最近50帧的轨迹
    
    std::cout << std::endl;
    
    // ========== 6. 主循环 ==========
    std::cout << "[主程序] 开始主循环..." << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "控制键：" << std::endl;
    std::cout << "  'q' - 退出程序" << std::endl;
    std::cout << "  'r' - 重置卡尔曼滤波器" << std::endl;
    std::cout << "  'c' - 清空轨迹" << std::endl;
    std::cout << "  'd' - 切换调试模式" << std::endl;
    std::cout << "  SPACE - 暂停/继续" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << std::endl;
    
    // 性能统计
    auto last_time = std::chrono::high_resolution_clock::now();
    auto last_frame_time = last_time;
    float fps = 0.0F;
    int frame_count = 0;
    bool paused = false;
    int lost_frames = 0;  // 连续丢失帧计数
    int max_lost_frames = kalman_config.getInt("kalman.max_lost_frames", 15);
    
    // 加载选择器与可视化配置
    ConfigReader selector_config(config_dir);
    selector_config.loadConfig("detector_params.yaml");
    ConfigReader viz_config(config_dir);
    viz_config.loadConfig("visualizer_params.yaml");
    
    bool use_prediction_gating = selector_config.getBool("detector.selection.use_prediction_gating", true);
    double gating_iou_min = selector_config.getDouble("detector.selection.gating_iou_min", 0.15);
    double gating_center_max_px = selector_config.getDouble("detector.selection.gating_center_max_px", 60.0);
    double weight_conf = selector_config.getDouble("detector.selection.weights.confidence", 0.6);
    double weight_iou = selector_config.getDouble("detector.selection.weights.iou", 0.4);
    double weight_ratio = selector_config.getDouble("detector.selection.weights.ratio", 0.2);
    double armor_ratio_ref = selector_config.getDouble("detector.selection.armor_ratio_ref", 3.5);
    
    bool use_variable_dt = kalman_config.getBool("kalman.use_variable_dt", true);
    bool use_viz_smooth = viz_config.getBool("visualizer.smooth.enabled", true);
    double viz_ema_alpha = viz_config.getDouble("visualizer.smooth.ema_alpha", 0.35);
    
    // 可视化平滑状态
    cv::Point2f smoothed_center(0, 0);
    cv::Size2f smoothed_size(0, 0);
    bool smooth_initialized = false;
    
    // 深度运动检测滞回状态
    bool depth_enabled = viz_config.getBool("visualizer.depth.enabled", true);
    double depth_hysteresis_in = viz_config.getDouble("visualizer.depth.hysteresis.in", 0.06);
    double depth_hysteresis_out = viz_config.getDouble("visualizer.depth.hysteresis.out", 0.03);
    double depth_hold_time = viz_config.getDouble("visualizer.depth.hold_time", 0.2);
    
    enum DepthState { DEPTH_NONE, DEPTH_APPROACHING, DEPTH_RETREATING };
    DepthState depth_state = DEPTH_NONE;
    auto depth_state_change_time = std::chrono::high_resolution_clock::now();
    double last_log_area = 0.0;
    bool depth_state_initialized = false;
    
    cv::namedWindow("Armor Detection & Prediction", cv::WINDOW_NORMAL);
    
    while (true) {
        // 采集图像
        cv::Mat frame;
        int64_t timestamp = 0;
        
        if (!camera.grabFrame(frame, timestamp)) {
            std::cerr << "[主程序] 警告：无法采集图像" << std::endl;
            cv::waitKey(10);
            continue;
        }
        
        if (frame.empty()) {
            std::cerr << "[主程序] 警告：图像为空" << std::endl;
            cv::waitKey(10);
            continue;
        }
        
        auto frame_start_time = std::chrono::high_resolution_clock::now();
        
        // 计算实际帧间隔
        double dt_meas = dt;
        if (use_variable_dt) {
            double elapsed_sec = std::chrono::duration<double>(frame_start_time - last_frame_time).count();
            if (elapsed_sec > 0.001 && elapsed_sec < 0.5) {  // 防止异常值
                dt_meas = elapsed_sec;
            }
        }
        last_frame_time = frame_start_time;
        
        // 检测装甲板（仅调用一次，后续复用）
        std::vector<Armor> armors;
        if (!paused) {
            // 如果有预测，设置ROI给detector用于Otsu
            if (kf.isInitialized()) {
                PredictionResult current_state = kf.getCurrentState();
                cv::Rect pred_roi(
                    static_cast<int>(current_state.position.x - current_state.size.width * otsu_roi_scale / 2),
                    static_cast<int>(current_state.position.y - current_state.size.height * otsu_roi_scale / 2),
                    static_cast<int>(current_state.size.width * otsu_roi_scale),
                    static_cast<int>(current_state.size.height * otsu_roi_scale)
                );
                detector.setPredictionROI(pred_roi);
            } else {
                detector.clearPredictionROI();
            }
            
            armors = detector.detect(frame);
            
            // 获取当前预测状态（用于门控）
            PredictionResult current_pred;
            bool has_pred_for_gating = kf.isInitialized();
            if (has_pred_for_gating) {
                current_pred = kf.getCurrentState();
            }
            
            // 选择最佳装甲板（基于预测门控与打分）
            Armor best_armor;
            if (!armors.empty()) {
                if (use_prediction_gating && has_pred_for_gating) {
                    // 基于预测的门控与打分
                    cv::Rect2f pred_rect = predictionToRect(current_pred);
                    
                    std::vector<std::pair<int, float>> scored_candidates;
                    for (size_t i = 0; i < armors.size(); i++) {
                        const Armor& armor = armors[i];
                        cv::Rect2f armor_rect = armorToRect(armor);
                        
                        // 门控检查
                        float iou = calculateIoU(pred_rect, armor_rect);
                        float center_dist = pointDistance(current_pred.position, armor.center);
                        
                        bool pass_gating = (iou >= gating_iou_min) || 
                                          (center_dist <= gating_center_max_px);
                        
                        // 放宽门控（若正在丢失中）
                        if (!pass_gating && lost_frames > 0 && lost_frames <= 5) {
                            pass_gating = (iou >= gating_iou_min * 0.7) ||
                                         (center_dist <= gating_center_max_px * 1.5);
                        }
                        
                        if (pass_gating) {
                            // 计算综合得分
                            float ratio = armor.width > 0 ? armor.width / armor.height : 0;
                            float ratio_dev = std::abs(ratio - armor_ratio_ref) / armor_ratio_ref;
                            
                            float score = weight_conf * armor.confidence + 
                                         weight_iou * iou - 
                                         weight_ratio * ratio_dev;
                            
                            scored_candidates.push_back({static_cast<int>(i), score});
                        }
                    }
                    
                    // 选择得分最高的
                    if (!scored_candidates.empty()) {
                        auto best_it = std::max_element(scored_candidates.begin(), scored_candidates.end(),
                            [](const auto& a, const auto& b) { return a.second < b.second; });
                        best_armor = armors[best_it->first];
                    }
                } else {
                    // 传统方法：仅按置信度最高
                    best_armor = *std::max_element(armors.begin(), armors.end(),
                        [](const Armor& a, const Armor& b) {
                            return a.confidence < b.confidence;
                        });
                }
            }
            
            // 卡尔曼滤波更新和预测
            PredictionResult pred_result;
            bool has_prediction = false;
            
            if (best_armor.isValid() && best_armor.confidence > 0.3) {
                // 提取当前装甲板的位置和尺寸
                cv::Point2f pos = best_armor.center;
                cv::Size2f size(best_armor.width, best_armor.height);
                
                // 更新8维滤波器（位置+尺寸）
                if (kf.isEnhancedMode()) {
                    kf.update(pos, size);
                } else {
                    Eigen::Vector2d measurement(pos.x, pos.y);
                    kf.update(measurement);
                }
                
                // 预测（使用可变dt）
                kf.predict(dt_meas);
                
                // 预测未来状态（位置+尺寸）
                pred_result = kf.predictFuture(prediction_time);
                has_prediction = true;
                
                // 添加轨迹点
                visualizer.addTrajectoryPoint(best_armor.center);
                
                // 初始化或更新平滑状态
                if (!smooth_initialized) {
                    smoothed_center = best_armor.center;
                    smoothed_size = cv::Size2f(best_armor.width, best_armor.height);
                    smooth_initialized = true;
                } else if (use_viz_smooth) {
                    // EMA平滑
                    smoothed_center.x = static_cast<float>(viz_ema_alpha * best_armor.center.x + 
                                                          (1.0 - viz_ema_alpha) * smoothed_center.x);
                    smoothed_center.y = static_cast<float>(viz_ema_alpha * best_armor.center.y + 
                                                          (1.0 - viz_ema_alpha) * smoothed_center.y);
                    smoothed_size.width = static_cast<float>(viz_ema_alpha * best_armor.width + 
                                                             (1.0 - viz_ema_alpha) * smoothed_size.width);
                    smoothed_size.height = static_cast<float>(viz_ema_alpha * best_armor.height + 
                                                              (1.0 - viz_ema_alpha) * smoothed_size.height);
                }
                
                // 重置丢失帧计数
                lost_frames = 0;
            } else {
                // 未检测到装甲板
                lost_frames++;
                
                // 如果连续丢失太多帧，重置滤波器
                if (lost_frames > max_lost_frames && kf.isInitialized()) {
                    std::cout << "[主程序] 连续丢失 " << lost_frames << " 帧，重置卡尔曼滤波器" << std::endl;
                    kf.reset();
                    visualizer.clearTrajectory();
                    smooth_initialized = false;
                }
            }
            
            // 可视化
            // 1. 绘制所有检测到的装甲板
            visualizer.drawArmors(frame, armors);
            
            // 2. 绘制轨迹
            visualizer.drawInternalTrajectory(frame);
            
            // 3. 绘制预测位置
            if (has_prediction) {
                visualizer.drawPrediction(frame, pred_result.position, best_armor.center);
                
                // 4. 如果使用8维模式，绘制预测尺寸框
                if (kf.isEnhancedMode() && pred_result.size.width > 0) {
                    cv::Rect pred_rect(
                        static_cast<int>(pred_result.position.x - (pred_result.size.width / 2)),
                        static_cast<int>(pred_result.position.y - (pred_result.size.height / 2)),
                        static_cast<int>(pred_result.size.width),
                        static_cast<int>(pred_result.size.height)
                    );
                    cv::rectangle(frame, pred_rect, cv::Scalar(0, 255, 255), 2);
                    
                    // 5. 显示运动方向（判断前后移动，带滞回）
                    if (depth_enabled) {
                        double current_log_area = std::log(best_armor.width * best_armor.height + 1.0);
                        
                        if (!depth_state_initialized) {
                            last_log_area = current_log_area;
                            depth_state_initialized = true;
                        }
                        
                        double area_change_rate = (current_log_area - last_log_area) / (dt_meas + 1e-6);
                        
                        // 滞回逻辑
                        DepthState new_state = depth_state;
                        if (area_change_rate > depth_hysteresis_in) {
                            new_state = DEPTH_APPROACHING;
                        } else if (area_change_rate < -depth_hysteresis_out) {
                            new_state = DEPTH_RETREATING;
                        } else if (std::abs(area_change_rate) < depth_hysteresis_out * 0.5) {
                            new_state = DEPTH_NONE;
                        }
                        
                        // 状态切换需要保持一段时间
                        auto now = std::chrono::high_resolution_clock::now();
                        if (new_state != depth_state) {
                            double elapsed = std::chrono::duration<double>(now - depth_state_change_time).count();
                            if (elapsed >= depth_hold_time) {
                                depth_state = new_state;
                                depth_state_change_time = now;
                            }
                        } else {
                            depth_state_change_time = now;
                        }
                        
                        // 显示状态
                        if (depth_state == DEPTH_APPROACHING) {
                            cv::putText(frame, "APPROACHING", cv::Point(10, 120),
                                       cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
                        } else if (depth_state == DEPTH_RETREATING) {
                            cv::putText(frame, "RETREATING", cv::Point(10, 120),
                                       cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);
                        }
                        
                        last_log_area = current_log_area;
                    }
                }
            }
        }
        
        // 计算FPS和延迟
        auto frame_end_time = std::chrono::high_resolution_clock::now();
        float latency = std::chrono::duration<float, std::milli>(
            frame_end_time - frame_start_time).count();
        
        frame_count++;
        auto current_time = std::chrono::high_resolution_clock::now();
        float elapsed = std::chrono::duration<float>(current_time - last_time).count();
        
        if (elapsed >= 1.0F) {  // 每秒更新一次FPS
            fps = static_cast<float>(frame_count) / elapsed;
            frame_count = 0;
            last_time = current_time;
        }
        
        // 绘制信息面板（复用检测结果，避免重复detect）
        bool has_prediction_display = kf.isInitialized() && lost_frames < max_lost_frames;
        int armor_count = 0;
        if (!paused && !armors.empty()) {
            armor_count = static_cast<int>(armors.size());
        }
        visualizer.drawInfoPanel(frame, fps, latency, armor_count, has_prediction_display);
        
        // 如果暂停，显示暂停标志
        if (paused) {
            cv::putText(frame, "PAUSED", 
                       cv::Point((frame.cols / 2) - 100, frame.rows / 2),
                       cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0, 255, 255), 3);
        }
        
        // 显示图像
        cv::imshow("Armor Detection & Prediction", frame);
        
        // 键盘控制
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {  // 'q' 或 ESC
            std::cout << "[主程序] 用户退出" << std::endl;
            break;
        }
        if (key == 'r') {  // 重置
            std::cout << "[主程序] 重置卡尔曼滤波器" << std::endl;
            kf.reset();
            visualizer.clearTrajectory();
            lost_frames = 0;
        } else if (key == 'c') {  // 清空轨迹
            std::cout << "[主程序] 清空轨迹" << std::endl;
            visualizer.clearTrajectory();
        } else if (key == 'd') {  // 切换调试模式
            bool debug = !detector_config.getBool("detector.debug_mode", false);
            detector.setDebugMode(debug);
            detector_config.loadConfig("detector_params.yaml");  // 重新加载配置
            std::cout << "[主程序] 调试模式: " << (debug ? "开启" : "关闭") << std::endl;
        } else if (key == ' ') {  // 暂停/继续
            paused = !paused;
            std::cout << "[主程序] " << (paused ? "暂停" : "继续") << std::endl;
        }
    }
    
    // ========== 7. 清理资源 ==========
    std::cout << std::endl;
    std::cout << "[主程序] 正在清理资源..." << std::endl;
    camera.stopGrabbing();
    camera.close();
    cv::destroyAllWindows();
    
    std::cout << "[主程序] 程序正常退出" << std::endl;
    std::cout << "========================================" << std::endl;
    
    return 0;
}


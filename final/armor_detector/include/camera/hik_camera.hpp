#ifndef HIK_CAMERA_HPP
#define HIK_CAMERA_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <memory>

// 海康SDK头文件（实际使用时需要安装SDK）
// #include "MvCameraControl.h"

namespace armor_detector {

/**
 * @brief 海康工业相机封装类
 * 提供相机初始化、参数设置、图像采集等功能
 */
class HikCamera {
public:
    HikCamera();
    ~HikCamera();
    
    /**
     * @brief 初始化相机
     * @param exposure_time 曝光时间（微秒）
     * @param gain 增益值
     * @param gamma 伽马值
     * @return 初始化是否成功
     */
    bool init(int exposure_time = 5000, float gain = 10.0f, float gamma = 0.7f);
    
    /**
     * @brief 打开相机设备
     * @param device_index 设备索引（默认0表示第一个相机）
     * @return 打开是否成功
     */
    bool open(int device_index = 0);
    
    /**
     * @brief 关闭相机
     */
    void close();
    
    /**
     * @brief 开始采集图像
     * @return 是否成功开始采集
     */
    bool startGrabbing();
    
    /**
     * @brief 停止采集图像
     * @return 是否成功停止采集
     */
    bool stopGrabbing();
    
    /**
     * @brief 抓取一帧图像
     * @param frame 输出图像
     * @param timestamp 输出时间戳（毫秒）
     * @return 是否成功抓取
     */
    bool grabFrame(cv::Mat& frame, int64_t& timestamp);
    
    /**
     * @brief 设置曝光时间
     * @param exposure_time 曝光时间（微秒）
     * @return 设置是否成功
     */
    bool setExposure(int exposure_time);
    
    /**
     * @brief 设置增益
     * @param gain 增益值
     * @return 设置是否成功
     */
    bool setGain(float gain);
    
    /**
     * @brief 设置伽马值
     * @param gamma 伽马值
     * @return 设置是否成功
     */
    bool setGamma(float gamma);
    
    /**
     * @brief 设置图像分辨率
     * @param width 宽度
     * @param height 高度
     * @return 设置是否成功
     */
    bool setResolution(int width, int height);
    
    /**
     * @brief 获取相机是否已打开
     */
    bool isOpened() const { return is_opened_; }
    
    /**
     * @brief 使用OpenCV VideoCapture作为备用方案（用于没有海康相机时的测试）
     * @param video_path 视频文件路径或摄像头设备号
     * @return 是否成功打开
     */
    bool openVideoCapture(const std::string& video_path);

private:
    bool is_opened_;                    // 相机是否已打开
    bool is_grabbing_;                  // 是否正在采集
    bool use_video_capture_;            // 是否使用VideoCapture备用方案
    
    // 海康SDK相关句柄
    void* camera_handle_;
    
    // OpenCV VideoCapture作为备用方案
    cv::VideoCapture video_capture_;
    
    // 相机参数
    int exposure_time_;                 // 曝光时间
    float gain_;                        // 增益
    float gamma_;                       // 伽马
    int width_;                         // 图像宽度
    int height_;                        // 图像高度
};

} // namespace armor_detector

#endif // HIK_CAMERA_HPP


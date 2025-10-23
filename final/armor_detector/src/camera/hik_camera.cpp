#include "camera/hik_camera.hpp"
#include "MvCameraControl.h"
#include <iostream>
#include <cstring>

namespace armor_detector {

HikCamera::HikCamera() 
    : is_opened_(false),
      is_grabbing_(false),
      use_video_capture_(false),
      camera_handle_(nullptr),
      exposure_time_(5000),
      gain_(10.0f),
      gamma_(0.7f),
      width_(1280),
      height_(1024) {
}

HikCamera::~HikCamera() {
    close();
}

bool HikCamera::init(int exposure_time, float gain, float gamma) {
    exposure_time_ = exposure_time;
    gain_ = gain;
    gamma_ = gamma;
    
    std::cout << "[HikCamera] 初始化相机参数：" << std::endl;
    std::cout << "  曝光时间: " << exposure_time_ << " us" << std::endl;
    std::cout << "  增益: " << gain_ << std::endl;
    std::cout << "  伽马: " << gamma_ << std::endl;
    
    return true;
}

bool HikCamera::open(int device_index) {
    // 1. 枚举设备
    MV_CC_DEVICE_INFO_LIST device_list;
    memset(&device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
    if (ret != MV_OK || device_list.nDeviceNum == 0) {
        std::cerr << "[HikCamera] 未找到相机设备，返回码: " << ret << std::endl;
        std::cerr << "[HikCamera] 提示：检查相机连接（USB3.0或网口）、SDK安装、权限" << std::endl;
        return false;
    }
    
    std::cout << "[HikCamera] 找到 " << device_list.nDeviceNum << " 个相机设备" << std::endl;
    
    if (device_index >= (int)device_list.nDeviceNum) {
        std::cerr << "[HikCamera] 设备索引超出范围" << std::endl;
        return false;
    }
    
    // 2. 创建句柄
    ret = MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[device_index]);
    if (ret != MV_OK) {
        std::cerr << "[HikCamera] 创建相机句柄失败" << std::endl;
        return false;
    }
    
    // 3. 打开设备
    ret = MV_CC_OpenDevice(camera_handle_);
    if (ret != MV_OK) {
        std::cerr << "[HikCamera] 打开相机失败" << std::endl;
        MV_CC_DestroyHandle(camera_handle_);
        camera_handle_ = nullptr;
        return false;
    }
    
    // 4. 设置触发模式为off（连续采集）
    ret = MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0);
    if (ret != MV_OK) {
        std::cerr << "[HikCamera] 设置触发模式失败" << std::endl;
    }
    
    // 5. 设置相机参数
    setExposure(exposure_time_);
    setGain(gain_);
    setGamma(gamma_);
    
    is_opened_ = true;
    std::cout << "[HikCamera] 海康相机打开成功！" << std::endl;
    return true;
}

void HikCamera::close() {
    if (is_grabbing_) {
        stopGrabbing();
    }
    
    if (use_video_capture_ && video_capture_.isOpened()) {
        video_capture_.release();
    }
    
    // 关闭海康相机
    if (is_opened_ && camera_handle_) {
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(camera_handle_);
        camera_handle_ = nullptr;
    }
    
    is_opened_ = false;
    use_video_capture_ = false;
    std::cout << "[HikCamera] 相机已关闭" << std::endl;
}

bool HikCamera::startGrabbing() {
    if (!is_opened_ && !use_video_capture_) {
        std::cerr << "[HikCamera] 相机未打开，无法开始采集" << std::endl;
        return false;
    }
    
    // 使用海康SDK
    if (is_opened_ && camera_handle_) {
        int ret = MV_CC_StartGrabbing(camera_handle_);
        if (ret != MV_OK) {
            std::cerr << "[HikCamera] 开始采集失败" << std::endl;
            return false;
        }
    }
    
    is_grabbing_ = true;
    std::cout << "[HikCamera] 开始采集图像" << std::endl;
    return true;
}

bool HikCamera::stopGrabbing() {
    if (!is_grabbing_) {
        return true;
    }
    
    // 使用海康SDK
    if (is_opened_ && camera_handle_) {
        int ret = MV_CC_StopGrabbing(camera_handle_);
        if (ret != MV_OK) {
            std::cerr << "[HikCamera] 停止采集失败" << std::endl;
            return false;
        }
    }
    
    is_grabbing_ = false;
    std::cout << "[HikCamera] 停止采集图像" << std::endl;
    return true;
}

bool HikCamera::grabFrame(cv::Mat& frame, int64_t& timestamp) {
    // 使用VideoCapture备用方案
    if (use_video_capture_) {
        if (!video_capture_.isOpened()) {
            return false;
        }
        
        if (!video_capture_.read(frame)) {
            return false;
        }
        
        timestamp = cv::getTickCount() * 1000 / cv::getTickFrequency();
        return true;
    }
    
    // 使用海康SDK
    if (!is_grabbing_) {
        return false;
    }
    
    MV_FRAME_OUT frame_info;
    memset(&frame_info, 0, sizeof(MV_FRAME_OUT));
    
    // 获取一帧图像，超时时间1000ms
    int ret = MV_CC_GetImageBuffer(camera_handle_, &frame_info, 1000);
    if (ret != MV_OK) {
        return false;
    }
    
    // 转换为BGR格式（OpenCV标准）
    MV_CC_PIXEL_CONVERT_PARAM convert_param;
    memset(&convert_param, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));
    
    // 创建BGR图像缓冲区
    unsigned int bgr_buf_size = frame_info.stFrameInfo.nWidth * frame_info.stFrameInfo.nHeight * 3;
    unsigned char* bgr_buf = new unsigned char[bgr_buf_size];
    
    // 设置转换参数
    convert_param.nWidth = frame_info.stFrameInfo.nWidth;
    convert_param.nHeight = frame_info.stFrameInfo.nHeight;
    convert_param.pSrcData = frame_info.pBufAddr;
    convert_param.nSrcDataLen = frame_info.stFrameInfo.nFrameLen;
    convert_param.enSrcPixelType = frame_info.stFrameInfo.enPixelType;
    convert_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    convert_param.pDstBuffer = bgr_buf;
    convert_param.nDstBufferSize = bgr_buf_size;
    
    // 执行转换
    ret = MV_CC_ConvertPixelType(camera_handle_, &convert_param);
    if (ret == MV_OK) {
        frame = cv::Mat(frame_info.stFrameInfo.nHeight, 
                       frame_info.stFrameInfo.nWidth, 
                       CV_8UC3, 
                       bgr_buf).clone();
    } else {
        std::cerr << "[HikCamera] 图像格式转换失败" << std::endl;
    }
    
    delete[] bgr_buf;
    
    // 获取时间戳
    timestamp = frame_info.stFrameInfo.nDevTimeStampHigh;
    timestamp = (timestamp << 32) + frame_info.stFrameInfo.nDevTimeStampLow;
    
    // 释放图像缓存
    MV_CC_FreeImageBuffer(camera_handle_, &frame_info);
    
    return ret == MV_OK;
}

bool HikCamera::setExposure(int exposure_time) {
    exposure_time_ = exposure_time;
    
    // 使用海康SDK
    if (is_opened_ && camera_handle_) {
        int ret = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", (float)exposure_time);
        if (ret != MV_OK) {
            std::cerr << "[HikCamera] 设置曝光时间失败，返回码: " << ret << std::endl;
            return false;
        }
    }
    
    std::cout << "[HikCamera] 设置曝光时间: " << exposure_time_ << " us" << std::endl;
    return true;
}

bool HikCamera::setGain(float gain) {
    gain_ = gain;
    
    // 使用海康SDK
    if (is_opened_ && camera_handle_) {
        int ret = MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
        if (ret != MV_OK) {
            std::cerr << "[HikCamera] 设置增益失败，返回码: " << ret << std::endl;
            return false;
        }
    }
    
    std::cout << "[HikCamera] 设置增益: " << gain_ << std::endl;
    return true;
}

bool HikCamera::setGamma(float gamma) {
    gamma_ = gamma;
    
    // 使用海康SDK
    if (is_opened_ && camera_handle_) {
        int ret = MV_CC_SetFloatValue(camera_handle_, "Gamma", gamma);
        if (ret != MV_OK) {
            std::cerr << "[HikCamera] 设置伽马值失败，返回码: " << ret << std::endl;
            return false;
        }
    }
    
    std::cout << "[HikCamera] 设置伽马: " << gamma_ << std::endl;
    return true;
}

bool HikCamera::setResolution(int width, int height) {
    width_ = width;
    height_ = height;
    
    // 使用海康SDK
    if (is_opened_ && camera_handle_) {
        MV_CC_SetIntValue(camera_handle_, "Width", width);
        MV_CC_SetIntValue(camera_handle_, "Height", height);
    }
    
    std::cout << "[HikCamera] 设置分辨率: " << width_ << "x" << height_ << std::endl;
    return true;
}

bool HikCamera::openVideoCapture(const std::string& video_path) {
    std::cout << "[HikCamera] 尝试打开视频源: " << video_path << std::endl;
    
    // 尝试将字符串转换为整数（摄像头设备号）
    try {
        int device_id = std::stoi(video_path);
        std::cout << "[HikCamera] 解析为设备ID: " << device_id << std::endl;
        
        if (!video_capture_.open(device_id)) {
            std::cerr << "[HikCamera] 无法打开摄像头设备 " << device_id << std::endl;
            return false;
        }
        std::cout << "[HikCamera] 成功打开摄像头设备 " << device_id << std::endl;
    } catch (...) {
        // 作为视频文件路径打开
        std::cout << "[HikCamera] 作为视频文件路径打开: " << video_path << std::endl;
        if (!video_capture_.open(video_path)) {
            std::cerr << "[HikCamera] 无法打开视频文件: " << video_path << std::endl;
            return false;
        }
    }
    
    // 设置分辨率
    video_capture_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    video_capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    
    use_video_capture_ = true;
    is_opened_ = true;
    is_grabbing_ = true;
    
    std::cout << "[HikCamera] 使用VideoCapture备用方案: " << video_path << std::endl;
    return true;
}

} // namespace armor_detector


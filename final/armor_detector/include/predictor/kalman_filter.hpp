#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include "common/armor.hpp"

namespace armor_detector {

/**
 * @brief 增强的8维卡尔曼滤波器
 * 
 * 功能升级：
 * - 从4维升级到8维，跟踪装甲板位置+尺寸
 * - 支持预测前后移动（通过尺寸变化）
 * - 自适应能力（渐消记忆，应对非匀速运动）
 * 
 * 状态向量（8维）: [x, y, vx, vy, w, h, vw, vh]
 * - (x, y): 装甲板中心位置（像素）
 * - (vx, vy): 平面速度（像素/秒）
 * - (w, h): 装甲板宽度和高度（像素）
 * - (vw, vh): 尺寸变化速度（像素/秒）- 反映前后移动
 * 
 * 观测向量（4维）: [x, y, w, h]
 * - 观测位置和尺寸，不观测速度
 */
class KalmanFilter {
public:
    /**
     * @brief 构造函数（升级为8维）
     * @param dt 时间步长（秒）
     * @param use_enhanced 是否使用8维增强模式（默认true）
     */
    KalmanFilter(double dt = 0.033, bool use_enhanced = true);
    
    /**
     * @brief 初始化滤波器状态（4维兼容模式）
     * @param initial_state 初始状态 [x, y, vx, vy]
     */
    void init(const Eigen::Vector4d& initial_state);
    
    /**
     * @brief 初始化滤波器（8维增强模式）
     * @param pos 初始位置
     * @param size 初始尺寸
     */
    void init(const cv::Point2f& pos, const cv::Size2f& size);
    
    /**
     * @brief 从观测初始化（速度设为0）- 4维模式
     * @param measurement 观测值 [x, y]
     */
    void initFromMeasurement(const Eigen::Vector2d& measurement);
    
    /**
     * @brief 从观测初始化（8维模式）
     * @param pos 位置
     * @param size 尺寸
     */
    void initFromMeasurement(const cv::Point2f& pos, const cv::Size2f& size);
    
    /**
     * @brief 预测下一时刻的状态
     * @param dt 预测的时间步长（如果不指定，使用构造函数中的dt）
     * @return 预测的状态（动态维度）
     */
    Eigen::VectorXd predict(double dt = -1.0);
    
    /**
     * @brief 更新滤波器（4维观测更新 - 兼容旧接口）
     * @param measurement 观测值 [x, y]
     */
    void update(const Eigen::Vector2d& measurement);
    
    /**
     * @brief 更新滤波器（8维观测更新 - 新接口）
     * @param measured_pos 测量的位置
     * @param measured_size 测量的尺寸
     */
    void update(const cv::Point2f& measured_pos, const cv::Size2f& measured_size);
    
    /**
     * @brief 预测未来某个时刻的位置（简单模式）
     * @param future_time 未来时间（秒）
     * @return 预测的位置 [x, y]
     */
    Eigen::Vector2d predictPosition(double future_time);
    
    /**
     * @brief 预测未来状态（增强模式 - 包含位置和尺寸）
     * @param future_time 未来时间（秒）
     * @return 预测结果（位置、尺寸、速度）
     */
    PredictionResult predictFuture(double future_time);
    
    /**
     * @brief 获取当前状态（作为PredictionResult）
     * @return 当前状态（位置、尺寸、速度）
     */
    PredictionResult getCurrentState();
    
    /**
     * @brief 获取当前状态向量
     * @return 当前状态（动态维度：4维或8维）
     */
    Eigen::VectorXd getState() const { return state_; }
    
    /**
     * @brief 获取当前位置
     * @return 当前位置 [x, y]
     */
    Eigen::Vector2d getPosition() const { return state_.head<2>(); }
    
    /**
     * @brief 获取当前速度
     * @return 当前速度 [vx, vy]
     */
    Eigen::Vector2d getVelocity() const;
    
    /**
     * @brief 获取当前尺寸（仅8维模式）
     * @return 当前尺寸 (w, h)
     */
    cv::Size2f getSize() const;
    
    /**
     * @brief 重置滤波器
     */
    void reset();
    
    /**
     * @brief 设置过程噪声协方差（8维）
     */
    void setProcessNoise(const Eigen::VectorXd& q);
    
    /**
     * @brief 设置测量噪声协方差（4维）
     */
    void setMeasurementNoise(const Eigen::VectorXd& r);
    
    /**
     * @brief 设置渐消因子（应对非匀速运动）
     * @param factor 渐消因子（1.0-1.2，越大越快适应变化）
     */
    void setFadingFactor(double factor) { fading_factor_ = factor; }
    
    /**
     * @brief 启用log域尺寸跟踪
     * @param enable 是否启用
     */
    void setLogScale(bool enable) { use_log_scale_ = enable; }
    
    /**
     * @brief 设置自适应R/Q参数
     * @param enable 是否启用
     * @param k_low 创新量小阈值
     * @param k_high 创新量大阈值
     * @param r_scale_low R缩小系数
     * @param r_scale_high R放大系数
     */
    void setAdaptiveNoise(bool enable, double k_low, double k_high, 
                          double r_scale_low, double r_scale_high);
    
    /**
     * @brief 判断滤波器是否已初始化
     */
    bool isInitialized() const { return is_initialized_; }
    
    /**
     * @brief 判断是否使用增强模式
     */
    bool isEnhancedMode() const { return use_enhanced_; }

private:
    // 状态向量和协方差矩阵（动态维度）
    Eigen::VectorXd state_;                     // 状态向量：4维或8维
    Eigen::MatrixXd covariance_;                // 状态协方差矩阵 P
    
    // 系统矩阵（动态维度）
    Eigen::MatrixXd transition_matrix_;         // 状态转移矩阵 F
    Eigen::MatrixXd observation_matrix_;        // 观测矩阵 H
    
    // 噪声协方差矩阵（动态维度）
    Eigen::MatrixXd process_noise_;             // 过程噪声协方差 Q
    Eigen::MatrixXd measurement_noise_;         // 测量噪声协方差 R
    
    // 参数
    double dt_;                                 // 时间步长
    bool is_initialized_;                       // 是否已初始化
    bool use_enhanced_;                         // 是否使用8维增强模式
    int state_dim_;                             // 状态维度（4或8）
    int measure_dim_;                           // 观测维度（2或4）
    
    // 自适应参数
    double fading_factor_;                      // 渐消因子（1.0-1.2）
    Eigen::VectorXd innovation_;                // 新息向量
    
    // log域尺寸跟踪
    bool use_log_scale_;                        // 是否使用log域
    
    // 自适应R/Q参数
    bool adaptive_noise_enabled_;               // 是否启用自适应
    double k_innovation_low_;                   // 创新量小阈值
    double k_innovation_high_;                  // 创新量大阈值
    double r_scale_low_;                        // R缩小系数
    double r_scale_high_;                       // R放大系数
    Eigen::VectorXd r_baseline_;                // R基准值
    
    /**
     * @brief 更新状态转移矩阵（基于时间步长）
     */
    void updateTransitionMatrix(double dt);
    
    /**
     * @brief 应用渐消记忆（增强对突变的响应）
     */
    void applyFading();
    
    /**
     * @brief 应用自适应噪声调整
     */
    void applyAdaptiveNoise();
};

} // namespace armor_detector

#endif // KALMAN_FILTER_HPP


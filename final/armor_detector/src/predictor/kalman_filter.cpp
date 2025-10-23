#include "predictor/kalman_filter.hpp"
#include <iostream>

namespace armor_detector {

KalmanFilter::KalmanFilter(double dt, bool use_enhanced)
    : dt_(dt), 
      is_initialized_(false),
      use_enhanced_(use_enhanced),
      fading_factor_(1.05),
      use_log_scale_(false),
      adaptive_noise_enabled_(false),
      k_innovation_low_(2.0),
      k_innovation_high_(8.0),
      r_scale_low_(0.7),
      r_scale_high_(1.4) {
    
    // 设置维度
    if (use_enhanced_) {
        state_dim_ = 8;    // [x, y, vx, vy, w, h, vw, vh]
        measure_dim_ = 4;  // [x, y, w, h]
        std::cout << "[KalmanFilter] 使用增强8维模式（跟踪位置+尺寸）" << std::endl;
    } else {
        state_dim_ = 4;    // [x, y, vx, vy]
        measure_dim_ = 2;  // [x, y]
        std::cout << "[KalmanFilter] 使用基础4维模式（仅跟踪位置）" << std::endl;
    }
    
    // 初始化状态向量
    state_ = Eigen::VectorXd::Zero(state_dim_);
    
    // 初始化状态协方差矩阵（初始不确定性较大）
    covariance_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * 1000.0;
    
    // 初始化观测矩阵 H
    observation_matrix_ = Eigen::MatrixXd::Zero(measure_dim_, state_dim_);
    observation_matrix_(0, 0) = 1.0;  // 观测x
    observation_matrix_(1, 1) = 1.0;  // 观测y
    if (use_enhanced_) {
        observation_matrix_(2, 4) = 1.0;  // 观测w
        observation_matrix_(3, 5) = 1.0;  // 观测h
    }
    
    // 设置过程噪声协方差矩阵 Q（对角矩阵）
    process_noise_ = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
    if (use_enhanced_) {
        // 8维：[x, y, vx, vy, w, h, vw, vh]
        process_noise_(0, 0) = 0.5;   // x
        process_noise_(1, 1) = 0.5;   // y
        process_noise_(2, 2) = 2.0;   // vx
        process_noise_(3, 3) = 2.0;   // vy
        process_noise_(4, 4) = 5.0;   // w
        process_noise_(5, 5) = 5.0;   // h
        process_noise_(6, 6) = 10.0;  // vw
        process_noise_(7, 7) = 10.0;  // vh
    } else {
        // 4维：[x, y, vx, vy]
        process_noise_(0, 0) = 0.1;
        process_noise_(1, 1) = 0.1;
        process_noise_(2, 2) = 0.5;
        process_noise_(3, 3) = 0.5;
    }
    
    // 设置测量噪声协方差矩阵 R（对角矩阵）
    measurement_noise_ = Eigen::MatrixXd::Zero(measure_dim_, measure_dim_);
    if (use_enhanced_) {
        // 4维：[x, y, w, h]
        measurement_noise_(0, 0) = 3.0;  // x
        measurement_noise_(1, 1) = 3.0;  // y
        measurement_noise_(2, 2) = 5.0;  // w
        measurement_noise_(3, 3) = 5.0;  // h
    } else {
        // 2维：[x, y]
        measurement_noise_(0, 0) = 5.0;
        measurement_noise_(1, 1) = 5.0;
    }
    
    // 初始化状态转移矩阵
    updateTransitionMatrix(dt_);
    
    // 初始化新息向量
    innovation_ = Eigen::VectorXd::Zero(measure_dim_);
    
    // 保存R基准值（用于自适应）
    r_baseline_ = Eigen::VectorXd::Zero(measure_dim_);
    for (int i = 0; i < measure_dim_; i++) {
        r_baseline_(i) = measurement_noise_(i, i);
    }
    
    std::cout << "[KalmanFilter] 卡尔曼滤波器初始化完成" << std::endl;
    std::cout << "  模式: " << (use_enhanced_ ? "8维增强" : "4维基础") << std::endl;
    std::cout << "  dt = " << dt_ << " s" << std::endl;
    std::cout << "  渐消因子 = " << fading_factor_ << std::endl;
}

void KalmanFilter::init(const Eigen::Vector4d& initial_state) {
    if (use_enhanced_) {
        std::cerr << "[KalmanFilter] 警告：8维模式下应使用 init(pos, size)" << std::endl;
        return;
    }
    
    state_ = initial_state;
    
    // 重置协方差矩阵
    covariance_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * 100.0;
    
    is_initialized_ = true;
    
    std::cout << "[KalmanFilter] 状态初始化: [" << state_.transpose() << "]" << std::endl;
}

void KalmanFilter::init(const cv::Point2f& pos, const cv::Size2f& size) {
    if (!use_enhanced_) {
        std::cerr << "[KalmanFilter] 警告：4维模式不支持尺寸初始化" << std::endl;
        return;
    }
    
    // 8维初始化：[x, y, vx, vy, w/log(w), h/log(h), vw/d_logw, vh/d_logh]
    state_(0) = pos.x;
    state_(1) = pos.y;
    state_(2) = 0.0;         // vx = 0
    state_(3) = 0.0;         // vy = 0
    
    if (use_log_scale_) {
        state_(4) = std::log(std::max(size.width, 1.0F));   // log(w)
        state_(5) = std::log(std::max(size.height, 1.0F));  // log(h)
    } else {
        state_(4) = size.width;
        state_(5) = size.height;
    }
    
    state_(6) = 0.0;         // vw/d_logw = 0
    state_(7) = 0.0;         // vh/d_logh = 0
    
    // 重置协方差矩阵
    covariance_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * 100.0;
    
    is_initialized_ = true;
    
    std::cout << "[KalmanFilter] 8维状态初始化: 位置=[" << pos.x << "," << pos.y 
              << "], 尺寸=[" << size.width << "," << size.height << "]"
              << (use_log_scale_ ? " (log域)" : "") << std::endl;
}

void KalmanFilter::initFromMeasurement(const Eigen::Vector2d& measurement) {
    if (use_enhanced_) {
        std::cerr << "[KalmanFilter] 警告：8维模式下应使用 initFromMeasurement(pos, size)" << std::endl;
        return;
    }
    
    state_(0) = measurement(0);  // x
    state_(1) = measurement(1);  // y
    state_(2) = 0.0;             // vx = 0
    state_(3) = 0.0;             // vy = 0
    
    // 重置协方差矩阵
    covariance_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * 100.0;
    
    is_initialized_ = true;
    
    std::cout << "[KalmanFilter] 从观测初始化: [" << state_.transpose() << "]" << std::endl;
}

void KalmanFilter::initFromMeasurement(const cv::Point2f& pos, const cv::Size2f& size) {
    init(pos, size);
}

void KalmanFilter::updateTransitionMatrix(double dt) {
    // 状态转移矩阵 F (匀速运动模型)
    transition_matrix_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    
    if (use_enhanced_) {
        // 8维模型：
        // x' = x + vx*dt
        // y' = y + vy*dt
        // vx' = vx
        // vy' = vy
        // w' = w + vw*dt  ← 跟踪宽度变化（前后移动）
        // h' = h + vh*dt  ← 跟踪高度变化
        // vw' = vw
        // vh' = vh
        //
        // F = [1 0 dt 0  0  0  0  0 ]
        //     [0 1 0  dt 0  0  0  0 ]
        //     [0 0 1  0  0  0  0  0 ]
        //     [0 0 0  1  0  0  0  0 ]
        //     [0 0 0  0  1  0  dt 0 ]
        //     [0 0 0  0  0  1  0  dt]
        //     [0 0 0  0  0  0  1  0 ]
        //     [0 0 0  0  0  0  0  1 ]
        
        transition_matrix_(0, 2) = dt;  // x += vx * dt
        transition_matrix_(1, 3) = dt;  // y += vy * dt
        transition_matrix_(4, 6) = dt;  // w += vw * dt
        transition_matrix_(5, 7) = dt;  // h += vh * dt
    } else {
        // 4维模型：
        // F = [1  0  dt 0 ]
        //     [0  1  0  dt]
        //     [0  0  1  0 ]
        //     [0  0  0  1 ]
        
        transition_matrix_(0, 2) = dt;  // x += vx * dt
        transition_matrix_(1, 3) = dt;  // y += vy * dt
    }
}

Eigen::VectorXd KalmanFilter::predict(double dt) {
    if (!is_initialized_) {
        std::cerr << "[KalmanFilter] 警告：滤波器未初始化" << std::endl;
        return state_;
    }
    
    // 如果指定了dt，更新状态转移矩阵
    if (dt > 0) {
        updateTransitionMatrix(dt);
    } else {
        updateTransitionMatrix(dt_);
    }
    
    // 应用渐消记忆（处理非匀速运动）
    applyFading();
    
    // 预测状态: x_pred = F * x
    state_ = transition_matrix_ * state_;
    
    // 预测协方差: P_pred = F * P * F^T + Q
    covariance_ = transition_matrix_ * covariance_ * transition_matrix_.transpose() + process_noise_;
    
    return state_;
}

void KalmanFilter::update(const Eigen::Vector2d& measurement) {
    if (use_enhanced_) {
        std::cerr << "[KalmanFilter] 警告：8维模式下应使用 update(pos, size)" << std::endl;
        return;
    }
    
    if (!is_initialized_) {
        // 如果未初始化，直接用观测值初始化
        initFromMeasurement(measurement);
        return;
    }
    
    // 计算观测预测: z_pred = H * x
    Eigen::VectorXd predicted_measurement = observation_matrix_ * state_;
    
    // 计算残差（新息）: y = z - z_pred
    innovation_ = measurement - predicted_measurement;
    
    // 计算残差协方差: S = H * P * H^T + R
    Eigen::MatrixXd innovation_covariance = 
        observation_matrix_ * covariance_ * observation_matrix_.transpose() + measurement_noise_;
    
    // 计算卡尔曼增益: K = P * H^T * S^(-1) - 使用LLT分解
    Eigen::LLT<Eigen::MatrixXd> llt(innovation_covariance);
    Eigen::MatrixXd kalman_gain;
    
    if (llt.info() == Eigen::Success) {
        kalman_gain = covariance_ * observation_matrix_.transpose() * 
                      llt.solve(Eigen::MatrixXd::Identity(measure_dim_, measure_dim_));
    } else {
        // 退化处理：使用伪逆
        kalman_gain = covariance_ * observation_matrix_.transpose() * 
                      innovation_covariance.completeOrthogonalDecomposition().pseudoInverse();
    }
    
    // 更新状态: x = x + K * y
    state_ = state_ + kalman_gain * innovation_;
    
    // 更新协方差: Joseph形式 - P = (I-K*H)*P*(I-K*H)^T + K*R*K^T
    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    Eigen::MatrixXd I_KH = identity - kalman_gain * observation_matrix_;
    covariance_ = I_KH * covariance_ * I_KH.transpose() + 
                  kalman_gain * measurement_noise_ * kalman_gain.transpose();
    
    // 应用自适应噪声调整
    if (adaptive_noise_enabled_) {
        applyAdaptiveNoise();
    }
}

void KalmanFilter::update(const cv::Point2f& measured_pos, const cv::Size2f& measured_size) {
    if (!use_enhanced_) {
        std::cerr << "[KalmanFilter] 警告：4维模式不支持尺寸更新" << std::endl;
        return;
    }
    
    if (!is_initialized_) {
        // 如果未初始化，直接用观测值初始化
        initFromMeasurement(measured_pos, measured_size);
        return;
    }
    
    // 构建观测向量：[x, y, w/log(w), h/log(h)]
    Eigen::VectorXd measurement(4);
    measurement(0) = measured_pos.x;
    measurement(1) = measured_pos.y;
    
    if (use_log_scale_) {
        // log域：观测尺寸取log
        measurement(2) = std::log(std::max(measured_size.width, 1.0F));
        measurement(3) = std::log(std::max(measured_size.height, 1.0F));
    } else {
        measurement(2) = measured_size.width;
        measurement(3) = measured_size.height;
    }
    
    // 计算观测预测: z_pred = H * x
    Eigen::VectorXd predicted_measurement = observation_matrix_ * state_;
    
    // 计算残差（新息）: y = z - z_pred
    innovation_ = measurement - predicted_measurement;
    
    // 计算残差协方差: S = H * P * H^T + R
    Eigen::MatrixXd innovation_covariance = 
        observation_matrix_ * covariance_ * observation_matrix_.transpose() + measurement_noise_;
    
    // 计算卡尔曼增益: K = P * H^T * S^(-1) - 使用LLT分解
    Eigen::LLT<Eigen::MatrixXd> llt(innovation_covariance);
    Eigen::MatrixXd kalman_gain;
    
    if (llt.info() == Eigen::Success) {
        // LLT分解成功，使用solve
        kalman_gain = covariance_ * observation_matrix_.transpose() * 
                      llt.solve(Eigen::MatrixXd::Identity(measure_dim_, measure_dim_));
    } else {
        // 退化处理：使用伪逆
        kalman_gain = covariance_ * observation_matrix_.transpose() * 
                      innovation_covariance.completeOrthogonalDecomposition().pseudoInverse();
    }
    
    // 更新状态: x = x + K * y
    state_ = state_ + kalman_gain * innovation_;
    
    // 更新协方差: Joseph形式 - P = (I-K*H)*P*(I-K*H)^T + K*R*K^T
    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    Eigen::MatrixXd I_KH = identity - kalman_gain * observation_matrix_;
    covariance_ = I_KH * covariance_ * I_KH.transpose() + 
                  kalman_gain * measurement_noise_ * kalman_gain.transpose();
    
    // 应用自适应噪声调整
    if (adaptive_noise_enabled_) {
        applyAdaptiveNoise();
    }
}

Eigen::Vector2d KalmanFilter::predictPosition(double future_time) {
    if (!is_initialized_) {
        std::cerr << "[KalmanFilter] 警告：滤波器未初始化，无法预测" << std::endl;
        return Eigen::Vector2d::Zero();
    }
    
    // 使用匀速运动模型预测未来位置
    // x_future = x_current + vx * future_time
    // y_future = y_current + vy * future_time
    
    Eigen::Vector2d future_position;
    future_position(0) = state_(0) + state_(2) * future_time;  // x + vx * t
    future_position(1) = state_(1) + state_(3) * future_time;  // y + vy * t
    
    return future_position;
}

PredictionResult KalmanFilter::predictFuture(double future_time) {
    PredictionResult result;
    
    if (!is_initialized_) {
        std::cerr << "[KalmanFilter] 警告：滤波器未初始化，无法预测" << std::endl;
        return result;
    }
    
    if (use_enhanced_) {
        // 8维预测：位置 + 尺寸
        result.position.x = state_(0) + state_(2) * future_time;  // x + vx*t
        result.position.y = state_(1) + state_(3) * future_time;  // y + vy*t
        
        if (use_log_scale_) {
            // log域：需要exp恢复
            double log_w_pred = state_(4) + state_(6) * future_time;
            double log_h_pred = state_(5) + state_(7) * future_time;
            result.size.width = std::exp(log_w_pred);
            result.size.height = std::exp(log_h_pred);
        } else {
            result.size.width = state_(4) + state_(6) * future_time;
            result.size.height = state_(5) + state_(7) * future_time;
        }
        
        result.velocity.x = state_(2);
        result.velocity.y = state_(3);
        result.size_velocity.x = state_(6);  // vw或d_logw
        result.size_velocity.y = state_(7);  // vh或d_logh
        
        result.confidence = 1.0f;
    } else {
        // 4维预测：仅位置
        result.position.x = state_(0) + state_(2) * future_time;
        result.position.y = state_(1) + state_(3) * future_time;
        result.velocity.x = state_(2);
        result.velocity.y = state_(3);
        result.size = cv::Size2f(0, 0);  // 无尺寸信息
        result.confidence = 0.5f;
    }
    
    return result;
}

PredictionResult KalmanFilter::getCurrentState() {
    PredictionResult result;
    
    if (!is_initialized_) {
        return result;
    }
    
    result.position.x = state_(0);
    result.position.y = state_(1);
    result.velocity.x = state_(2);
    result.velocity.y = state_(3);
    
    if (use_enhanced_) {
        if (use_log_scale_) {
            // log域：需要exp恢复
            result.size.width = std::exp(state_(4));
            result.size.height = std::exp(state_(5));
        } else {
            result.size.width = state_(4);
            result.size.height = state_(5);
        }
        result.size_velocity.x = state_(6);
        result.size_velocity.y = state_(7);
    }
    
    return result;
}

Eigen::Vector2d KalmanFilter::getVelocity() const {
    if (state_dim_ >= 4) {
        return Eigen::Vector2d(state_(2), state_(3));
    }
    return Eigen::Vector2d::Zero();
}

cv::Size2f KalmanFilter::getSize() const {
    if (use_enhanced_ && state_dim_ >= 6) {
        if (use_log_scale_) {
            return cv::Size2f(std::exp(state_(4)), std::exp(state_(5)));
        }
        return cv::Size2f(state_(4), state_(5));
    }
    return cv::Size2f(0, 0);
}

void KalmanFilter::reset() {
    state_ = Eigen::VectorXd::Zero(state_dim_);
    covariance_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * 1000.0;
    innovation_ = Eigen::VectorXd::Zero(measure_dim_);
    is_initialized_ = false;
    
    std::cout << "[KalmanFilter] 滤波器已重置" << std::endl;
}

void KalmanFilter::setProcessNoise(const Eigen::VectorXd& q) {
    if (q.size() != state_dim_) {
        std::cerr << "[KalmanFilter] 警告：过程噪声维度不匹配" << std::endl;
        return;
    }
    
    for (int i = 0; i < state_dim_; i++) {
        process_noise_(i, i) = q(i);
    }
    
    std::cout << "[KalmanFilter] 更新过程噪声: [" << q.transpose() << "]" << std::endl;
}

void KalmanFilter::setMeasurementNoise(const Eigen::VectorXd& r) {
    if (r.size() != measure_dim_) {
        std::cerr << "[KalmanFilter] 警告：测量噪声维度不匹配" << std::endl;
        return;
    }
    
    for (int i = 0; i < measure_dim_; i++) {
        measurement_noise_(i, i) = r(i);
    }
    
    std::cout << "[KalmanFilter] 更新测量噪声: [" << r.transpose() << "]" << std::endl;
}

void KalmanFilter::applyFading() {
    // 渐消记忆滤波：P = fading_factor * P
    // 增大协方差，减小对历史数据的依赖，快速响应突变
    if (fading_factor_ > 1.0) {
        covariance_ *= fading_factor_;
    }
}

void KalmanFilter::applyAdaptiveNoise() {
    // 基于创新量大小自适应调整R
    double innovation_norm = innovation_.norm();
    
    if (innovation_norm < k_innovation_low_) {
        // 观测与预测接近，观测可靠，减小R（更信任观测）
        for (int i = 0; i < measure_dim_; i++) {
            measurement_noise_(i, i) *= r_scale_low_;
            // 限制下界
            measurement_noise_(i, i) = std::max(measurement_noise_(i, i), r_baseline_(i) * 0.3);
        }
    } else if (innovation_norm > k_innovation_high_) {
        // 观测与预测偏差大，观测可能异常，增大R（更信任模型）
        for (int i = 0; i < measure_dim_; i++) {
            measurement_noise_(i, i) *= r_scale_high_;
            // 限制上界
            measurement_noise_(i, i) = std::min(measurement_noise_(i, i), r_baseline_(i) * 3.0);
        }
    } else {
        // 中间区域，逐渐恢复到基准值
        for (int i = 0; i < measure_dim_; i++) {
            double diff = r_baseline_(i) - measurement_noise_(i, i);
            measurement_noise_(i, i) += diff * 0.1;  // 缓慢恢复
        }
    }
}

void KalmanFilter::setAdaptiveNoise(bool enable, double k_low, double k_high, 
                                     double r_scale_low, double r_scale_high) {
    adaptive_noise_enabled_ = enable;
    k_innovation_low_ = k_low;
    k_innovation_high_ = k_high;
    r_scale_low_ = r_scale_low;
    r_scale_high_ = r_scale_high;
    
    std::cout << "[KalmanFilter] 自适应噪声: " << (enable ? "启用" : "禁用") << std::endl;
    if (enable) {
        std::cout << "  创新量阈值: [" << k_low << ", " << k_high << "]" << std::endl;
        std::cout << "  R缩放: [" << r_scale_low << ", " << r_scale_high << "]" << std::endl;
    }
}

} // namespace armor_detector


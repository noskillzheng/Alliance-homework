#include "core/polynomial_trajectory.hpp"
#include <stdexcept>

namespace trajectory {

PolynomialTrajectory::PolynomialTrajectory(const Eigen::VectorXd& coefficients, 
                                          double duration)
    : coefficients_(coefficients), duration_(duration) {
    if (coefficients.size() != 6) {
        throw std::invalid_argument("Coefficients must have size 6 for 5th order polynomial");
    }
    if (duration <= 0) {
        throw std::invalid_argument("Duration must be positive");
    }
}

double PolynomialTrajectory::position(double t) const {
    if (t < 0 || t > duration_) {
        t = std::max(0.0, std::min(t, duration_));
    }
    
    // p(t) = a5*t^5 + a4*t^4 + a3*t^3 + a2*t^2 + a1*t + a0
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;
    
    return coefficients_[0] * t5 + 
           coefficients_[1] * t4 + 
           coefficients_[2] * t3 + 
           coefficients_[3] * t2 + 
           coefficients_[4] * t + 
           coefficients_[5];
}

double PolynomialTrajectory::velocity(double t) const {
    if (t < 0 || t > duration_) {
        t = std::max(0.0, std::min(t, duration_));
    }
    
    // v(t) = 5*a5*t^4 + 4*a4*t^3 + 3*a3*t^2 + 2*a2*t + a1
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    
    return 5.0 * coefficients_[0] * t4 + 
           4.0 * coefficients_[1] * t3 + 
           3.0 * coefficients_[2] * t2 + 
           2.0 * coefficients_[3] * t + 
           coefficients_[4];
}

double PolynomialTrajectory::acceleration(double t) const {
    if (t < 0 || t > duration_) {
        t = std::max(0.0, std::min(t, duration_));
    }
    
    // a(t) = 20*a5*t^3 + 12*a4*t^2 + 6*a3*t + 2*a2
    double t2 = t * t;
    double t3 = t2 * t;
    
    return 20.0 * coefficients_[0] * t3 + 
           12.0 * coefficients_[1] * t2 + 
           6.0 * coefficients_[2] * t + 
           2.0 * coefficients_[3];
}

double PolynomialTrajectory::jerk(double t) const {
    if (t < 0 || t > duration_) {
        t = std::max(0.0, std::min(t, duration_));
    }
    
    // j(t) = 60*a5*t^2 + 24*a4*t + 6*a3
    double t2 = t * t;
    
    return 60.0 * coefficients_[0] * t2 + 
           24.0 * coefficients_[1] * t + 
           6.0 * coefficients_[2];
}

double PolynomialTrajectory::computeJerkSquareIntegral() const {
    // ∫[0,T] jerk^2 dt = ∫[0,T] (60*a5*t^2 + 24*a4*t + 6*a3)^2 dt
    // 展开并积分：
    // = 720*a5^2*T^5 + 720*a5*a4*T^4 + 192*a4^2*T^3 + 240*a5*a3*T^3 + 144*a4*a3*T^2 + 36*a3^2*T
    
    double a5 = coefficients_[0];
    double a4 = coefficients_[1];
    double a3 = coefficients_[2];
    double T = duration_;
    
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;
    
    return 720.0 * a5 * a5 * T5 + 
           720.0 * a5 * a4 * T4 + 
           192.0 * a4 * a4 * T3 + 
           240.0 * a5 * a3 * T3 + 
           144.0 * a4 * a3 * T2 + 
           36.0 * a3 * a3 * T;
}

double PolynomialTrajectory::getVelocityExtremum() const {
    // v(t) = 5*a5*t^4 + 4*a4*t^3 + 3*a3*t^2 + 2*a2*t + a1
    // v'(t) = 20*a5*t^3 + 12*a4*t^2 + 6*a3*t + 2*a2 = 0
    // 求根找极值点，检查端点和内部极值
    
    double max_v = std::max(std::abs(velocity(0.0)), std::abs(velocity(duration_)));
    
    // 采样检查（简化实现，可用牛顿法精确求根）
    for (int i = 1; i < 100; ++i) {
        double t = (i / 100.0) * duration_;
        max_v = std::max(max_v, std::abs(velocity(t)));
    }
    
    return max_v;
}

double PolynomialTrajectory::getAccelerationExtremum() const {
    // a(t) = 20*a5*t^3 + 12*a4*t^2 + 6*a3*t + 2*a2
    // a'(t) = 60*a5*t^2 + 24*a4*t + 6*a3 = 0
    
    double max_a = std::max(std::abs(acceleration(0.0)), std::abs(acceleration(duration_)));
    
    // 解二次方程求极值点
    double A = 60.0 * coefficients_[0];
    double B = 24.0 * coefficients_[1];
    double C = 6.0 * coefficients_[2];
    
    if (std::abs(A) > 1e-10) {
        double discriminant = B * B - 4.0 * A * C;
        if (discriminant >= 0) {
            double sqrt_disc = std::sqrt(discriminant);
            double t1 = (-B + sqrt_disc) / (2.0 * A);
            double t2 = (-B - sqrt_disc) / (2.0 * A);
            
            if (t1 >= 0 && t1 <= duration_) {
                max_a = std::max(max_a, std::abs(acceleration(t1)));
            }
            if (t2 >= 0 && t2 <= duration_) {
                max_a = std::max(max_a, std::abs(acceleration(t2)));
            }
        }
    }
    
    return max_a;
}

} // namespace trajectory


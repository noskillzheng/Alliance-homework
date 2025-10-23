#ifndef POLYNOMIAL_TRAJECTORY_HPP
#define POLYNOMIAL_TRAJECTORY_HPP

#include <Eigen/Dense>
#include <vector>
#include <cmath>

namespace trajectory {

/**
 * @brief 多项式轨迹类
 * 
 * 使用5阶多项式表示轨迹段（Quintic Polynomial）：
 * T(t) = a5*t^5 + a4*t^4 + a3*t^3 + a2*t^2 + a1*t + a0
 * 
 * 5阶多项式有6个系数，恰好满足6个边界条件：
 * p(0), v(0), a(0), p(T), v(T), a(T)
 * 从而保证跨段 C^2 连续（位置、速度、加速度连续）
 */
class PolynomialTrajectory {
public:
    /**
     * @brief 构造函数
     * @param coefficients 多项式系数向量 [a5, a4, a3, a2, a1, a0]
     * @param duration 该段轨迹的持续时间
     */
    PolynomialTrajectory(const Eigen::VectorXd& coefficients, double duration);
    
    /**
     * @brief 默认构造函数
     */
    PolynomialTrajectory() = default;
    
    /**
     * @brief 计算位置
     * @param t 时间 [0, duration]
     * @return 位置值
     */
    double position(double t) const;
    
    /**
     * @brief 计算速度（一阶导数）
     * @param t 时间 [0, duration]
     * @return 速度值
     */
    double velocity(double t) const;
    
    /**
     * @brief 计算加速度（二阶导数）
     * @param t 时间 [0, duration]
     * @return 加速度值
     */
    double acceleration(double t) const;
    
    /**
     * @brief 计算加加速度/jerk（三阶导数）
     * @param t 时间 [0, duration]
     * @return jerk值
     */
    double jerk(double t) const;
    
    /**
     * @brief 获取轨迹持续时间
     */
    double getDuration() const { return duration_; }
    
    /**
     * @brief 获取多项式系数
     */
    const Eigen::VectorXd& getCoefficients() const { return coefficients_; }
    
    /**
     * @brief 计算该段轨迹的jerk的平方积分（优化目标）
     * @return ∫(jerk^2)dt
     */
    double computeJerkSquareIntegral() const;
    
    /**
     * @brief 计算速度的极值点（用于精确约束检查）
     * @return 速度极值（可能为最大或最小）
     */
    double getVelocityExtremum() const;
    
    /**
     * @brief 计算加速度的极值点（用于精确约束检查）
     * @return 加速度极值
     */
    double getAccelerationExtremum() const;

private:
    Eigen::VectorXd coefficients_;  // [a5, a4, a3, a2, a1, a0]
    double duration_;               // 该段轨迹持续时间
};

/**
 * @brief 二维轨迹（x和y方向的多项式轨迹组合）
 */
struct Trajectory2D {
    PolynomialTrajectory x_traj;  // x方向轨迹
    PolynomialTrajectory y_traj;  // y方向轨迹
    
    /**
     * @brief 获取二维位置
     */
    Eigen::Vector2d position(double t) const {
        return Eigen::Vector2d(x_traj.position(t), y_traj.position(t));
    }
    
    /**
     * @brief 获取二维速度
     */
    Eigen::Vector2d velocity(double t) const {
        return Eigen::Vector2d(x_traj.velocity(t), y_traj.velocity(t));
    }
    
    /**
     * @brief 获取二维加速度
     */
    Eigen::Vector2d acceleration(double t) const {
        return Eigen::Vector2d(x_traj.acceleration(t), y_traj.acceleration(t));
    }
    
    /**
     * @brief 计算速度的模
     */
    double speed(double t) const {
        return velocity(t).norm();
    }
    
    /**
     * @brief 计算加速度的模
     */
    double accelerationMagnitude(double t) const {
        return acceleration(t).norm();
    }
};

} // namespace trajectory

#endif // POLYNOMIAL_TRAJECTORY_HPP


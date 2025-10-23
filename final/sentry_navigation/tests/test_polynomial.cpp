#include "core/polynomial_trajectory.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace trajectory;

// 简单的单元测试框架
#define TEST(name) void test_##name()
#define ASSERT_NEAR(a, b, eps) \
    if (std::abs((a) - (b)) > eps) { \
        std::cerr << "FAIL: " << #a << " (" << (a) << ") != " << #b << " (" << (b) << ")\n"; \
        std::exit(1); \
    }
#define ASSERT_TRUE(cond) \
    if (!(cond)) { \
        std::cerr << "FAIL: " << #cond << " is false\n"; \
        std::exit(1); \
    }

TEST(quintic_boundary_conditions) {
    // 测试5阶多项式是否满足边界条件
    // p(0)=0, v(0)=0, a(0)=0, p(1)=1, v(1)=0, a(1)=0
    
    Eigen::VectorXd coeffs(6);
    // 手工计算的系数：10t^3 - 15t^4 + 6t^5
    coeffs << 6, -15, 10, 0, 0, 0;
    
    PolynomialTrajectory traj(coeffs, 1.0);
    
    ASSERT_NEAR(traj.position(0.0), 0.0, 1e-10);
    ASSERT_NEAR(traj.velocity(0.0), 0.0, 1e-10);
    ASSERT_NEAR(traj.acceleration(0.0), 0.0, 1e-10);
    ASSERT_NEAR(traj.position(1.0), 1.0, 1e-10);
    ASSERT_NEAR(traj.velocity(1.0), 0.0, 1e-10);
    ASSERT_NEAR(traj.acceleration(1.0), 0.0, 1e-10);
    
    std::cout << "✓ test_quintic_boundary_conditions passed\n";
}

TEST(jerk_integral) {
    // 测试jerk积分计算
    Eigen::VectorXd coeffs(6);
    coeffs << 1, 0, 0, 0, 0, 0;  // p(t) = t^5
    
    PolynomialTrajectory traj(coeffs, 1.0);
    
    // j(t) = 60*t^2, ∫j^2 dt = ∫3600*t^4 dt = 720*t^5 = 720
    double jerk_integral = traj.computeJerkSquareIntegral();
    ASSERT_NEAR(jerk_integral, 720.0, 1e-6);
    
    std::cout << "✓ test_jerk_integral passed\n";
}

TEST(extremum_detection) {
    // 测试极值点检测
    Eigen::VectorXd coeffs(6);
    coeffs << 6, -15, 10, 0, 0, 0;
    
    PolynomialTrajectory traj(coeffs, 1.0);
    
    double v_max = traj.getVelocityExtremum();
    double a_max = traj.getAccelerationExtremum();
    
    ASSERT_TRUE(v_max > 0);
    ASSERT_TRUE(a_max > 0);
    ASSERT_TRUE(v_max < 3.0);  // 合理范围检查
    ASSERT_TRUE(a_max < 20.0);
    
    std::cout << "✓ test_extremum_detection passed (v_max=" << v_max 
              << ", a_max=" << a_max << ")\n";
}

int main() {
    std::cout << "\n========== 运行单元测试 ==========\n";
    
    test_quintic_boundary_conditions();
    test_jerk_integral();
    test_extremum_detection();
    
    std::cout << "\n所有测试通过 ✓\n";
    std::cout << "==================================\n\n";
    
    return 0;
}


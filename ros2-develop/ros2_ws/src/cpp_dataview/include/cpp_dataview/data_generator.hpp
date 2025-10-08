#ifndef CPP_DATAVIEW__DATA_GENERATOR_HPP_
#define CPP_DATAVIEW__DATA_GENERATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class SignalGenerator : public rclcpp::Node {
public:
    SignalGenerator();

private:
    void timer_callback();
    
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sine_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr square_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
};

#endif  // CPP_DATAVIEW__DATA_GENERATOR_HPP_

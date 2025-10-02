#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>

using namespace std::chrono_literals;

class SignalGenerator : public rclcpp::Node {
public:
    SignalGenerator() : Node("signal_generator"), start_time_(this->now()) {
        // 创建发布者 (500Hz频率)
        sine_pub_ = this->create_publisher<std_msgs::msg::Float32>("sine_wave", 10);
        square_pub_ = this->create_publisher<std_msgs::msg::Float32>("square_wave", 10);
        
        // 500Hz定时器 (2ms间隔)
        timer_ = this->create_wall_timer(2ms, std::bind(&SignalGenerator::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "信号发生器节点已启动 (500Hz)");
    }

private:
    void timer_callback() {
        auto current_time = (this->now() - start_time_).seconds();
        
        // 生成10Hz正弦波
        auto sine_msg = std_msgs::msg::Float32();
        sine_msg.data = std::sin(2 * M_PI * 10 * current_time);
        sine_pub_->publish(sine_msg);
        
        // 生成1Hz方波
        auto square_msg = std_msgs::msg::Float32();
        square_msg.data = (std::fmod(current_time, 1.0) < 0.5) ? 1.0 : -1.0;
        square_pub_->publish(square_msg);
    }
    
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sine_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr square_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalGenerator>());
    rclcpp::shutdown();
    return 0;
}

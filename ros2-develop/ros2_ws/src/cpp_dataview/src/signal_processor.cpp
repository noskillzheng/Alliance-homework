#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class SignalProcessor : public rclcpp::Node {
public:
    SignalProcessor() : Node("signal_processor") {
        // 订阅信号
        sine_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "sine_wave", 10, std::bind(&SignalProcessor::sine_callback, this, std::placeholders::_1));
        
        square_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "square_wave", 10, std::bind(&SignalProcessor::square_callback, this, std::placeholders::_1));
        
        // 创建结果发布者
        output_pub_ = this->create_publisher<std_msgs::msg::Float32>("processed_signal", 10);
        
        RCLCPP_INFO(this->get_logger(), "信号处理器节点已启动");
    }

private:
    void sine_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        latest_sine_ = msg->data;
        process_signals();
    }

    void square_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        latest_square_ = msg->data;
        process_signals();
    }

    void process_signals() {
        auto output_msg = std_msgs::msg::Float32();
        
        // 同号检测：正弦和方波符号相同
        if (latest_sine_ * latest_square_ >= 0) {
            output_msg.data = latest_sine_;
        } else {
            output_msg.data = 0.0;
        }
        
        output_pub_->publish(output_msg);
    }
    
    float latest_sine_ = 0.0;
    float latest_square_ = 1.0;  // 初始化为正数确保初始同号
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sine_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr square_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr output_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalProcessor>());
    rclcpp::shutdown();
    return 0;
}

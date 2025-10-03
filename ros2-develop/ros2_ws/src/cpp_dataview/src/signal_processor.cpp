
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace message_filters;

class SignalProcessor : public rclcpp::Node {
public:
    SignalProcessor() : Node("signal_processor") {
        processed_pub_ = this->create_publisher<std_msgs::msg::Float32>("/processed_signal", 10);
        
        // 创建同步订阅器
        sine_sub_.subscribe(this, "/sine_wave");
        square_sub_.subscribe(this, "/square_wave");
        
        // 配置时间同步策略 (10ms容差)
        sync_.reset(new Sync(ApproxPolicy(10), sine_sub_, square_sub_));
        sync_->registerCallback(&SignalProcessor::signal_callback, this);
        
        RCLCPP_INFO(this->get_logger(), "信号处理器节点已启动");
    }

private:
    void signal_callback(
        const std_msgs::msg::Float32::ConstSharedPtr& sine_msg,
        const std_msgs::msg::Float32::ConstSharedPtr& square_msg) {
        
        auto output_msg = std_msgs::msg::Float32();
        
        // 同号检测逻辑
        if (sine_msg->data * square_msg->data >= 0) {
            output_msg.data = sine_msg->data;
        } else {
            output_msg.data = 0.0f;
        }
        
        processed_pub_->publish(output_msg);
    }
    
    typedef sync_policies::ApproximateTime<std_msgs::msg::Float32, std_msgs::msg::Float32> ApproxPolicy;
    typedef Synchronizer<ApproxPolicy> Sync;
    std::shared_ptr<Sync> sync_;
    
    Subscriber<std_msgs::msg::Float32> sine_sub_;
    Subscriber<std_msgs::msg::Float32> square_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr processed_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalProcessor>());
    rclcpp::shutdown();
    return 0;
}


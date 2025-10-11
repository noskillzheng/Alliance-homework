#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class LowPassFilterNode : public rclcpp::Node
{
public:
  LowPassFilterNode()
  : Node("lowpass_filter_node"), last_filtered_value_(0.0), is_first_(true)
  {
    this->declare_parameter<double>("alpha", 0.1);
    alpha_ = this->get_parameter("alpha").as_double();

    subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "/noisy_data", 10, std::bind(&LowPassFilterNode::topic_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("/lowpass_filtered_data", 10);
  }

private:
  void topic_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    double current_value = msg->data;
    double filtered_value;

    if (is_first_) {
      filtered_value = current_value;
      is_first_ = false;
    } else {
      filtered_value = alpha_ * current_value + (1.0 - alpha_) * last_filtered_value_;
    }
    
    last_filtered_value_ = filtered_value;
    
    auto filtered_msg = std_msgs::msg::Float64();
    filtered_msg.data = filtered_value;
    publisher_->publish(filtered_msg);
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  double alpha_;
  double last_filtered_value_;
  bool is_first_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LowPassFilterNode>());
  rclcpp::shutdown();
  return 0;
}

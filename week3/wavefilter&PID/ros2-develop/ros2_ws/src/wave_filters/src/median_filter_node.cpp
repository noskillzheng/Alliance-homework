#include <memory>
#include <vector>
#include <algorithm>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class MedianFilterNode : public rclcpp::Node
{
public:
  MedianFilterNode()
  : Node("median_filter_node")
  {
    this->declare_parameter<int>("window_size", 5);
    window_size_ = this->get_parameter("window_size").as_int();

    subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "/noisy_data", 10, std::bind(&MedianFilterNode::topic_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("/median_filtered_data", 10);
  }

private:
  void topic_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    window_.push_back(msg->data);

    if (window_.size() > window_size_) {
      window_.erase(window_.begin());
    }

    if (window_.size() == window_size_) {
      std::vector<double> sorted_window = window_;
      std::sort(sorted_window.begin(), sorted_window.end());
      
      double median;
      if (window_size_ % 2 == 0) {
        median = (sorted_window[window_size_ / 2 - 1] + sorted_window[window_size_ / 2]) / 2.0;
      } else {
        median = sorted_window[window_size_ / 2];
      }
      
      auto median_msg = std_msgs::msg::Float64();
      median_msg.data = median;
      publisher_->publish(median_msg);
    }
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  std::vector<double> window_;
  size_t window_size_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MedianFilterNode>());
  rclcpp::shutdown();
  return 0;
}

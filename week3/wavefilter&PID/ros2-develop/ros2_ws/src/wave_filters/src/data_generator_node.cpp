#include <chrono>
#include <memory>
#include <cmath>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class DataGeneratorNode : public rclcpp::Node
{
public:
  DataGeneratorNode()
  : Node("data_generator_node"), time_(0.0)
  {
    this->declare_parameter<double>("frequency", 0.5);
    this->declare_parameter<double>("amplitude", 2.0);
    this->declare_parameter<double>("noise_std_dev", 0.3);

    frequency_ = this->get_parameter("frequency").as_double();
    amplitude_ = this->get_parameter("amplitude").as_double();
    noise_std_dev_ = this->get_parameter("noise_std_dev").as_double();

    publisher_ = this->create_publisher<std_msgs::msg::Float64>("/noisy_data", 10);
    timer_ = this->create_wall_timer(
      50ms, std::bind(&DataGeneratorNode::timer_callback, this));

    std::random_device rd;
    rng_ = std::mt19937(rd());
    dist_ = std::normal_distribution<double>(0.0, noise_std_dev_);
  }

private:
  void timer_callback()
  {
    auto msg = std_msgs::msg::Float64();
    double signal = amplitude_ * sin(2 * M_PI * frequency_ * time_);
    double noise = dist_(rng_);
    msg.data = signal + noise;
    
    publisher_->publish(msg);
    
    time_ += 0.05; // Corresponds to 50ms timer interval
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  double time_;
  double frequency_;
  double amplitude_;
  double noise_std_dev_;
  std::mt19937 rng_;
  std::normal_distribution<double> dist_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}

#ifndef CPP_DATAVIEW__SIGNAL_PROCESSOR_HPP_
#define CPP_DATAVIEW__SIGNAL_PROCESSOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace message_filters;

class SignalProcessor : public rclcpp::Node {
public:
    SignalProcessor();

private:
    void signal_callback(
        const std_msgs::msg::Float32::ConstSharedPtr& sine_msg,
        const std_msgs::msg::Float32::ConstSharedPtr& square_msg);
    
    typedef sync_policies::ApproximateTime<std_msgs::msg::Float32, std_msgs::msg::Float32> ApproxPolicy;
    typedef Synchronizer<ApproxPolicy> Sync;
    std::shared_ptr<Sync> sync_;
    
    Subscriber<std_msgs::msg::Float32> sine_sub_;
    Subscriber<std_msgs::msg::Float32> square_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr processed_pub_;
};

#endif  // CPP_DATAVIEW__SIGNAL_PROCESSOR_HPP_

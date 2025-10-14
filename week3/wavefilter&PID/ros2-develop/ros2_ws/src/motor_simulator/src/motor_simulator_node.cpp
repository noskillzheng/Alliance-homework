#include <chrono>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include "motor_simulator/motor_model.hpp"
#include "std_msgs/msg/float64.hpp"

namespace motor_simulator {
class MotorSimulatorNode : public rclcpp::Node {
public:
    MotorSimulatorNode()
        : Node("motor_simulator") {

        subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "motor_simulator_control_input", 10,
            std::bind(&MotorSimulatorNode::subscriber_callback, this, std::placeholders::_1));

        angle_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>("simulated_motor_angle", 10);
        velocity_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>("simulated_motor_velocity", 10);
        torque_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>("simulated_motor_torque", 10);

        timer_ = this->create_wall_timer(
            std::chrono::microseconds(1000), std::bind(&MotorSimulatorNode::data_publish, this));
        RCLCPP_INFO(get_logger(), "motor simualtor launch");
    }

private:
    void data_publish() {
        motor_reset_judge();

        std_msgs::msg::Float64 angle_msg;
        angle_msg.set__data(motor_model_.get_angle());
        angle_publisher_->publish(angle_msg);

        std_msgs::msg::Float64 velocity_msg;
        velocity_msg.set__data(motor_model_.get_velocity());
        velocity_publisher_->publish(velocity_msg);

        std_msgs::msg::Float64 torque_msg;
        torque_msg.set__data(motor_model_.get_torque());
        torque_publisher_->publish(torque_msg);
    }

    void subscriber_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        last_control_time_point_ = std::chrono::steady_clock::now();
        motor_model_.set_control_torque(msg->data);
        motor_model_.update_status();
    }

    void motor_reset_judge() {
        auto delta_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - last_control_time_point_);
        if (delta_time.count() > 3000) {
            motor_model_.reset_states();
        }
    }

    MotorBaseModel motor_model_{1e-3, 2e-3, 1.5, 0.00001};

    std::chrono::steady_clock::time_point last_control_time_point_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr torque_publisher_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_;
};
} // namespace motor_simulator

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<motor_simulator::MotorSimulatorNode>());

    rclcpp::shutdown();
    return 0;
}
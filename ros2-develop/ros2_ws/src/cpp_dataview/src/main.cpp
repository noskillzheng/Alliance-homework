#include "cpp_dataview/data_generator.hpp"
#include "cpp_dataview/signal_processor.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::executors::SingleThreadedExecutor executor;
    
    auto generator_node = std::make_shared<SignalGenerator>();
    auto processor_node = std::make_shared<SignalProcessor>();
    
    executor.add_node(generator_node);
    executor.add_node(processor_node);
    
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}

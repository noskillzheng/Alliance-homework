cmake_minimum_required(VERSION 3.8)
project(cpp_dataview)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(signal_generator src/signal_generator.cpp)
add_executable(signal_processor src/signal_processor.cpp)

ament_target_dependencies(signal_generator rclcpp std_msgs)
ament_target_dependencies(signal_processor rclcpp std_msgs)

install(TARGETS
  signal_generator
  signal_processor
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()

cmake_minimum_required(VERSION 3.10)

project(cpp_dataview)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)

add_executable(data_generator src/data_generator.cpp)
ament_target_dependencies(data_generator rclcpp std_msgs)

add_executable(signal_processor src/signal_processor.cpp)
ament_target_dependencies(signal_processor rclcpp std_msgs message_filters)

install(TARGETS
  data_generator
  signal_processor
  DESTINATION lib/${PROJECT_NAME}
)


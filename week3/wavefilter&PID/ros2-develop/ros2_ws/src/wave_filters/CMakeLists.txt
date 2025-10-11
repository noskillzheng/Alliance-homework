cmake_minimum_required(VERSION 3.8)
project(wave_filters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(data_generator_node src/data_generator_node.cpp)
ament_target_dependencies(data_generator_node rclcpp std_msgs)

add_executable(median_filter_node src/median_filter_node.cpp)
ament_target_dependencies(median_filter_node rclcpp std_msgs)

add_executable(lowpass_filter_node src/lowpass_filter_node.cpp)
ament_target_dependencies(lowpass_filter_node rclcpp std_msgs)

install(TARGETS
  data_generator_node
  median_filter_node
  lowpass_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

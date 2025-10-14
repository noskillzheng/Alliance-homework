cmake_minimum_required(VERSION 3.10)
project(ArmorDetection)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置可执行文件输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)

# 添加编译选项，为clangd生成compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找OpenCV
find_package(OpenCV REQUIRED)

# 包含头文件目录
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

# 为clangd设置编译标志
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# 1. 传统视觉版本（仅OpenCV，支持图片）
add_executable(armor_detection_traditional
    src/main_traditional.cpp
    src/armor_detector_traditional.cpp
)

# 为传统视觉版本设置编译选项
target_compile_options(armor_detection_traditional PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra>
)

# 链接库
target_link_libraries(armor_detection_traditional
    ${OpenCV_LIBS}
)

# 2. YOLO视觉版本（基于深度学习，支持图片和视频）
add_executable(armor_detection_yolo
    src/main_cmd.cpp
    src/armor_detector_cmd.cpp
)

# 为YOLO版本设置编译选项
target_compile_options(armor_detection_yolo PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra>
)

# 链接库
target_link_libraries(armor_detection_yolo
    ${OpenCV_LIBS}
)

# 打印信息
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "Building C++ traditional vision version for armor detection")
message(STATUS "Building C++ YOLO vision version for armor detection")
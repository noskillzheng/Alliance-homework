cmake_minimum_required(VERSION 3.10)

project(armor_detector VERSION 1.0.0 LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2")

# 输出编译信息
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")

# ========== 查找依赖库 ==========

# 查找 OpenCV
find_package(OpenCV 4 REQUIRED)
if(OpenCV_FOUND)
    message(STATUS "Found OpenCV ${OpenCV_VERSION}")
    message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "OpenCV not found!")
endif()

# 查找 Eigen3
# Eigen3 是header-only库，直接包含头文件目录即可
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
if(EXISTS ${EIGEN3_INCLUDE_DIR})
    message(STATUS "Found Eigen3")
    message(STATUS "Eigen3 include dirs: ${EIGEN3_INCLUDE_DIR}")
else()
    # 尝试其他常见路径
    set(EIGEN3_INCLUDE_DIR "/usr/local/include/eigen3")
    if(EXISTS ${EIGEN3_INCLUDE_DIR})
        message(STATUS "Found Eigen3 at ${EIGEN3_INCLUDE_DIR}")
    else()
        message(FATAL_ERROR "Eigen3 not found! Please install: sudo apt install libeigen3-dev")
    endif()
endif()

# 海康 MVS SDK
set(MVS_DIR /opt/MVS)
set(MVS_INCLUDE_DIR ${MVS_DIR}/include)
set(MVS_LIB_DIR ${MVS_DIR}/lib/64)
if(EXISTS ${MVS_INCLUDE_DIR})
    message(STATUS "Found HikVision MVS SDK at ${MVS_DIR}")
    include_directories(${MVS_INCLUDE_DIR})
    link_directories(${MVS_LIB_DIR})
    set(MVS_LIBS MvCameraControl)
else()
    message(WARNING "HikVision MVS SDK not found. Camera will use VideoCapture fallback.")
    set(MVS_LIBS "")
endif()

# ========== 包含目录 ==========

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# ========== 源文件 ==========

set(SOURCES
    src/main.cpp
    src/camera/hik_camera.cpp
    src/detector/traditional_detector.cpp
    src/predictor/kalman_filter.cpp
    src/utils/visualizer.cpp
    src/utils/config_reader.cpp
)

set(HEADERS
    include/common/armor.hpp
    include/camera/hik_camera.hpp
    include/detector/traditional_detector.hpp
    include/predictor/kalman_filter.hpp
    include/utils/visualizer.hpp
    include/utils/config_reader.hpp
)

# ========== 可执行文件 ==========

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# ========== 链接库 ==========

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${MVS_LIBS}
)

# ========== 安装设置 ==========

# 安装可执行文件
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)

# 安装配置文件
install(DIRECTORY config/
    DESTINATION config
    FILES_MATCHING PATTERN "*.yaml"
)

# ========== 编译信息 ==========

message(STATUS "========================================")
message(STATUS "Configuration Summary:")
message(STATUS "  Project Name: ${PROJECT_NAME}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  OpenCV Version: ${OpenCV_VERSION}")
message(STATUS "  Eigen3 Version: ${Eigen3_VERSION}")
message(STATUS "========================================")


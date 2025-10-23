cmake_minimum_required(VERSION 3.16)
project(sentry_trajectory_planner VERSION 1.0.0 LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 生成 compile_commands.json 用于 clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 编译选项
add_compile_options(-Wall -Wextra -Wpedantic -O3)

# 查找依赖
find_package(Eigen3 REQUIRED)

 

# 包含目录
include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
)

# 源文件
set(CORE_SOURCES
    src/core/polynomial_trajectory.cpp
    src/core/trajectory_optimizer.cpp
    src/core/constraint_checker.cpp
)

set(UTILS_SOURCES
    src/utils/csv_writer.cpp
    src/utils/config_loader.cpp
)

set(VIS_SOURCES
    src/visualization/trajectory_visualizer.cpp
)

# 核心库
add_library(trajectory_core STATIC
    ${CORE_SOURCES}
)
target_link_libraries(trajectory_core
    Eigen3::Eigen
)

# 工具库
add_library(trajectory_utils STATIC
    ${UTILS_SOURCES}
)

# 可视化库
add_library(trajectory_vis STATIC
    ${VIS_SOURCES}
)
target_link_libraries(trajectory_vis
    trajectory_core
)

# 主可执行文件
add_executable(${PROJECT_NAME}
    src/main.cpp
)

target_link_libraries(${PROJECT_NAME}
    trajectory_vis
    trajectory_core
    trajectory_utils
)

# 安装规则
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)

# 复制配置文件
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config/
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/config/)

# 测试（可选）
option(BUILD_TESTS "Build unit tests" OFF)
if(BUILD_TESTS)
    add_executable(test_polynomial
        tests/test_polynomial.cpp
    )
    target_link_libraries(test_polynomial
        trajectory_core
    )
    enable_testing()
    add_test(NAME polynomial_tests COMMAND test_polynomial)
endif()

# Sanitizers支持（Debug模式）
option(ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(ENABLE_TSAN "Enable ThreadSanitizer" OFF)

if(ENABLE_ASAN)
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address)
    message(STATUS "AddressSanitizer enabled")
endif()

if(ENABLE_TSAN)
    add_compile_options(-fsanitize=thread)
    add_link_options(-fsanitize=thread)
    message(STATUS "ThreadSanitizer enabled")
endif()

# clang-tidy支持
option(ENABLE_CLANG_TIDY "Enable clang-tidy checks" OFF)
if(ENABLE_CLANG_TIDY)
    find_program(CLANG_TIDY_EXE NAMES "clang-tidy" "clangd-20")
    if(CLANG_TIDY_EXE)
        set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE};-checks=modernize-*,performance-*,readability-*")
        message(STATUS "clang-tidy enabled: ${CLANG_TIDY_EXE}")
    else()
        message(WARNING "clang-tidy not found")
    endif()
endif()


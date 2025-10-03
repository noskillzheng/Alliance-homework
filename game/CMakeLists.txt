cmake_minimum_required(VERSION 3.10)

project(game)

find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)
include_directories(include)
aux_source_directory(./tools TOOLS)

add_executable(${PROJECT_NAME}
    main/main.cpp    ${TOOLS})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
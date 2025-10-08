cmake_minimum_required(VERSION 3.16)

project(qt-game VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 设置Qt路径
set(CMAKE_PREFIX_PATH "/home/zheng/Qt/6.8.3/gcc_64/lib/cmake")

# 查找Qt库
find_package(QT NAMES Qt6 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

# 设置包含目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# 设置源文件和头文件
set(PROJECT_SOURCES
<<<<<<< HEAD
    src/main.cpp
    src/mainwindow.cpp
    src/game.cpp
    src/map.cpp
    src/player.cpp
    src/fog.cpp
    src/solver.cpp
    src/trap.cpp
    src/imageprocessor.cpp
=======
        src/main.cpp
        src/mainwindow.cpp
        include/mainwindow.h
        include/mainwindow.ui
        src/player.cpp
        include/player.h
        src/map.cpp
        include/map.h
        src/trap.cpp
        include/trap.h
        src/fog.cpp
        include/fog.h
        src/solver.cpp
        include/solver.h
        src/imageprocessor.cpp
        include/imageprocessor.h
        src/game.cpp
        include/game.h
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
)

set(PROJECT_HEADERS
    include/mainwindow.h
    include/game.h
    include/map.h
    include/player.h
    include/fog.h
    include/solver.h
    include/trap.h
    include/imageprocessor.h
)

# UI文件
set(PROJECT_UI
    include/mainwindow.ui
)

# 如果使用Qt6
if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(qt-game
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
        ${PROJECT_HEADERS}
        ${PROJECT_UI}
    )
else()
    if(ANDROID)
        add_library(qt-game SHARED
            ${PROJECT_SOURCES}
            ${PROJECT_HEADERS}
            ${PROJECT_UI}
        )
    else()
        add_executable(qt-game
            ${PROJECT_SOURCES}
            ${PROJECT_HEADERS}
            ${PROJECT_UI}
        )
    endif()
endif()

# 链接Qt库
target_link_libraries(qt-game PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)

# 设置目标属性
if(${QT_VERSION} VERSION_LESS 6.1.0)
    set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.qt-game)
endif()

set_target_properties(qt-game PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

# 安装配置
include(GNUInstallDirs)
install(TARGETS qt-game
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# 如果使用Qt6，需要最终化可执行文件
if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(qt-game)
endif()

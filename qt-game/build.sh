#!/bin/bash

# 通用编译脚本 - 支持本地和容器环境
# 适用于任何Qt6开发环境

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检测运行环境
detect_environment() {
    if [ -f /.dockerenv ]; then
        ENVIRONMENT="container"
        print_info "检测到容器环境"
    else
        ENVIRONMENT="local"
        print_info "检测到本地环境"
    fi
}

# 检测Qt6安装
detect_qt6() {
    print_info "检测Qt6安装..."
    
    # 常见的Qt6安装路径
    QT_PATHS=(
        "/opt/qt6"
        "/usr/lib/x86_64-linux-gnu/cmake/Qt6"
        "/usr/lib/cmake/Qt6"
        "/home/$USER/Qt/6.*/gcc_64"
        "/opt/Qt/6.*/gcc_64"
    )
    
    QT_FOUND=false
    
    for path in "${QT_PATHS[@]}"; do
        if [ -d "$path" ]; then
            print_success "找到Qt6安装: $path"
            QT_FOUND=true
            break
        fi
    done
    
    if [ "$QT_FOUND" = false ]; then
        print_warning "未找到Qt6安装，尝试使用系统默认路径"
    fi
}

# 检测编译工具
detect_tools() {
    print_info "检测编译工具..."
    
    # 检测CMake
    if command -v cmake &> /dev/null; then
        CMAKE_VERSION=$(cmake --version | head -n1 | cut -d' ' -f3)
        print_success "CMake版本: $CMAKE_VERSION"
    else
        print_error "CMake未安装"
        exit 1
    fi
    
    # 检测Make
    if command -v make &> /dev/null; then
        MAKE_VERSION=$(make --version | head -n1 | cut -d' ' -f3)
        print_success "Make版本: $MAKE_VERSION"
    else
        print_error "Make未安装"
        exit 1
    fi
    
    # 检测GCC
    if command -v g++ &> /dev/null; then
        GCC_VERSION=$(g++ --version | head -n1 | cut -d' ' -f4)
        print_success "GCC版本: $GCC_VERSION"
    else
        print_error "GCC未安装"
        exit 1
    fi
}

# 清理构建目录
clean_build() {
    if [ "$1" = "--clean" ] || [ "$1" = "-c" ]; then
        print_info "清理构建目录..."
        if [ -d "build" ]; then
            rm -rf build
            print_success "构建目录已清理"
        fi
    fi
}

# 检查并清理CMake缓存（解决路径不一致问题）
check_cmake_cache() {
    if [ -f "build/CMakeCache.txt" ]; then
        print_info "检查CMake缓存..."
        
        # 检查缓存中的源目录路径
        CACHED_SOURCE_DIR=$(grep "CMAKE_HOME_DIRECTORY:INTERNAL=" build/CMakeCache.txt 2>/dev/null | cut -d'=' -f2)
        CURRENT_SOURCE_DIR=$(pwd)
        
        if [ "$CACHED_SOURCE_DIR" != "$CURRENT_SOURCE_DIR" ]; then
            print_warning "检测到CMake缓存路径不一致:"
            print_warning "  缓存路径: $CACHED_SOURCE_DIR"
            print_warning "  当前路径: $CURRENT_SOURCE_DIR"
            print_info "清理CMake缓存..."
            rm -rf build/CMakeCache.txt build/CMakeFiles/
            print_success "CMake缓存已清理"
        else
            print_success "CMake缓存路径一致"
        fi
    fi
}

# 检查并清理CMake缓存
check_cmake_cache() {
    print_info "检查CMake缓存..."
    
    if [ -f "build/CMakeCache.txt" ]; then
        # 读取缓存中的路径
        CACHED_PATH=$(grep "CMAKE_CACHEFILE_DIR:STATIC" build/CMakeCache.txt 2>/dev/null | cut -d'=' -f2 || echo "")
        CURRENT_PATH=$(pwd)
        
        if [ -n "$CACHED_PATH" ] && [ "$CACHED_PATH" != "$CURRENT_PATH" ]; then
            print_warning "检测到CMake缓存路径不一致:"
            print_warning "  缓存路径: $CACHED_PATH"
            print_warning "  当前路径: $CURRENT_PATH"
            print_info "清理CMake缓存..."
            rm -rf build CMakeCache.txt CMakeFiles/ cmake_install.cmake Makefile
            print_success "CMake缓存已清理"
        fi
    fi
}

# 创建构建目录
create_build_dir() {
    print_info "创建构建目录..."
    mkdir -p build
}

# 配置CMake
configure_cmake() {
    print_info "配置CMake..."
    
    # 根据环境设置不同的CMake参数
    CMAKE_ARGS=""
    
    if [ "$ENVIRONMENT" = "container" ]; then
        # 容器环境
        CMAKE_ARGS="-DCMAKE_PREFIX_PATH=/opt/Qt/6.6.1/gcc_64/lib/cmake"
        print_info "使用容器Qt6路径"
    else
        # 本地环境 - 让CMake自动检测
        print_info "使用系统Qt6路径"
    fi
    
    # 进入构建目录并执行CMake配置
    cd build
    if cmake .. $CMAKE_ARGS; then
        print_success "CMake配置成功"
    else
        print_error "CMake配置失败"
        exit 1
    fi
}

# 编译项目
build_project() {
    print_info "编译项目..."
    
    # 获取CPU核心数
    if command -v nproc &> /dev/null; then
        CORES=$(nproc)
    else
        CORES=4  # 默认使用4个核心
    fi
    
    print_info "使用 $CORES 个CPU核心进行编译"
    
    if make -j$CORES; then
        print_success "编译成功"
    else
        print_error "编译失败"
        exit 1
    fi
}

# 验证可执行文件
verify_executable() {
    print_info "验证可执行文件..."
    
    if [ -f "qt-game" ]; then
        print_success "可执行文件 qt-game 已生成"
        
        # 检查文件权限
        if [ -x "qt-game" ]; then
            print_success "可执行文件权限正确"
        else
            print_warning "设置可执行文件权限..."
            chmod +x qt-game
        fi
        
        # 显示文件信息
        FILE_SIZE=$(du -h qt-game | cut -f1)
        print_info "可执行文件大小: $FILE_SIZE"
        
    else
        print_error "可执行文件未生成"
        exit 1
    fi
}

# 显示使用说明
show_usage() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -c, --clean     清理构建目录后重新编译"
    echo "  -h, --help      显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0              # 普通编译"
    echo "  $0 --clean      # 清理后重新编译"
    echo ""
}

# 主函数
main() {
    echo "=========================================="
    echo "Qt6 迷宫游戏编译脚本"
    echo "=========================================="
    
    # 处理命令行参数
    case "$1" in
        -h|--help)
            show_usage
            exit 0
            ;;
        -c|--clean)
            clean_build "$1"
            ;;
        "")
            # 无参数，正常编译
            ;;
        *)
            print_error "未知参数: $1"
            show_usage
            exit 1
            ;;
    esac
    
    # 检测环境
    detect_environment
    detect_qt6
    detect_tools
    
    # 编译流程
    check_cmake_cache
    create_build_dir
    configure_cmake
    build_project
    verify_executable
    
    echo ""
    echo "=========================================="
    print_success "编译完成！"
    echo "=========================================="
    echo ""
    echo "运行游戏:"
    echo "  ./build/qt-game"
    echo ""
    echo "调试游戏:"
    echo "  gdb ./build/qt-game"
    echo ""
    
    # 返回项目根目录
    cd ..
}

# 执行主函数
main "$@"

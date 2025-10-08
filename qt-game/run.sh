#!/bin/bash

# 通用运行脚本 - 支持本地和容器环境
# 自动检测环境并运行游戏

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

# 检查可执行文件
check_executable() {
    if [ ! -f "build/qt-game" ]; then
        print_error "可执行文件不存在: build/qt-game"
        print_info "请先运行编译脚本: ./build.sh"
        exit 1
    fi
    
    if [ ! -x "build/qt-game" ]; then
        print_warning "设置可执行文件权限..."
        chmod +x build/qt-game
    fi
    
    print_success "可执行文件检查通过"
}

# 检查显示环境
check_display() {
    if [ -z "$DISPLAY" ]; then
        print_warning "DISPLAY环境变量未设置"
        
        if [ "$ENVIRONMENT" = "container" ]; then
            print_info "容器环境，尝试设置DISPLAY..."
            export DISPLAY=:0
        else
            print_info "本地环境，尝试自动检测DISPLAY..."
            if command -v xdpyinfo &> /dev/null; then
                export DISPLAY=$(xdpyinfo | grep "name of display" | cut -d: -f2 | xargs)
            else
                export DISPLAY=:0
            fi
        fi
    fi
    
    print_info "使用DISPLAY: $DISPLAY"
}

# 检查X11权限
check_x11_permissions() {
    if [ "$ENVIRONMENT" = "container" ]; then
        print_info "容器环境，检查X11权限..."
        
        # 检查X11转发
        if ! xhost &> /dev/null; then
            print_warning "X11权限可能有问题，尝试修复..."
            xhost +local:docker 2>/dev/null || true
        fi
    fi
}

# 运行游戏
run_game() {
    print_info "启动游戏..."
    
    # 设置Qt平台插件
    if [ "$ENVIRONMENT" = "container" ]; then
        export QT_QPA_PLATFORM=xcb
        print_info "容器环境，使用xcb平台插件"
    fi
    
    # 运行游戏
    if ./build/qt-game; then
        print_success "游戏正常退出"
    else
        print_error "游戏运行失败"
        exit 1
    fi
}

# 显示使用说明
show_usage() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -d, --debug     使用GDB调试运行"
    echo "  -h, --help      显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0              # 普通运行"
    echo "  $0 --debug      # 调试运行"
    echo ""
}

# 调试运行
debug_run() {
    print_info "使用GDB调试运行..."
    
    if ! command -v gdb &> /dev/null; then
        print_error "GDB未安装"
        exit 1
    fi
    
    gdb -ex "run" -ex "bt" -ex "quit" --args ./build/qt-game
}

# 主函数
main() {
    echo "=========================================="
    echo "Qt6 迷宫游戏运行脚本"
    echo "=========================================="
    
    # 处理命令行参数
    case "$1" in
        -h|--help)
            show_usage
            exit 0
            ;;
        -d|--debug)
            DEBUG_MODE=true
            ;;
        "")
            # 无参数，正常运行
            ;;
        *)
            print_error "未知参数: $1"
            show_usage
            exit 1
            ;;
    esac
    
    # 检测环境
    detect_environment
    
    # 检查前置条件
    check_executable
    check_display
    check_x11_permissions
    
    # 运行游戏
    if [ "$DEBUG_MODE" = true ]; then
        debug_run
    else
        run_game
    fi
}

# 执行主函数
main "$@"

#!/bin/bash

# DevContainer 后创建命令脚本
# 解决CMake缓存路径问题

set -e  # 遇到错误立即退出

echo "=========================================="
echo "DevContainer 项目构建"
echo "=========================================="

# 清理可能存在的构建目录和CMake缓存
echo "清理旧的构建目录和CMake缓存..."
rm -rf build CMakeCache.txt CMakeFiles/ cmake_install.cmake Makefile
echo "✅ 旧构建目录和缓存已清理"

# 使用build.sh脚本编译项目
echo "使用build.sh脚本编译项目..."
# shellcheck disable=SC2295
if [ -f "./build.sh" ]; then
    ./build.sh
else
    echo "⚠️  build.sh脚本不存在,跳过自动编译。"
fi

echo ""
echo "=========================================="
echo "✅ DevContainer 项目构建完成"
echo "=========================================="
echo ""
echo "项目已编译完成，可以开始开发："
echo "- 使用 F5 开始调试"
echo "- 使用 Ctrl+Shift+P -> Tasks: Run Task -> Build Script 重新编译"
echo "- 使用 ./run.sh 运行游戏"
echo ""

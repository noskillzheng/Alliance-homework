#!/bin/bash

# RoboMaster 装甲板检测系统构建脚本

echo "🔧 开始构建 RoboMaster 装甲板检测系统..."

# 创建构建目录
mkdir -p build

# 进入构建目录
cd build

# 配置CMake
echo "📋 配置CMake..."
cmake ..

# 编译项目
echo "🔨 编译项目..."
make

# 复制可执行文件到项目根目录
echo "📦 复制可执行文件到项目根目录..."
cp armor_detection_traditional ..
cp armor_detection_yolo ..

echo "✅ 构建完成！"
echo ""
echo "🎯 使用方法："
echo "  传统视觉版本: ./armor_detection_traditional --image data/test_images/test.PNG"
echo "  YOLO版本: ./armor_detection_yolo --image data/test_images/test.PNG"
echo ""
echo "💡 提示: 如果您有OpenCV和CMake，可以在VSCode中直接打开项目进行开发和调试。"

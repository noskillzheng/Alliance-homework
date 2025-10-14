# RoboMaster 装甲板检测系统

## 项目简介

这是一个基于OpenCV和YOLO的RoboMaster装甲板检测系统，提供了传统计算机视觉方法和深度学习方法两种实现方式。项目支持图片和视频的装甲板检测，并提供了完整的训练和推理流程。

## 功能特点

- 🎯 **双模式检测**：支持传统计算机视觉和YOLO深度学习两种检测方法
- 🖼️ **多格式支持**：支持图片和视频输入
- 🎨 **颜色识别**：支持蓝色和红色装甲板检测
- 🚀 **实时处理**：支持实时视频流处理
- 📊 **训练工具**：提供完整的模型训练脚本
- 🔧 **VSCode友好**：提供完整的VSCode开发环境配置

## 项目结构

```
vision/
├── src/                    # C++源代码
│   ├── main_traditional.cpp    # 传统视觉主程序
│   ├── main_cmd.cpp            # 命令行版本主程序
│   ├── armor_detector_traditional.cpp  # 传统视觉检测器
│   └── armor_detector_cmd.cpp  # 命令行检测器
├── scripts/                # Python脚本
│   ├── armor_detection_display.py  # 显示检测结果
│   ├── train_armor_fixed.py       # 训练脚本
│   ├── convert_labelme_to_yolo.py # 数据转换
│   └── convert_to_onnx.py         # 模型转换
├── models/                 # 预训练模型
├── dataset/                # 数据集
├── data/                   # 测试数据
├── include/                # 头文件
├── build/                  # 构建目录
└── yolo_env/              # Python虚拟环境
```

## 环境要求

- **操作系统**：Linux (推荐Ubuntu 18.04+)
- **编译器**：GCC/G++ 支持C++14标准
- **Python**：3.8+
- **依赖库**：
  - OpenCV 4.x
  - CMake 3.10+
  - PyTorch
  - Ultralytics YOLO

## 安装步骤

### 1. 克隆仓库

```bash
git clone https://github.com/您的用户名/vision.git
cd vision
```

### 2. 安装系统依赖

```bash
# 安装OpenCV和CMake
sudo apt update
sudo apt install build-essential cmake pkg-config
sudo apt install libopencv-dev python3-opencv

# 安装Python依赖
sudo apt install python3-pip python3-venv
```

### 3. 设置Python环境

```bash
# 创建并激活虚拟环境
python3 -m venv yolo_env
source yolo_env/bin/activate

# 安装Python依赖
pip install torch torchvision ultralytics opencv-python numpy
```

## 使用方法

### 传统视觉方法

```bash
# 检测图片中的蓝色装甲板
./build/armor_detection_traditional --image data/test_images/test.PNG

# 指定输出路径
./build/armor_detection_traditional --image data/test_images/test.PNG --output ./results/
```

### YOLO深度学习方法

```bash
# 检测图片中的装甲板
./build/armor_detection_yolo --image data/test_images/test.PNG

# 检测视频中的装甲板
./build/armor_detection_yolo --video data/video/test_hik_1.avi

# 使用自定义模型
./build/armor_detection_yolo --image data/test_images/test.PNG --model models/trained/best.pt

# 设置置信度阈值
./build/armor_detection_yolo --image data/test_images/test.PNG --confidence 0.5
```

### Python深度学习方法

```bash
# 激活Python环境
source yolo_env/bin/activate

# 使用预训练模型检测图片
python scripts/armor_detection_display.py --image data/test_images/test.PNG

# 检测视频
python scripts/armor_detection_display.py --video data/video/test_hik_1.avi

# 使用自定义模型
python scripts/armor_detection_display.py --image data/test_images/test.PNG --model models/trained/best.pt
```

### 训练自定义模型

```bash
# 激活Python环境
source yolo_env/bin/activate

# 训练模型
python scripts/train_armor_fixed.py

# 转换模型为ONNX格式
python scripts/convert_to_onnx.py --model models/trained/best.pt
```

## 示例结果

检测后的结果将保存在 `results/` 目录中，包括：
- 原始图片
- 标注了装甲板位置的图片
- 检测统计信息

## 故障排除

### 常见问题

1. **OpenCV找不到**：
   ```bash
   pkg-config --modversion opencv4
   # 如果没有输出，请重新安装OpenCV
   ```

2. **Python环境问题**：
   ```bash
   # 确保激活了正确的虚拟环境
   which python
   # 应该显示 ./yolo_env/bin/python
   ```

3. **编译错误**：
   ```bash
   # 清理构建目录并重新编译
   rm -rf build/*
   cd build
   cmake ..
   make
   ```

4. **显示问题**：
   如果在远程环境中运行，可能需要设置X11转发：
   ```bash
   export DISPLAY=:0
   ```

## 贡献指南

1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启 Pull Request

## 许可证

本项目采用 MIT 许可证 - 查看 [LICENSE](LICENSE) 文件了解详情

## 致谢

- [OpenCV](https://opencv.org/) - 开源计算机视觉库
- [Ultralytics YOLO](https://ultralytics.com/) - YOLO实现
- [RoboMaster](https://www.robomaster.com/) - 机器人比赛平台

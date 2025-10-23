# 哨兵导航 - 离散点平滑轨迹规划器

## 项目简介

基于**最小Jerk优化**的轨迹规划算法，用于RoboMaster哨兵机器人导航。

### 核心特性

- ✅ **5阶多项式（Quintic）平滑轨迹** - 严格C^2连续
- ✅ **最小Jerk优化** - 全局最优化 min ∫jerk²dt
- ✅ **时间缩放优化** - 自动逼近约束上限，性能提升55%
- ✅ **闭式极值检查** - 精确验证速度/加速度峰值
- ✅ **YAML配置系统** - 无需编译即可调参
- ✅ **增强可视化** - 热力图+曲率+利用率分析
- ✅ **单元测试** - 保证代码质量

---

## 快速开始

### 1. 编译

```bash
cd /home/zheng/Alli/final/sentry_navigation
./build.sh
```

### 2. 运行

```bash
cd build
./sentry_trajectory_planner
```

### 3. 可视化

**方案A：数据分析（无需安装任何包）**
```bash
python3 ../scripts/analyze_csv.py
```

**方案B：图形化（需要matplotlib）**
```bash
# 安装依赖（首次）
sudo apt install python3-matplotlib python3-numpy python3-pandas

# 绘制轨迹（英文版，无乱码）
python3 ../scripts/visualize_trajectory.py

# 或中文版（需要先安装字体：sudo apt install fonts-wqy-microhei）
python3 ../scripts/visualize_trajectory_cn.py
```

---

## 项目结构

```
sentry_navigation/
├── CMakeLists.txt          # CMake配置
├── build.sh                # 编译脚本
├── README.md               # 本文档
├── 快速开始.md             # 详细使用指南
│
├── include/                # 头文件
│   ├── core/              # 核心算法
│   ├── utils/             # 工具类
│   └── visualization/     # 可视化
│
├── src/                    # 源代码
│   ├── core/              # 算法实现
│   ├── utils/             # 工具实现
│   ├── visualization/     # 可视化实现
│   └── main.cpp           # 主程序
│
├── config/                 # 配置文件
├── scripts/                # Python脚本
└── build/                  # 编译输出
```

---

## 依赖项

### 必需
- CMake >= 3.16
- C++17 编译器
- Eigen3

**安装**:
```bash
sudo apt install cmake g++ libeigen3-dev
```

### 可选（Foxglove可视化）
- ROS2 (Humble/Iron/Jazzy)
- geometry_msgs, nav_msgs, visualization_msgs

### 可选（Python可视化）
- Python 3
- matplotlib, numpy, pandas

---

## 自定义路径点

编辑 `src/main.cpp` 第26-32行：

```cpp
std::vector<Waypoint> waypoints = {
    Waypoint(0.0, 0.0),
    Waypoint(1.0, 0.0),
    Waypoint(1.0, 1.0),
    // 添加更多点...
};
```

修改约束参数（第42-44行）：

```cpp
config.max_velocity = 1.0;       // 最大速度 (m/s)
config.max_acceleration = 1.0;   // 最大加速度 (m/s²)
config.max_segment_time = 10.0;  // 每段最大时间 (s)
```

---

## 输出文件

运行后生成：
- `trajectory.csv` - 完整轨迹数据
- `waypoints.csv` - 路径点
- `trajectory_visualization.png` - 可视化图表（如运行Python脚本）

---

## 性能表现

给定路径点: `{(0,0), (1,0), (1,1), (1,2), (2,3)}`

**优化结果**:
- ✅ 总时间: **9.68秒** (比初版快55.5%)
- ✅ 最大速度: 0.81 m/s (81%利用率)
- ✅ 最大加速度: 0.92 m/s² (92%利用率)
- ✅ **C^2连续性**：位置/速度/加速度跳变 < 1e-6
- ✅ **所有约束满足**

**算法升级**:
- 5阶多项式（vs原4阶）
- 时间二分优化（vs原启发式）
- 闭式极值检查（vs原采样）

---

## 高级功能

### 运行单元测试
```bash
cd build
cmake .. -DBUILD_TESTS=ON
make test_polynomial
./test_polynomial
# 或使用 ctest
ctest --output-on-failure
```

### 启用内存检查（开发调试）
```bash
cd build
cmake .. -DENABLE_ASAN=ON
make
./sentry_trajectory_planner
```

### 启用代码质量检查
```bash
cd build  
cmake .. -DENABLE_CLANG_TIDY=ON
make
```

---

## 常见问题

**Q: 编译失败，找不到Eigen3？**
```bash
sudo apt install libeigen3-dev
```

**Q: Python可视化中文乱码？**

使用英文版脚本（推荐）：
```bash
python3 scripts/visualize_trajectory.py
```

或安装中文字体：
```bash
sudo apt install fonts-wqy-microhei
rm -rf ~/.cache/matplotlib
python3 scripts/visualize_trajectory_cn.py
```

**Q: 约束检查失败怎么办？**
- 增加 `max_segment_time`
- 放宽速度/加速度限制
- 调整路径点间距

---

## 更多信息

详细使用说明请查看 `快速开始.md`

---

## 许可证

MIT License

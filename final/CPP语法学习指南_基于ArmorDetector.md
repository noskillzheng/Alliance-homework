# C++ 语法学习指南

> 基于 `armor_detector` 项目的实战学习
> 
> **学习目标**：通过真实项目代码理解 C++ 核心语法和编程规范

---

## 📚 目录

1. [项目概览](#项目概览)
2. [第一章：头文件与代码组织](#第一章头文件与代码组织)
3. [第二章：命名空间](#第二章命名空间)
4. [第三章：结构体 vs 类](#第三章结构体-vs-类)
5. [第四章：类的封装](#第四章类的封装)
6. [第五章：构造函数与初始化](#第五章构造函数与初始化)
7. [第六章：const 正确性](#第六章const-正确性)
8. [第七章：引用与指针](#第七章引用与指针)
9. [第八章：STL 容器](#第八章stl-容器)
10. [第九章：内存管理与 RAII](#第九章内存管理与-raii)
11. [第十章：Doxygen 注释规范](#第十章doxygen-注释规范)
12. [第十一章：编码规范总结](#第十一章编码规范总结)
13. [实战练习](#实战练习)

---

## 项目概览

### 项目结构

```
armor_detector/
├── include/           # 头文件（.hpp）
│   ├── camera/       # 相机模块
│   ├── common/       # 公共数据结构
│   ├── detector/     # 检测器模块
│   ├── predictor/    # 预测器模块
│   └── utils/        # 工具类
├── src/              # 源文件（.cpp）
│   ├── camera/
│   ├── detector/
│   ├── predictor/
│   ├── utils/
│   └── main.cpp      # 主程序入口
├── config/           # 配置文件（.yaml）
└── CMakeLists.txt    # 构建配置
```

### 核心模块

| 模块 | 功能 | 关键类 |
|------|------|--------|
| `common/` | 数据结构定义 | `Armor`, `LightBar`, `PredictionResult` |
| `camera/` | 相机控制 | `HikCamera` |
| `detector/` | 装甲板检测 | `TraditionalDetector` |
| `predictor/` | 卡尔曼滤波预测 | `KalmanFilter` |
| `utils/` | 工具类 | `ConfigReader`, `Visualizer` |

---

## 第一章：头文件与代码组织

### 1.1 Include Guard（包含保护）

**作用**：防止头文件被重复包含，导致重定义错误。

**示例**：`include/common/armor.hpp`

```cpp
#ifndef ARMOR_HPP          // 如果没有定义 ARMOR_HPP
#define ARMOR_HPP          // 则定义 ARMOR_HPP

// 头文件内容...
struct Armor { ... };

#endif // ARMOR_HPP        // 结束条件编译
```

**工作原理**：
1. 第一次包含时：`ARMOR_HPP` 未定义，执行内容并定义它
2. 第二次包含时：`ARMOR_HPP` 已定义，跳过内容

**命名规范**：
- 使用文件名的大写形式
- 将 `.` 替换为 `_`
- 例如：`hik_camera.hpp` → `HIK_CAMERA_HPP`

### 1.2 头文件 vs 源文件

**头文件（.hpp/.h）**：
- 声明类、函数、结构体
- 定义内联函数、模板
- 包含常量定义

**源文件（.cpp）**：
- 实现类的成员函数
- 定义全局函数
- 包含具体逻辑

**示例对比**：

```cpp
// ========== hik_camera.hpp（声明） ==========
class HikCamera {
public:
    bool init(int exposure_time = 5000, float gain = 10.0f, float gamma = 0.7f);
    bool open(int device_index = 0);
    // ... 其他声明
private:
    bool is_opened_;
};

// ========== hik_camera.cpp（实现） ==========
bool HikCamera::init(int exposure_time, float gain, float gamma) {
    exposure_time_ = exposure_time;
    gain_ = gain;
    gamma_ = gamma;
    // ... 实现逻辑
    return true;
}
```

**为什么要分离？**
- **编译效率**：修改 .cpp 不需要重新编译所有依赖的文件
- **接口清晰**：.hpp 展示"做什么"，.cpp 隐藏"怎么做"
- **减少依赖**：头文件只包含必要的声明

### 1.3 包含顺序规范

**推荐顺序**（参考 `src/main.cpp`）：

```cpp
// 1. 本项目的头文件
#include "camera/hik_camera.hpp"
#include "detector/traditional_detector.hpp"
#include "predictor/kalman_filter.hpp"

// 2. 第三方库头文件
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

// 3. C++ 标准库头文件
#include <iostream>
#include <vector>
#include <string>
```

**原因**：
- 先包含自己的头文件，暴露依赖问题
- 按依赖层级排序，便于查找

---

## 第二章：命名空间

### 2.1 为什么需要命名空间？

**问题**：不同库可能有同名类/函数，导致冲突。

```cpp
// opencv 有 Mat 类
// eigen 也有 Mat 类
// 如何区分？
```

**解决方案**：使用命名空间（namespace）

### 2.2 定义命名空间

**示例**：`include/common/armor.hpp`

```cpp
namespace armor_detector {  // 定义命名空间

struct Armor {
    cv::Point2f center;
    float width;
    float height;
    // ...
};

struct LightBar {
    // ...
};

} // namespace armor_detector  // 结束命名空间（建议添加注释）
```

### 2.3 使用命名空间

**方式一：完全限定名**

```cpp
armor_detector::Armor armor;
armor_detector::LightBar bar;
```

**方式二：using 声明**

```cpp
using armor_detector::Armor;
Armor armor;  // 可以直接使用
```

**方式三：using 指令（不推荐）**

```cpp
using namespace armor_detector;  // 引入整个命名空间
Armor armor;
LightBar bar;
```

**最佳实践**（参考 `src/main.cpp`）：

```cpp
using namespace armor_detector;  // 在 .cpp 文件中可以用

int main() {
    HikCamera camera;           // 简洁
    TraditionalDetector detector;
    // ...
}
```

**注意**：
- ✅ 在 .cpp 文件中使用 `using namespace`
- ❌ 在头文件中使用 `using namespace`（会污染全局命名空间）

---

## 第三章：结构体 vs 类

### 3.1 结构体（struct）

**特点**：
- 默认成员是 **public**
- 用于简单的数据聚合
- 一般不包含复杂逻辑

**示例**：`include/common/armor.hpp`

```cpp
struct Armor {
    ArmorCorners corners;       // 公共成员，直接访问
    cv::Point2f center;
    float width;
    float height;
    int color;
    float confidence;
    int64_t timestamp;
    
    // 默认构造函数
    Armor() : width(0), height(0), color(0), confidence(0), timestamp(0) {}
    
    // 简单的工具函数
    bool isValid() const {
        return confidence > 0.0f && width > 0 && height > 0;
    }
};
```

**使用**：

```cpp
Armor armor;
armor.width = 100;          // 直接访问，无需 getter/setter
armor.height = 50;
if (armor.isValid()) {
    // ...
}
```

### 3.2 类（class）

**特点**：
- 默认成员是 **private**
- 用于封装复杂的逻辑和数据
- 提供接口（public），隐藏实现（private）

**示例**：`include/detector/traditional_detector.hpp`

```cpp
class TraditionalDetector {
public:
    // 公共接口
    explicit TraditionalDetector(const std::string& target_color = "red");
    std::vector<Armor> detect(const cv::Mat& frame);
    void setTargetColor(const std::string& color);
    
private:
    // 私有数据（外部无法访问）
    std::string target_color_;
    int binary_threshold_;
    float min_light_area_;
    
    // 私有辅助函数
    cv::Mat colorSegmentation(const cv::Mat& frame);
    std::vector<LightBar> findLightBars(const cv::Mat& binary);
};
```

**使用**：

```cpp
TraditionalDetector detector("red");
detector.detect(frame);         // 调用公共接口
// detector.target_color_ = "blue";  // ❌ 错误！私有成员不可访问
detector.setTargetColor("blue");   // ✅ 通过公共接口修改
```

### 3.3 何时用 struct，何时用 class？

| 场景 | 使用 | 示例 |
|------|------|------|
| 纯数据容器 | `struct` | `Armor`, `LightBar`, `PredictionResult` |
| 需要封装和逻辑 | `class` | `TraditionalDetector`, `KalmanFilter` |
| 简单配置 | `struct` | `ArmorCorners` |
| 复杂状态管理 | `class` | `HikCamera` |

**经验法则**：
- 如果所有成员都是 public，用 `struct`
- 如果需要隐藏实现细节，用 `class`

---

## 第四章：类的封装

### 4.1 访问控制

**三种访问权限**：

```cpp
class HikCamera {
public:
    // 任何地方都可以访问
    bool open(int device_index = 0);
    bool isOpened() const { return is_opened_; }
    
protected:
    // 子类可以访问（本项目未使用继承，所以较少见）
    void logMessage(const std::string& msg);
    
private:
    // 只有类内部可以访问
    bool is_opened_;
    int exposure_time_;
    cv::VideoCapture video_capture_;
};
```

### 4.2 封装的好处

**示例**：`include/detector/traditional_detector.hpp`

```cpp
class TraditionalDetector {
public:
    // 用户只需要知道这个接口
    void setBinaryThreshold(int threshold) { 
        binary_threshold_ = threshold; 
    }
    
private:
    // 实现细节被隐藏
    int binary_threshold_;
    
    // 内部辅助函数，用户不需要知道
    cv::Mat colorSegmentation(const cv::Mat& frame);
    bool isValidLightBar(const LightBar& bar);
};
```

**好处**：
1. **安全性**：防止外部错误修改内部状态
2. **灵活性**：可以修改内部实现，不影响外部代码
3. **清晰性**：公共接口清楚地展示"能做什么"

**对比**：

```cpp
// ❌ 不好的设计（全部 public）
class Detector {
public:
    int threshold;
    cv::Mat processImage(cv::Mat img);
};

detector.threshold = -100;  // 可能导致错误！

// ✅ 好的设计（封装）
class Detector {
public:
    void setThreshold(int t) {
        if (t >= 0 && t <= 255) {  // 验证输入
            threshold_ = t;
        }
    }
private:
    int threshold_;
};
```

### 4.3 Getter 和 Setter

**Getter（获取值）**：

```cpp
cv::Mat getDebugImage() const { return debug_image_; }
bool isOpened() const { return is_opened_; }
```

**Setter（设置值）**：

```cpp
void setDebugMode(bool enable) { debug_mode_ = enable; }
void setMinLightArea(float min_area) { min_light_area_ = min_area; }
```

**内联 Getter/Setter**：
- 简单的函数可以直接在类定义中实现
- 编译器会自动内联，无性能损失

```cpp
class TraditionalDetector {
public:
    // 内联 getter
    cv::Mat getDebugImage() const { return debug_image_; }
    
    // 内联 setter
    void setDebugMode(bool enable) { debug_mode_ = enable; }
    
private:
    cv::Mat debug_image_;
    bool debug_mode_;
};
```

---

## 第五章：构造函数与初始化

### 5.1 默认构造函数

**示例**：`include/common/armor.hpp`

```cpp
struct Armor {
    float width;
    float height;
    int color;
    float confidence;
    int64_t timestamp;
    
    // 默认构造函数：初始化所有成员
    Armor() : width(0), height(0), color(0), confidence(0), timestamp(0) {}
};
```

**使用**：

```cpp
Armor armor;  // 调用默认构造函数
// armor.width == 0, armor.height == 0, ...
```

### 5.2 带参数的构造函数

**示例**：`include/detector/traditional_detector.hpp`

```cpp
class TraditionalDetector {
public:
    /**
     * @brief 构造函数
     * @param target_color 目标颜色："red" 或 "blue"
     */
    explicit TraditionalDetector(const std::string& target_color = "red");
};
```

**`explicit` 关键字**：
- 防止隐式类型转换
- 强制必须明确调用构造函数

```cpp
// 有 explicit
explicit TraditionalDetector(const std::string& target_color);

TraditionalDetector detector1("red");           // ✅ OK
TraditionalDetector detector2 = "red";          // ❌ 错误！不能隐式转换
TraditionalDetector detector3 = {"red"};        // ❌ 错误！

// 无 explicit
TraditionalDetector(const std::string& target_color);

TraditionalDetector detector4 = "red";          // ✅ OK（但不推荐）
```

### 5.3 初始化列表（重要！）

**示例**：`include/common/armor.hpp`

```cpp
struct LightBar {
    cv::RotatedRect rect;
    cv::Point2f center;
    float angle;
    float length;
    
    // 使用初始化列表
    explicit LightBar(const cv::RotatedRect& r) : rect(r) {
        center = r.center;
        angle = r.angle;
        length = std::max(r.size.width, r.size.height);
    }
};
```

**初始化列表 vs 赋值**：

```cpp
// ❌ 不推荐：在构造函数体内赋值
LightBar(const cv::RotatedRect& r) {
    rect = r;        // 先默认构造，再赋值（效率低）
    center = r.center;
    angle = r.angle;
}

// ✅ 推荐：使用初始化列表
LightBar(const cv::RotatedRect& r) 
    : rect(r),       // 直接构造（效率高）
      center(r.center),
      angle(r.angle),
      length(std::max(r.size.width, r.size.height)) {
    // 构造函数体（可为空）
}
```

**优势**：
1. **效率更高**：直接构造，避免二次赋值
2. **必需性**：const 成员、引用成员必须用初始化列表
3. **清晰性**：明确初始化顺序

### 5.4 默认参数

**示例**：`include/camera/hik_camera.hpp`

```cpp
bool init(int exposure_time = 5000, float gain = 10.0f, float gamma = 0.7f);
```

**使用**：

```cpp
camera.init();                    // 使用默认值：5000, 10.0, 0.7
camera.init(8000);                // 指定第一个参数：8000, 10.0, 0.7
camera.init(8000, 12.0f);         // 指定前两个：8000, 12.0, 0.7
camera.init(8000, 12.0f, 0.8f);   // 全部指定：8000, 12.0, 0.8
```

**注意**：
- 默认参数只能从右往左指定
- 声明和定义只能有一处指定默认参数（通常在声明处）

```cpp
// ✅ 正确
void func(int a, int b = 10, int c = 20);

// ❌ 错误：跳过中间参数
void func(int a, int b = 10, int c);

// ❌ 错误：从左边开始
void func(int a = 5, int b, int c);
```

---

## 第六章：const 正确性

### 6.1 const 成员函数

**作用**：承诺该函数不会修改对象的状态。

**示例**：`include/detector/traditional_detector.hpp`

```cpp
class TraditionalDetector {
public:
    // const 成员函数：不会修改对象
    cv::Mat getDebugImage() const { return debug_image_; }
    
    // 非 const 成员函数：可能修改对象
    void setDebugMode(bool enable) { debug_mode_ = enable; }
    
private:
    cv::Mat debug_image_;
    bool debug_mode_;
};
```

**规则**：
- const 对象只能调用 const 成员函数
- 非 const 对象可以调用任何函数

```cpp
const TraditionalDetector detector("red");

cv::Mat img = detector.getDebugImage();  // ✅ OK（const 函数）
detector.setDebugMode(true);             // ❌ 错误！（非 const 函数）
```

**最佳实践**：
- 所有不修改对象的函数都应该声明为 const
- 提高代码安全性和可读性

### 6.2 const 引用参数

**示例**：`include/detector/traditional_detector.hpp`

```cpp
std::vector<Armor> detect(const cv::Mat& frame);
void setHSVThreshold(const cv::Scalar& hsv_low, const cv::Scalar& hsv_high);
```

**为什么使用 `const &`？**

```cpp
// ❌ 按值传递：会复制整个对象（效率低）
std::vector<Armor> detect(cv::Mat frame);

// ✅ const 引用：不复制，不修改（高效且安全）
std::vector<Armor> detect(const cv::Mat& frame);
```

**何时使用 const 引用？**

| 类型 | 传递方式 | 原因 |
|------|---------|------|
| 基本类型（int, float） | 按值 | 复制成本低 |
| 大对象（cv::Mat, std::vector） | const 引用 | 避免复制 |
| 字符串（std::string） | const 引用 | 避免复制 |

**示例对比**：

```cpp
// 基本类型：按值传递
void setThreshold(int threshold);

// 大对象：const 引用
std::vector<Armor> detect(const cv::Mat& frame);
void setColor(const std::string& color);
```

### 6.3 返回 const

**示例**：`include/predictor/kalman_filter.hpp`

```cpp
Eigen::VectorXd getState() const { return state_; }
Eigen::Vector2d getPosition() const { return state_.head<2>(); }
```

**注意**：
- 返回值本身不需要 const（会被复制）
- 返回引用时才需要考虑 const

```cpp
// 返回值（会复制，不需要 const）
Eigen::VectorXd getState() const;

// 返回引用（需要 const 保护）
const Eigen::VectorXd& getStateRef() const;
```

---

## 第七章：引用与指针

### 7.1 引用（Reference）

**定义**：别名，绑定到另一个对象。

**示例**：

```cpp
int x = 10;
int& ref = x;   // ref 是 x 的别名

ref = 20;       // 修改 ref 就是修改 x
// 现在 x == 20
```

**引用作为参数**：

```cpp
// 值传递：复制参数
void updateValue(int value) {
    value = 100;  // 只修改副本，不影响原变量
}

// 引用传递：直接操作原变量
void updateRef(int& value) {
    value = 100;  // 修改原变量
}

int x = 10;
updateValue(x);  // x 仍然是 10
updateRef(x);    // x 变成 100
```

**const 引用**：只读，不能修改

```cpp
void printValue(const int& value) {
    std::cout << value << std::endl;
    // value = 100;  // ❌ 错误！不能修改 const 引用
}
```

### 7.2 指针（Pointer）

**定义**：存储对象地址的变量。

**示例**：`include/camera/hik_camera.hpp`

```cpp
class HikCamera {
private:
    void* camera_handle_;  // 指针：存储相机句柄的地址
};
```

**引用 vs 指针**：

| 特性 | 引用 | 指针 |
|------|------|------|
| 可以为空 | ❌ 不能 | ✅ 可以（nullptr） |
| 可以重新绑定 | ❌ 不能 | ✅ 可以 |
| 语法 | 自动解引用 | 需要 `*` 和 `->` |
| 安全性 | 更安全 | 需要检查空指针 |

**示例对比**：

```cpp
// 引用
int x = 10;
int& ref = x;
ref = 20;  // 直接使用

// 指针
int* ptr = &x;
*ptr = 20;  // 需要解引用
```

### 7.3 项目中的使用

**函数参数**：优先使用引用

```cpp
// ✅ 使用 const 引用（推荐）
std::vector<Armor> detect(const cv::Mat& frame);

// ❌ 使用指针（不推荐，除非需要表示"可选"）
std::vector<Armor> detect(const cv::Mat* frame);
```

**可选参数**：使用指针或 std::optional

```cpp
// 指针可以为 nullptr 表示"没有"
bool update(const Armor* armor) {
    if (armor == nullptr) {
        return false;  // 没有装甲板
    }
    // 处理装甲板...
}
```

---

## 第八章：STL 容器

### 8.1 std::vector（动态数组）

**最常用的容器**，类似于 Python 的 list。

**示例**：`include/detector/traditional_detector.hpp`

```cpp
std::vector<Armor> detect(const cv::Mat& frame);
std::vector<LightBar> findLightBars(const cv::Mat& binary);
```

**基本操作**：

```cpp
std::vector<int> numbers;

// 添加元素
numbers.push_back(10);
numbers.push_back(20);
numbers.push_back(30);

// 访问元素
int first = numbers[0];        // 索引访问（不检查边界）
int second = numbers.at(1);    // at() 会检查边界

// 大小
int size = numbers.size();     // 3
bool empty = numbers.empty();  // false

// 遍历
for (int num : numbers) {      // 范围 for 循环（C++11）
    std::cout << num << std::endl;
}

// 传统循环
for (size_t i = 0; i < numbers.size(); ++i) {
    std::cout << numbers[i] << std::endl;
}
```

**初始化**：

```cpp
// 空向量
std::vector<Armor> armors;

// 预留空间（提高效率）
armors.reserve(10);

// 初始化列表
std::vector<int> nums = {1, 2, 3, 4, 5};

// 指定大小和初值
std::vector<float> values(100, 0.0f);  // 100 个 0.0
```

**项目实例**：

```cpp
// 检测到多个装甲板
std::vector<Armor> armors = detector.detect(frame);

// 处理每个装甲板
for (const Armor& armor : armors) {
    if (armor.isValid()) {
        std::cout << "装甲板中心: (" << armor.center.x 
                  << ", " << armor.center.y << ")" << std::endl;
    }
}
```

### 8.2 std::map（键值对）

**关联容器**，类似于 Python 的 dict。

**示例**：`include/utils/config_reader.hpp`

```cpp
class ConfigReader {
private:
    std::map<std::string, std::string> config_;  // 配置键值对
};
```

**基本操作**：

```cpp
std::map<std::string, int> scores;

// 插入元素
scores["Alice"] = 95;
scores["Bob"] = 87;
scores.insert({"Charlie", 92});

// 访问元素
int alice_score = scores["Alice"];   // 95

// 检查键是否存在
if (scores.find("David") != scores.end()) {
    // 找到了
} else {
    // 没找到
}

// 遍历
for (const auto& pair : scores) {
    std::cout << pair.first << ": " << pair.second << std::endl;
}
```

**项目实例**：

```cpp
// 配置键值对
std::map<std::string, std::string> config_;

config_["camera.width"] = "1280";
config_["camera.height"] = "720";
config_["detector.threshold"] = "80";

// 获取配置
std::string width_str = config_["camera.width"];
int width = std::stoi(width_str);  // 字符串转整数
```

### 8.3 std::string（字符串）

**C++ 风格的字符串**（比 C 风格的 `char*` 安全）。

**基本操作**：

```cpp
std::string name = "RoboMaster";

// 拼接
std::string greeting = "Hello, " + name + "!";

// 长度
int len = name.length();  // 或 name.size()

// 比较
if (name == "RoboMaster") {
    // ...
}

// 子串
std::string sub = name.substr(0, 5);  // "Robo"

// 查找
size_t pos = name.find("Master");     // 5
if (pos != std::string::npos) {
    // 找到了
}
```

**项目实例**：

```cpp
// 设置目标颜色
void setTargetColor(const std::string& color) {
    if (color == "red" || color == "blue") {
        target_color_ = color;
    }
}

// 配置文件路径
std::string config_path = config_dir_ + "detector_params.yaml";
```

---

## 第九章：内存管理与 RAII

### 9.1 RAII（Resource Acquisition Is Initialization）

**核心思想**：资源的获取即初始化，资源的释放在析构函数中自动完成。

**问题**：手动管理内存容易出错

```cpp
// ❌ 手动管理（容易忘记释放）
void processImage() {
    cv::Mat* image = new cv::Mat();
    // ... 处理图像
    delete image;  // 如果中途 return，会忘记释放！
}
```

**解决方案**：使用 RAII

```cpp
// ✅ RAII（自动管理）
void processImage() {
    cv::Mat image;  // 栈上分配，函数结束自动释放
    // ... 处理图像
}  // 自动调用析构函数，释放资源
```

### 9.2 析构函数

**示例**：`include/camera/hik_camera.hpp`

```cpp
class HikCamera {
public:
    HikCamera();   // 构造函数：获取资源
    ~HikCamera();  // 析构函数：释放资源
    
private:
    void* camera_handle_;
    cv::VideoCapture video_capture_;
};
```

**实现**：

```cpp
HikCamera::HikCamera() : camera_handle_(nullptr), is_opened_(false) {
    // 初始化
}

HikCamera::~HikCamera() {
    // 清理资源
    close();
    if (camera_handle_ != nullptr) {
        // 释放相机句柄
        camera_handle_ = nullptr;
    }
}
```

**自动调用**：

```cpp
void useCamera() {
    HikCamera camera;
    camera.open(0);
    // ... 使用相机
}  // camera 超出作用域，自动调用 ~HikCamera()，释放资源
```

### 9.3 智能指针（Smart Pointers）

**问题**：裸指针需要手动 delete

**解决**：使用智能指针自动管理

**std::unique_ptr**：独占所有权

```cpp
#include <memory>

std::unique_ptr<HikCamera> camera = std::make_unique<HikCamera>();
camera->open(0);
// 无需手动 delete，自动释放
```

**std::shared_ptr**：共享所有权

```cpp
std::shared_ptr<cv::Mat> image = std::make_shared<cv::Mat>();
std::shared_ptr<cv::Mat> image_copy = image;  // 引用计数 +1
// 当最后一个 shared_ptr 销毁时，资源才释放
```

**项目中的应用**：

虽然项目中大部分使用栈对象，但在需要动态分配时，可以使用智能指针：

```cpp
// ❌ 不好
cv::Mat* frame = new cv::Mat();
// ... 使用
delete frame;

// ✅ 更好
std::unique_ptr<cv::Mat> frame = std::make_unique<cv::Mat>();
// 自动释放
```

---

## 第十章：Doxygen 注释规范

### 10.1 为什么需要 Doxygen？

**Doxygen** 是一个文档生成工具，可以从代码注释生成 HTML/PDF 文档。

**好处**：
- 自动生成 API 文档
- 规范化注释格式
- 提高代码可读性

### 10.2 注释语法

**示例**：`include/detector/traditional_detector.hpp`

```cpp
/**
 * @brief 传统图像处理装甲板检测器
 * 使用颜色分割+轮廓检测+灯条匹配的方法检测装甲板
 */
class TraditionalDetector {
public:
    /**
     * @brief 构造函数
     * @param target_color 目标颜色："red" 或 "blue"
     */
    explicit TraditionalDetector(const std::string& target_color = "red");
    
    /**
     * @brief 检测装甲板
     * @param frame 输入图像
     * @return 检测到的装甲板列表
     */
    std::vector<Armor> detect(const cv::Mat& frame);
    
    /**
     * @brief 设置HSV颜色阈值
     * @param hsv_low HSV下限 [H, S, V]
     * @param hsv_high HSV上限 [H, S, V]
     */
    void setHSVThreshold(const cv::Scalar& hsv_low, const cv::Scalar& hsv_high);
};
```

### 10.3 常用标签

| 标签 | 用途 | 示例 |
|------|------|------|
| `@brief` | 简短描述 | `@brief 检测装甲板` |
| `@param` | 参数说明 | `@param frame 输入图像` |
| `@return` | 返回值说明 | `@return 检测到的装甲板列表` |
| `@note` | 注意事项 | `@note 输入图像必须是 BGR 格式` |
| `@warning` | 警告 | `@warning 线程不安全` |
| `@see` | 参考 | `@see KalmanFilter` |

### 10.4 完整示例

```cpp
/**
 * @brief 卡尔曼滤波器预测未来位置
 * 
 * 使用8维状态向量跟踪装甲板的位置和尺寸，
 * 支持预测前后移动。
 * 
 * @param future_time 未来时间（秒）
 * @return 预测结果，包含位置、尺寸和速度
 * 
 * @note 必须先调用 init() 初始化滤波器
 * @warning 如果 future_time < 0，会使用默认预测时间
 * 
 * @see PredictionResult
 */
PredictionResult predictFuture(double future_time);
```

---

## 第十一章：编码规范总结

### 11.1 命名规范

项目遵循以下命名规范：

| 类型 | 规范 | 示例 |
|------|------|------|
| 类名 | PascalCase | `TraditionalDetector`, `KalmanFilter` |
| 函数名 | camelCase | `detectArmor()`, `predictFuture()` |
| 成员变量 | snake_case_ | `target_color_`, `binary_threshold_` |
| 局部变量 | snake_case | `frame`, `light_bars` |
| 常量 | UPPER_CASE 或 kConstantName | `MAX_ARMORS`, `kDefaultThreshold` |
| 命名空间 | snake_case | `armor_detector` |
| 文件名 | snake_case.hpp | `traditional_detector.hpp` |

**示例**：

```cpp
namespace armor_detector {

const int MAX_ARMORS = 10;  // 常量

class TraditionalDetector {  // 类名：PascalCase
public:
    void detectArmor();      // 函数名：camelCase
    
private:
    int binary_threshold_;   // 成员变量：snake_case_
    std::string target_color_;
};

void processFrame() {        // 函数名：camelCase
    int frame_count = 0;     // 局部变量：snake_case
    cv::Mat current_frame;
}

}  // namespace armor_detector
```

### 11.2 代码组织规范

**类的成员排序**：

```cpp
class MyClass {
public:
    // 1. 构造函数和析构函数
    MyClass();
    ~MyClass();
    
    // 2. 公共接口函数
    void publicMethod();
    
    // 3. Getter/Setter
    int getValue() const { return value_; }
    void setValue(int v) { value_ = v; }

protected:
    // 4. 保护成员（如果有继承）
    void protectedMethod();

private:
    // 5. 私有辅助函数
    void helperMethod();
    
    // 6. 私有成员变量（末尾带下划线）
    int value_;
    std::string name_;
};
```

### 11.3 注释规范

**文件头注释**：

```cpp
/**
 * @file traditional_detector.hpp
 * @brief 传统图像处理装甲板检测器
 * @author Your Name
 * @date 2024-10-23
 */
```

**函数注释**：

```cpp
/**
 * @brief 函数的简短描述
 * 
 * 更详细的说明（可选）
 * 
 * @param param1 参数1的说明
 * @param param2 参数2的说明
 * @return 返回值说明
 */
```

**行内注释**：

```cpp
int threshold = 80;  // 二值化阈值

// 查找所有轮廓
std::vector<std::vector<cv::Point>> contours;
cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
```

### 11.4 最佳实践

**1. const 正确性**

```cpp
// ✅ 不修改的函数声明为 const
cv::Mat getImage() const;

// ✅ 不修改的参数使用 const 引用
void process(const cv::Mat& frame);
```

**2. 避免魔法数字**

```cpp
// ❌ 不好
if (threshold > 100) { ... }

// ✅ 好
const int MAX_THRESHOLD = 100;
if (threshold > MAX_THRESHOLD) { ... }
```

**3. 使用初始化列表**

```cpp
// ✅ 使用初始化列表
MyClass::MyClass(int value) : value_(value), name_("default") {
    // 构造函数体
}
```

**4. 合理使用命名空间**

```cpp
// ✅ 在 .cpp 文件中使用
using namespace armor_detector;

// ❌ 避免在头文件中使用
// using namespace std;  // 不要这样做！
```

---

## 实战练习

### 练习1：定义一个简单的类

**任务**：创建一个 `Rectangle` 类，封装矩形的宽度和高度。

**要求**：
1. 使用 private 成员变量 `width_` 和 `height_`
2. 提供构造函数初始化宽度和高度
3. 提供 getter 函数获取宽度和高度
4. 提供 `area()` 函数计算面积
5. 提供 `isSquare()` 函数判断是否为正方形

**参考答案**：

```cpp
class Rectangle {
public:
    // 构造函数（使用初始化列表）
    Rectangle(float width, float height) : width_(width), height_(height) {}
    
    // Getter（const 成员函数）
    float getWidth() const { return width_; }
    float getHeight() const { return height_; }
    
    // 计算面积
    float area() const {
        return width_ * height_;
    }
    
    // 判断是否为正方形
    bool isSquare() const {
        return width_ == height_;
    }
    
private:
    float width_;
    float height_;
};

// 使用
Rectangle rect(10.0f, 20.0f);
float area = rect.area();        // 200.0
bool is_square = rect.isSquare(); // false
```

### 练习2：使用 std::vector

**任务**：处理一组装甲板数据。

**要求**：
1. 创建一个 `std::vector<Armor>` 存储装甲板
2. 遍历所有装甲板，打印中心坐标
3. 找出置信度最高的装甲板
4. 过滤出宽度大于 100 的装甲板

**参考答案**：

```cpp
#include <vector>
#include <iostream>
#include <algorithm>

void processArmors(const std::vector<Armor>& armors) {
    // 1. 遍历打印
    for (const Armor& armor : armors) {
        std::cout << "中心: (" << armor.center.x << ", " 
                  << armor.center.y << ")" << std::endl;
    }
    
    // 2. 找最高置信度
    if (!armors.empty()) {
        auto max_it = std::max_element(armors.begin(), armors.end(),
            [](const Armor& a, const Armor& b) {
                return a.confidence < b.confidence;
            });
        std::cout << "最高置信度: " << max_it->confidence << std::endl;
    }
    
    // 3. 过滤宽度 > 100
    std::vector<Armor> wide_armors;
    for (const Armor& armor : armors) {
        if (armor.width > 100) {
            wide_armors.push_back(armor);
        }
    }
    std::cout << "宽装甲板数量: " << wide_armors.size() << std::endl;
}
```

### 练习3：理解 const 引用

**任务**：解释以下代码的区别。

```cpp
// 版本 1
void process1(cv::Mat frame);

// 版本 2
void process2(const cv::Mat& frame);

// 版本 3
void process3(cv::Mat& frame);
```

**答案**：
- **版本 1**：按值传递，会复制整个图像（效率低）
- **版本 2**：const 引用，不复制，不修改原图像（推荐用于只读）
- **版本 3**：非 const 引用，不复制，可以修改原图像（用于需要修改输入）

### 练习4：阅读项目代码

**任务**：阅读以下代码，理解其功能。

**文件**：`include/common/armor.hpp`

```cpp
struct LightBar {
    cv::RotatedRect rect;
    cv::Point2f center;
    float angle;
    
    explicit LightBar(const cv::RotatedRect& r) : rect(r) {
        center = r.center;
        angle = r.angle;
    }
    
    float aspectRatio() const {
        return width > 0 ? length / width : 0;
    }
};
```

**问题**：
1. `explicit` 的作用是什么？
2. 为什么使用初始化列表 `: rect(r)`？
3. `aspectRatio()` 为什么是 const 函数？

**答案**：
1. **explicit**：防止隐式类型转换，必须显式调用构造函数
2. **初始化列表**：直接构造成员，避免先默认构造再赋值（效率更高）
3. **const 函数**：承诺不修改对象状态，const 对象也可以调用

---

## 深入学习建议

### 推荐阅读顺序

1. **从结构体开始**：阅读 `include/common/armor.hpp`
   - 理解数据结构的定义
   - 学习简单的成员函数

2. **理解类的封装**：阅读 `include/camera/hik_camera.hpp`
   - 观察 public/private 的使用
   - 理解接口设计

3. **学习复杂逻辑**：阅读 `include/detector/traditional_detector.hpp`
   - 观察多个 setter/getter
   - 理解私有辅助函数

4. **阅读实现**：对照头文件阅读 `src/` 下的 `.cpp` 文件
   - 理解声明与实现的分离
   - 学习函数实现细节

5. **理解主程序**：阅读 `src/main.cpp`
   - 观察如何使用各个类
   - 理解整体程序流程

### 进阶主题

学完基础后，可以继续学习：

1. **模板编程**：理解 Eigen 库的模板使用
2. **异常处理**：项目中的错误处理策略
3. **CMake 构建**：理解 `CMakeLists.txt`
4. **多线程**：扩展项目支持并发处理
5. **设计模式**：识别项目中的设计模式

### 练习建议

1. **修改参数**：尝试修改配置文件，观察效果
2. **添加功能**：为 `Armor` 结构体添加新方法
3. **重构代码**：提取重复代码，抽象为函数
4. **编写测试**：为某个类编写单元测试
5. **扩展项目**：添加新的检测算法或优化方法

---

## 总结

通过学习 `armor_detector` 项目，您已经接触到了 C++ 的核心语法：

✅ **代码组织**：头文件、源文件、命名空间
✅ **面向对象**：类、封装、构造函数、析构函数
✅ **内存管理**：RAII、智能指针、栈对象
✅ **const 正确性**：const 函数、const 引用
✅ **STL 容器**：vector、map、string
✅ **编码规范**：命名规范、注释规范、代码风格

**下一步**：
1. 编译并运行项目，观察实际效果
2. 逐个阅读源文件，理解实现细节
3. 尝试修改代码，验证理解
4. 完成练习题，巩固知识

**记住**：最好的学习方法是**动手实践**！

---

*Happy Coding! 🚀*


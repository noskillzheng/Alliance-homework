# DevContainer 配置说明

## 概述

这个 devcontainer 配置使用 `stateoftheartio/qt6:6.6-gcc-apt` 镜像，提供了完整的 Qt6 开发环境。

## 使用方法

### 1. 在 VSCode 中打开项目

1. 确保已安装 VSCode 和 Dev Containers 扩展
2. 在 VSCode 中打开项目文件夹
3. 按 `Ctrl+Shift+P`，选择 "Dev Containers: Reopen in Container"
4. 等待容器启动和配置完成

### 2. 构建项目

容器启动后会自动执行构建，你也可以手动构建：

```bash
# 使用构建脚本
./.devcontainer/build.sh

# 或使用 CMake 命令
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

### 3. 运行游戏

```bash
# 使用运行脚本
./.devcontainer/run.sh

# 或直接运行
./build/qt-game
```

### 4. VSCode 任务

在 VSCode 中可以使用以下任务：

- `Ctrl+Shift+P` → "Tasks: Run Task" → "CMake: Build"
- `Ctrl+Shift+P` → "Tasks: Run Task" → "Run Game"
- `Ctrl+Shift+P` → "Tasks: Run Task" → "Clean Build"

### 5. 调试

- 按 `F5` 开始调试
- 或使用 "Run and Debug" 面板选择 "Debug Qt Game"

## 环境配置

### 包含的扩展

- **C++ 支持**: `ms-vscode.cpptools`, `llvm-vs-code-extensions.vscode-clangd`
- **CMake 支持**: `ms-vscode.cmake-tools`, `twxs.cmake`
- **Qt 支持**: `tonka3000.qtvsctools`
- **调试支持**: `ms-vscode.cpp-debug`
- **其他工具**: Git Graph, Code Spell Checker

### 环境变量

- `DISPLAY`: X11 显示设置
- `QT_QPA_PLATFORM`: Qt 平台插件 (xcb)

### 挂载点

- `/dev`: 设备文件系统
- `/tmp/.X11-unix`: X11 套接字

## 故障排除

### 1. 图形界面无法显示

```bash
# 检查 DISPLAY 变量
echo $DISPLAY

# 允许 X11 转发
xhost +local:docker
```

### 2. Qt 库找不到

```bash
# 检查 Qt 安装
find /opt -name "Qt6Config.cmake" 2>/dev/null
find /usr -name "Qt6Config.cmake" 2>/dev/null

# 设置 Qt 路径
export QT_DIR=/opt/qt6
```

### 3. 构建失败

```bash
# 清理构建目录
rm -rf build/*

# 重新配置
cd build
cmake ..
make -j$(nproc)
```

### 4. 权限问题

```bash
# 确保脚本有执行权限
chmod +x .devcontainer/*.sh
```

## 项目结构

```
.devcontainer/
├── devcontainer.json    # DevContainer 配置
├── build.sh            # 构建脚本
├── run.sh              # 运行脚本
└── README.md           # 说明文档

.vscode/
├── tasks.json          # VSCode 任务配置
├── launch.json         # 调试配置
└── settings.json       # 工作区设置
```

## 注意事项

1. **网络访问**: 容器使用 `--network host` 模式
2. **特权模式**: 容器以特权模式运行以支持调试
3. **X11 转发**: 需要 X11 服务器来显示图形界面
4. **自动构建**: 容器启动时会自动构建项目

## 支持的平台

- Linux (推荐)
- macOS (需要 XQuartz)
- Windows (需要 WSL2 + X11 服务器)



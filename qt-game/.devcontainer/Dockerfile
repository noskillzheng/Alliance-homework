# 基于Qt6镜像构建开发环境，添加OpenGL支持
FROM stateoftheartio/qt6:6.6-gcc-aqt

# 切换到root用户进行系统级安装
USER root

# 安装必要的系统依赖
RUN apt-get update && apt-get install -y \
    # OpenGL开发库 (必需)
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    # X11开发库 (必需)
    libx11-dev \
    libxext-dev \
    libxrender-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    # 系统开发库 (必需)
    libudev-dev \
    libxkbcommon-dev \
    libxkbcommon-x11-dev \
    # 调试工具
    gdb \
    # 清理缓存
    && rm -rf /var/lib/apt/lists/*

# 设置环境变量
ENV DISPLAY=:0
ENV QT_QPA_PLATFORM=xcb
ENV CMAKE_PREFIX_PATH=/opt/Qt/6.6.1/gcc_64/lib/cmake

# 设置工作目录
WORKDIR /workspace

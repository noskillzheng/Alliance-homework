#!/bin/bash
# 海康相机连接检查脚本

echo "========================================="
echo "  海康相机连接诊断工具"
echo "========================================="
echo ""

# 1. USB设备检查
echo "【1】检查USB设备..."
if lsusb | grep -iq hik; then
    echo "✅ 找到海康设备："
    lsusb | grep -i hik
else
    echo "❌ 未找到海康设备"
    echo "   请检查："
    echo "   - 相机是否插入"
    echo "   - USB线是否正常"
    echo "   - 是否需要外部供电"
fi
echo ""

# 2. Video设备检查
echo "【2】检查Video设备..."
if ls /dev/video* &>/dev/null; then
    echo "✅ 找到Video设备："
    ls -l /dev/video*
else
    echo "❌ 未找到Video设备"
fi
echo ""

# 3. 权限检查
echo "【3】检查用户权限..."
if groups $USER | grep -q video; then
    echo "✅ 用户在video组"
else
    echo "❌ 用户不在video组"
    echo "   执行: sudo usermod -a -G video $USER"
    echo "   然后重新登录"
fi
echo ""

# 4. SDK检查
echo "【4】检查海康SDK..."
if [ -d "/opt/MVS" ]; then
    echo "✅ SDK已安装：/opt/MVS"
    if [ -f "/opt/MVS/lib/64/libMvCameraControl.so" ]; then
        echo "✅ 核心库存在"
    else
        echo "❌ 核心库缺失"
    fi
else
    echo "❌ SDK未安装"
    echo "   请从海康官网下载并安装MVS SDK"
fi
echo ""

# 5. 库路径检查
echo "【5】检查库路径..."
if echo $LD_LIBRARY_PATH | grep -q MVS; then
    echo "✅ LD_LIBRARY_PATH已设置"
    echo "   当前值: $LD_LIBRARY_PATH"
else
    echo "⚠️  LD_LIBRARY_PATH未设置"
    echo "   执行: export LD_LIBRARY_PATH=/opt/MVS/lib/64:\$LD_LIBRARY_PATH"
    echo "   或运行: source setup_env.sh"
fi
echo ""

# 6. 最近的内核消息
echo "【6】最近的USB事件（最新10条）..."
dmesg | grep -i usb | tail -10
echo ""

# 7. 测试建议
echo "========================================="
echo "  测试建议"
echo "========================================="
echo ""
echo "如果以上检查都通过，尝试运行："
echo "  cd build"
echo "  ./armor_detector"
echo ""
echo "如果仍有问题："
echo "  sudo ./armor_detector  # 使用sudo运行"
echo ""
echo "如果没有海康相机，使用备用方案："
echo "  # 修改代码使用VideoCapture"
echo "  # 或使用测试视频"
echo ""
echo "========================================="





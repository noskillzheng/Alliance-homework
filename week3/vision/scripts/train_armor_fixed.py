#!/usr/bin/env python3
"""
修复版的装甲板识别模型训练脚本
优化内存使用和批次大小
"""

import os
import sys
from ultralytics import YOLO

def train_armor_detection_model():
    """训练装甲板检测模型 - 优化版"""
    print("🎯 开始训练装甲板识别模型（优化版）")
    print("=" * 50)

    # 检查数据集
    dataset_config = "dataset/yolo_dataset.yaml"
    if not os.path.exists(dataset_config):
        print(f"❌ 数据集配置文件不存在: {dataset_config}")
        return False

    print(f"✅ 数据集配置: {dataset_config}")

    # 加载YOLOv8模型
    print("🔄 加载YOLOv8模型...")
    model = YOLO('yolov8n.pt')

    # 优化的训练参数
    print("🚀 开始训练（内存优化版）...")

    try:
        # 训练模型 - 极限内存优化参数
        results = model.train(
            data=dataset_config,
            epochs=30,         # 进一步减少训练轮数
            imgsz=416,         # 减小图像尺寸
            batch=1,           # 最小批次大小
            lr0=0.01,          # 初始学习率
            name='armor_detector_fixed',
            project='runs/train',
            device='cpu',       # 明确使用CPU
            workers=1,         # 减少工作进程
            save_period=3,     # 更频繁保存
            plots=False,       # 禁用图表节省内存
            val=True,           # 启用验证
            patience=30,       # 更早停止
            cache=False,       # 禁用缓存节省内存
            amp=False,         # 禁用自动混合精度
            deterministic=True, # 确保可重现
            single_cls=True,   # 单一类别优化
            mosaic=0.0,        # 禁用mosaic数据增强
            mixup=0.0,         # 禁用mixup数据增强
            copy_paste=0.0,    # 禁用copy-paste增强
            hsv_h=0.0,         # 禁用HSV增强
            hsv_s=0.0,         # 禁用HSV增强
            hsv_v=0.0          # 禁用HSV增强
        )

        print("✅ 模型训练完成！")
        print(f"📁 模型保存位置: runs/train/armor_detector_fixed/weights/")

        # 导出ONNX格式
        print("🔄 导出ONNX格式...")
        model.export(
            format='onnx',
            imgsz=640,
            simplify=True,
            opset=11
        )

        print("✅ ONNX模型导出完成！")
        return True

    except Exception as e:
        print(f"❌ 训练过程中出现错误: {e}")
        print("💡 可能的原因:")
        print("   - 内存不足")
        print("   - 数据加载问题")
        print("   - 数据集格式错误")
        return False

def main():
    print("🎯 RoboMaster 装甲板识别模型训练（优化版）")
    print("=" * 50)

    # 检查环境
    try:
        import torch
        print(f"✅ PyTorch版本: {torch.__version__}")
        print(f"✅ CUDA可用: {torch.cuda.is_available()}")

        # 显示内存信息
        import psutil
        memory = psutil.virtual_memory()
        print(f"💾 可用内存: {memory.available // (1024**3):.1f} GB")

    except ImportError:
        print("❌ PyTorch未安装")
        return

    # 删除现有训练目录重新开始
    train_dir = "runs/train/armor_detector"
    if os.path.exists(train_dir):
        print(f"⚠️  检测到现有训练目录: {train_dir}")
        import shutil
        shutil.rmtree(train_dir)
        print(f"✅ 已删除: {train_dir}，重新开始训练")

    # 开始训练
    if train_armor_detection_model():
        print("\n🎉 训练完成！")
        print("\n📝 下一步操作:")
        print("1. 查看训练结果: runs/train/armor_detector_fixed/")
        print("2. 测试模型效果")
        print("3. 部署到C++应用")
    else:
        print("\n❌ 训练失败")

if __name__ == "__main__":
    main()
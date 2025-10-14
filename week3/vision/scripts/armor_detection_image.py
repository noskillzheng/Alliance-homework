#!/usr/bin/env python3
"""
RoboMaster 装甲板检测系统 - 图片检测版本
"""

import sys
import os
import argparse
import cv2
from pathlib import Path
from ultralytics import YOLO

def main():
    parser = argparse.ArgumentParser(
        description="🎯 RoboMaster 装甲板检测系统 - 图片检测版"
    )

    parser.add_argument('-i', '--image', required=True, help='检测指定图片')
    parser.add_argument('-m', '--model',
                       default='models/trained/best.pt',
                       help='指定模型文件路径 (默认: models/trained/best.pt)')
    parser.add_argument('-o', '--output', default='./results/',
                       help='指定输出路径 (默认: ./results/)')
    parser.add_argument('-c', '--confidence', type=float, default=0.3,
                       help='置信度阈值 (默认: 0.3)')

    args = parser.parse_args()

    # 检查模型文件
    if not os.path.exists(args.model):
        print(f"❌ 模型文件不存在: {args.model}")
        return 1

    # 检查图片文件
    if not os.path.exists(args.image):
        print(f"❌ 图片文件不存在: {args.image}")
        return 1

    # 创建输出目录
    os.makedirs(args.output, exist_ok=True)

    print("🎯 RoboMaster 装甲板检测系统 - 图片检测版")
    print("=" * 50)
    print(f"📸 图片路径: {args.image}")
    print(f"🧠 模型路径: {args.model}")
    print(f"🎯 置信度阈值: {args.confidence}")
    print(f"📁 输出路径: {args.output}")
    print()

    # 加载模型
    print("🔄 正在加载模型...")
    try:
        model = YOLO(args.model)
        print("✅ 模型加载成功")
    except Exception as e:
        print(f"❌ 模型加载失败: {e}")
        return 1

    # 读取图片
    print("📖 正在读取图片...")
    try:
        image = cv2.imread(args.image)
        if image is None:
            print("❌ 无法读取图片")
            return 1
        print(f"✅ 图片读取成功，尺寸: {image.shape[1]}x{image.shape[0]}")
    except Exception as e:
        print(f"❌ 图片读取失败: {e}")
        return 1

    # 进行检测
    print("🔍 正在进行检测...")
    try:
        results = model(image, conf=args.confidence)
        print("✅ 检测完成")
    except Exception as e:
        print(f"❌ 检测失败: {e}")
        return 1

    # 处理结果
    print("📊 正在处理结果...")
    detection_count = 0
    
    # 在图片上绘制检测结果
    for result in results:
        boxes = result.boxes
        if boxes is not None:
            detection_count = len(boxes)
            for box in boxes:
                # 获取边界框坐标
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = box.conf[0].cpu().numpy()
                
                # 绘制边界框
                cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                
                # 添加标签
                label = f"Armor {conf:.2f}"
                cv2.putText(image, label, (int(x1), int(y1) - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # 添加检测信息
    info_text = f"检测到 {detection_count} 个装甲板"
    cv2.putText(image, info_text, (10, 30), 
               cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

    # 保存结果
    output_filename = f"result_{Path(args.image).name}"
    output_path = os.path.join(args.output, output_filename)
    
    try:
        cv2.imwrite(output_path, image)
        print(f"✅ 结果已保存: {output_path}")
    except Exception as e:
        print(f"❌ 保存结果失败: {e}")
        return 1

    print()
    print("🎉 检测完成！")
    print(f"📊 检测到 {detection_count} 个装甲板")
    print(f"💾 结果已保存到: {output_path}")

    return 0

if __name__ == "__main__":
    sys.exit(main())

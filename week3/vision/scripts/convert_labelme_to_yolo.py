#!/usr/bin/env python3
"""
将labelme标注转换为YOLO格式
支持装甲板检测模型训练
"""

import json
import os
import glob
import shutil
import numpy as np
from pathlib import Path

def labelme_to_yolo(labelme_json_path):
    """
    将单个labelme标注文件转换为YOLO格式

    Args:
        labelme_json_path: labelme标注文件路径

    Returns:
        yolo_lines: YOLO格式的标注行列表
    """
    with open(labelme_json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    # 获取图像尺寸
    width = data['imageWidth']
    height = data['imageHeight']

    yolo_lines = []

    # 处理每个标注
    for shape in data['shapes']:
        if shape['shape_type'] not in ['polygon', 'rectangle']:
            continue

        if shape['shape_type'] == 'polygon':
            # 获取多边形顶点
            points = shape['points']
            if len(points) < 3:
                continue

            # 计算多边形的最小外接矩形
            points_np = np.array(points)
            min_x = np.min(points_np[:, 0])
            max_x = np.max(points_np[:, 0])
            min_y = np.min(points_np[:, 1])
            max_y = np.max(points_np[:, 1])

        elif shape['shape_type'] == 'rectangle':
            # 矩形标注
            points = shape['points']
            if len(points) != 2:
                continue

            min_x = min(points[0][0], points[1][0])
            max_x = max(points[0][0], points[1][0])
            min_y = min(points[0][1], points[1][1])
            max_y = max(points[0][1], points[1][1])

        # 计算YOLO格式坐标（归一化）
        center_x = (min_x + max_x) / 2.0 / width
        center_y = (min_y + max_y) / 2.0 / height
        bbox_width = (max_x - min_x) / width
        bbox_height = (max_y - min_y) / height

        # 类别ID（armor = 0）
        class_id = 0

        # YOLO格式: class_id center_x center_y width height
        yolo_line = f"{class_id} {center_x:.6f} {center_y:.6f} {bbox_width:.6f} {bbox_height:.6f}"
        yolo_lines.append(yolo_line)

    return yolo_lines

def create_yolo_dataset():
    """创建YOLO格式的训练数据集"""
    print("🎯 转换labelme标注为YOLO格式")

    # 源目录和目标目录
    source_dir = "dataset/raw_images"
    target_dir = "dataset/yolo_dataset"

    # 创建YOLO数据集目录结构
    dirs = [
        f"{target_dir}/images/train",
        f"{target_dir}/images/val",
        f"{target_dir}/labels/train",
        f"{target_dir}/labels/val"
    ]

    for dir_path in dirs:
        os.makedirs(dir_path, exist_ok=True)

    # 查找所有labelme标注文件
    json_files = glob.glob(os.path.join(source_dir, "*.json"))
    print(f"📁 找到 {len(json_files)} 个标注文件")

    if len(json_files) == 0:
        print("❌ 没有找到标注文件")
        return False

    # 数据集划分（80%训练，20%验证）
    split_idx = int(len(json_files) * 0.8)
    train_files = json_files[:split_idx]
    val_files = json_files[split_idx:]

    print(f"📊 数据集划分:")
    print(f"   训练集: {len(train_files)} 张")
    print(f"   验证集: {len(val_files)} 张")

    # 转换训练集
    train_count = 0
    for json_file in train_files:
        # 转换标注
        yolo_lines = labelme_to_yolo(json_file)
        if not yolo_lines:
            continue

        # 保存YOLO标注文件
        base_name = os.path.splitext(os.path.basename(json_file))[0]
        image_file = base_name

        # 查找对应的图像文件（不区分大小写）
        possible_extensions = ['.jpg', '.png', '.jpeg', '.bmp']
        image_path = None
        for ext in possible_extensions:
            test_path = os.path.join(source_dir, image_file + ext)
            test_path_lower = os.path.join(source_dir, image_file.lower() + ext)
            test_path_upper = os.path.join(source_dir, image_file.upper() + ext)

            if os.path.exists(test_path):
                image_path = test_path
                break
            elif os.path.exists(test_path_lower):
                image_path = test_path_lower
                break
            elif os.path.exists(test_path_upper):
                image_path = test_path_upper
                break

        if image_path is None:
            print(f"⚠️  找不到图像文件: {image_file}")
            continue

        # 复制图像到训练集
        target_image_path = os.path.join(f"{target_dir}/images/train", os.path.basename(image_path))
        shutil.copy2(image_path, target_image_path)

        # 保存YOLO标注
        target_label_path = os.path.join(f"{target_dir}/labels/train", base_name + '.txt')
        with open(target_label_path, 'w') as f:
            for line in yolo_lines:
                f.write(line + '\n')

        train_count += 1

    # 转换验证集
    val_count = 0
    for json_file in val_files:
        # 转换标注
        yolo_lines = labelme_to_yolo(json_file)
        if not yolo_lines:
            continue

        # 保存YOLO标注文件
        base_name = os.path.splitext(os.path.basename(json_file))[0]
        image_file = base_name

        # 查找对应的图像文件（不区分大小写）
        possible_extensions = ['.jpg', '.png', '.jpeg', '.bmp']
        image_path = None
        for ext in possible_extensions:
            test_path = os.path.join(source_dir, image_file + ext)
            test_path_lower = os.path.join(source_dir, image_file.lower() + ext)
            test_path_upper = os.path.join(source_dir, image_file.upper() + ext)

            if os.path.exists(test_path):
                image_path = test_path
                break
            elif os.path.exists(test_path_lower):
                image_path = test_path_lower
                break
            elif os.path.exists(test_path_upper):
                image_path = test_path_upper
                break

        if image_path is None:
            print(f"⚠️  找不到图像文件: {image_file}")
            continue

        # 复制图像到验证集
        target_image_path = os.path.join(f"{target_dir}/images/val", os.path.basename(image_path))
        shutil.copy2(image_path, target_image_path)

        # 保存YOLO标注
        target_label_path = os.path.join(f"{target_dir}/labels/val", base_name + '.txt')
        with open(target_label_path, 'w') as f:
            for line in yolo_lines:
                f.write(line + '\n')

        val_count += 1

    print(f"✅ 转换完成！")
    print(f"   训练集: {train_count} 张图像")
    print(f"   验证集: {val_count} 张图像")
    print(f"   数据集目录: {target_dir}")

    return True

def create_dataset_yaml():
    """创建YOLO数据集配置文件"""
    yaml_content = """# RoboMaster 装甲板数据集配置
path: ./dataset/yolo_dataset
train: images/train
val: images/val

nc: 1  # 类别数量
names: ['armor']  # 装甲板类别

# 数据集信息
dataset_info:
  name: "RoboMaster Armor Detection"
  version: "1.0"
  author: "Vision Team"
  description: "装甲板目标检测数据集"
"""

    yaml_path = "dataset/yolo_dataset.yaml"
    with open(yaml_path, 'w', encoding='utf-8') as f:
        f.write(yaml_content)

    print(f"✅ 数据集配置文件: {yaml_path}")

def main():
    print("🎯 Labelme to YOLO 转换工具")
    print("=" * 50)

    # 检查源目录
    source_dir = "dataset/raw_images"
    if not os.path.exists(source_dir):
        print(f"❌ 源目录不存在: {source_dir}")
        return

    # 转换数据集
    if create_yolo_dataset():
        # 创建配置文件
        create_dataset_yaml()

        print("\n🎉 数据集转换完成！")
        print("\n📝 下一步操作:")
        print("1. 检查转换结果:")
        print("   - dataset/yolo_dataset/images/train/")
        print("   - dataset/yolo_dataset/labels/train/")
        print("   - dataset/yolo_dataset.yaml")
        print("2. 准备训练模型")
        print("3. 告诉我继续，我将开始训练装甲板识别模型")
    else:
        print("❌ 数据集转换失败")

if __name__ == "__main__":
    main()
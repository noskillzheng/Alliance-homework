#!/usr/bin/env python3
"""
将PyTorch模型转换为ONNX格式
用于C++版本兼容
"""

import os
import sys
from pathlib import Path
from ultralytics import YOLO

def convert_pt_to_onnx(pt_model_path, output_dir="models"):
    """
    将PyTorch模型转换为ONNX格式
    
    Args:
        pt_model_path: PyTorch模型路径
        output_dir: 输出目录
    """
    print(f"🔄 开始转换模型: {pt_model_path}")
    
    # 检查输入文件
    if not os.path.exists(pt_model_path):
        print(f"❌ 模型文件不存在: {pt_model_path}")
        return False
    
    try:
        # 加载YOLO模型
        print("📥 加载YOLO模型...")
        model = YOLO(pt_model_path)
        
        # 获取模型名称
        model_name = Path(pt_model_path).stem
        onnx_path = os.path.join(output_dir, f"{model_name}.onnx")
        
        # 确保输出目录存在
        os.makedirs(output_dir, exist_ok=True)
        
        # 导出为ONNX格式
        print(f"🔄 导出为ONNX格式: {onnx_path}")
        model.export(format='onnx', imgsz=416, optimize=True)
        
        # 检查导出结果
        if os.path.exists(onnx_path):
            print(f"✅ ONNX模型导出成功: {onnx_path}")
            print(f"📊 文件大小: {os.path.getsize(onnx_path) / 1024 / 1024:.2f} MB")
            return True
        else:
            print("❌ ONNX模型导出失败")
            return False
            
    except Exception as e:
        print(f"❌ 转换过程中出错: {e}")
        return False

def main():
    """主函数"""
    print("🎯 PyTorch模型转ONNX工具")
    print("=" * 40)
    
    # 检查models目录
    models_dir = "models"
    if not os.path.exists(models_dir):
        print(f"❌ 模型目录不存在: {models_dir}")
        return
    
    # 查找所有.pt文件
    pt_files = []
    for file in os.listdir(models_dir):
        if file.endswith('.pt'):
            pt_files.append(os.path.join(models_dir, file))
    
    if not pt_files:
        print("❌ 未找到.pt模型文件")
        return
    
    print(f"📁 找到 {len(pt_files)} 个PyTorch模型:")
    for pt_file in pt_files:
        print(f"  - {pt_file}")
    
    # 转换每个模型
    success_count = 0
    for pt_file in pt_files:
        print(f"\n🔄 处理: {pt_file}")
        if convert_pt_to_onnx(pt_file, models_dir):
            success_count += 1
    
    print(f"\n✅ 转换完成! 成功转换 {success_count}/{len(pt_files)} 个模型")
    
    # 列出所有ONNX文件
    print("\n📋 生成的ONNX文件:")
    for file in os.listdir(models_dir):
        if file.endswith('.onnx'):
            file_path = os.path.join(models_dir, file)
            size_mb = os.path.getsize(file_path) / 1024 / 1024
            print(f"  - {file} ({size_mb:.2f} MB)")

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轨迹数据分析脚本（无外部依赖）
"""

import csv
import sys
import math

def read_csv(filename):
    """读取CSV文件"""
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        return list(reader)

def analyze_trajectory(trajectory_file='trajectory.csv', waypoints_file='waypoints.csv'):
    """分析轨迹数据"""
    try:
        traj_data = read_csv(trajectory_file)
        waypoints = read_csv(waypoints_file)
    except FileNotFoundError as e:
        print(f"错误: 找不到文件 {e}")
        return
    
    print("\n" + "="*50)
    print("       轨迹分析报告")
    print("="*50)
    
    # 路径点
    print(f"\n【路径点】({len(waypoints)} 个)")
    for i, wp in enumerate(waypoints):
        print(f"  P{i}: ({float(wp['x']):.3f}, {float(wp['y']):.3f})")
    
    # 提取速度和加速度数据
    speeds = [float(row['speed']) for row in traj_data]
    accelerations = [float(row['acceleration']) for row in traj_data]
    
    # 时间统计
    total_time = float(traj_data[-1]['time'])
    print(f"\n【时间统计】")
    print(f"  总时间: {total_time:.4f} s")
    
    # 速度统计
    max_speed = max(speeds)
    avg_speed = sum(speeds) / len(speeds)
    print(f"\n【速度统计】")
    print(f"  最大速度: {max_speed:.4f} m/s")
    print(f"  平均速度: {avg_speed:.4f} m/s")
    print(f"  速度限制: 1.0000 m/s")
    print(f"  约束满足: {'✓' if max_speed <= 1.0 else '✗'}")
    
    # 加速度统计
    max_acc = max(accelerations)
    avg_acc = sum(accelerations) / len(accelerations)
    print(f"\n【加速度统计】")
    print(f"  最大加速度: {max_acc:.4f} m/s²")
    print(f"  平均加速度: {avg_acc:.4f} m/s²")
    print(f"  加速度限制: 1.0000 m/s²")
    print(f"  约束满足: {'✓' if max_acc <= 1.0 else '✗'}")
    
    # 路径长度
    path_length = 0.0
    for i in range(len(traj_data) - 1):
        x1 = float(traj_data[i]['x'])
        y1 = float(traj_data[i]['y'])
        x2 = float(traj_data[i+1]['x'])
        y2 = float(traj_data[i+1]['y'])
        path_length += math.sqrt((x2-x1)**2 + (y2-y1)**2)
    
    print(f"\n【轨迹统计】")
    print(f"  路径长度: {path_length:.4f} m")
    print(f"  数据点数: {len(traj_data)}")
    
    print("\n" + "="*50)
    print("✓ 数据分析完成")
    print("="*50 + "\n")

if __name__ == '__main__':
    if len(sys.argv) > 2:
        analyze_trajectory(sys.argv[1], sys.argv[2])
    else:
        analyze_trajectory()


#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轨迹可视化脚本 - 中文版本
使用matplotlib绘制优化后的轨迹
需要安装中文字体
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import sys
from pathlib import Path

# 配置中文字体
try:
    plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'DejaVu Sans', 'SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
    plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题
    use_chinese = True
except:
    print("警告: 中文字体未安装，使用英文显示")
    use_chinese = False

def plot_trajectory(trajectory_file='trajectory.csv', waypoints_file='waypoints.csv'):
    """
    绘制轨迹和路径点
    
    Args:
        trajectory_file: 轨迹CSV文件路径
        waypoints_file: 路径点CSV文件路径
    """
    # 读取数据
    try:
        traj_data = pd.read_csv(trajectory_file)
        waypoints = pd.read_csv(waypoints_file)
    except FileNotFoundError as e:
        print(f"错误: 找不到文件 {e.filename}")
        print("请先运行轨迹规划程序生成CSV文件")
        return
    
    # 创建图形
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    if use_chinese:
        fig.suptitle('哨兵导航 - 离散点平滑轨迹规划结果', fontsize=16, fontweight='bold')
        
        # 1. 轨迹路径 (x-y平面)
        ax1 = axes[0, 0]
        ax1.plot(traj_data['x'], traj_data['y'], 'b-', linewidth=2, label='优化轨迹')
        ax1.plot(waypoints['x'], waypoints['y'], 'ro', markersize=10, label='路径点', zorder=5)
        
        for i, (x, y) in enumerate(zip(waypoints['x'], waypoints['y'])):
            ax1.annotate(f'P{i}', (x, y), xytext=(5, 5), textcoords='offset points',
                        fontsize=10, fontweight='bold')
        
        ax1.set_xlabel('X (m)', fontsize=12)
        ax1.set_ylabel('Y (m)', fontsize=12)
        ax1.set_title('轨迹路径', fontsize=14, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.legend(fontsize=10)
        ax1.axis('equal')
        
        # 2. 速度随时间变化
        ax2 = axes[0, 1]
        ax2.plot(traj_data['time'], traj_data['speed'], 'g-', linewidth=2, label='速度模')
        ax2.axhline(y=1.0, color='r', linestyle='--', linewidth=1.5, label='速度限制')
        ax2.set_xlabel('时间 (s)', fontsize=12)
        ax2.set_ylabel('速度 (m/s)', fontsize=12)
        ax2.set_title('速度曲线', fontsize=14, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.legend(fontsize=10)
        ax2.set_ylim(bottom=0)
        
        # 3. 加速度随时间变化
        ax3 = axes[1, 0]
        ax3.plot(traj_data['time'], traj_data['acceleration'], 'orange', linewidth=2, label='加速度模')
        ax3.axhline(y=1.0, color='r', linestyle='--', linewidth=1.5, label='加速度限制')
        ax3.set_xlabel('时间 (s)', fontsize=12)
        ax3.set_ylabel('加速度 (m/s²)', fontsize=12)
        ax3.set_title('加速度曲线', fontsize=14, fontweight='bold')
        ax3.grid(True, alpha=0.3)
        ax3.legend(fontsize=10)
        ax3.set_ylim(bottom=0)
        
        # 4. 速度分量
        ax4 = axes[1, 1]
        ax4.plot(traj_data['time'], traj_data['vx'], 'b-', linewidth=2, label='vx')
        ax4.plot(traj_data['time'], traj_data['vy'], 'r-', linewidth=2, label='vy')
        ax4.set_xlabel('时间 (s)', fontsize=12)
        ax4.set_ylabel('速度分量 (m/s)', fontsize=12)
        ax4.set_title('速度分量', fontsize=14, fontweight='bold')
        ax4.grid(True, alpha=0.3)
        ax4.legend(fontsize=10)
    else:
        # English version
        fig.suptitle('Sentry Navigation - Trajectory Planning Results', fontsize=16, fontweight='bold')
        
        ax1 = axes[0, 0]
        ax1.plot(traj_data['x'], traj_data['y'], 'b-', linewidth=2, label='Trajectory')
        ax1.plot(waypoints['x'], waypoints['y'], 'ro', markersize=10, label='Waypoints', zorder=5)
        for i, (x, y) in enumerate(zip(waypoints['x'], waypoints['y'])):
            ax1.annotate(f'P{i}', (x, y), xytext=(5, 5), textcoords='offset points')
        ax1.set_xlabel('X (m)', fontsize=12)
        ax1.set_ylabel('Y (m)', fontsize=12)
        ax1.set_title('Path', fontsize=14, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.legend(fontsize=10)
        ax1.axis('equal')
        
        ax2 = axes[0, 1]
        ax2.plot(traj_data['time'], traj_data['speed'], 'g-', linewidth=2, label='Speed')
        ax2.axhline(y=1.0, color='r', linestyle='--', linewidth=1.5, label='Limit')
        ax2.set_xlabel('Time (s)', fontsize=12)
        ax2.set_ylabel('Speed (m/s)', fontsize=12)
        ax2.set_title('Velocity', fontsize=14, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.legend(fontsize=10)
        ax2.set_ylim(bottom=0)
        
        ax3 = axes[1, 0]
        ax3.plot(traj_data['time'], traj_data['acceleration'], 'orange', linewidth=2, label='Accel')
        ax3.axhline(y=1.0, color='r', linestyle='--', linewidth=1.5, label='Limit')
        ax3.set_xlabel('Time (s)', fontsize=12)
        ax3.set_ylabel('Acceleration (m/s^2)', fontsize=12)
        ax3.set_title('Acceleration', fontsize=14, fontweight='bold')
        ax3.grid(True, alpha=0.3)
        ax3.legend(fontsize=10)
        ax3.set_ylim(bottom=0)
        
        ax4 = axes[1, 1]
        ax4.plot(traj_data['time'], traj_data['vx'], 'b-', linewidth=2, label='vx')
        ax4.plot(traj_data['time'], traj_data['vy'], 'r-', linewidth=2, label='vy')
        ax4.set_xlabel('Time (s)', fontsize=12)
        ax4.set_ylabel('Velocity (m/s)', fontsize=12)
        ax4.set_title('Components', fontsize=14, fontweight='bold')
        ax4.grid(True, alpha=0.3)
        ax4.legend(fontsize=10)
    
    plt.tight_layout()
    
    # 保存图片
    output_file = 'trajectory_visualization.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"图片已保存到: {output_file}")
    
    # 显示图形
    plt.show()
    
    # 打印统计信息
    print("\n========== 轨迹统计信息 ==========")
    print(f"总时间: {traj_data['time'].iloc[-1]:.4f} s")
    print(f"最大速度: {traj_data['speed'].max():.4f} m/s")
    print(f"最大加速度: {traj_data['acceleration'].max():.4f} m/s²")
    print(f"路径长度: {len(waypoints)} 个点")
    print("==================================\n")

if __name__ == '__main__':
    if len(sys.argv) > 2:
        plot_trajectory(sys.argv[1], sys.argv[2])
    else:
        plot_trajectory()


#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轨迹可视化脚本
使用matplotlib绘制优化后的轨迹
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import sys
from pathlib import Path

# 配置中文字体
plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'DejaVu Sans', 'SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

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
    
    # 创建图形（2x3布局以显示更多信息）
    fig, axes = plt.subplots(2, 3, figsize=(20, 10))
    fig.suptitle('Sentry Navigation - Trajectory Planning Results', fontsize=16, fontweight='bold')
    
    # 1. 轨迹路径 (x-y平面) - 速度热力图
    ax1 = axes[0, 0]
    # 按速度着色
    scatter = ax1.scatter(traj_data['x'], traj_data['y'], c=traj_data['speed'], 
                         cmap='jet', s=5, alpha=0.6, label='Trajectory')
    ax1.plot(waypoints['x'], waypoints['y'], 'ro', markersize=10, label='Waypoints', zorder=5)
    plt.colorbar(scatter, ax=ax1, label='Speed (m/s)')
    
    # 标注路径点
    for i, (x, y) in enumerate(zip(waypoints['x'], waypoints['y'])):
        ax1.annotate(f'P{i}', (x, y), xytext=(5, 5), textcoords='offset points',
                    fontsize=10, fontweight='bold')
    
    ax1.set_xlabel('X (m)', fontsize=12)
    ax1.set_ylabel('Y (m)', fontsize=12)
    ax1.set_title('Trajectory Path', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=10)
    ax1.axis('equal')
    
    # 2. 速度随时间变化
    ax2 = axes[0, 1]
    ax2.plot(traj_data['time'], traj_data['speed'], 'g-', linewidth=2, label='Speed')
    ax2.axhline(y=1.0, color='r', linestyle='--', linewidth=1.5, label='Speed Limit')
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Speed (m/s)', fontsize=12)
    ax2.set_title('Velocity Profile', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=10)
    ax2.set_ylim(bottom=0)
    
    # 3. 加速度随时间变化
    ax3 = axes[1, 0]
    ax3.plot(traj_data['time'], traj_data['acceleration'], 'orange', linewidth=2, label='Acceleration')
    ax3.axhline(y=1.0, color='r', linestyle='--', linewidth=1.5, label='Accel Limit')
    ax3.set_xlabel('Time (s)', fontsize=12)
    ax3.set_ylabel('Acceleration (m/s^2)', fontsize=12)
    ax3.set_title('Acceleration Profile', fontsize=14, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend(fontsize=10)
    ax3.set_ylim(bottom=0)
    
    # 4. 速度分量
    ax4 = axes[1, 1]
    ax4.plot(traj_data['time'], traj_data['vx'], 'b-', linewidth=2, label='vx')
    ax4.plot(traj_data['time'], traj_data['vy'], 'r-', linewidth=2, label='vy')
    ax4.set_xlabel('Time (s)', fontsize=12)
    ax4.set_ylabel('Velocity Components (m/s)', fontsize=12)
    ax4.set_title('Velocity Components', fontsize=14, fontweight='bold')
    ax4.grid(True, alpha=0.3)
    ax4.legend(fontsize=10)
    
    # 5. 曲率曲线（新增）
    ax5 = axes[0, 2]
    if 'curvature' in traj_data.columns:
        ax5.plot(traj_data['time'], traj_data['curvature'], 'purple', linewidth=2, label='Curvature')
        ax5.set_xlabel('Time (s)', fontsize=12)
        ax5.set_ylabel('Curvature (1/m)', fontsize=12)
        ax5.set_title('Curvature Profile', fontsize=14, fontweight='bold')
        ax5.grid(True, alpha=0.3)
        ax5.legend(fontsize=10)
        ax5.set_ylim(bottom=0)
    
    # 6. 利用率热力图（新增）
    ax6 = axes[1, 2]
    if 'v_util' in traj_data.columns and 'a_util' in traj_data.columns:
        v_percent = traj_data['v_util'] / 1.0 * 100  # 假设max=1.0
        a_percent = traj_data['a_util'] / 1.0 * 100
        ax6.fill_between(traj_data['time'], 0, v_percent, alpha=0.3, color='green', label='Velocity %')
        ax6.fill_between(traj_data['time'], 0, a_percent, alpha=0.3, color='orange', label='Acceleration %')
        ax6.axhline(y=100, color='r', linestyle='--', linewidth=1, label='100% Limit')
        ax6.set_xlabel('Time (s)', fontsize=12)
        ax6.set_ylabel('Utilization (%)', fontsize=12)
        ax6.set_title('Constraint Utilization', fontsize=14, fontweight='bold')
        ax6.grid(True, alpha=0.3)
        ax6.legend(fontsize=10)
        ax6.set_ylim(0, 110)
    
    plt.tight_layout()
    
    # 保存图片
    output_file = 'trajectory_visualization.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Image saved to: {output_file}")
    
    # 显示图形
    plt.show()
    
    # 打印统计信息
    print("\n========== Trajectory Statistics ==========")
    print(f"Total Time: {traj_data['time'].iloc[-1]:.4f} s")
    print(f"Max Velocity: {traj_data['speed'].max():.4f} m/s")
    print(f"Max Acceleration: {traj_data['acceleration'].max():.4f} m/s^2")
    print(f"Number of Waypoints: {len(waypoints)}")
    print("==========================================\n")

if __name__ == '__main__':
    if len(sys.argv) > 2:
        plot_trajectory(sys.argv[1], sys.argv[2])
    else:
        plot_trajectory()


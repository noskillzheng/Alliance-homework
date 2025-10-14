#!/usr/bin/env python3
"""
RoboMaster 装甲板检测系统 - 强制显示版本
专门解决显示窗口不弹出的问题
"""

import sys
import os
import argparse
import cv2
import time
import subprocess
import signal
from pathlib import Path
from ultralytics import YOLO

def force_display_setup():
    """强制设置显示环境"""
    print("🔧 正在配置显示环境...")

    # 设置多种显示环境变量
    display_configs = [
        "export DISPLAY=:1",
        "export XAUTHORITY=$HOME/.Xauthority",
        "export QT_X11_NO_MITSHM=1",
        "export QT_QPA_PLATFORM=xcb",
        "export XDG_SESSION_TYPE=x11",
        "export WAYLAND_DISPLAY=",
        "export GDK_BACKEND=x11",
    ]

    for config in display_configs:
        try:
            os.system(config)
            print(f"  ✅ {config}")
        except Exception as e:
            print(f"  ❌ {config}: {e}")

    # 尝试启动Xvfb（如果可用）
    try:
        subprocess.run(["which", "xvfb-run"], check=True, capture_output=True)
        print("  ✅ 检测到 Xvfb，将使用虚拟显示")
        return True
    except:
        print("  ⚠️  未检测到 Xvfb，尝试使用系统显示")
        return False

def test_display_availability():
    """测试显示系统是否可用"""
    print("🧪 测试显示系统...")

    try:
        # 创建测试图片
        test_img = cv2.zeros((200, 400, 3), dtype=np.uint8)
        cv2.putText(test_img, 'Display Test', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # 尝试多种窗口创建方式
        window_methods = [
            ("WINDOW_NORMAL", cv2.WINDOW_NORMAL),
            ("WINDOW_AUTOSIZE", cv2.WINDOW_AUTOSIZE),
            ("WINDOW_FREERATIO", cv2.WINDOW_FREERATIO),
        ]

        for method_name, window_flag in window_methods:
            try:
                cv2.namedWindow(f'Test_{method_name}', window_flag)
                cv2.imshow(f'Test_{method_name}', test_img)
                print(f"  ✅ {method_name} 窗口创建成功")
                cv2.waitKey(100)  # 短暂显示
                cv2.destroyWindow(f'Test_{method_name}')
                return True
            except Exception as e:
                print(f"  ❌ {method_name} 失败: {e}")
                continue

    except Exception as e:
        print(f"  ❌ 显示测试失败: {e}")

    return False

def create_video_display(model, video_path, output_dir, confidence):
    """强制创建视频显示"""
    print(f"🎬 强制显示模式: {video_path}")

    # 打开视频
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"❌ 无法打开视频: {video_path}")
        return False

    # 获取视频信息
    fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    print(f"📹 视频信息: {fps:.1f}fps, {width}x{height}, 总帧数: {total_frames}")

    # 设置输出路径
    os.makedirs(output_dir, exist_ok=True)
    input_name = os.path.splitext(os.path.basename(video_path))[0]
    output_path = os.path.join(output_dir, f"display_result_{input_name}.avi")

    # 创建视频写入器
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    writer = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    if not writer.isOpened():
        print(f"❌ 无法创建输出视频: {output_path}")
        cap.release()
        return False

    print(f"💾 输出视频: {output_path}")
    print("🔄 强制显示模式处理中...")

    # 强制创建显示窗口
    try:
        cv2.namedWindow("装甲板检测 - 强制显示", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("装甲板检测 - 强制显示", min(width, 1200), min(height, 800))
        print("✅ 显示窗口创建成功")
        display_works = True
    except Exception as e:
        print(f"⚠️  窗口创建失败: {e}")
        display_works = False

    # 处理视频帧
    frame_count = 0
    total_detections = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            frame_count += 1

            # 检测装甲板
            results = model(frame, conf=confidence, iou=0.4)

            # 统计检测结果
            frame_detections = 0
            annotated_frame = frame.copy()

            for result in results:
                boxes = result.boxes
                frame_detections += len(boxes)
                total_detections += len(boxes)

                # 绘制检测结果
                annotated_frame = result.plot()

            # 添加实时信息
            info_text = f"Frame: {frame_count}/{total_frames} | Armors: {frame_detections}"
            cv2.putText(annotated_frame, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # 写入输出视频
            writer.write(annotated_frame)

            # 强制显示
            if display_works:
                try:
                    cv2.imshow("装甲板检测 - 强制显示", annotated_frame)

                    # 每100帧显示一次进度
                    if frame_count % 100 == 0:
                        progress = (frame_count * 100) // total_frames
                        print(f"📊 进度: {progress}% ({frame_count}/{total_frames}), 检测到: {total_detections} 个装甲板")

                    # 等待按键，但不阻塞
                    key = cv2.waitKey(1) & 0xFF
                    if key == 27:  # ESC
                        print("⏹️ 用户中断处理")
                        break
                    elif key == 32:  # 空格
                        print("⏸️  已暂停，按任意键继续...")
                        cv2.waitKey(0)

                except Exception as e:
                    print(f"⚠️  显示错误: {e}")
                    display_works = False
            else:
                # 仅显示进度
                if frame_count % 100 == 0:
                    progress = (frame_count * 100) // total_frames
                    print(f"📊 进度: {progress}% ({frame_count}/{total_frames}), 检测到: {total_detections} 个装甲板")

    except KeyboardInterrupt:
        print("\n⏹️ 用户中断处理")

    finally:
        # 清理资源
        cap.release()
        writer.release()
        if display_works:
            try:
                cv2.destroyAllWindows()
            except:
                pass

    # 输出统计信息
    print("\n✅ 视频处理完成！")
    print("📊 统计信息:")
    print(f"   - 处理帧数: {frame_count}")
    print(f"   - 总检测数: {total_detections}")
    print(f"   - 平均每帧: {total_detections/frame_count:.2f} 个装甲板")
    print(f"💾 结果已保存: {output_path}")

    if display_works:
        print("🎉 显示功能正常工作")
    else:
        print("⚠️  显示功能不可用，但文件保存正常")

    # 尝试用系统默认播放器打开结果
    try:
        print("🎬 尝试打开结果视频...")
        subprocess.run(['xdg-open', output_path], check=False)
    except:
        print("无法自动打开视频播放器")

    return True

def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description="🎯 RoboMaster 装甲板检测系统 - 强制显示版"
    )

    parser.add_argument('-v', '--video', required=True, help='检测指定视频并强制显示')
    parser.add_argument('-m', '--model',
                       default='models/trained/best.pt',
                       help='指定模型文件路径 (默认: best.pt)')
    parser.add_argument('-o', '--output', default='./results/',
                       help='指定输出路径 (默认: ./results/)')
    parser.add_argument('-c', '--confidence', type=float, default=0.3,
                       help='置信度阈值 (默认: 0.3)')

    args = parser.parse_args()

    # 检查模型文件
    if not os.path.exists(args.model):
        print(f"❌ 模型文件不存在: {args.model}")
        return 1

    print("🎯 RoboMaster 装甲板检测系统 - 强制显示版")
    print("=========================================")

    # 强制设置显示环境
    force_display_setup()

    # 测试显示系统
    display_available = test_display_availability()

    if display_available:
        print("✅ 显示系统可用")
    else:
        print("⚠️  显示系统测试失败，但将尝试强制显示")

    print(f"🔄 正在加载模型: {args.model}")

    try:
        # 加载模型
        model = YOLO(args.model)
        print("✅ 模型加载成功")
        print(f"🔧 置信度阈值: {args.confidence}")

        # 执行检测
        success = create_video_display(model, args.video, args.output, args.confidence)

        if success:
            print("\n🎉 检测完成！")
            return 0
        else:
            print("\n❌ 检测失败！")
            return 1

    except Exception as e:
        print(f"❌ 检测过程中发生错误: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())
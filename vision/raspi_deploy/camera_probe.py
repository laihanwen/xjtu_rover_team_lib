"""摄像头探测：枚举本机 OpenCV 可用的摄像头、硬件名称与支持分辨率。

从原 xbhdcc_tools.py 中拆出，只保留 detect_cameras() 及其 __main__ 演示入口。
支持 Windows (DSHOW 快速扫描) 和 Linux (sysfs 硬件树分析) 双平台。
"""

import glob
import os
import platform
import sys
import time

import cv2


def detect_cameras(max_to_test=15):
    """
    探测当前系统连接的所有摄像头，获取它们的 OpenCV 编号、真实硬件名称及支持的分辨率。
    支持 Windows (DSHOW 快速扫描) 和 Linux (sysfs 硬件树分析)
    """
    system_name = platform.system()
    print("=" * 60)
    print(f" 🔍 开始探测系统摄像头 (当前系统: {system_name})")
    print("=" * 60)

    available_cameras = []
    test_resolutions = [(1920, 1080), (1280, 720), (640, 480), (320, 240)]
    candidates = []

    if system_name == "Linux":
        # Linux 专属高级扫描：读取 V4L2 硬件树并过滤虚拟节点
        video_paths = glob.glob('/sys/class/video4linux/video*')
        if video_paths:
            sorted_paths = sorted(
                video_paths, key=lambda x: int(os.path.basename(x).replace('video', '')))
            for path in sorted_paths:
                dev_name = os.path.basename(path)
                idx = int(dev_name.replace('video', ''))

                friendly_name = "未知摄像头"
                name_file = os.path.join(path, 'name')
                if os.path.exists(name_file):
                    try:
                        with open(name_file, 'r', encoding='utf-8') as f:
                            friendly_name = f.read().strip()
                    except Exception:
                        pass

                # 过滤掉无法成像的虚拟节点
                ignore_keywords = ["metadata", "association", "statistics", "params", "meta"]
                if any(kw in friendly_name.lower() for kw in ignore_keywords):
                    continue

                candidates.append((idx, friendly_name))

        if not candidates:
            candidates = [(i, f"Camera {i}") for i in range(max_to_test)]
    else:
        # Windows / macOS 默认顺序扫描
        candidates = [(i, f"Camera {i}") for i in range(max_to_test)]

    # 遍历设备获取详细参数
    for index, name in candidates:
        if system_name == "Windows":
            cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        elif system_name == "Linux":
            cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        else:
            cap = cv2.VideoCapture(index)

        if cap.isOpened():
            default_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            default_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            default_fps = cap.get(cv2.CAP_PROP_FPS)

            supported_res = []
            for w, h in test_resolutions:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
                act_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                act_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

                res_str = f"{act_w}x{act_h}"
                if res_str not in supported_res:
                    supported_res.append(res_str)

            # 恢复默认值
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, default_w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, default_h)

            fps_str = f"{default_fps:.1f}" if default_fps > 0 else "未知/动态"
            cam_info = {
                "index": index,
                "name": name,
                "default_res": f"{default_w}x{default_h}",
                "default_fps": fps_str,
                "supported_resolutions": supported_res
            }
            available_cameras.append(cam_info)

            print(f"\n[+] 发现摄像头 [编号: {index}]")
            print(f"    - 设备名称: {name}")
            print(f"    - 默认启动分辨率: {cam_info['default_res']}")
            print(f"    - 默认帧率 (FPS) : {cam_info['default_fps']}")
            print(f"    - 硬件支持分辨率: {', '.join(supported_res)}")

            if system_name == "Windows":
                print(f"    -OpenCV 启动代码建议: cv2.VideoCapture({index}, cv2.CAP_DSHOW)")
            elif system_name == "Linux":
                print(f"    -OpenCV 启动代码建议: cv2.VideoCapture({index}, cv2.CAP_V4L2)")
            else:
                print(f"    -OpenCV 启动代码建议: cv2.VideoCapture({index})")

            cap.release()

    print("\n" + "=" * 60)
    if not available_cameras:
        print("未检测到任何可用的摄像头设备！")
    else:
        print(f"探测完成！共发现 {len(available_cameras)} 个可用摄像头。")
    print("=" * 60)

    return available_cameras


# ==========================================
# 极简测试运行入口
# ==========================================
if __name__ == '__main__':
    # 推流类只在演示入口用到，放在这里，避免探测功能硬依赖 mjpeg_stream
    from mjpeg_stream import WebStreamer

    # 1. 探测摄像头
    cams = detect_cameras()
    if not cams:
        print("错误：未检测到任何摄像头，无法启动演示！")
        sys.exit(1)

    # 2. 启动网页服务器
    streamer = WebStreamer(port=8080)

    # 3. 打开第一个检测到的摄像头
    target_idx = cams[0]['index']
    sys_name = platform.system()
    if sys_name == "Windows":
        cap = cv2.VideoCapture(target_idx, cv2.CAP_DSHOW)
    elif sys_name == "Linux":
        cap = cv2.VideoCapture(target_idx, cv2.CAP_V4L2)
    else:
        cap = cv2.VideoCapture(target_idx)

    print(f"\n[演示] 正在读取摄像头 {target_idx} 并推流到网页...")
    print("请在浏览器中打开: http://localhost:8080")
    print("在终端按 Ctrl+C 退出。")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # 【通道 0】：直接推送原始彩色画面
            streamer.update_frame(0, frame)

            # 【通道 1】：直接转成灰度图并推送（imencode 会自动处理通道数）
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            streamer.update_frame(1, gray_frame)

            # 控制主循环速度，避免 CPU 空转
            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        streamer.stop()

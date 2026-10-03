"""MJPEG 双路推流服务：WebStreamer + MJPEGHandler。

从原 xbhdcc_tools.py 中拆出，只负责网页视频流显示；摄像头探测见 camera_probe.py。
推流基于标准库 http.server，不需要 flask。
"""

import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingTCPServer

import cv2
import numpy as np


class MJPEGHandler(BaseHTTPRequestHandler):
    """HTTP 处理器：根路径返回监控主页，/stream/<id> 返回 MJPEG 视频流。"""

    def log_message(self, format, *args):
        """静默服务器控制台的频繁日志输出。"""

    def do_GET(self):
        # 1. 根目录：返回监控主页 HTML
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-type", "text/html; charset=utf-8")
            self.end_headers()

            html = """<!DOCTYPE html>
            <html>
            <head>
                <meta charset="UTF-8">
                <title>xbhdcc 视觉监控面板</title>
                <style>
                    body { font-family: 'Segoe UI', Arial, sans-serif; margin: 0;
                           padding: 20px; background: #0f172a; color: #f1f5f9;
                           text-align: center; }
                    h1 { color: #38bdf8; margin-bottom: 5px; font-weight: 800; }
                    .subtitle { color: #94a3b8; font-size: 14px;
                                margin-bottom: 25px; }
                    /* 强制左右并排的 Grid 布局 */
                    .container { display: grid;
                                 grid-template-columns: repeat(2, 1fr);
                                 gap: 20px; max-width: 1400px; margin: 0 auto;
                                 padding: 10px; }
                    /* 手机等窄屏下自动退化为上下堆叠 */
                    @media (max-width: 900px) {
                        .container { grid-template-columns: 1fr; }
                    }
                    .stream-box { background: #1e293b; padding: 15px;
                                  border-radius: 12px; border: 1px solid #334155;
                                  box-shadow: 0 10px 25px rgba(0, 0, 0, 0.5);
                                  display: flex; flex-direction: column;
                                  align-items: center; }
                    .stream-box h2 { margin-top: 0; color: #38bdf8;
                                     font-size: 18px;
                                     border-bottom: 1px solid #334155;
                                     padding-bottom: 8px; width: 100%; }
                    img { width: 100%; max-width: 640px; height: auto;
                          border-radius: 6px; background: #000; }
                    .footer { margin-top: 40px; color: #64748b;
                              font-size: 12px; }
                </style>
            </head>
            <body>
                <h1>xbhdcc 实时视频流监视器</h1>
                <div class="subtitle">支持局域网多设备同时访问 | 左右双路画面对比</div>
                <div class="container">
                    <div class="stream-box">
                        <h2>视频流 1</h2>
                        <img src="/stream/0" />
                    </div>
                    <div class="stream-box">
                        <h2>视频流 2</h2>
                        <img src="/stream/1" />
                    </div>
                </div>
                <div class="footer">Powered by OpenCV &amp; Python BaseHTTPServer</div>
            </body>
            </html>
            """
            self.wfile.write(html.encode("utf-8"))

        # 2. 视频流路径：/stream/0 或 /stream/1
        elif self.path.startswith("/stream/"):
            try:
                stream_id = int(self.path.split("/")[-1])
            except ValueError:
                self.send_error(400, "Invalid stream ID")
                return

            if stream_id not in (0, 1):
                self.send_error(404, "Stream not found")
                return

            # streamer 绑定在服务器实例上（见 _ThreadedHTTPServer），不是全局单例
            streamer = self.server.streamer

            # 设置 MJPEG 流的 HTTP 响应头
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()

            try:
                while True:
                    # 从 WebStreamer 获取对应通道的图像
                    frame = streamer.get_frame(stream_id)

                    # 编码为 JPG (支持 3 通道彩色和 1 通道灰度)
                    ret, jpeg = cv2.imencode(
                        ".jpg", frame,
                        [int(cv2.IMWRITE_JPEG_QUALITY), streamer.jpeg_quality])
                    if ret:
                        self.wfile.write(b"--frame\r\n")
                        # 注意：MJPEG 的 part 头按 HTTP 规范应直接写字节到 wfile；
                        # 这里沿用 send_header()/end_headers() 的写法，虽然绕过了
                        # 正常的 header 流程，但实测可稳定工作，故保留不改。
                        self.send_header("Content-Type", "image/jpeg")
                        self.send_header("Content-Length", str(len(jpeg)))
                        self.end_headers()
                        self.wfile.write(jpeg.tobytes())
                        self.wfile.write(b"\r\n")

                    # 控制推流帧率，默认约 30 FPS（可通过构造参数传入）
                    time.sleep(streamer.frame_interval)
            except (ConnectionResetError, BrokenPipeError):
                # 客户端关闭网页时静默退出
                pass
            except Exception as e:
                print(f"[WebStreamer] 推流异常: {e}")
        else:
            self.send_error(404)


class _ThreadedHTTPServer(ThreadingTCPServer, HTTPServer):
    """多线程 HTTP 服务器；streamer 绑定在服务器实例上，避免全局单例互相覆盖。"""

    allow_reuse_address = True

    def __init__(self, server_address, handler_cls, streamer):
        self.streamer = streamer
        super().__init__(server_address, handler_cls)


class WebStreamer:
    """网页视频流服务器，最多支持 2 路图像同时显示（stream_id 只能是 0/1）。"""

    def __init__(self, port=8080, jpeg_quality=80, frame_interval=0.03):
        self.port = port
        self.jpeg_quality = int(jpeg_quality)      # MJPEG 单帧 JPEG 质量
        self.frame_interval = float(frame_interval)  # 推流间隔秒数，默认约 30 FPS
        self.frames = {0: None, 1: None}
        self.lock = threading.Lock()
        self.server = None
        self.server_thread = None
        self._start_server()

    def _generate_placeholder(self, stream_id):
        """当用户还没有传入图像时，生成一个等待占位图。"""
        placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
        placeholder[:] = (30, 30, 30)
        cv2.rectangle(placeholder, (15, 15), (625, 465), (100, 100, 100), 2)
        cv2.putText(placeholder, f"Waiting for Stream {stream_id}...", (130, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
        return placeholder

    def _start_server(self):
        self.server = _ThreadedHTTPServer(("0.0.0.0", self.port), MJPEGHandler, self)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        print("\n[WebStreamer] 网页服务已启动！")
        print(f"本地预览: http://localhost:{self.port}")
        print(f"局域网预览: http://<树莓派IP>:{self.port}\n")

    def update_frame(self, stream_id, frame):
        """更新指定通道的图像 (stream_id 只能是 0 或 1)"""
        if stream_id not in (0, 1):
            raise ValueError("stream_id 必须是 0 或者 1")
        with self.lock:
            if frame is not None:
                self.frames[stream_id] = frame.copy()
            else:
                self.frames[stream_id] = None

    def get_frame(self, stream_id):
        """获取指定通道的图像，若为空则返回占位图"""
        with self.lock:
            if self.frames[stream_id] is None:
                return self._generate_placeholder(stream_id)
            return self.frames[stream_id]

    def stop(self):
        """停止服务器"""
        if self.server:
            self.server.shutdown()
            self.server.server_close()
            print("[WebStreamer] 服务器已安全关闭。")

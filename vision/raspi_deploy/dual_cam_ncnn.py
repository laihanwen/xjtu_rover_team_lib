"""
dual_cam_ncnn.py
CSI + USB 双路摄像头 -> YOLOv11n(ncnn) 推理 -> 画框 -> Web 推流

本版改动：
  1. 配置外置到 config.yaml：每路摄像头独立配置 enable / infer / infer_fps /
     model_dir / class_names / conf / nms / input_size / pixel，用 --config
     指定路径；代码内保留一份默认值兜底（DEFAULT_CONFIG）。
  2. 每路各自一条 in_q / out_q，投递前先清空队列只留最新帧 —— 两路互不饿死。
  3. 推理调度两版本共用一份代码，用 runtime.infer_mode 切换（"A" 独立进程 / "B" 轮流）。
  4. 子进程默认 fork，且在相机初始化之前就 fork —— 避开 spawn 的
     AttributeError: Can't get attribute 'infer_proc_xxx' 问题。
  5. 推理频率改为 infer_fps（次/秒，按时间门控），并打印真实单帧推理耗时。
"""

import argparse
import glob
import multiprocessing as mp
import os
import queue
import sys
import time
import traceback
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml

try:
    from .mjpeg_stream import WebStreamer
except ImportError:  # Direct script execution from this directory.
    from mjpeg_stream import WebStreamer

# =====================================================================
#                     配置默认值（可用 config.yaml 覆盖）
# =====================================================================
DEFAULT_CONFIG = {
    "cameras": {
        # ---------------- 0 号：CSI（libcamera / Picamera2） ----------------
        0: {
            "label": "CAM0-CSI",
            "enable": 1,
            "infer": 0,  # CSI 路当前只推流不推理（排线未到位）
            "infer_fps": 0,  # 0 = 该路不推理
            "model_dir": "",
            "class_names": ["sea_cucumber", "starfish", "turtle"],
            "conf": 0.25,
            "nms": 0.45,
            "input_size": 416,
            "pixel": "BGR2RGB",
            "csi_candidates": [(640, 480), (800, 600), (960, 720)],
        },
        # ---------------- 1 号：USB（UVC） ----------------
        1: {
            "label": "CAM1-USB",
            "enable": 1,
            "infer": 1,
            "infer_fps": 2,
            "model_dir": "",
            "class_names": ["sea_cucumber", "starfish", "turtle"],
            "conf": 0.40,
            "nms": 0.45,
            "input_size": 416,
            "pixel": "BGR2RGB",
            "usb_size": (960, 540),
            "usb_candidates": ["/dev/video0", "/dev/video2", "/dev/video4", 0, 1, 2, 3],
        },
    },
    "stream": {"fps": 10, "port": 8080, "mpeg_quality": 80},
    "runtime": {
        "infer_mode": "A",
        "mp_start_method": "fork",
        "ncnn_threads": 3,
        "extra_site_packages": "",
    },
    "debug": {"diag_print": False},
}

# 按类别序号取色
CLASS_COLORS = [(0, 255, 0), (0, 165, 255), (255, 0, 0)]


# =====================================================================
#                            配置读取
# =====================================================================
def _parse_args(argv=None):
    default_cfg = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.yaml")
    parser = argparse.ArgumentParser(description="CSI + USB 双路摄像头 NCNN 推理 + MJPEG 推流")
    parser.add_argument(
        "--config", default=default_cfg, help="配置文件路径（默认：与本脚本同目录的 config.yaml）"
    )
    return parser.parse_args(argv)


def _load_config(path):
    """读取 config.yaml，文件缺失或缺字段时回落到 DEFAULT_CONFIG。"""
    raw = {}
    if path and os.path.exists(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                raw = yaml.safe_load(f)
        except (OSError, yaml.YAMLError) as e:
            print(f"[CFG] 解析 {path} 失败：{type(e).__name__}: {e}，改用内置默认配置")
            raw = {}
    else:
        print(f"[CFG] 未找到配置文件 {path}，改用内置默认配置")
    if not isinstance(raw, dict):
        raw = {}

    cfg = {"cameras": {}}
    for section in ("stream", "runtime", "debug"):
        merged = dict(DEFAULT_CONFIG[section])
        loaded = raw.get(section)
        if isinstance(loaded, dict):
            merged.update(loaded)
        cfg[section] = merged

    for cid in (0, 1):
        merged = dict(DEFAULT_CONFIG["cameras"][cid])
        loaded = (raw.get("cameras") or {}).get(cid)
        if isinstance(loaded, dict):
            merged.update(loaded)
        cfg["cameras"][cid] = merged
    return cfg


# =====================================================================
#                          推理引擎（子进程内使用）
# =====================================================================
@dataclass(frozen=True)
class LetterboxTransform:
    scale: float
    pad_x: float
    pad_y: float
    original_width: int
    original_height: int


def letterbox(frame, size):
    """Resize with preserved aspect ratio and Ultralytics-compatible padding."""
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3 or size <= 0:
        raise ValueError("letterbox expects a BGR image and positive size")
    height, width = frame.shape[:2]
    scale = min(size / width, size / height)
    resized_width = max(1, round(width * scale))
    resized_height = max(1, round(height * scale))
    pad_x = (size - resized_width) // 2
    pad_y = (size - resized_height) // 2
    resized = cv2.resize(frame, (resized_width, resized_height))
    output = np.full((size, size, 3), 114, dtype=np.uint8)
    output[pad_y : pad_y + resized_height, pad_x : pad_x + resized_width] = resized
    return np.ascontiguousarray(output), LetterboxTransform(
        scale=scale,
        pad_x=float(pad_x),
        pad_y=float(pad_y),
        original_width=width,
        original_height=height,
    )


def _ordered_names(names):
    if isinstance(names, list) and names:
        return [str(name) for name in names]
    if isinstance(names, dict) and names:
        try:
            return [str(names[key]) for key in sorted(names, key=lambda value: int(value))]
        except (TypeError, ValueError) as error:
            raise ValueError("metadata names keys must be integer-like") from error
    raise ValueError("metadata must contain a non-empty names list or mapping")


def validate_model_metadata(model_dir, cfg):
    """Require the exported metadata to match runtime classes and input size."""
    metadata_path = Path(model_dir) / "metadata.yaml"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"missing model metadata: {metadata_path}")
    try:
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"unable to read model metadata: {metadata_path}") from error
    if not isinstance(metadata, dict):
        raise ValueError(f"invalid model metadata: {metadata_path}")  # noqa: TRY004
    actual_names = _ordered_names(metadata.get("names"))
    expected_names = [str(name) for name in cfg.get("class_names", [])]
    if actual_names != expected_names:
        raise ValueError(f"class_names mismatch: metadata={actual_names}, config={expected_names}")
    image_size = metadata.get("imgsz")
    if isinstance(image_size, (list, tuple)):
        dimensions = [int(value) for value in image_size]
    else:
        dimensions = [int(image_size)] if image_size is not None else []
    expected_size = int(cfg.get("input_size", 0))
    if not dimensions or any(value != expected_size for value in dimensions):
        raise ValueError(f"input_size mismatch: metadata={image_size}, config={expected_size}")
    return metadata


class YoloEngine:
    """一路摄像头的独立推理引擎：自己的模型、类别、阈值、通道序。"""

    def __init__(self, cfg, ncnn_mod, diag_print=False):
        self.cfg = cfg
        self.ncnn = ncnn_mod
        self.diag = bool(diag_print)
        self.input_size = int(cfg.get("input_size", 640))
        self.conf = float(cfg.get("conf", 0.25))
        self.nms = float(cfg.get("nms", 0.45))
        self.names = list(cfg.get("class_names", []))
        pixel = str(cfg.get("pixel", "BGR2RGB")).upper()
        if pixel != "BGR2RGB":
            raise ValueError("Ultralytics NCNN deployment requires pixel: BGR2RGB")
        self.pixel = ncnn_mod.Mat.PixelType.PIXEL_BGR2RGB
        self.net = None
        self._dbg_t = 0.0

    def load(self, model_dir, threads):
        if not model_dir:
            print(
                "[infer] model_dir 为空，本路不推理。请在 config.yaml 里填成含 "
                "model.ncnn.param + model.ncnn.bin 的目录"
            )
            return False
        try:
            validate_model_metadata(model_dir, self.cfg)
        except (FileNotFoundError, ValueError) as error:
            print(f"[infer] 模型元数据校验失败：{error}")
            return False
        net = self.ncnn.Net()
        try:
            net.opt.num_threads = threads  # 必须在 load 之前设置，否则不生效
        except AttributeError:
            print("[infer] 当前 ncnn 绑定不支持设置 num_threads，使用运行库默认值")
        model_path = Path(model_dir)
        if net.load_param(str(model_path / "model.ncnn.param")) != 0:
            print(f"[infer] load_param 失败：{model_dir}")
            return False
        if net.load_model(str(model_path / "model.ncnn.bin")) != 0:
            print(f"[infer] load_model 失败：{model_dir}")
            return False
        self.net = net
        return True

    def forward(self, frame):
        s = self.input_size
        if isinstance(frame, tuple) and len(frame) == 2:
            resized, transform = frame
        else:
            resized, transform = letterbox(frame, s)
        mat = self.ncnn.Mat.from_pixels_resize(resized, self.pixel, s, s, s, s)
        mat.substract_mean_normalize([0.0, 0.0, 0.0], [1 / 255.0, 1 / 255.0, 1 / 255.0])
        ex = self.net.create_extractor()
        ex.input("in0", mat)
        _, out = ex.extract("out0")
        a = np.array(out)
        if a.ndim == 3:
            a = a[0]
        if a.shape[0] < a.shape[1]:
            a = a.T
        return a, transform

    def detect(self, frame):
        a, transform = self.forward(frame)
        if self.diag:
            now = time.time()
            if now - self._dbg_t > 2.0:
                self._dbg_t = now
                per_cls = a[:, 4:].max(axis=0)
                print(
                    "[DIAG] 各类最高分: "
                    + ", ".join(f"{n}={float(per_cls[i]):.3f}" for i, n in enumerate(self.names))
                )
        return self._nms(a, transform)

    def _nms(self, a, transform):
        if a.shape[0] == 0:
            return [], [], []
        cs = a[:, 4:]
        cids = np.argmax(cs, axis=1)
        scs = cs[np.arange(a.shape[0]), cids]
        m = scs >= self.conf
        if not bool(m.any()):
            return [], [], []
        xywh = a[m, :4].copy()
        scs = scs[m]
        cids = cids[m]
        xywh[:, 0] = (xywh[:, 0] - transform.pad_x) / transform.scale
        xywh[:, 1] = (xywh[:, 1] - transform.pad_y) / transform.scale
        xywh[:, 2:] /= transform.scale
        cx, cy, w, h = xywh[:, 0], xywh[:, 1], xywh[:, 2], xywh[:, 3]
        boxes = np.stack([cx - w / 2.0, cy - h / 2.0, w, h], axis=1).astype(np.float64).tolist()
        scores = scs.astype(np.float64).tolist()
        cids = cids.astype(int).tolist()
        if not boxes:
            return [], [], []
        kept = []
        for class_id in sorted(set(cids)):
            source_indexes = [index for index, value in enumerate(cids) if value == class_id]
            class_boxes = [boxes[index] for index in source_indexes]
            class_scores = [scores[index] for index in source_indexes]
            indexes = cv2.dnn.NMSBoxes(class_boxes, class_scores, self.conf, self.nms)
            indexes = indexes.flatten() if hasattr(indexes, "flatten") else [i[0] for i in indexes]
            kept.extend(source_indexes[int(index)] for index in indexes)
        kept.sort(key=lambda index: scores[index], reverse=True)
        clipped_boxes = []
        kept_scores = []
        kept_classes = []
        for index in kept:
            x, y, width, height = boxes[index]
            left = max(0.0, min(float(transform.original_width), x))
            top = max(0.0, min(float(transform.original_height), y))
            right = max(left, min(float(transform.original_width), x + width))
            bottom = max(top, min(float(transform.original_height), y + height))
            if right <= left or bottom <= top:
                continue
            clipped_boxes.append([left, top, right - left, bottom - top])
            kept_scores.append(scores[index])
            kept_classes.append(cids[index])
        return clipped_boxes, kept_scores, kept_classes


# ---------------------------------------------------------------------
#  推理耗时统计：用来判断 infer_fps 是不是设得超过了硬件能力
# ---------------------------------------------------------------------
_PERF_STATS = {}


def _tick_perf(tag, cam_id, dt_sec):
    """累计单帧推理耗时，每 5 秒打印一次平均值和该路理论上限。"""
    st = _PERF_STATS.get(cam_id)
    if st is None:
        st = _PERF_STATS[cam_id] = {"n": 0, "ms": 0.0, "t": time.monotonic()}
    st["n"] += 1
    st["ms"] += dt_sec * 1000.0
    now = time.monotonic()
    if now - st["t"] >= 5.0:
        avg = st["ms"] / max(1, st["n"])
        print(
            f"[PERF] {tag} cam{cam_id} 平均单帧推理 {avg:.0f} ms"
            f" → 该路天花板约 {1000.0 / max(avg, 1e-6):.1f} 次/秒"
        )
        st["n"], st["ms"], st["t"] = 0, 0.0, now


def _import_ncnn(extra_site_packages):
    try:
        import ncnn
    except ImportError:
        if extra_site_packages and extra_site_packages not in sys.path:
            sys.path.insert(0, extra_site_packages)
        import ncnn
    return ncnn


def _put_latest(q, value):
    """投递前先清空队列，只保留最新一条 —— 这是两路互不饿死的关键。"""
    try:
        while True:
            q.get_nowait()
    except queue.Empty:
        pass
    try:
        q.put_nowait(value)
    except queue.Full:
        pass


def _stop_processes(processes):
    """Terminate and reap inference children so systemd restarts cleanly."""
    for _label, process in processes:
        if process.is_alive():
            process.terminate()
    for _label, process in processes:
        process.join(timeout=2.0)


# =====================================================================
#                      模式 A：每路一个独立推理进程
# =====================================================================
def infer_proc_single(cam_id, cfg, threads, extra_site_packages, diag_print, in_q, out_q):
    ncnn = _import_ncnn(extra_site_packages)
    eng = YoloEngine(cfg, ncnn, diag_print)
    if not eng.load(cfg["model_dir"], threads):
        print(f"[infer-A] cam{cam_id} 模型加载失败，本路不推理")
        return
    print(f"[infer-A] cam{cam_id} 就绪 模型={cfg['model_dir']}")
    # --- 临时诊断：在子进程内部量一次纯算力 ---
    _dbg = np.ascontiguousarray(
        np.random.randint(0, 255, (eng.input_size, eng.input_size, 3), dtype=np.uint8)
    )
    eng.detect(_dbg)  # 预热
    _tb = time.monotonic()
    for _ in range(3):
        eng.detect(_dbg)
    _bench_ms = (time.monotonic() - _tb) / 3 * 1000
    print(f"[CHILD-BENCH] cam{cam_id} 子进程内单帧 {_bench_ms:.0f} ms  (NCNN_THREADS={threads})")

    while True:
        _tw = time.monotonic()
        frame = in_q.get()
        _tw = time.monotonic() - _tw
        t0 = time.monotonic()
        try:
            det = eng.detect(frame)
        except Exception as e:  # noqa: BLE001 -- third-party NCNN raises binding-specific errors.
            print(f"[infer-A] cam{cam_id} 推理异常:", e)
            det = ([], [], [])
        _tc = time.monotonic() - t0
        _tick_perf("infer-A", cam_id, _tc)
        if not hasattr(eng, "_n"):
            eng._n = 0
        eng._n += 1
        if eng._n % 5 == 0:
            print(
                f"[CHILD] cam{cam_id} 等待取帧 {_tw * 1000:.0f} ms  +  纯推理 {_tc * 1000:.0f} ms"
            )
        _put_latest(out_q, det)


# =====================================================================
#                    模式 B：单进程在两路之间轮流
# =====================================================================
def infer_proc_rr(cam_cfgs, threads, extra_site_packages, diag_print, in_qs, out_qs):
    """cam_cfgs: {cid: cfg}，只包含开启了推理的那几路。"""
    ncnn = _import_ncnn(extra_site_packages)

    engs = {}
    for cid, cfg in cam_cfgs.items():
        eng = YoloEngine(cfg, ncnn, diag_print)
        if not eng.load(cfg["model_dir"], threads):
            print(f"[infer-B] cam{cid} 模型加载失败，该路不推理")
            continue
        engs[cid] = eng
        print(f"[infer-B] cam{cid} 就绪 模型={cfg['model_dir']}")

    if not engs:
        print("[infer-B] 没有可用的推理引擎，进程退出")
        return

    order = sorted(engs.keys())
    rr = 0
    while True:
        taken = None
        for k in range(len(order)):
            cid = order[(rr + k) % len(order)]
            try:
                frame = in_qs[cid].get_nowait()
            except queue.Empty:
                continue
            taken = (cid, frame)
            rr = (order.index(cid) + 1) % len(order)  # 下次从另一路开始
            break

        if taken is None:
            time.sleep(0.005)
            continue

        cid, frame = taken
        t0 = time.monotonic()
        try:
            det = engs[cid].detect(frame)
        except Exception as e:  # noqa: BLE001 -- third-party NCNN raises binding-specific errors.
            print(f"[infer-B] cam{cid} 推理异常:", e)
            det = ([], [], [])
        _tick_perf("infer-B", cid, time.monotonic() - t0)
        _put_latest(out_qs[cid], det)


# =====================================================================
#                            摄像头封装
# =====================================================================
class CamSource:
    """统一封装一路摄像头：kind = 'picam'(picamera2) 或 'cv2'(VideoCapture)。"""

    def __init__(self, kind, obj, size, swap=False):
        self.kind = kind
        self.obj = obj
        self.size = size
        self.swap = swap
        self.fail = 0

    def read(self):
        if self.kind == "picam":
            frame = self.obj.capture_array()
        else:
            ok, frame = self.obj.read()
            if not ok or frame is None:
                return None
        frame = np.ascontiguousarray(frame)
        if self.swap and frame.ndim == 3 and frame.shape[2] == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame

    def close(self):
        try:
            if self.kind == "picam":
                self.obj.stop()
                self.obj.close()
            else:
                self.obj.release()
        except Exception as error:  # noqa: BLE001 -- camera backends vary by platform.
            print(f"[CAM] 关闭相机失败：{error}")


def enum_libcamera():
    """枚举 libcamera 看到的相机，返回 [(index, info_dict), ...]。"""
    try:
        from picamera2 import Picamera2
    except ImportError as e:
        print(f"[CAM0] picamera2 不可用：{type(e).__name__}: {e}")
        return []
    try:
        infos = Picamera2.global_camera_info()
    except Exception as e:  # noqa: BLE001 -- libcamera exposes backend-specific errors.
        print(f"[CAM0] 枚举 libcamera 相机失败：{type(e).__name__}: {e}")
        return []

    print(f"[CAM0] libcamera 共枚举到 {len(infos)} 个相机：")
    for i, info in enumerate(infos):
        print(f"        #{i} Model={info.get('Model')} Id={info.get('Id')}")
    return list(enumerate(infos))


def _pick_csi_index(infos):
    """从枚举结果里挑 CSI 相机编号；只有 USB 时返回 None。"""
    for i, info in infos:
        s = f"{info.get('Model', '')} {info.get('Id', '')}".lower()
        if "usb" in s or "uvc" in s:
            continue
        if any(k in s for k in ("ov", "imx", "i2c", "unicam", "csi")):
            return i
    return None


def init_csi(candidates):
    infos = enum_libcamera()
    if not infos:
        print("[CAM0] CSI 本路停用：libcamera 没有枚举到任何相机")
        return None

    idx = _pick_csi_index(infos)
    if idx is None:
        print("[CAM0] CSI 本路停用：libcamera 里只有 USB 相机，没有 CSI 相机")
        print("       请检查 rpicam-hello --list-cameras，以及排线 / config.txt 的 dtoverlay")
        return None

    try:
        from picamera2 import Picamera2
    except ImportError as e:
        print(f"[CAM0] picamera2 不可用：{type(e).__name__}: {e}")
        return None

    last_err = None
    for size in candidates:
        cam = None
        try:
            cam = Picamera2(camera_num=idx)
            cam.configure(cam.create_preview_configuration(main={"format": "RGB888", "size": size}))
            cam.start()
            time.sleep(1.0)
            frame = cam.capture_array()
            real = (frame.shape[1], frame.shape[0])
            print(f"[CAM0] CSI 就绪 @ 请求{size} 实际{real} (libcamera #{idx})")
            return CamSource("picam", cam, real, swap=False)
        except Exception as e:  # noqa: BLE001 -- libcamera exposes backend-specific errors.
            last_err = e
            print(f"[CAM0] {size} 启动失败：{type(e).__name__}: {e}")
            if cam is not None:
                try:
                    cam.stop()
                    cam.close()
                except Exception as cleanup_error:  # noqa: BLE001 -- best-effort hardware cleanup.
                    print(f"[CAM0] 清理失败：{cleanup_error}")

    print("[CAM0] CSI 所有分辨率都失败，本路停用。真实异常如下：")
    if last_err is not None:
        traceback.print_exception(type(last_err), last_err, last_err.__traceback__)
    return None


def _usb_candidates(extra_candidates):
    """自动探测 /dev/video*，再并入 config.yaml 里配置的额外候选节点。"""
    seen = sorted(glob.glob("/dev/video*"))
    for extra in extra_candidates:
        if extra not in seen:
            seen.append(extra)
    return seen


def init_usb(size, extra_candidates):
    """只用 cv2 打开 USB 摄像头：cv2 的 V4L2 后端直接输出 BGR，颜色正确。"""
    cands = _usb_candidates(extra_candidates)
    print(f"[CAM1] 逐个尝试 USB 采集节点：{cands}")
    for dev in cands:
        cap = None
        try:
            cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
            ok, frame = cap.read()
            if not ok or frame is None:
                cap.release()
                continue
            real = (frame.shape[1], frame.shape[0])
            print(f"[CAM1] USB 就绪 @ {dev} 实际{real}")
            return CamSource("cv2", cap, real, swap=False)
        except Exception as e:  # noqa: BLE001 -- OpenCV camera backends vary by platform.
            print(f"[CAM1] {dev} 打开异常：{type(e).__name__}: {e}")
            if cap is not None:
                try:
                    cap.release()
                except Exception as cleanup_error:  # noqa: BLE001 -- best-effort hardware cleanup.
                    print(f"[CAM1] 清理失败：{cleanup_error}")
    print("[CAM1] USB 本路停用：没有可用的 /dev/video* 采集节点")
    return None


# =====================================================================
#                              工具
# =====================================================================
def check_ncnn(extra_site_packages):
    try:
        import ncnn

        return True
    except ImportError:
        pass
    if extra_site_packages and extra_site_packages not in sys.path:
        sys.path.insert(0, extra_site_packages)
    try:
        import ncnn  # noqa: F401

        return True
    except ImportError:
        print("[FATAL] 当前 Python 找不到 ncnn，请先激活装好 ncnn 的虚拟环境后重跑：")
        print("        source <你的 venv 目录>/bin/activate")
        return False


def make_offline_frame(cid, w=640, h=480):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    cv2.putText(
        img, f"CAM{cid} OFFLINE", (40, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2
    )
    return img


def draw(frame, boxes, scores, class_ids, cfg):
    if not boxes:
        return frame
    names = cfg.get("class_names", [])
    for box, score, cid in zip(boxes, scores, class_ids):
        x, y = int(box[0]), int(box[1])
        bw, bh = int(box[2]), int(box[3])
        color = CLASS_COLORS[int(cid) % len(CLASS_COLORS)]
        cv2.rectangle(frame, (x, y), (x + bw, y + bh), color, 2)
        label = names[int(cid)] if int(cid) < len(names) else str(cid)
        cv2.putText(
            frame,
            f"{label} {score:.2f}",
            (x, max(20, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            color,
            2,
        )
    return frame


# =====================================================================
#                              主程序
# =====================================================================
def main():
    args = _parse_args()
    cfg = _load_config(args.config)
    cam_cfg = cfg["cameras"]
    runtime_cfg = cfg["runtime"]
    stream_cfg = cfg["stream"]

    infer_mode = str(runtime_cfg.get("infer_mode", "A")).upper()
    mp_start_method = str(runtime_cfg.get("mp_start_method", "fork"))
    ncnn_threads = int(runtime_cfg.get("ncnn_threads", 3))
    extra_site_packages = str(runtime_cfg.get("extra_site_packages", "") or "")
    stream_fps = float(stream_cfg.get("fps", 10))
    stream_port = int(stream_cfg.get("port", 8080))
    mpeg_quality = int(stream_cfg.get("mpeg_quality", 80))
    diag_print = bool(cfg["debug"].get("diag_print", False))

    if not check_ncnn(extra_site_packages):
        return 1

    # 自检：入口函数必须顶格定义在模块最外层（换回 spawn 时尤其重要）
    for fn_name in ("infer_proc_single", "infer_proc_rr"):
        if not callable(globals().get(fn_name)):
            print(f"[FATAL] {fn_name} 不在模块顶层（被缩进进了 main()，或文件保存不完整）。")
            print("        请跑：grep -n '^def \\|^class \\|^if __name__' dual_cam_ncnn.py")
            return 1

    if infer_mode not in ("A", "B"):
        print(f"[FATAL] runtime.infer_mode 只能是 'A' 或 'B'，当前 = {infer_mode}")
        return 1

    infer_on = {
        cid: bool(
            int(cam_cfg[cid].get("enable", 1))
            and int(cam_cfg[cid].get("infer", 0))
            and float(cam_cfg[cid].get("infer_fps", 0)) > 0
        )
        for cid in (0, 1)
    }
    print(
        f"[INFO] 启动方式={mp_start_method} | 调度模式={infer_mode} | "
        f"开启推理={[c for c in (0, 1) if infer_on[c]]}"
    )

    # ---------- 1) 先建队列 + 起推理进程 ----------
    # fork 模式下这一步放在相机初始化"之前"：此刻父进程还没启动 libcamera，
    # 子进程继承到的是最干净的状态（没有 libcamera 的线程和 DMA 缓冲）。
    ctx = mp.get_context(mp_start_method)
    in_qs, out_qs, procs = {}, {}, []  # procs 元素是 (名字, Process)

    for cid in (0, 1):
        if not infer_on[cid]:
            continue
        in_qs[cid] = ctx.Queue(maxsize=1)  # 深度 1 + 投递前清空 => 永远只有最新帧
        out_qs[cid] = ctx.Queue(maxsize=1)

    if infer_mode == "A":
        for cid in (0, 1):
            if not infer_on[cid]:
                continue
            p = ctx.Process(
                target=infer_proc_single,
                args=(
                    cid,
                    cam_cfg[cid],
                    ncnn_threads,
                    extra_site_packages,
                    diag_print,
                    in_qs[cid],
                    out_qs[cid],
                ),
                daemon=True,
            )
            p.start()
            procs.append((f"cam{cid}", p))
            print(f"[INFO] cam{cid} 独立推理进程已启动 (pid={p.pid})")
    else:
        rr_cfgs = {cid: cam_cfg[cid] for cid in (0, 1) if infer_on[cid]}
        if rr_cfgs:
            p = ctx.Process(
                target=infer_proc_rr,
                args=(rr_cfgs, ncnn_threads, extra_site_packages, diag_print, in_qs, out_qs),
                daemon=True,
            )
            p.start()
            procs.append(("rr", p))
            print(f"[INFO] 单进程轮流推理已启动 (pid={p.pid})")

    # ---------- 2) 再建摄像头 ----------
    cams = {}
    if int(cam_cfg[0].get("enable", 1)):
        csi_cands = cam_cfg[0].get("csi_candidates", DEFAULT_CONFIG["cameras"][0]["csi_candidates"])
        cams[0] = init_csi([tuple(s) for s in csi_cands])
    if int(cam_cfg[1].get("enable", 1)):
        usb_size = tuple(cam_cfg[1].get("usb_size", DEFAULT_CONFIG["cameras"][1]["usb_size"]))
        usb_cands = cam_cfg[1].get("usb_candidates", DEFAULT_CONFIG["cameras"][1]["usb_candidates"])
        cams[1] = init_usb(usb_size, usb_cands)

    alive = {cid: (cams.get(cid) is not None) for cid in (0, 1)}
    if not any(alive.values()):
        print("[FATAL] 两路摄像头都不可用，退出")
        _stop_processes(procs)
        return 1
    print(f"[INFO] 摄像头可用={[c for c in (0, 1) if alive[c]]}")
    for cid in (0, 1):
        if infer_on[cid]:
            print(
                f"[INFO] cam{cid} 目标推理频率 = {cam_cfg[cid].get('infer_fps')} 次/秒"
                f"（实际 = min(该值, 单帧耗时倒数, {stream_fps})）"
            )

    streamer = WebStreamer(port=stream_port, jpeg_quality=mpeg_quality)

    latest = {0: None, 1: None}
    detections = {0: ([], [], []), 1: ([], [], [])}
    last_infer_t = {0: 0.0, 1: 0.0}
    frames_in_window = 0
    last_perf = time.time()

    try:
        while True:
            t0 = time.time()
            frames_in_window += 1

            # ---- 两路独立取帧 / 独立投递推理 / 独立推流 ----
            for cid in (0, 1):
                cfg_i = cam_cfg[cid]

                if not int(cfg_i.get("enable", 1)) or not alive.get(cid):
                    streamer.update_frame(cid, make_offline_frame(cid))
                    continue

                cam = cams[cid]
                frame = None
                try:
                    frame = cam.read()
                except Exception as e:  # noqa: BLE001 -- camera backends vary by platform.
                    print(f"[CAM{cid}] 取帧异常：{type(e).__name__}: {e}")

                if frame is None:
                    cam.fail += 1
                    if cam.fail >= 10:
                        print(f"[CAM{cid}] 连续 {cam.fail} 次取帧失败，本路停用")
                        alive[cid] = False
                        cam.close()
                    streamer.update_frame(cid, make_offline_frame(cid))
                    continue

                cam.fail = 0
                latest[cid] = frame

                # 按时间门控投递推理：距上次投递达到 1/infer_fps 秒才投一帧
                if infer_on[cid]:
                    fps = float(cfg_i.get("infer_fps", 0))
                    if fps > 0:
                        now_t = time.monotonic()
                        if now_t - last_infer_t[cid] >= 1.0 / fps:
                            last_infer_t[cid] = now_t
                            s = int(cfg_i.get("input_size", 640))
                            _put_latest(in_qs[cid], letterbox(frame, s))

                out = draw(frame.copy(), *detections[cid], cfg_i)
                streamer.update_frame(cid, out)

            # ---- 收结果：每路独立队列，不会互相覆盖 ----
            got = False
            for cid in (0, 1):
                if not infer_on[cid]:
                    continue
                try:
                    detections[cid] = out_qs[cid].get_nowait()
                    got = True
                except queue.Empty:
                    pass
            if got:
                for cid in (0, 1):
                    if alive.get(cid) and latest.get(cid) is not None:
                        out = draw(latest[cid].copy(), *detections[cid], cam_cfg[cid])
                        streamer.update_frame(cid, out)

            now = time.time()
            if now - last_perf >= 5.0:
                print(f"[PERF] 推流帧率: {frames_in_window / (now - last_perf):.1f} fps")
                last_perf = now
                frames_in_window = 0
                for label, p in procs:
                    if not p.is_alive():
                        print(f"[WARN] {label} 推理进程已退出")

            dt = time.time() - t0
            if dt < 1.0 / stream_fps:
                time.sleep(1.0 / stream_fps - dt)

    except KeyboardInterrupt:
        print("\n[INFO] 退出中...")
    finally:
        streamer.stop()
        _stop_processes(procs)
        for cam in cams.values():
            if cam is not None:
                cam.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

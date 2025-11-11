#!/usr/bin/env python3
# jetson_imx519_cuda_autofocus.py
# Clean-room autofocus implementation for IMX519 + AK7375 on Jetson (Orin Nano, etc.).
# Capture path: GStreamer nvarguscamerasrc -> OpenCV. Focus metric: PyTorch CUDA.
# Lens control: V4L2 focus_absolute on /dev/v4l-subdev* via v4l2-ctl.
#
# Notes:
# - Requires: OpenCV with GStreamer, PyTorch with CUDA, v4l2-ctl installed.
# - This does not copy vendor code. It reimplements standard contrast-AF patterns.
# - Tune steps and timing for your module.
#
# Usage examples:
#   python3 jetson_imx519_cuda_autofocus.py --sensor-id 0 --width 1280 --height 720
#   python3 jetson_imx519_cuda_autofocus.py --focuser /dev/v4l-subdev2 --burst-period 1.0
#
# MIT License (for this script).

import argparse
import subprocess
import sys
import time
import re
from typing import Tuple, Optional

import cv2
import numpy as np
import torch
import torch.nn.functional as F

def check_cuda():
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA not available. Install JetPack PyTorch or enable CUDA.")
    torch.zeros(1, device='cuda')  # warmup

# ------------------------
# Focuser via V4L2 controls
# ------------------------

class V4L2Focuser:
    def __init__(self, device: Optional[str] = None, min_pos: Optional[int] = None,
                 max_pos: Optional[int] = None, settle_ms: int = 30):
        self.device = device or self._probe_device()
        self.min_pos, self.max_pos, self.step = self._read_focus_range() if (min_pos is None or max_pos is None) else (min_pos, max_pos, 1)
        self.settle_ms = settle_ms
        self.current_pos = self.get_focus()  # may fail; ignore errors

    def _probe_device(self) -> str:
        # Find a subdev that exposes focus_absolute control.
        for n in range(0, 16):
            dev = f"/dev/v4l-subdev{n}"
            try:
                out = subprocess.check_output(["v4l2-ctl", "-d", dev, "-l"], stderr=subprocess.STDOUT, text=True)
                if "focus_absolute" in out:
                    return dev
            except subprocess.CalledProcessError:
                continue
        raise RuntimeError("No focuser subdevice with focus_absolute found in /dev/v4l-subdev*. Specify --focuser explicitly.")

    def _read_focus_range(self) -> Tuple[int, int, int]:
        out = subprocess.check_output(["v4l2-ctl", "-d", self.device, "-l"], stderr=subprocess.STDOUT, text=True)
        # Example line: focus_absolute 0x009a090a (int) : min=0 max=1023 step=1 default=0 value=100
        m = re.search(r"focus_absolute.*min=(\d+)\s+max=(\d+)\s+step=(\d+)", out)
        if not m:
            # Fallback to typical AK7375 range
            return (0, 1023, 1)
        return (int(m.group(1)), int(m.group(2)), int(m.group(3)))

    def set_focus(self, pos: int):
        pos = int(max(self.min_pos, min(self.max_pos, pos)))
        try:
            subprocess.check_call(["v4l2-ctl", "-d", self.device, "--set-ctrl", f"focus_absolute={pos}"],
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.current_pos = pos
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Failed to set focus to {pos} on {self.device}: {e}")
        if self.settle_ms > 0:
            time.sleep(self.settle_ms / 1000.0)

    def get_focus(self) -> Optional[int]:
        try:
            out = subprocess.check_output(["v4l2-ctl", "-d", self.device, "-C", "focus_absolute"],
                                          stderr=subprocess.STDOUT, text=True)
            m = re.search(r"focus_absolute:\s*(\d+)", out)
            if m:
                return int(m.group(1))
        except subprocess.CalledProcessError:
            pass
        return None

# ------------------------
# Camera via GStreamer
# ------------------------

def build_gst_pipeline(sensor_id:int, width:int, height:int, fps:int, exposure_time_us:int=None, gain:int=None) -> str:
    # nvarguscamerasrc yields NV12 in NVMM. We convert to BGR for OpenCV.
    # You can add exposure/gain controls through camera properties if supported.
    caps_nvmm = f"video/x-raw(memory:NVMM), width={width}, height={height}, format=NV12, framerate={fps}/1"
    pipeline = (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"{caps_nvmm} ! "
        f"nvvidconv ! video/x-raw, format=BGRx ! "
        f"videoconvert ! video/x-raw, format=BGR ! "
        f"appsink drop=true sync=false"
    )
    return pipeline

class GStreamerCamera:
    def __init__(self, sensor_id=0, width=1280, height=720, fps=30):
        self.pipeline = build_gst_pipeline(sensor_id, width, height, fps)
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open camera with GStreamer. Check sensor-id and Jetson multimedia stack.")
        self.width = width
        self.height = height
        self._warmup()

    def _warmup(self, n=5):
        for _ in range(n):
            self.read()

    def read(self) -> np.ndarray:
        ok, frame = self.cap.read()
        if not ok or frame is None:
            raise RuntimeError("Camera read failed.")
        return frame

    def release(self):
        self.cap.release()

# ------------------------
# CUDA autofocus metric
# ------------------------

def rgb_to_gray_torch(bgr: torch.Tensor) -> torch.Tensor:
    # bgr: BxHxWx3, float32 [0,1]
    # Convert to luma Y' = 0.114*B + 0.587*G + 0.299*R
    b = bgr[..., 0]
    g = bgr[..., 1]
    r = bgr[..., 2]
    y = 0.114 * b + 0.587 * g + 0.299 * r
    return y.unsqueeze(1)  # Bx1xHxW

def laplacian_variance_metric_cuda(frame_bgr_np: np.ndarray, roi: float = 0.6, downscale: int = 2) -> float:
    # Upload to GPU
    img = torch.from_numpy(frame_bgr_np).to(device='cuda', dtype=torch.float32) / 255.0  # HxWx3
    img = img.unsqueeze(0)  # 1xHxWx3
    gray = rgb_to_gray_torch(img)  # 1x1xHxW

    # Downscale to reduce compute
    if downscale > 1:
        h, w = gray.shape[-2:]
        new_h, new_w = h // downscale, w // downscale
        gray = F.interpolate(gray, size=(new_h, new_w), mode='area')

    # Center ROI crop
    _, _, H, W = gray.shape
    rh = int(H * roi)
    rw = int(W * roi)
    y0 = (H - rh) // 2
    x0 = (W - rw) // 2
    gray = gray[:, :, y0:y0+rh, x0:x0+rw]

    # 3x3 Laplacian kernel
    kernel = torch.tensor([[0., 1., 0.],
                           [1., -4., 1.],
                           [0., 1., 0.]], device='cuda', dtype=torch.float32).view(1, 1, 3, 3)
    lap = F.conv2d(gray, kernel, padding=1)
    # Metric: variance of Laplacian (higher is sharper)
    metric = lap.pow(2).mean() - lap.mean().pow(2)
    return float(metric.item())

# ------------------------
# AF strategy: coarse -> fine -> hill-climb
# ------------------------

class Autofocus:
    def __init__(self, focuser: V4L2Focuser, camera: GStreamerCamera,
                 coarse_step: int = 40, fine_step: int = 8, settle_ms: int = 30,
                 window: int = 160, roi: float = 0.6, downscale: int = 2,
                 verbose: bool = True):
        self.focuser = focuser
        self.camera = camera
        self.coarse_step = max(1, coarse_step)
        self.fine_step = max(1, fine_step)
        self.settle_ms = settle_ms
        self.window = window
        self.roi = roi
        self.downscale = downscale
        self.verbose = verbose

    def _measure(self) -> float:
        frame = self.camera.read()
        return laplacian_variance_metric_cuda(frame, roi=self.roi, downscale=self.downscale)

    def _scan(self, start: int, stop: int, step: int) -> Tuple[int, float]:
        best_pos = start
        best_m = -1.0
        direction = 1 if stop >= start else -1
        for pos in range(start, stop + direction, step * direction):
            self.focuser.set_focus(pos)
            time.sleep(self.settle_ms / 1000.0)  # lens settle
            m = self._measure()
            if self.verbose:
                print(f"[scan] pos={pos} metric={m:.6f}")
            if m > best_m:
                best_m, best_pos = m, pos
        return best_pos, best_m

    def _hill_climb(self, start_pos: int, init_metric: float, step: int = 4, max_iters: int = 20) -> Tuple[int, float]:
        best_pos = start_pos
        best_m = init_metric
        pos = start_pos
        last_dir = 0
        for it in range(max_iters):
            # Try both directions, favor last improvement
            candidates = [pos + step, pos - step] if last_dir >= 0 else [pos - step, pos + step]
            improved = False
            for p in candidates:
                p = int(max(self.focuser.min_pos, min(self.focuser.max_pos, p)))
                if p == pos:
                    continue
                self.focuser.set_focus(p)
                time.sleep(self.settle_ms / 1000.0)
                m = self._measure()
                if self.verbose:
                    print(f"[hill] pos={p} metric={m:.6f}")
                if m > best_m * 1.002:  # 0.2% threshold to avoid noise
                    best_m, best_pos, pos, improved = m, p, p, True
                    last_dir = 1 if p > pos else -1
                    break
            if not improved:
                break
        return best_pos, best_m

    def run_burst(self) -> Tuple[int, float]:
        # Coarse scan across full range
        start = self.focuser.min_pos
        stop = self.focuser.max_pos
        best_pos, best_m = self._scan(start, stop, self.coarse_step)
        # Fine scan around best
        fine_start = max(self.focuser.min_pos, best_pos - self.window)
        fine_stop = min(self.focuser.max_pos, best_pos + self.window)
        best_pos, best_m = self._scan(fine_start, fine_stop, self.fine_step)
        # Hill-climb refine
        best_pos, best_m = self._hill_climb(best_pos, best_m, step=max(1, self.fine_step // 2), max_iters=20)
        # Move to final
        self.focuser.set_focus(best_pos)
        if self.verbose:
            print(f"[result] best_pos={best_pos} metric={best_m:.6f}")
        return best_pos, best_m

def main():
    parser = argparse.ArgumentParser(description="CUDA autofocus for IMX519 + AK7375 on Jetson")
    parser.add_argument("--sensor-id", type=int, default=0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--focuser", type=str, default=None, help="Path to /dev/v4l-subdevX with focus control")
    parser.add_argument("--settle-ms", type=int, default=30)
    parser.add_argument("--coarse-step", type=int, default=40)
    parser.add_argument("--fine-step", type=int, default=8)
    parser.add_argument("--window", type=int, default=160)
    parser.add_argument("--roi", type=float, default=0.6)
    parser.add_argument("--downscale", type=int, default=2)
    parser.add_argument("--burst-period", type=float, default=1.0, help="Seconds between AF bursts")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    check_cuda()
    cam = GStreamerCamera(sensor_id=args.sensor_id, 
                          width=args.width, height=args.height, fps=args.fps)
    focuser = V4L2Focuser(device=args.focuser, settle_ms=args.settle_ms)
    af = Autofocus(focuser=focuser, camera=cam, coarse_step=args.coarse_step, fine_step=args.fine_step,
                   settle_ms=args.settle_ms, window=args.window, roi=args.roi, downscale=args.downscale,
                   verbose=args.verbose)

    print(f"Camera opened {args.width}x{args.height}@{args.fps}. Focuser on {focuser.device} range [{focuser.min_pos},{focuser.max_pos}] step {focuser.step}")
    next_time = time.time()
    try:
        while True:
            now = time.time()
            if now >= next_time:
                af.run_burst()
                next_time = now + args.burst_period
            # Drain a frame to keep pipeline alive
            _ = cam.read()
    except KeyboardInterrupt:
        pass
    finally:
        cam.release()

if __name__ == "__main__":
    main()

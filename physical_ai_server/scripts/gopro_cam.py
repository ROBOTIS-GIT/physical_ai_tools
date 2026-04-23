#!/usr/bin/env python3
"""
GoPro Webcam + MJPEG HTTP Stream Server (All-in-One)
노트북에서 이 파일 하나만 실행하면 GoPro 시작 → MJPEG 서버까지 자동 실행.
AI Server 브라우저에서 http://<노트북IP>:8090/stream 으로 접속.

========================================
  새 노트북에서 처음 설정할 때 필요한 것들
========================================

1. Python 패키지:
    pip install opencv-python

2. GoPro 웹캠 도구:
    sudo pip install gopro-webcam
    (또는 https://github.com/jschmid1/gopro_as_webcam_on_linux)

3. v4l2loopback 커널 모듈:
    sudo apt install v4l2loopback-dkms v4l2loopback-utils

4. FFmpeg:
    sudo apt install ffmpeg

5. 방화벽 (로봇 브라우저에서 접속하려면 포트 열기):
    sudo ufw allow 8090/tcp
    (또는 방화벽 비활성화: sudo ufw disable)

========================================

Usage:
    # 기본 실행 (GoPro 시작 + MJPEG 서버)
    sudo python3 gopro_mjpeg_server.py

    # 포트/해상도/품질 변경
    sudo python3 gopro_mjpeg_server.py --port 9000 --width 640 --height 360

    # GoPro 이미 켜져있고 서버만 실행
    python3 gopro_mjpeg_server.py --server-only --source /dev/video42
"""

import os
# ──────────────────────────────────────
# 버퍼 최소화 설정 (리눅스 지연 문제 해결)
# ──────────────────────────────────────
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
    "fflags;nobuffer|flags;low_delay|rtbufsize;1M|"
    "probesize;32000|analyzeduration;0|max_delay;0|"
    "thread_queue_size;16|"
    "reorder_queue_size;0"
)

import argparse
import glob
import subprocess
import threading
import time
import signal
import sys
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

import cv2


# ──────────────────────────────────────
#  GoPro Startup
# ──────────────────────────────────────

GOPRO_DEVICE = "/dev/video42"
GOPRO_WAIT_TIMEOUT = 20  # seconds


def _device_has_frames(dev_path: str, timeout_sec: float = 2.0) -> bool:
    """Return True when the device can be opened and at least one frame is readable."""
    cap = cv2.VideoCapture(dev_path, cv2.CAP_V4L2)
    if not cap.isOpened():
        return False

    start = time.time()
    ok = False
    while time.time() - start < timeout_sec:
        ret, frame = cap.read()
        if ret and frame is not None:
            ok = True
            break
        time.sleep(0.05)

    cap.release()
    return ok


def _detect_enx_host_ip() -> str:
    """Best-effort detect host IPv4 on enx* interface used by GoPro USB ethernet."""
    try:
        out = subprocess.check_output(["ip", "-4", "addr", "show"], text=True)
    except Exception:
        return ""

    current_iface = ""
    for raw in out.splitlines():
        line = raw.strip()
        if raw and raw[0].isdigit() and ":" in raw:
            current_iface = raw.split(":", 2)[1].strip().split("@", 1)[0]
            continue
        if current_iface.startswith("enx") and line.startswith("inet "):
            cidr = line.split()[1]
            return cidr.split("/", 1)[0]
    return ""


def _candidate_gopro_ips(explicit_ip: str = ""):
    """Build candidate GoPro control IPs for startup retries."""
    if explicit_ip:
        return [explicit_ip]

    host_ip = _detect_enx_host_ip()
    if not host_ip:
        return []

    parts = host_ip.split(".")
    if len(parts) != 4:
        return []

    # Common GoPro USB control IPs observed in the wild.
    base = ".".join(parts[:3])
    candidates = [f"{base}.51", f"{base}.1", f"{base}.52", f"{base}.53"]

    # Keep order but deduplicate.
    seen = set()
    ordered = []
    for ip in candidates:
        if ip in seen:
            continue
        seen.add(ip)
        ordered.append(ip)
    return ordered


def _run_quiet(cmd):
    try:
        subprocess.run(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
            timeout=3,
        )
    except (subprocess.TimeoutExpired, KeyboardInterrupt, OSError):
        pass


def _terminate_process_tree(proc):
    """Terminate subprocess and its process group when available."""
    if proc is None:
        return
    if proc.poll() is not None:
        return

    try:
        # start_new_session=True creates a dedicated process group.
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=2)
        return
    except Exception:
        pass

    try:
        proc.terminate()
        proc.wait(timeout=2)
    except Exception:
        pass


def _as_root_cmd(cmd):
    if os.geteuid() == 0:
        return cmd
    return ["sudo", "-n", *cmd]


def stop_gopro_stack(port=8090, source=GOPRO_DEVICE):
    """Force-stop GoPro stack and release related port/device locks."""
    _run_quiet(_as_root_cmd(["pkill", "-f", "gopro webcam"]))
    _run_quiet(_as_root_cmd(["pkill", "-f", "ffmpeg"]))
    _run_quiet(_as_root_cmd(["fuser", "-k", f"{port}/tcp"]))
    if source and os.path.exists(source):
        _run_quiet(_as_root_cmd(["fuser", "-k", source]))


def kill_previous(port=8090):
    """Kill any leftover gopro/ffmpeg/server processes (자기 자신 제외)."""
    stop_gopro_stack(port=port, source=GOPRO_DEVICE)
    time.sleep(1)


def start_gopro(port=8090, explicit_ip="", retries=1, width=None, height=None):
    """Start GoPro webcam and wait for readable v4l2loopback frames.
    
    Note: --width/height는 OpenCV에서 리스케일링하는 데만 사용됩니다.
    GoPro는 항상 720p로 시작합니다 (버퍼 최소화를 위해).
    """
    kill_previous(port=port)

    candidates = _candidate_gopro_ips(explicit_ip=explicit_ip)
    attempts = []
    if candidates:
        for _ in range(max(1, retries)):
            attempts.extend(candidates)
    else:
        attempts = [""] * max(1, retries)

    print(f"[*] GoPro 캡처 해상도: 720p (버퍼 최소화), OpenCV 리스케일: {width}x{height} (if set)")

    last_exit = None
    for idx, ip in enumerate(attempts, start=1):
        print(f"[*] GoPro 웹캠 시작... (시도 {idx}/{len(attempts)})")
        cmd = ["gopro", "webcam", "-p", "enx", "-n", "-a", "-r", "720"]
        if ip:
            cmd.extend(["-i", ip])
            print(f"[*] GoPro IP 강제 지정: {ip}")

        gopro_proc = subprocess.Popen(cmd, start_new_session=True)

        # Wait until loopback device is actually readable.
        print(f"[*] FFmpeg 스트림 시작 대기 중 (최대 {GOPRO_WAIT_TIMEOUT}초)...")
        start = time.time()
        last_status = ""
        while time.time() - start < GOPRO_WAIT_TIMEOUT:
            if gopro_proc.poll() is not None:
                last_exit = gopro_proc.returncode
                print(f"[!] GoPro 웹캠 시작 실패 (exit={last_exit})")
                break

            if os.path.exists(GOPRO_DEVICE) and _device_has_frames(GOPRO_DEVICE, timeout_sec=0.7):
                print(f"[OK] {GOPRO_DEVICE} 프레임 준비 완료")
                return gopro_proc

            status = "device exists but no frames yet" if os.path.exists(GOPRO_DEVICE) else "device not found yet"
            if status != last_status:
                print(f"[*] 대기 중: {status}")
                last_status = status
            time.sleep(0.5)

        # Cleanup failed attempt before next retry.
        try:
            if gopro_proc.poll() is None:
                gopro_proc.terminate()
                gopro_proc.wait(timeout=2)
        except Exception:
            pass
        stop_gopro_stack(port=port, source=GOPRO_DEVICE)
        time.sleep(1)

    print(
        f"[!] {GOPRO_DEVICE} 프레임 준비 실패. GoPro Webcam 모드(403) 또는 USB 연결 상태를 확인하세요."
    )
    if last_exit is not None:
        print(f"[!] 마지막 gopro webcam 종료 코드: {last_exit}")
    sys.exit(1)


# ──────────────────────────────────────
#  Frame Grabber
# ──────────────────────────────────────

class FrameGrabber:
    """Capture thread: always holds the latest frame only."""

    def __init__(self, source, width, height, quality):
        self.source = source
        self.width = width
        self.height = height
        self.quality = quality

        self._frame_jpeg = None
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._running = False
        self._thread = None
        self._backend = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)

    def get_frame(self, timeout=2.0):
        """Return latest JPEG bytes, or None on timeout."""
        with self._condition:
            if self._frame_jpeg is None:
                self._condition.wait(timeout=timeout)
            return self._frame_jpeg

    def get_frame_nowait(self):
        """Get latest frame without blocking (for real-time streaming)."""
        with self._lock:
            return self._frame_jpeg

    def _open_capture(self):
        """Open video source with buffer minimization."""
        if self.source.startswith('/dev/'):
            cap = cv2.VideoCapture(self.source, cv2.CAP_V4L2)
            if cap.isOpened():
                # Minimize buffer at OpenCV level
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                cap.set(cv2.CAP_PROP_FPS, 30)  # Explicit FPS
                self._backend = 'v4l2'
                print(f"[OK] Opened {self.source} via V4L2 (buffer=1)")
                return cap
            print(f"[!] Failed to open {self.source}")
            return None

        # Auto-detect v4l2loopback device
        v4l2_devices = sorted(glob.glob('/dev/video*'))
        for dev in v4l2_devices:
            sysfs = f"/sys/devices/virtual/video4linux/{dev.split('/')[-1]}"
            if not os.path.exists(sysfs):
                continue
            try:
                cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
                if cap.isOpened():
                    # Minimize buffer
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    cap.set(cv2.CAP_PROP_FPS, 30)
                    ret, _ = cap.read()
                    if ret:
                        self._backend = f'v4l2-auto({dev})'
                        print(f"[OK] Auto-detected v4l2loopback: {dev} (buffer=1)")
                        return cap
                    cap.release()
            except Exception:
                continue
        return None

    def _capture_loop(self):
        cap = None
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
        frame_count = 0
        prev_timestamp = time.time()

        while self._running:
            if cap is None or not cap.isOpened():
                if cap:
                    cap.release()
                print("[*] Opening video source...")
                cap = self._open_capture()
                if cap is None:
                    print("[!] Failed to open source, retrying in 2s...")
                    time.sleep(2)
                    continue

            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.001)  # 1ms只在无帧时等待
                continue

            # Resize if needed (skip if buffer minimization sufficient)
            if self.width and self.height:
                h, w = frame.shape[:2]
                if w != self.width or h != self.height:
                    frame = cv2.resize(
                        frame, (self.width, self.height), interpolation=cv2.INTER_LINEAR
                    )

            # JPEG encode (optimized quality)
            success, jpeg = cv2.imencode('.jpg', frame, encode_params)
            if not success:
                continue

            # Drop old frame, keep only latest (critical for latency)
            with self._condition:
                self._frame_jpeg = jpeg.tobytes()
                self._condition.notify_all()

            frame_count += 1
            
            # Log at reduced frequency to avoid overhead
            if frame_count == 1:
                print(f"[OK] First frame captured ({self._backend})")
            elif frame_count % 1000 == 0:
                elapsed = time.time() - prev_timestamp
                fps = 1000 / elapsed if elapsed > 0 else 0
                print(f"[*] {frame_count} frames @ {fps:.1f} FPS (버퍼 최소화 모드)")
                prev_timestamp = time.time()

        if cap:
            cap.release()


# ──────────────────────────────────────
#  MJPEG HTTP Server
# ──────────────────────────────────────

class MJPEGHandler(BaseHTTPRequestHandler):
    grabber = None

    def do_GET(self):
        if self.path.startswith('/stream'):
            self._handle_stream()
        elif self.path.startswith('/snapshot'):
            self._handle_snapshot()
        else:
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(
                b'<html><body>'
                b'<h2>GoPro MJPEG Server</h2>'
                b'<p><a href="/stream">/stream</a> - MJPEG live stream</p>'
                b'<p><a href="/snapshot">/snapshot</a> - Single JPEG frame</p>'
                b'<img src="/stream" style="max-width:100%">'
                b'</body></html>'
            )

    def _handle_stream(self):
        self.send_response(200)
        self.send_header(
            'Content-Type', 'multipart/x-mixed-replace; boundary=frame'
        )
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        prev_frame = None
        try:
            while True:
                # Use nowait() to always get latest frame, minimize latency
                jpeg = self.grabber.get_frame_nowait()
                if jpeg is None:
                    time.sleep(0.001)  # 1ms wait for next frame
                    continue
                if jpeg is prev_frame:
                    time.sleep(0.001)  # Already streaming this frame
                    continue
                prev_frame = jpeg

                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(f'Content-Length: {len(jpeg)}\r\n'.encode())
                self.wfile.write(b'\r\n')
                self.wfile.write(jpeg)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _handle_snapshot(self):
        jpeg = self.grabber.get_frame(timeout=5.0)
        if jpeg is None:
            self.send_response(503)
            self.end_headers()
            self.wfile.write(b'No frame available')
            return

        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(jpeg)))
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(jpeg)

    def log_message(self, format, *args):
        pass


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


# ──────────────────────────────────────
#  Main
# ──────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='GoPro Webcam + MJPEG HTTP Stream Server'
    )
    parser.add_argument(
        '--server-only', action='store_true',
        help='MJPEG 서버만 실행 (GoPro는 이미 켜져있을 때)'
    )
    parser.add_argument(
        '--source', default=GOPRO_DEVICE,
        help=f'Video source device (default: {GOPRO_DEVICE})'
    )
    parser.add_argument(
        '--port', type=int, default=8090,
        help='HTTP server port (default: 8090)'
    )
    parser.add_argument(
        '--quality', type=int, default=60,
        help='JPEG quality 1-100 (default: 60, 낮을수록 지연 감소)'
    )
    parser.add_argument(
        '--width', type=int, default=0,
        help='Resize width (0 = no resize)'
    )
    parser.add_argument(
        '--height', type=int, default=0,
        help='Resize height (0 = no resize)'
    )
    parser.add_argument(
        '--gopro-ip', default='',
        help='GoPro control IPv4 override (ex: 172.21.172.51)'
    )
    parser.add_argument(
        '--startup-retries', type=int, default=2,
        help='GoPro startup retry count (default: 2)'
    )
    args = parser.parse_args()

    gopro_proc = None
    grabber = None
    server = None
    cleanup_done = False

    def cleanup():
        nonlocal cleanup_done
        if cleanup_done:
            return
        cleanup_done = True

        # Ignore repeated Ctrl+C while cleanup is running.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)

        print("\n[!] 종료 중...")

        if grabber is not None:
            grabber.stop()

        if server is not None:
            try:
                server.shutdown()
                server.server_close()
            except Exception:
                pass

        _terminate_process_tree(gopro_proc)

        # Always release GoPro-related locks, even in --server-only mode.
        stop_gopro_stack(port=args.port, source=args.source)
        # Second pass helps when ffmpeg/gopro exits slightly later.
        time.sleep(0.3)
        stop_gopro_stack(port=args.port, source=args.source)

        print("[OK] 정리 완료.")

    def _raise_keyboard_interrupt(sig, frame):
        # Signal handler에서는 최소 작업만 하고 정리는 finally에서 수행.
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

    try:
        # 1. GoPro 시작 (--server-only 가 아닐 때)
        if not args.server_only:
            gopro_proc = start_gopro(
                port=args.port,
                explicit_ip=args.gopro_ip.strip(),
                retries=max(1, args.startup_retries),
                width=args.width if args.width > 0 else None,
                height=args.height if args.height > 0 else None,
            )
        else:
            # --server-only 에서도 포트 정리
            _run_quiet(_as_root_cmd(["fuser", "-k", f"{args.port}/tcp"]))
            time.sleep(0.5)

        # 2. MJPEG 서버 시작
        width = args.width if args.width > 0 else None
        height = args.height if args.height > 0 else None

        grabber = FrameGrabber(
            source=args.source,
            width=width,
            height=height,
            quality=args.quality,
        )
        MJPEGHandler.grabber = grabber
        grabber.start()

        # Fail fast for stale loopback nodes with no actual frames.
        first = grabber.get_frame(timeout=6.0)
        if first is None:
            raise RuntimeError(
                "카메라 프레임을 받지 못했습니다. GoPro Webcam 모드 실패(403) 또는 장치 점유를 확인하세요."
            )

        server = ThreadedHTTPServer(('0.0.0.0', args.port), MJPEGHandler)

        print("")
        print("========================================")
        print("  GoPro MJPEG Stream Ready!")
        print(f"  Stream:   http://0.0.0.0:{args.port}/stream")
        print(f"  Snapshot: http://0.0.0.0:{args.port}/snapshot")
        print(f"  Source:   {args.source}")
        print(f"  Quality:  {args.quality}")
        if width and height:
            print(f"  Resize:   {width}x{height}")
        print("  Ctrl+C 로 종료")
        print("========================================")
        print("")

        server.serve_forever(poll_interval=0.5)
    except RuntimeError as e:
        print(f"[!] {e}")
    except KeyboardInterrupt:
        print("\n[!] Ctrl+C 감지, 종료합니다...")
    finally:
        cleanup()


if __name__ == '__main__':
    main()

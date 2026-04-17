#!/usr/bin/env python3
"""
Multi Webcam + MJPEG HTTP Stream Server

Usage:
	# 연결된 카메라 자동 감지 후 서버 실행
	python3 all_cams.py

	# 포트/해상도/품질 변경
	python3 all_cams.py --port 8091 --width 640 --height 360 --quality 80

	# GoPro/가상 장치도 포함
	python3 all_cams.py --include-gopro

	# 연결 확인만 수행
	python3 all_cams.py --check-only
"""

import argparse
import glob
import json
import os
import re
import signal
import socket
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from urllib.parse import urlparse

import cv2


DEFAULT_PORT = 8091


def _read_file_text(path: str) -> str:
	try:
		with open(path, "r", encoding="utf-8") as f:
			return f.read().strip()
	except OSError:
		return ""


def _resolve_video_path(dev_path: str) -> str:
	resolved = os.path.realpath(dev_path)
	if resolved.startswith("/dev/video") and os.path.exists(resolved):
		return resolved
	return ""


def _to_video_node(dev_path: str) -> str:
	return os.path.basename(_resolve_video_path(dev_path) or dev_path)


def _read_sysfs_attr(dev_path: str, attr: str) -> str:
	video_node = _to_video_node(dev_path)
	return _read_file_text(f"/sys/class/video4linux/{video_node}/{attr}")


def _device_name(dev_path: str) -> str:
	return _read_sysfs_attr(dev_path, "name")


def _device_index(dev_path: str):
	index_raw = _read_sysfs_attr(dev_path, "index")
	try:
		return int(index_raw)
	except ValueError:
		return None


def _get_usb_vid_pid(dev_path: str):
	"""Return (vendor_id, product_id) for a video device, or ('', '')."""
	try:
		video_node = _to_video_node(dev_path)
		devpath = os.path.realpath(f"/sys/class/video4linux/{video_node}")
		for _ in range(10):
			vendor_path = os.path.join(devpath, "idVendor")
			product_path = os.path.join(devpath, "idProduct")
			if os.path.exists(vendor_path) and os.path.exists(product_path):
				vendor = _read_file_text(vendor_path).lower()
				product = _read_file_text(product_path).lower()
				return vendor, product
			parent = os.path.dirname(devpath)
			if parent == devpath:
				break
			devpath = parent
	except OSError:
		pass
	return "", ""


def _video_sort_key(path: str):
	name = os.path.basename(path)
	m = re.search(r"(\d+)$", name)
	idx = int(m.group(1)) if m else 10**9
	return idx, path


def _build_by_id_map():
	"""Map resolved /dev/videoX -> stable by-id label (index0 only)."""
	mapping = {}
	for by_id in sorted(glob.glob("/dev/v4l/by-id/*video-index0")):
		resolved = _resolve_video_path(by_id)
		if not resolved:
			continue
		mapping[resolved] = os.path.basename(by_id)
	return mapping


def _slugify(text: str) -> str:
	base = re.sub(r"[^a-zA-Z0-9]+", "-", text).strip("-").lower()
	return base or "cam"


def _get_local_ipv4_addresses():
	"""Return preferred local IPv4 addresses for URL printing.

	Only 192.168.* addresses are returned when available.
	"""
	ips = set()

	# Primary route address (works even without DNS).
	try:
		with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
			s.connect(("8.8.8.8", 80))
			ips.add(s.getsockname()[0])
	except OSError:
		pass

	# Additional host addresses.
	try:
		for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
			ip = info[4][0]
			if not ip.startswith("127."):
				ips.add(ip)
	except OSError:
		pass

	preferred = sorted(ip for ip in ips if ip.startswith("192.168."))
	if preferred:
		return preferred

	# Fallback to whatever non-loopback address exists.
	return sorted(ips)


def check_camera_ready(source: str, timeout_sec: float = 2.0):
	"""Open camera and verify at least one frame can be read."""
	cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
	if not cap.isOpened():
		return False, "open failed"

	start = time.time()
	ok = False
	while time.time() - start < timeout_sec:
		ret, frame = cap.read()
		if ret and frame is not None:
			ok = True
			break
		time.sleep(0.03)

	cap.release()
	if not ok:
		return False, "no frame"
	return True, "ready"


@dataclass
class CameraInfo:
	cam_id: str
	source: str
	name: str
	label: str
	vendor_id: str
	product_id: str


def discover_cameras(include_gopro: bool = False):
	"""Discover usable capture nodes, preferring index0 devices with readable frames."""
	by_id_map = _build_by_id_map()
	devices = sorted(glob.glob("/dev/video*"), key=_video_sort_key)
	seen = set()
	used_ids = set()
	found = []

	for dev in devices:
		resolved = _resolve_video_path(dev)
		if not resolved or resolved in seen:
			continue
		seen.add(resolved)

		if _device_index(resolved) != 0:
			continue

		name = _device_name(resolved)
		name_l = name.lower()
		if not include_gopro and "gopro" in name_l:
			continue

		ok, _ = check_camera_ready(resolved, timeout_sec=1.5)
		if not ok:
			continue

		vendor_id, product_id = _get_usb_vid_pid(resolved)
		label = by_id_map.get(resolved, os.path.basename(resolved))
		base_id = _slugify(label)
		cam_id = base_id
		suffix = 2
		while cam_id in used_ids:
			cam_id = f"{base_id}-{suffix}"
			suffix += 1
		used_ids.add(cam_id)

		found.append(
			CameraInfo(
				cam_id=cam_id,
				source=resolved,
				name=name,
				label=label,
				vendor_id=vendor_id,
				product_id=product_id,
			)
		)

	return found


class FrameGrabber:
	"""Capture thread: always holds latest JPEG frame only."""

	def __init__(self, camera: CameraInfo, width, height, quality):
		self.camera = camera
		self.width = width
		height = height
		self.height = height
		self.quality = quality

		self._frame_jpeg = None
		self._lock = threading.Lock()
		self._condition = threading.Condition(self._lock)
		self._running = False
		self._thread = None

	def start(self):
		self._running = True
		self._thread = threading.Thread(target=self._capture_loop, daemon=True)
		self._thread.start()

	def stop(self):
		self._running = False
		if self._thread:
			self._thread.join(timeout=3)

	def get_frame(self, timeout=2.0):
		with self._condition:
			if self._frame_jpeg is None:
				self._condition.wait(timeout=timeout)
			return self._frame_jpeg

	def _capture_loop(self):
		cap = None
		encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
		frame_count = 0
		source = self.camera.source
		cam_id = self.camera.cam_id

		while self._running:
			if cap is None or not cap.isOpened():
				if cap:
					cap.release()
				cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
				if not cap.isOpened():
					print(f"[!] [{cam_id}] Failed to open {source}, retry in 2s")
					time.sleep(2)
					continue
				print(f"[OK] [{cam_id}] Opened {source}")

			ret, frame = cap.read()
			if not ret or frame is None:
				time.sleep(0.01)
				continue

			if self.width and self.height:
				h, w = frame.shape[:2]
				if w != self.width or h != self.height:
					frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_AREA)

			success, jpeg = cv2.imencode(".jpg", frame, encode_params)
			if not success:
				continue

			with self._condition:
				self._frame_jpeg = jpeg.tobytes()
				self._condition.notify_all()

			frame_count += 1
			if frame_count == 1:
				print(f"[OK] [{cam_id}] First frame captured")

		if cap:
			cap.release()


class CameraRegistry:
	def __init__(self, cameras, grabbers):
		self.cameras = {c.cam_id: c for c in cameras}
		self.grabbers = grabbers

	def first_cam_id(self):
		return next(iter(self.cameras), "")


class MJPEGHandler(BaseHTTPRequestHandler):
	registry = None

	def do_GET(self):
		parsed = urlparse(self.path)
		path = parsed.path
		parts = [p for p in path.strip("/").split("/") if p]

		if not parts:
			self._handle_index()
			return

		if path == "/cameras.json":
			self._handle_cameras_json()
			return

		if len(parts) == 1 and parts[0] == "stream":
			cam_id = self.registry.first_cam_id()
			if not cam_id:
				self._send_text(503, "No camera available")
				return
			self._handle_stream(cam_id)
			return

		if len(parts) == 3 and parts[0] == "cam" and parts[2] == "stream":
			cam_id = parts[1]
			if cam_id not in self.registry.grabbers:
				self._send_text(404, f"Unknown camera id: {cam_id}")
				return
			self._handle_stream(cam_id)
			return

		if path.endswith("/snapshot") or path == "/snapshot":
			self._send_text(410, "Snapshot endpoint removed. Use /cam/<camera_id>/stream")
			return

		self._send_text(404, "Not found")

	def do_OPTIONS(self):
		self.send_response(204)
		self.send_header("Access-Control-Allow-Origin", "*")
		self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
		self.send_header("Access-Control-Allow-Headers", "*")
		self.send_header("Access-Control-Max-Age", "86400")
		self.end_headers()

	def _send_text(self, code: int, text: str):
		data = text.encode("utf-8", errors="replace")
		self.send_response(code)
		self.send_header("Content-Type", "text/plain; charset=utf-8")
		self.send_header("Content-Length", str(len(data)))
		self.end_headers()
		self.wfile.write(data)

	def _handle_cameras_json(self):
		host_header = self.headers.get("Host", "").strip()
		if not host_header:
			server_ip = self.server.server_address[0]
			server_port = self.server.server_address[1]
			host_header = f"{server_ip}:{server_port}"

		cameras_payload = []
		for cam_id, cam in self.registry.cameras.items():
			cameras_payload.append({
				"cam_id": cam_id,
				"name": cam.name,
				"label": cam.label,
				"source": cam.source,
				"stream_url": f"http://{host_header}/cam/{cam_id}/stream",
			})

		body = json.dumps({
			"server": {"port": self.server.server_address[1]},
			"cameras": cameras_payload,
		}).encode("utf-8")

		self.send_response(200)
		self.send_header("Content-Type", "application/json; charset=utf-8")
		self.send_header("Content-Length", str(len(body)))
		self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
		self.send_header("Access-Control-Allow-Origin", "*")
		self.end_headers()
		self.wfile.write(body)

	def _handle_index(self):
		rows = []
		for cam_id, cam in self.registry.cameras.items():
			rows.append(
				"<li>"
				f"<b>{cam_id}</b> | {cam.name} | {cam.source} "
				f"| <a href='/cam/{cam_id}/stream'>stream</a>"
				"</li>"
			)

		body = (
			"<html><body>"
			"<h2>Multi Camera MJPEG Server</h2>"
			"<p>Route format: /cam/&lt;camera_id&gt;/stream</p>"
			"<ul>"
			+ "".join(rows)
			+ "</ul>"
			"</body></html>"
		).encode("utf-8")

		self.send_response(200)
		self.send_header("Content-Type", "text/html; charset=utf-8")
		self.send_header("Content-Length", str(len(body)))
		self.end_headers()
		self.wfile.write(body)

	def _handle_stream(self, cam_id: str):
		grabber = self.registry.grabbers[cam_id]
		self.send_response(200)
		self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
		self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
		self.send_header("Access-Control-Allow-Origin", "*")
		self.end_headers()

		prev_frame = None
		try:
			while True:
				jpeg = grabber.get_frame(timeout=5.0)
				if jpeg is None:
					continue
				if jpeg is prev_frame:
					time.sleep(0.005)
					continue
				prev_frame = jpeg

				self.wfile.write(b"--frame\r\n")
				self.wfile.write(b"Content-Type: image/jpeg\r\n")
				self.wfile.write(f"Content-Length: {len(jpeg)}\r\n".encode())
				self.wfile.write(b"\r\n")
				self.wfile.write(jpeg)
				self.wfile.write(b"\r\n")
				self.wfile.flush()
		except (BrokenPipeError, ConnectionResetError):
			pass

	def log_message(self, format, *args):
		pass


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
	daemon_threads = True
	allow_reuse_address = True


def main():
	parser = argparse.ArgumentParser(description="Multi Webcam + MJPEG HTTP Stream Server")
	parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="HTTP server port")
	parser.add_argument("--quality", type=int, default=80, help="JPEG quality 1-100")
	parser.add_argument("--width", type=int, default=0, help="Resize width (0 = no resize)")
	parser.add_argument("--height", type=int, default=0, help="Resize height (0 = no resize)")
	parser.add_argument("--include-gopro", action="store_true", help="Include GoPro/loopback-like nodes")
	parser.add_argument("--check-only", action="store_true", help="Exit after camera connection check")
	args = parser.parse_args()

	cameras = discover_cameras(include_gopro=args.include_gopro)
	local_ips = _get_local_ipv4_addresses()
	if not cameras:
		print("[!] 사용 가능한 카메라를 찾지 못했습니다.")
		print("    팁: 장치 연결 상태, 권한(video 그룹), 장치 점유 여부를 확인하세요.")
		raise SystemExit(1)

	print("[*] 감지된 카메라:")
	for cam in cameras:
		usb = f"{cam.vendor_id}:{cam.product_id}" if cam.vendor_id and cam.product_id else "-"
		print(f"    - {cam.cam_id} | {cam.source} | {cam.name} | usb={usb}")

	print("[*] Stream URL:")
	for cam in cameras:
		for ip in local_ips:
			print(f"    - {cam.cam_id}: http://{ip}:{args.port}/cam/{cam.cam_id}/stream")

	print("[*] Discovery (JSON) URL:")
	for ip in local_ips:
		print(f"    - http://{ip}:{args.port}/cameras.json")

	if args.check_only:
		print("[OK] 카메라 연결/프레임 확인 완료")
		raise SystemExit(0)

	width = args.width if args.width > 0 else None
	height = args.height if args.height > 0 else None

	grabbers = {cam.cam_id: FrameGrabber(cam, width, height, args.quality) for cam in cameras}
	registry = CameraRegistry(cameras, grabbers)
	MJPEGHandler.registry = registry
	server = ThreadedHTTPServer(("0.0.0.0", args.port), MJPEGHandler)

	cleanup_done = False

	def cleanup():
		nonlocal cleanup_done
		if cleanup_done:
			return
		cleanup_done = True
		print("\n[!] 종료 중...")
		for g in grabbers.values():
			g.stop()
		try:
			server.server_close()
		except Exception:
			pass
		print("[OK] 정리 완료.")

	def _raise_keyboard_interrupt(sig, frame):
		raise KeyboardInterrupt

	signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
	signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

	try:
		for g in grabbers.values():
			g.start()

		print("")
		print("========================================")
		print("  Multi Camera MJPEG Stream Ready!")
		print(f"  Index: http://0.0.0.0:{args.port}/")
		print("  Route: /cam/<camera_id>/stream")
		print(f"  Quality:  {args.quality}")
		if width and height:
			print(f"  Resize:   {width}x{height}")
		print("  Ctrl+C 로 종료")
		print("========================================")
		print("")

		server.serve_forever(poll_interval=0.5)
	except KeyboardInterrupt:
		print("\n[!] Ctrl+C 감지, 종료합니다...")
	finally:
		cleanup()


if __name__ == "__main__":
	main()
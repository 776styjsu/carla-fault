#!/usr/bin/env python3
"""
inject_and_collect_pcla.py

Connect to an already-running CARLA server, spawn ego vehicle + sensors, inject a camera fault,
and drive using a PCLA agent (from external/PCLA submodule).
"""

import argparse
import os
import sys
import time
import random
import threading
import queue
import json
import copy
import subprocess
from pathlib import Path
from typing import Tuple, Optional

import numpy as np
from PIL import Image
from tqdm import tqdm

try:
    import cv2
except Exception:
    cv2 = None
try:
    import imageio
except Exception:
    imageio = None

import carla

# Add PCLA to path
pcla_path = Path(__file__).parent.parent / "external" / "PCLA"
if pcla_path.exists():
    sys.path.append(str(pcla_path))
else:
    print(f"[warning] PCLA submodule not found at {pcla_path}. Please run: git submodule update --init --recursive")

from carla_runner import EgoSpawner, FaultInjector
from faults import create_fault, list_available_faults


# -------------------------------------------------
# Global Fault Registry
# -------------------------------------------------
class GlobalFaultRegistry:
    current_fault = None
    writer = None
    rgb_queue = None
    sync_mode = False


# -------------------------------------------------
# Monkeypatching CallBack (BEFORE importing PCLA)
# -------------------------------------------------
try:
    import leaderboard_codes.sensor_interface as lcsi
    OriginalCallBack = lcsi.CallBack

    class FaultInjectionCallBack(OriginalCallBack):
        def __init__(self, tag, sensor_type, sensor, data_provider):
            super().__init__(tag, sensor_type, sensor, data_provider)
            self.sensor_type = sensor_type

        def _parse_image_cb(self, image, tag):
            # Convert raw data to numpy (BGRA)
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = copy.deepcopy(array)
            array = np.reshape(array, (image.height, image.width, 4))
            
            # Inject fault if it's an RGB camera and fault is set
            # PCLA agents typically use 'sensor.camera.rgb'
            if self.sensor_type == 'sensor.camera.rgb' and GlobalFaultRegistry.current_fault:
                # Extract RGB for fault injection
                bgr = array[:, :, :3]
                # IMPORTANT: Create a COPY for the clean reference, otherwise it's a view
                # and will be modified when we update 'array' below.
                rgb = bgr[:, :, ::-1].copy()
                
                # Apply fault
                rgb_faulty = GlobalFaultRegistry.current_fault.apply_to_image(rgb)
                
                # Convert back to BGR and update array
                bgr_faulty = rgb_faulty[:, :, ::-1]
                array[:, :, :3] = bgr_faulty

                # Record if this is the main camera and writer is available
                # We assume 'rgb_front' or 'Center' is the main camera
                if tag in ['rgb_front', 'Center', 'RGB', 'rgb_0'] and GlobalFaultRegistry.writer:
                    if GlobalFaultRegistry.sync_mode and GlobalFaultRegistry.rgb_queue:
                        GlobalFaultRegistry.rgb_queue.put({"frame": image.frame, "clean": rgb, "faulty": rgb_faulty})
                    else:
                        GlobalFaultRegistry.writer.enqueue_rgb_pair(image.frame, rgb, rgb_faulty)
            
            self._data_provider.update_sensor(tag, array, image.frame)

    # Apply the monkeypatch to the source module
    lcsi.CallBack = FaultInjectionCallBack
    print("[info] Monkeypatched leaderboard_codes.sensor_interface.CallBack for fault injection.")

except ImportError as e:
    print(f"[warning] Could not import leaderboard_codes.sensor_interface: {e}")


try:
    import PCLA as PCLA_module
    from PCLA import PCLA, route_maker, location_to_waypoint
except ImportError:
    PCLA_module = None
    PCLA = None
    print("[error] Could not import PCLA. Make sure the submodule is initialized and dependencies are installed.")


# -------------------------------------------------
# Argument parsing
# -------------------------------------------------
def parse_args():
    available_faults = list_available_faults()
    default_fault = "camera_blackout" if "camera_blackout" in available_faults else (
        available_faults[0] if available_faults else "camera_blackout"
    )

    p = argparse.ArgumentParser(
        description="Spawn ego + sensors, inject fault, drive with PCLA agent."
    )
    p.add_argument("--carla-root", type=Path, required=True,
                   help="Path to CARLA install root (contains CarlaUE4.sh and PythonAPI/).")
    p.add_argument("--port", type=int, default=2000, help="RPC port for CARLA server (default: 2000).")
    p.add_argument("--time", type=float, default=60.0, help="How long (s) to run closed-loop before cleanup.")
    
    # PCLA specific
    p.add_argument("--agent", type=str, default="tf_tf", 
                   help="PCLA agent name (e.g., tf_tf, lav_lav). "
                        "For AIM (single image), use 'tfpp_aim_0'.")
    p.add_argument("--route-file", type=str, default="temp_route.xml", help="Path to save/load the generated route XML.")

    # Frame (image) saving
    p.add_argument("--save-rgb", action="store_true", help="Save clean & faulted RGB frames as PNG.")
    p.add_argument("--save-depth", action="store_true", help="Save depth frames as PNG.")
    # Video saving
    p.add_argument("--video-rgb", action="store_true", help="Save RGB videos (clean & faulty) as MP4.")
    p.add_argument("--video-depth", action="store_true", help="Save depth video as MP4.")
    p.add_argument("--video-fps", type=float, default=None,
                   help="FPS for videos. Default: CARLA fixed delta if set, else 20.")
    p.add_argument("--video-backend", type=str, default="auto", choices=["auto", "cv2", "imageio", "ffmpeg"],
                   help="Backend for video recording. 'ffmpeg' requires ffmpeg installed. 'auto' tries cv2 then imageio.")
    # Vehicle log
    p.add_argument("--log-vehicle", action="store_true",
                   help="Log vehicle control/state to out_dir/vehicle.jsonl (one row per tick).")
    p.add_argument("--sync", action="store_true", default=True,
                   help="Force synchronous mode (recommended for correct video timing). Default: True.")
    p.add_argument("--fault-severity", type=float, default=0.8, help="Fault severity (0..1).")
    fault_help = "Fault to inject."
    if available_faults:
        fault_help += f" Available: {', '.join(sorted(available_faults))}."
    fault_help += f" Default: {default_fault}."
    p.add_argument("--fault", type=str, default=default_fault, help=fault_help)
    p.add_argument("--out-dir", type=Path, default=Path("out_debug_pcla"),
                   help="Directory where captures (frames/videos/logs) will be stored.")
    return p.parse_args()


# -------------------------------------------------
# Helpers
# -------------------------------------------------
def connect_existing(port: int, max_wait_s: float = 10.0, timeout_s: float = 2.0) -> Tuple[carla.Client, carla.World]:
    start = time.time()
    last_err = None
    while time.time() - start < max_wait_s:
        try:
            client = carla.Client("localhost", port)
            client.set_timeout(timeout_s)
            world = client.get_world()
            _ = world.get_map()
            print(f"[info] Connected to CARLA on port {port}")
            return client, world
        except Exception as e:
            last_err = e
            time.sleep(0.5)
    raise RuntimeError(f"Failed to connect to CARLA on port {port} within {max_wait_s}s. Last error: {last_err}")


def carla_rgb_to_numpy(img: carla.Image) -> np.ndarray:
    bgra = np.frombuffer(img.raw_data, dtype=np.uint8).reshape((img.height, img.width, 4))
    bgr = bgra[:, :, :3]
    rgb = bgr[:, :, ::-1]
    return rgb


def depth_bytes_to_numpy(raw_bytes: bytes, h: int, w: int) -> np.ndarray:
    arr = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((h, w, 4))
    bgr = arr[:, :, :3]
    rgb_like = bgr[:, :, ::-1]
    return rgb_like


def vec3_to_dict(v: carla.Vector3D):
    return {"x": float(v.x), "y": float(v.y), "z": float(v.z)}


def rot_to_dict(r: carla.Rotation):
    return {"pitch": float(r.pitch), "yaw": float(r.yaw), "roll": float(r.roll)}


def get_image_for_frame(q: queue.Queue, frame_id: int, timeout: float = 2.0) -> Tuple[np.ndarray, np.ndarray]:
    """
    Block until we receive the camera image matching the given simulator frame id.
    Returns (clean_rgb, faulty_rgb).
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            item = q.get(timeout=max(0.0, deadline - time.time()))
            if item["frame"] == frame_id:
                return item["clean"], item["faulty"]
            # If we somehow jumped ahead (shouldn't happen in sync mode), return the latest.
            if item["frame"] > frame_id:
                return item["clean"], item["faulty"]
        except queue.Empty:
            pass
    raise RuntimeError(f"Timed out waiting for sensor frame {frame_id}")


# ---------------------- Video helpers ----------------------
class LazyVideoWriter:
    """
    Lazily opens a video writer on first frame.
    Uses OpenCV if available, otherwise imageio. Expects RGB uint8 frames by default.
    """
    def __init__(self, path: Path, fps: float, backend: str = "auto"):
        self.path = str(path)
        self.fps = float(fps)
        self.backend_pref = backend
        self._backend = None  # 'cv2', 'imageio', or 'ffmpeg'
        self._writer = None
        self._size = None  # (w, h)
        self._proc = None  # for ffmpeg subprocess

    def _open(self, frame: np.ndarray):
        h, w = frame.shape[:2]
        self._size = (w, h)
        
        # Decide backend
        if self.backend_pref == "ffmpeg":
            self._open_ffmpeg(w, h)
            return
        elif self.backend_pref == "cv2":
            if cv2 is None:
                raise RuntimeError("cv2 backend requested but opencv-python not installed.")
            self._open_cv2(w, h)
            return
        elif self.backend_pref == "imageio":
            if imageio is None:
                raise RuntimeError("imageio backend requested but imageio not installed.")
            self._open_imageio(w, h)
            return
        
        # Auto mode
        if cv2 is not None:
            try:
                self._open_cv2(w, h)
            except Exception as e:
                print(f"[warn] cv2 video writer failed: {e}. Trying imageio...")
                if imageio is not None:
                    self._open_imageio(w, h)
                else:
                    print("[warn] imageio not installed. Trying ffmpeg subprocess...")
                    self._open_ffmpeg(w, h)
        elif imageio is not None:
            self._open_imageio(w, h)
        else:
            # Fallback to ffmpeg subprocess if nothing else
            self._open_ffmpeg(w, h)

    def _open_cv2(self, w, h):
        # Try mp4v first, then avc1
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self._writer = cv2.VideoWriter(self.path, fourcc, self.fps, (w, h))
        if not self._writer or not self._writer.isOpened():
            # Try avc1
            fourcc = cv2.VideoWriter_fourcc(*"avc1")
            self._writer = cv2.VideoWriter(self.path, fourcc, self.fps, (w, h))
            if not self._writer or not self._writer.isOpened():
                raise RuntimeError(f"OpenCV VideoWriter failed to open {self.path}")
        self._backend = "cv2"
        print(f"[info] VideoWriter: using cv2 backend for {self.path}")

    def _open_imageio(self, w, h):
        self._writer = imageio.get_writer(self.path, fps=self.fps, codec="libx264", quality=7)
        self._backend = "imageio"
        print(f"[info] VideoWriter: using imageio backend for {self.path}")

    def _open_ffmpeg(self, w, h):
        # ffmpeg -y -f rawvideo -vcodec rawvideo -s {w}x{h} -pix_fmt rgb24 -r {fps} -i - -c:v libx264 -pix_fmt yuv420p -preset fast {path}
        cmd = [
            "ffmpeg", "-y",
            "-f", "rawvideo",
            "-vcodec", "rawvideo",
            "-s", f"{w}x{h}",
            "-pix_fmt", "rgb24",
            "-r", str(self.fps),
            "-i", "-",
            "-c:v", "libx264",
            "-pix_fmt", "yuv420p",
            "-preset", "fast",
            self.path
        ]
        try:
            # Use DEVNULL for stderr to prevent deadlock if buffer fills up
            self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)
            self._backend = "ffmpeg"
            print(f"[info] VideoWriter: using ffmpeg subprocess for {self.path}")
        except FileNotFoundError:
            raise RuntimeError("ffmpeg backend requested but ffmpeg binary not found in PATH.")

    def write_rgb(self, frame_rgb: np.ndarray):
        if self._backend is None:
            self._open(frame_rgb)
        if (frame_rgb.shape[1], frame_rgb.shape[0]) != self._size:
            raise ValueError("Frame size changed during recording.")
        
        if self._backend == "cv2":
            self._writer.write(frame_rgb[:, :, ::-1])  # RGB->BGR
        elif self._backend == "imageio":
            self._writer.append_data(frame_rgb)
        elif self._backend == "ffmpeg":
            try:
                self._proc.stdin.write(frame_rgb.tobytes())
            except BrokenPipeError:
                print(f"[error] ffmpeg process died for {self.path}")

    def write_bgr(self, frame_bgr: np.ndarray):
        if self._backend is None:
            self._open(frame_bgr)
        if (frame_bgr.shape[1], frame_bgr.shape[0]) != self._size:
            raise ValueError("Frame size changed during recording.")
            
        if self._backend == "cv2":
            self._writer.write(frame_bgr)
        elif self._backend == "imageio":
            self._writer.append_data(frame_bgr[:, :, ::-1]) # BGR->RGB
        elif self._backend == "ffmpeg":
            # Convert BGR to RGB for ffmpeg (since we told it pix_fmt rgb24)
            frame_rgb = frame_bgr[:, :, ::-1]
            try:
                self._proc.stdin.write(frame_rgb.tobytes())
            except BrokenPipeError:
                print(f"[error] ffmpeg process died for {self.path}")

    def close(self):
        if self._backend == "cv2":
            if self._writer:
                self._writer.release()
        elif self._backend == "imageio":
            if self._writer:
                self._writer.close()
        elif self._backend == "ffmpeg":
            if self._proc:
                self._proc.stdin.close()
                self._proc.wait()
                if self._proc.returncode != 0:
                    print(f"[warn] ffmpeg exited with code {self._proc.returncode} for {self.path}")
                    # print(self._proc.stderr.read().decode())
        
        self._writer = None
        self._proc = None
        self._backend = None


# -------------------------------------------------
# AsyncWriter for frames + videos + vehicle logs (no drops)
# -------------------------------------------------
class AsyncWriter:
    """
    - Unbounded queue: enqueue EVERY captured item (no drops).
    - One worker thread drains the queue in arrival order.
    - Can write PNG frames, append to MP4 videos, and append JSONL logs.
    """
    def __init__(self, out_dir: Path, save_rgb: bool, save_depth: bool,
                 video_rgb: bool, video_depth: bool, video_fps: float,
                 log_vehicle: bool, video_backend: str = "auto"):
        self.q = queue.Queue()
        self.stop_flag = False

        # frames
        self.save_rgb = save_rgb
        self.save_depth = save_depth

        # videos
        self.video_rgb = video_rgb
        self.video_depth = video_depth
        self.video_fps = float(video_fps)
        self.video_backend = video_backend

        # vehicle logs
        self.log_vehicle = log_vehicle

        # directories
        self.out_dir = out_dir
        self.clean_dir = out_dir / "clean_rgb"
        self.fault_dir = out_dir / "faulty_rgb"
        self.depth_dir = out_dir / "depth"
        self.video_dir = out_dir / "video"
        self.log_path = out_dir / "vehicle.jsonl"

        if self.save_rgb:
            self.clean_dir.mkdir(parents=True, exist_ok=True)
            self.fault_dir.mkdir(parents=True, exist_ok=True)
        if self.save_depth:
            self.depth_dir.mkdir(parents=True, exist_ok=True)
        if self.video_rgb or self.video_depth:
            self.video_dir.mkdir(parents=True, exist_ok=True)
        if self.log_vehicle:
            self.out_dir.mkdir(parents=True, exist_ok=True)

        # lazy video writers
        self._vid_rgb_clean: Optional[LazyVideoWriter] = None
        self._vid_rgb_faulty: Optional[LazyVideoWriter] = None
        self._vid_depth: Optional[LazyVideoWriter] = None

        # lazy log file handle
        self._log_fp = None

        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def _ensure_video_writers(self, frame_rgb: Optional[np.ndarray] = None, depth_rgb: Optional[np.ndarray] = None):
        if self.video_rgb and self._vid_rgb_clean is None and frame_rgb is not None:
            self._vid_rgb_clean = LazyVideoWriter(self.video_dir / "rgb_clean.mp4", self.video_fps, backend=self.video_backend)
            self._vid_rgb_faulty = LazyVideoWriter(self.video_dir / "rgb_faulty.mp4", self.video_fps, backend=self.video_backend)
        if self.video_depth and self._vid_depth is None and depth_rgb is not None:
            self._vid_depth = LazyVideoWriter(self.video_dir / "depth.mp4", self.video_fps, backend=self.video_backend)

    def _ensure_log_file(self):
        if self.log_vehicle and self._log_fp is None:
            self._log_fp = open(self.log_path, "a", buffering=1)  # line-buffered

    def _worker(self):
        while True:
            try:
                item = self.q.get(timeout=0.1)
            except queue.Empty:
                if self.stop_flag:
                    break
                continue

            kind = item["kind"]

            if kind == "rgb_pair":
                frame_id = item["frame_id"]
                clean_arr = item["clean"]    # RGB uint8
                faulty_arr = item["faulty"]  # RGB uint8

                # frames
                if self.save_rgb:
                    Image.fromarray(clean_arr).save(self.clean_dir / f"rgb_{frame_id:06d}.png")
                    Image.fromarray(faulty_arr).save(self.fault_dir / f"rgb_{frame_id:06d}.png")

                # videos
                if self.video_rgb:
                    self._ensure_video_writers(frame_rgb=clean_arr)
                    self._vid_rgb_clean.write_rgb(clean_arr)
                    self._vid_rgb_faulty.write_rgb(faulty_arr)

            elif kind == "depth":
                frame_id = item["frame_id"]
                depth_arr = depth_bytes_to_numpy(item["raw_bytes"], item["h"], item["w"])

                if self.save_depth:
                    Image.fromarray(depth_arr).save(self.depth_dir / f"depth_{frame_id:06d}.png")

                if self.video_depth:
                    self._ensure_video_writers(depth_rgb=depth_arr)
                    self._vid_depth.write_rgb(depth_arr)

            elif kind == "veh_log":
                self._ensure_log_file()
                # Write a compact JSON line
                self._log_fp.write(json.dumps(item["record"], separators=(",", ":")) + "\n")

            self.q.task_done()

    def enqueue_rgb_pair(self, frame_id: int, clean_arr: np.ndarray, faulty_arr: np.ndarray):
        self.q.put({"kind": "rgb_pair", "frame_id": frame_id, "clean": clean_arr, "faulty": faulty_arr})

    def enqueue_depth_raw(self, frame_id: int, h: int, w: int, raw_bytes: bytes):
        self.q.put({"kind": "depth", "frame_id": frame_id, "h": h, "w": w, "raw_bytes": raw_bytes})

    def enqueue_vehicle_record(self, record: dict):
        if self.log_vehicle:
            self.q.put({"kind": "veh_log", "record": record})

    def shutdown(self):
        self.stop_flag = True
        
        # If queue has items, show a progress bar while they drain
        q_size = self.q.qsize()
        if q_size > 0:
            print(f"[info] Waiting for writer queue to drain ({q_size} items)...")
            with tqdm(total=q_size, unit="items", desc="Writing remaining data") as pbar:
                last_size = q_size
                while self.thread.is_alive():
                    current_size = self.q.qsize()
                    diff = last_size - current_size
                    if diff > 0:
                        pbar.update(diff)
                        last_size = current_size
                    
                    if self.q.empty():
                        break
                    time.sleep(0.1)
                # Ensure bar completes
                pbar.update(last_size - self.q.qsize())

        self.thread.join()
        # close video writers
        if self._vid_rgb_clean: self._vid_rgb_clean.close()
        if self._vid_rgb_faulty: self._vid_rgb_faulty.close()
        if self._vid_depth: self._vid_depth.close()
        # close log fp
        if self._log_fp:
            self._log_fp.close()
            self._log_fp = None


# -------------------------------------------------
# main logic
# -------------------------------------------------
def main():
    args = parse_args()

    if PCLA is None:
        print("[fatal] PCLA not imported. Exiting.")
        sys.exit(1)

    # Early check: if video requested, ensure at least one backend exists
    if (args.video_rgb or args.video_depth):
        if args.video_backend == "cv2" and cv2 is None:
            raise RuntimeError("Video backend 'cv2' requested but opencv-python is not installed.")
        if args.video_backend == "imageio" and imageio is None:
            raise RuntimeError("Video backend 'imageio' requested but imageio is not installed.")
        if args.video_backend == "auto" and cv2 is None and imageio is None:
             # Check for ffmpeg binary
             try:
                 subprocess.run(["ffmpeg", "-version"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
             except (FileNotFoundError, subprocess.CalledProcessError):
                 raise RuntimeError(
                    "Video saving requested but neither 'opencv-python' nor 'imageio' is installed, "
                    "and 'ffmpeg' binary was not found in PATH."
                )

    # Connect to existing CARLA server
    client, world = connect_existing(args.port)

    # Setup Synchronous Mode if requested
    original_settings = world.get_settings()
    settings = world.get_settings()
    
    if args.sync:
        settings.synchronous_mode = True
        # If video FPS is set, use it. Otherwise default to 20.
        target_fps = args.video_fps if (args.video_fps and args.video_fps > 0) else 20.0
        settings.fixed_delta_seconds = 1.0 / target_fps
        world.apply_settings(settings)
        print(f"[info] Synchronous mode enabled. Fixed delta: {settings.fixed_delta_seconds}s ({target_fps} FPS)")
        video_fps = target_fps
    else:
        # Derive default video FPS if not provided
        if args.video_rgb or args.video_depth:
            fps = args.video_fps
            if fps is None:
                if settings.fixed_delta_seconds and settings.fixed_delta_seconds > 0:
                    fps = 1.0 / settings.fixed_delta_seconds
                else:
                    fps = 20.0
            video_fps = float(fps)
            print(f"[info] Video FPS set to {video_fps:.2f}")
        else:
            video_fps = 0.0  # unused

    # Spawn ego
    spawner = EgoSpawner(client)
    vehicle = spawner.spawn()
    vehicle.set_autopilot(False)
    
    # Tick world once to ensure vehicle location is updated on client side
    if args.sync:
        world.tick()
    else:
        world.wait_for_tick()

    # Prepare fault injector (just for registry)
    injector = FaultInjector()
    fault = create_fault(args.fault, severity=args.fault_severity)
    injector.register_fault(fault)
    
    # Set global fault for PCLA agent AND ACTIVATE IT
    # Since we are manually injecting via monkeypatch, we must manually activate the fault.
    fault.activate()
    GlobalFaultRegistry.current_fault = fault
    GlobalFaultRegistry.sync_mode = args.sync

    # Async writer for frames + videos + logs
    writer = AsyncWriter(
        out_dir=args.out_dir,
        save_rgb=args.save_rgb,
        save_depth=args.save_depth,
        video_rgb=args.video_rgb,
        video_depth=args.video_depth,
        video_fps=video_fps,
        log_vehicle=args.log_vehicle,
        video_backend=args.video_backend,
    )
    GlobalFaultRegistry.writer = writer
    
    # Queue for sync mode
    rgb_queue = queue.Queue()
    GlobalFaultRegistry.rgb_queue = rgb_queue

    collision_sensor_bp = world.get_blueprint_library().find('sensor.other.collision')
    collision_sensor = world.spawn_actor(collision_sensor_bp, carla.Transform(), attach_to=vehicle)
    collision_events=[]

    def collision_callback(event):
        actor = event.other_actor
        impulse = event.normal_impulse
        loc = event.transform.location

        collision_events.append({
           "frame": event.frame,
           "timestamp": world.get_snapshot().timestamp.elapsed_seconds,
           "other_actor": actor.type_id,
           "location": {
               "x": loc.x,
               "y": loc.y,
               "z": loc.z
           },
           "impulse": {
               "x": impulse.x,
               "y": impulse.y,
               "z": impulse.z
           }
        })
    collision_sensor.listen(collision_callback)

    # ---------------------------------------------------------
    # PCLA Setup
    # ---------------------------------------------------------
    print(f"[info] Setting up PCLA agent: {args.agent}")
    
    # Generate route
    spawn_points = world.get_map().get_spawn_points()
    # Start from current vehicle location (which is one of the spawn points usually)
    # But vehicle is already spawned.
    start_loc = vehicle.get_location()
    
    # Pick a random destination that is at least 100m away
    dest_tf = None
    for _ in range(20): # Try 20 times
        candidate = random.choice(spawn_points)
        if candidate.location.distance(start_loc) > 100.0:
            dest_tf = candidate
            break
    
    if dest_tf is None:
        print("[warn] Could not find a destination > 100m away. Using random.")
        dest_tf = random.choice(spawn_points)

    end_loc = dest_tf.location
    
    print(f"[info] Generating route from {start_loc} to {end_loc}")
    waypoints = location_to_waypoint(client, start_loc, end_loc)
    route_maker(waypoints, args.route_file)
    print(f"[info] Route saved to {args.route_file}")

    # Initialize PCLA
    # PCLA(agent, vehicle, route, client)
    pcla_agent = PCLA(args.agent, vehicle, args.route_file, client)

    # ---------------------------------------------------------
    # Inspect and Print Agent Modalities
    # ---------------------------------------------------------
    print("\n" + "="*60)
    print(f"AGENT INPUT MODALITIES ({args.agent})")
    print("="*60)
    try:
        agent_sensors = pcla_agent.agent_instance.sensors()
        for i, s in enumerate(agent_sensors):
            s_type = s.get('type', 'unknown')
            s_id = s.get('id', 'unknown')
            print(f"Sensor #{i+1}: {s_id} ({s_type})")
            # Print key details like resolution, fov, position
            if 'width' in s and 'height' in s:
                print(f"  Resolution: {s['width']}x{s['height']}")
            if 'fov' in s:
                print(f"  FOV: {s['fov']}")
            if 'x' in s:
                print(f"  Position: x={s.get('x')}, y={s.get('y')}, z={s.get('z')}")
            if 'sensor_tick' in s:
                print(f"  Tick: {s['sensor_tick']}")
            print("-" * 30)
    except Exception as e:
        print(f"[error] Could not inspect agent sensors: {e}")
    print("="*60 + "\n")
    
    # If depth saving is requested, we need to spawn a depth camera matching the agent's RGB camera
    depth_cam = None
    if args.save_depth or args.video_depth:
        try:
            agent_sensors = pcla_agent.agent_instance.sensors()
            rgb_conf = next((s for s in agent_sensors if s['type'] == 'sensor.camera.rgb' and s['id'] in ['rgb_front', 'Center', 'RGB']), None)
            
            if rgb_conf:
                print(f"[info] Spawning depth camera matching agent's RGB config: {rgb_conf}")
                bp_lib = world.get_blueprint_library()
                depth_bp = bp_lib.find('sensor.camera.depth')
                depth_bp.set_attribute('image_size_x', str(rgb_conf['width']))
                depth_bp.set_attribute('image_size_y', str(rgb_conf['height']))
                depth_bp.set_attribute('fov', str(rgb_conf['fov']))
                if args.sync:
                    depth_bp.set_attribute('sensor_tick', '0.0')
                else:
                    # Match video FPS if async
                    tick_time = 1.0 / video_fps if video_fps > 0 else 0.0
                    depth_bp.set_attribute('sensor_tick', str(tick_time))

                spawn_point = carla.Transform(
                    carla.Location(x=rgb_conf['x'], y=rgb_conf['y'], z=rgb_conf['z']),
                    carla.Rotation(pitch=rgb_conf['pitch'], yaw=rgb_conf['yaw'], roll=rgb_conf['roll'])
                )
                depth_cam = world.spawn_actor(depth_bp, spawn_point, attach_to=vehicle)
                
                def depth_callback(img):
                    writer.enqueue_depth_raw(img.frame, img.height, img.width, bytes(img.raw_data))
                
                depth_cam.listen(depth_callback)
            else:
                print("[warn] Could not find main RGB camera in agent sensors. Depth camera not spawned.")
        except Exception as e:
            print(f"[error] Failed to spawn matching depth camera: {e}")

    current_goal = end_loc
    spectator = world.get_spectator()

    # Get initial snapshot to establish start time
    snapshot0 = world.get_snapshot()
    start_sim_time = snapshot0.timestamp.elapsed_seconds
    
    pbar = tqdm(total=args.time, unit="s", desc="Simulating")
    last_elapsed = 0.0
    step_count = 0

    try:
        while True:
            # PCLA Step
            ego_action = pcla_agent.get_action()
            vehicle.apply_control(ego_action)
            
            step_count += 1
            if step_count % 20 == 0:  # Print control every ~1s (at 20fps)
                print(f"\n[debug] Step {step_count}: Throttle={ego_action.throttle:.2f}, Steer={ego_action.steer:.2f}, Brake={ego_action.brake:.2f}")

            # Step world and get snapshot for consistent frame/timestamp
            if args.sync:
                frame_id = world.tick()
                snapshot = world.get_snapshot()
                
                # Retrieve the exact frame for this tick
                if args.save_rgb or args.video_rgb:
                    try:
                        clean, faulty = get_image_for_frame(rgb_queue, frame_id)
                        writer.enqueue_rgb_pair(frame_id, clean, faulty)
                    except RuntimeError as e:
                        print(f"[warn] {e}")
            else:
                snapshot = world.wait_for_tick()
            
            ts = snapshot.timestamp
            elapsed = ts.elapsed_seconds - start_sim_time
            
            # Update pbar
            delta = elapsed - last_elapsed
            if delta > 0:
                pbar.update(delta)
                last_elapsed = elapsed
            
            # Update queue size in description
            q_size = writer.q.qsize()
            pbar.set_postfix(write_q=q_size)
            
            # Check simulation time duration
            if elapsed > args.time:
                pbar.close()
                print(f"[info] Reached target simulation time {args.time}s. Stopping.")
                break

            # Move spectator (optional)
            # spectator.set_transform(vehicle.get_transform())

            # -------- Vehicle log (per tick) --------
            if args.log_vehicle:
                tf = vehicle.get_transform()
                vel = vehicle.get_velocity()
                ang = vehicle.get_angular_velocity()
                acc = vehicle.get_acceleration()
                speed_mps = float(np.sqrt(vel.x**2 + vel.y**2 + vel.z**2))
                loc = vehicle.get_location()
                speed_limit_kph = vehicle.get_speed_limit()
                wp = world.get_map().get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
                lane_deviation_m = loc.distance(wp.transform.location)
                
                collisions_this_frame = [
                    c for c in collision_events if c["frame"] == snapshot.frame
                ]

                record = {
                    "frame": int(snapshot.frame),
                    "timestamp": {
                        "delta_seconds": float(ts.delta_seconds),
                        "elapsed_seconds": float(getattr(ts, "elapsed_seconds", 0.0)),
                    },
                    "control": {
                        "throttle": float(ego_action.throttle),
                        "steer": float(ego_action.steer),
                        "brake": float(ego_action.brake),
                        "hand_brake": bool(ego_action.hand_brake),
                        "reverse": bool(ego_action.reverse),
                        "manual_gear_shift": bool(ego_action.manual_gear_shift),
                        "gear": int(ego_action.gear),
                    },
                    "transform": {
                        "location": {"x": float(tf.location.x), "y": float(tf.location.y), "z": float(tf.location.z)},
                        "rotation": rot_to_dict(tf.rotation),
                    },
                    "velocity": vec3_to_dict(vel),
                    "angular_velocity": vec3_to_dict(ang),
                    "acceleration": vec3_to_dict(acc),
                    "speed_mps": speed_mps,
                    "speed_kph": speed_mps * 3.6,
                    "speed_limit_kph" : speed_limit_kph,
                    "collisions_this_frame": collisions_this_frame,
                    "lane_deviation_m": lane_deviation_m,
                    "lane_width": wp.lane_width,
                    "goal": {"x": float(current_goal.x), "y": float(current_goal.y), "z": float(current_goal.z)},
                    "fault": {"name": fault.name, "severity": float(getattr(fault, "severity", 0.0))},
                    "capture_flags": {
                        "save_rgb": bool(args.save_rgb),
                        "save_depth": bool(args.save_depth),
                        "video_rgb": bool(args.video_rgb),
                        "video_depth": bool(args.video_depth),
                    },
                }
                writer.enqueue_vehicle_record(record)
                collision_events = [
                  c for c in collision_events if c["frame"] > snapshot.frame
                ]

    finally:
        # Stop manual sensors first
        if depth_cam:
            if depth_cam.is_listening:
                depth_cam.stop()
            if depth_cam.is_alive:
                depth_cam.destroy()
        
        if collision_sensor:
            if collision_sensor.is_listening:
                collision_sensor.stop()
            if collision_sensor.is_alive:
                collision_sensor.destroy()

        # Cleanup agent (which cleans up its own sensors)
        if pcla_agent:
            try:
                pcla_agent.cleanup()
            except Exception as e:
                print(f"[warning] PCLA cleanup failed: {e}")

        # Destroy vehicle last
        try:
            if vehicle and vehicle.is_alive:
                vehicle.destroy()
        except RuntimeError:
            pass

        # Restore settings if we changed them
        if args.sync:
            world.apply_settings(original_settings)

        writer.shutdown()
        print("[info] Episode finished, actors destroyed, data flushed.")


if __name__ == "__main__":
    main()

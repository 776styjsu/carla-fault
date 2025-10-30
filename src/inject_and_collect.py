#!/usr/bin/env python3
"""
inject_and_collect.py

Connect to an already-running CARLA server (launched separately, e.g. via carla_server.sh),
spawn an ego vehicle + sensors, inject a camera fault, optionally record RGB/depth (frames and/or MP4),
optionally log vehicle control/state each tick, drive with BasicAgent for a fixed duration, then clean up.
"""

import argparse
import os
import sys
import time
import random
import threading
import queue
import json
from pathlib import Path
from typing import Tuple, Optional

import numpy as np
from PIL import Image

try:
    import cv2
except Exception:
    cv2 = None
try:
    import imageio
except Exception:
    imageio = None

import carla  # assumes the CARLA .egg/.whl is importable

from carla_runner import EgoSpawner, SensorManager, FaultInjector
from faults import create_fault, list_available_faults


# -------------------------------------------------
# Argument parsing
# -------------------------------------------------
def parse_args():
    available_faults = list_available_faults()
    default_fault = "camera_blackout" if "camera_blackout" in available_faults else (
        available_faults[0] if available_faults else "camera_blackout"
    )

    p = argparse.ArgumentParser(
        description="Spawn ego + sensors, inject fault, optionally save frames/MP4 and vehicle logs."
    )
    p.add_argument("--carla-root", type=Path, required=True,
                   help="Path to CARLA install root (contains CarlaUE4.sh and PythonAPI/).")
    p.add_argument("--port", type=int, default=2000, help="RPC port for CARLA server (default: 2000).")
    p.add_argument("--time", type=float, default=60.0, help="How long (s) to run closed-loop before cleanup.")
    # Frame (image) saving
    p.add_argument("--save-rgb", action="store_true", help="Save clean & faulted RGB frames as PNG.")
    p.add_argument("--save-depth", action="store_true", help="Save depth frames as PNG.")
    # Video saving
    p.add_argument("--video-rgb", action="store_true", help="Save RGB videos (clean & faulty) as MP4.")
    p.add_argument("--video-depth", action="store_true", help="Save depth video as MP4.")
    p.add_argument("--video-fps", type=float, default=None,
                   help="FPS for videos. Default: CARLA fixed delta if set, else 20.")
    # Vehicle log
    p.add_argument("--log-vehicle", action="store_true",
                   help="Log vehicle control/state to out_dir/vehicle.jsonl (one row per tick).")
    p.add_argument("--fault-severity", type=float, default=0.8, help="Fault severity (0..1).")
    fault_help = "Fault to inject."
    if available_faults:
        fault_help += f" Available: {', '.join(sorted(available_faults))}."
    fault_help += f" Default: {default_fault}."
    p.add_argument("--fault", type=str, default=default_fault, help=fault_help)
    p.add_argument("--out-dir", type=Path, default=Path("out_debug"),
                   help="Directory where captures (frames/videos/logs) will be stored.")
    return p.parse_args()


# -------------------------------------------------
# Helpers
# -------------------------------------------------
def ensure_carla_agents_on_path(carla_root: Path):
    pythonapi_dir = carla_root / "PythonAPI" / "carla"
    pythonapi_dir_str = str(pythonapi_dir)

    if not pythonapi_dir.is_dir():
        raise RuntimeError(
            f"Couldn't find {pythonapi_dir}.\n"
            "Did you pass --carla-root pointing at the CARLA folder with CarlaUE4.sh and PythonAPI/?"
        )
    if pythonapi_dir_str not in sys.path:
        sys.path.append(pythonapi_dir_str)

    from agents.navigation.basic_agent import BasicAgent
    return BasicAgent


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


# ---------------------- Video helpers ----------------------
class LazyVideoWriter:
    """
    Lazily opens a video writer on first frame.
    Uses OpenCV if available, otherwise imageio. Expects RGB uint8 frames by default.
    """
    def __init__(self, path: Path, fps: float, expect_bgr: bool = False):
        self.path = str(path)
        self.fps = float(fps)
        self.expect_bgr = expect_bgr  # if True, frames passed to write() are BGR; else RGB
        self._backend = None  # 'cv2' or 'imageio'
        self._writer = None
        self._size = None  # (w, h)

    def _open(self, frame: np.ndarray):
        h, w = frame.shape[:2]
        self._size = (w, h)
        if cv2 is not None:
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self._writer = cv2.VideoWriter(self.path, fourcc, self.fps, (w, h))
            if not self._writer or not self._writer.isOpened():
                raise RuntimeError(f"OpenCV VideoWriter failed to open {self.path}")
            self._backend = "cv2"
        elif imageio is not None:
            self._writer = imageio.get_writer(self.path, fps=self.fps, codec="libx264", quality=7)
            self._backend = "imageio"
        else:
            raise RuntimeError("Video saving requested but neither opencv-python nor imageio is installed.")

    def write_rgb(self, frame_rgb: np.ndarray):
        if self._writer is None:
            self._open(frame_rgb)
        if (frame_rgb.shape[1], frame_rgb.shape[0]) != self._size:
            raise ValueError("Frame size changed during recording.")
        if self._backend == "cv2":
            self._writer.write(frame_rgb[:, :, ::-1])  # RGB->BGR
        else:
            self._writer.append_data(frame_rgb)

    def write_bgr(self, frame_bgr: np.ndarray):
        if self._writer is None:
            self._open(frame_bgr)
        if (frame_bgr.shape[1], frame_bgr.shape[0]) != self._size:
            raise ValueError("Frame size changed during recording.")
        if self._backend == "cv2":
            self._writer.write(frame_bgr)
        else:
            self._writer.append_data(frame_bgr[:, :, ::-1])

    def close(self):
        if self._writer is not None:
            if self._backend == "cv2":
                self._writer.release()
            else:
                self._writer.close()
            self._writer = None


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
                 log_vehicle: bool):
        self.q = queue.Queue()
        self.stop_flag = False

        # frames
        self.save_rgb = save_rgb
        self.save_depth = save_depth

        # videos
        self.video_rgb = video_rgb
        self.video_depth = video_depth
        self.video_fps = float(video_fps)

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
            self._vid_rgb_clean = LazyVideoWriter(self.video_dir / "rgb_clean.mp4", self.video_fps)
            self._vid_rgb_faulty = LazyVideoWriter(self.video_dir / "rgb_faulty.mp4", self.video_fps)
        if self.video_depth and self._vid_depth is None and depth_rgb is not None:
            self._vid_depth = LazyVideoWriter(self.video_dir / "depth.mp4", self.video_fps)

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

    # Early check: if video requested, ensure at least one backend exists
    if (args.video_rgb or args.video_depth) and (cv2 is None and imageio is None):
        raise RuntimeError(
            "Video saving requested but neither 'opencv-python' nor 'imageio[ffmpeg]' is installed.\n"
            "Install one of them, e.g.: pip install opencv-python  (or)  pip install imageio imageio-ffmpeg"
        )

    # Ensure BasicAgent is importable
    BasicAgent = ensure_carla_agents_on_path(args.carla_root)

    # Connect to existing CARLA server
    client, world = connect_existing(args.port)

    # Derive default video FPS if not provided
    if args.video_rgb or args.video_depth:
        fps = args.video_fps
        if fps is None:
            settings = world.get_settings()
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

    # Spawn sensors
    sensor_mgr = SensorManager(client, vehicle)
    config = sensor_mgr.load_config("../configs/carla/sensors_camera.yaml")
    sensor_mgr.spawn_sensors(config)

    rgb_cam = sensor_mgr.get_sensor("camera_rgb")
    depth_cam = sensor_mgr.get_sensor("camera_depth")

    # Prepare fault injector
    injector = FaultInjector()
    fault = create_fault(args.fault, severity=args.fault_severity)
    injector.register_fault(fault)

    # Async writer for frames + videos + logs
    writer = AsyncWriter(
        out_dir=args.out_dir,
        save_rgb=args.save_rgb,
        save_depth=args.save_depth,
        video_rgb=args.video_rgb,
        video_depth=args.video_depth,
        video_fps=video_fps,
        log_vehicle=args.log_vehicle,
    )

    listener_active = True

    # sensor callbacks
    def rgb_callback(img: carla.Image):
        nonlocal listener_active
        if not listener_active:
            return
        frame_id = img.frame
        rgb_clean = carla_rgb_to_numpy(img).copy()
        rgb_faulty = fault.apply_to_image(rgb_clean.copy())
        writer.enqueue_rgb_pair(frame_id, rgb_clean, rgb_faulty)
        if frame_id % 30 == 0:
            print(f"[rgb frame {frame_id}] mean={float(rgb_clean.mean()):.2f}")

    def depth_callback(img: carla.Image):
        nonlocal listener_active
        if not listener_active:
            return
        frame_id = img.frame
        writer.enqueue_depth_raw(frame_id=frame_id, h=img.height, w=img.width, raw_bytes=bytes(img.raw_data))

    # Register callbacks BEFORE injecting
    rgb_cam.listen(rgb_callback)
    depth_cam.listen(depth_callback)

    # Mark that the RGB cam is faulted via injector metadata
    injector.inject_fault(fault.name, rgb_cam)

    # Driving agent
    agent = BasicAgent(vehicle, target_speed=20)
    spawn_points = world.get_map().get_spawn_points()
    dest_tf = random.choice(spawn_points)
    try:
        agent.set_destination(dest_tf.location)
    except TypeError:
        agent.set_destination(vehicle.get_location(), dest_tf.location)

    current_goal = dest_tf.location  # keep for logging
    spectator = world.get_spectator()

    try:
        t0 = time.time()
        while time.time() - t0 < args.time:
            # Closed-loop step
            control = agent.run_step()
            vehicle.apply_control(control)

            # Step world and get snapshot for consistent frame/timestamp
            snapshot = world.wait_for_tick()
            ts = snapshot.timestamp

            # Update goal if agent reports done()
            if hasattr(agent, "done") and agent.done():
                new_tf = random.choice(spawn_points)
                try:
                    agent.set_destination(new_tf.location)
                except TypeError:
                    agent.set_destination(vehicle.get_location(), new_tf.location)
                current_goal = new_tf.location

            # Move spectator (optional)
            spectator.set_transform(rgb_cam.get_transform())

            # -------- Vehicle log (per tick) --------
            if args.log_vehicle:
                tf = vehicle.get_transform()
                vel = vehicle.get_velocity()
                ang = vehicle.get_angular_velocity()
                acc = vehicle.get_acceleration()
                speed_mps = float(np.sqrt(vel.x**2 + vel.y**2 + vel.z**2))
                record = {
                    "frame": int(snapshot.frame),
                    "timestamp": {
                        "delta_seconds": float(ts.delta_seconds),
                        "elapsed_seconds": float(getattr(ts, "elapsed_seconds", 0.0)),
                    },
                    "control": {
                        "throttle": float(control.throttle),
                        "steer": float(control.steer),
                        "brake": float(control.brake),
                        "hand_brake": bool(control.hand_brake),
                        "reverse": bool(control.reverse),
                        "manual_gear_shift": bool(control.manual_gear_shift),
                        "gear": int(control.gear),
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

    finally:
        listener_active = False
        rgb_cam.stop()
        depth_cam.stop()

        sensor_mgr.destroy_sensors()
        try:
            vehicle.destroy()
        except RuntimeError:
            pass

        writer.shutdown()
        print("[info] Episode finished, actors destroyed, data flushed.")


if __name__ == "__main__":
    main()

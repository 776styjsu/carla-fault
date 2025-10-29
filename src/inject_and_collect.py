#!/usr/bin/env python3
"""
inject_and_collect.py

Connect to an already-running CARLA server (launched separately, e.g. via carla_server.sh),
spawn an ego vehicle + sensors, inject a camera fault, optionally record RGB/depth,
drive with BasicAgent for a fixed duration, then clean up our actors.
"""

import argparse
import sys
import time
import random
import threading
import queue
from pathlib import Path
from typing import Tuple

import numpy as np
from PIL import Image

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
        description="Spawn ego + sensors, inject camera fault, optionally log RGB/depth, drive with BasicAgent."
    )
    p.add_argument(
        "--carla-root",
        type=Path,
        required=True,
        help="Path to CARLA install root (contains CarlaUE4.sh and PythonAPI/). "
             "Used to locate agents.navigation.basic_agent.",
    )
    p.add_argument(
        "--port",
        type=int,
        default=2000,
        help="RPC port for CARLA server (default: 2000).",
    )
    p.add_argument(
        "--time",
        type=float,
        default=60.0,
        help="How long (in seconds) to run closed-loop before cleanup.",
    )
    p.add_argument(
        "--save-rgb",
        action="store_true",
        help="If set, save clean & faulted RGB frames every tick.",
    )
    p.add_argument(
        "--save-depth",
        action="store_true",
        help="If set, save depth frames every tick.",
    )
    p.add_argument(
        "--fault-severity",
        type=float,
        default=0.8,
        help="Fault severity (0..1).",
    )
    fault_help = "Fault to inject."
    if available_faults:
        fault_help += f" Available: {', '.join(sorted(available_faults))}."
    fault_help += f" Default: {default_fault}."
    p.add_argument(
        "--fault",
        type=str,
        default=default_fault,
        help=fault_help,
    )
    p.add_argument(
        "--out-dir",
        type=Path,
        default=Path("out_debug"),
        help="Directory where captured images will be stored.",
    )
    return p.parse_args()


# -------------------------------------------------
# Helpers
# -------------------------------------------------
def ensure_carla_agents_on_path(carla_root: Path):
    """
    Ensure availability of agents.navigation.basic_agent.
    Basic Agent is used as a baseline driving agent for the ego vehicle for testing purposes.
    NOTE: injected faults do affect its behavior as it does not rely on sensor data for driving.
    """
    pythonapi_dir = carla_root / "PythonAPI" / "carla"
    pythonapi_dir_str = str(pythonapi_dir)

    if not pythonapi_dir.is_dir():
        raise RuntimeError(
            f"Couldn't find {pythonapi_dir}.\n"
            "Did you pass --carla-root pointing at the CARLA folder "
            "that has CarlaUE4.sh and PythonAPI/?"
        )

    if pythonapi_dir_str not in sys.path:
        sys.path.append(pythonapi_dir_str)

    from agents.navigation.basic_agent import BasicAgent
    return BasicAgent


# -------------------------------------------------
# Connect to an existing CARLA server
# -------------------------------------------------
def connect_existing(port: int, max_wait_s: float = 10.0, timeout_s: float = 2.0) -> Tuple[carla.Client, carla.World]:
    """
    Try repeatedly to connect to CARLA on localhost:<port> and get a world.
    If we can't within max_wait_s, raise.
    """
    start = time.time()
    last_err = None

    while time.time() - start < max_wait_s:
        try:
            client = carla.Client("localhost", port)
            client.set_timeout(timeout_s)
            world = client.get_world()
            _ = world.get_map()  # sanity check
            print(f"[info] Connected to CARLA on port {port}")
            return client, world
        except Exception as e:
            last_err = e
            time.sleep(0.5)

    raise RuntimeError(
        f"Failed to connect to CARLA on port {port} within {max_wait_s}s. "
        f"Last error: {last_err}"
    )


# -------------------------------------------------
# Image conversion helpers
# -------------------------------------------------
def carla_rgb_to_numpy(img: carla.Image) -> np.ndarray:
    """
    CARLA camera.rgb gives BGRA uint8.
    Return np.uint8 [H, W, 3] in RGB order.
    """
    bgra = np.frombuffer(img.raw_data, dtype=np.uint8)
    bgra = bgra.reshape((img.height, img.width, 4))  # [H,W,B,G,R,A]
    bgr = bgra[:, :, :3]                             # [H,W,B,G,R]
    rgb = bgr[:, :, ::-1]                            # -> [R,G,B]
    return rgb


def depth_bytes_to_numpy(raw_bytes: bytes, h: int, w: int) -> np.ndarray:
    """
    Reconstruct a viewable 3-channel image from CARLA's depth raw_data bytes
    (the same style CARLA would save via save_to_disk).
    """
    arr = np.frombuffer(raw_bytes, dtype=np.uint8)
    arr = arr.reshape((h, w, 4))   # BGRA-like
    bgr = arr[:, :, :3]
    rgb_like = bgr[:, :, ::-1]
    return rgb_like


# -------------------------------------------------
# AsyncWriter for logging images
# -------------------------------------------------
class AsyncWriter:
    """
    - Unbounded queue: we enqueue EVERY captured frame (no drops).
    - Single background thread drains the queue and writes PNGs.
    - On shutdown(), we block until queue is empty and thread exits.
    """

    def __init__(self, out_dir: Path, save_rgb: bool, save_depth: bool):
        self.q = queue.Queue()  # unbounded
        self.stop_flag = False

        self.save_rgb = save_rgb
        self.save_depth = save_depth

        self.clean_dir = out_dir / "clean_rgb"
        self.fault_dir = out_dir / "faulty_rgb"
        self.depth_dir = out_dir / "depth"

        if self.save_rgb:
            self.clean_dir.mkdir(parents=True, exist_ok=True)
            self.fault_dir.mkdir(parents=True, exist_ok=True)
        if self.save_depth:
            self.depth_dir.mkdir(parents=True, exist_ok=True)

        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def _worker(self):
        """
        Keep pulling items and writing them until stop_flag is True AND queue is empty.
        """
        while True:
            try:
                item = self.q.get(timeout=0.1)
            except queue.Empty:
                if self.stop_flag:
                    break
                continue

            kind = item["kind"]
            frame_id = item["frame_id"]

            if kind == "rgb_pair" and self.save_rgb:
                clean_arr = item["clean"]
                faulty_arr = item["faulty"]
                Image.fromarray(clean_arr).save(self.clean_dir / f"rgb_{frame_id:06d}.png")
                Image.fromarray(faulty_arr).save(self.fault_dir  / f"rgb_{frame_id:06d}.png")

            elif kind == "depth" and self.save_depth:
                depth_arr = depth_bytes_to_numpy(
                    item["raw_bytes"],
                    item["h"],
                    item["w"],
                )
                Image.fromarray(depth_arr).save(self.depth_dir / f"depth_{frame_id:06d}.png")

            self.q.task_done()

    def enqueue_rgb_pair(self, frame_id: int, clean_arr: np.ndarray, faulty_arr: np.ndarray):
        if not self.save_rgb:
            return
        self.q.put({
            "kind": "rgb_pair",
            "frame_id": frame_id,
            "clean": clean_arr,
            "faulty": faulty_arr,
        })

    def enqueue_depth_raw(self, frame_id: int, h: int, w: int, raw_bytes: bytes):
        if not self.save_depth:
            return
        self.q.put({
            "kind": "depth",
            "frame_id": frame_id,
            "h": h,
            "w": w,
            "raw_bytes": raw_bytes,
        })

    def shutdown(self):
        """
        Signal no more incoming work, then wait for queue drain.
        """
        self.stop_flag = True
        self.thread.join()


# -------------------------------------------------
# main logic
# -------------------------------------------------
def main():
    args = parse_args()

    # Ensure BasicAgent is importable
    BasicAgent = ensure_carla_agents_on_path(args.carla_root)

    # Connect to existing CARLA server
    client, world = connect_existing(args.port)

    # Spawn ego
    spawner = EgoSpawner(client)
    vehicle = spawner.spawn()  # EgoSpawner already tries spawn points
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

    # Async writer for optional logging
    writer = AsyncWriter(out_dir=args.out_dir, save_rgb=args.save_rgb, save_depth=args.save_depth)

    listener_active = True

    # sensor callbacks
    def rgb_callback(img: carla.Image):
        nonlocal listener_active
        if not listener_active or not args.save_rgb:
            return

        frame_id = img.frame
        rgb_clean = carla_rgb_to_numpy(img).copy()
        rgb_faulty = fault.apply_to_image(rgb_clean.copy())
        writer.enqueue_rgb_pair(frame_id, rgb_clean, rgb_faulty)

        if frame_id % 30 == 0:
            print(f"[rgb frame {frame_id}] mean={float(rgb_clean.mean()):.2f}")

    def depth_callback(img: carla.Image):
        nonlocal listener_active
        if not listener_active or not args.save_depth:
            return

        frame_id = img.frame
        # Copy raw bytes + dims now, so we don't depend on img staying alive.
        writer.enqueue_depth_raw(
            frame_id=frame_id,
            h=img.height,
            w=img.width,
            raw_bytes=bytes(img.raw_data),
        )

    # Register callbacks BEFORE injecting (ordering just for sanity/consistency)
    rgb_cam.listen(rgb_callback)
    depth_cam.listen(depth_callback)

    # Mark that the RGB cam is faulted via your injector metadata
    injector.inject_fault(fault.name, rgb_cam)

    # Driving agent
    agent = BasicAgent(vehicle, target_speed=20)

    spawn_points = world.get_map().get_spawn_points()
    dest_tf = random.choice(spawn_points)
    try:
        agent.set_destination(dest_tf.location)
    except TypeError:
        # Some CARLA versions use (start, end)
        agent.set_destination(vehicle.get_location(), dest_tf.location)

    spectator = world.get_spectator()

    try:
        t0 = time.time()
        while time.time() - t0 < args.time:
            # Closed-loop step
            control = agent.run_step()
            vehicle.apply_control(control)

            # Show camera POV in the UE window if you're running interactive
            spectator.set_transform(rgb_cam.get_transform())

            # Step world forward in sync-ish mode
            world.wait_for_tick()

            # If agent reached goal, pick a new one so it keeps driving
            if hasattr(agent, "done") and agent.done():
                new_tf = random.choice(spawn_points)
                try:
                    agent.set_destination(new_tf.location)
                except TypeError:
                    agent.set_destination(vehicle.get_location(), new_tf.location)

    finally:
        # stop any new callbacks immediately
        listener_active = False
        rgb_cam.stop()
        depth_cam.stop()

        # cleanup actors we spawned
        sensor_mgr.destroy_sensors()
        try:
            vehicle.destroy()
        except RuntimeError:
            pass

        # flush and stop writer thread
        writer.shutdown()

        print("[info] Episode finished, actors destroyed, data flushed.")


if __name__ == "__main__":
    main()

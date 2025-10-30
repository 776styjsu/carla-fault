# CARLA Fault Injection Project & Toolkit

This project evaluates how injected sensor faults (e.g., camera outages) affect state-of-the-art autonomous driving policies such as SimLingo, InterFuser, and TCP. The repository provides a toolkit designed expressly for that purpose.  

This toolkit supports reproducible fault-injection experiments inside CARLA. The main runner scripts connect to a running simulator, spawn an ego vehicle with configured sensors, inject camera-focused faults, and optionally record RGB/depth frames, MP4 videos, and vehicle telemetry for downstream analysis.

## Features
- Connects to an existing CARLA server and keeps it alive across runs.
- Spawns an ego vehicle plus camera sensors from YAML configs.
- Dynamically selects fault implementations via `--fault <name>` (no code edits required).
- Supports frame dumps (PNG), MP4 capture, and vehicle state logging in parallel using an async writer.
- Includes helpers for launching/shutting down the CARLA server (`src/carla_server.sh`).

## Project Structure
- `src/inject_and_collect.py` – main entry point for closed-loop runs, recording, and fault injection.
- `src/carla_runner/` – wrappers for spawning vehicles, managing sensors, launching CARLA, and orchestrating faults.
- `src/faults/` – fault base class plus concrete fault implementations. New faults dropped here are auto-discovered.
- `configs/` – YAML sensor layouts (e.g., RGB and depth cameras mounted on the ego vehicle).
- `env/environment.yml` – conda environment spec with CARLA Python API and required Python packages.

## Prerequisites
- CARLA simulator installation (e.g., `/opt/carla-0.9.15`).
- Python 3.8 (the provided environment.yml pins this version).
- Conda for environment management.
- GPU drivers/X11 forwarding as required by CARLA (or run in headless mode).

## Environment Setup
```bash
conda env create -f env/environment.yml
conda activate carla-fault-injection
export CARLA_ROOT=/path/to/carla-0.9.15  # used by carla_server.sh and CLI
```

If you plan to save MP4 videos, ensure either `opencv-python` or `imageio[ffmpeg]` is installed in the environment (already included in `environment.yml`).

## Running the Simulator
Use the helper script to start/stop CARLA. It records the server PID in `src/.carla_server.pid` for easy shutdown.

```bash
cd src
./carla_server.sh start 2000 interactive  # or headless
# ... later ...
./carla_server.sh stop
```

Alternatively, launch CARLA manually and ensure it listens on the port you pass to the runner script.

## Running an Experiment
The recommended entry point is `inject_and_collect.py` (superset of the older `main.py`). Example:

```bash
cd src
python inject_and_collect.py \
	--carla-root "$CARLA_ROOT" \
	--port 2000 \
	--time 120 \
	--fault camera_blackout \
	--fault-severity 0.8 \
	--save-rgb \
	--video-rgb \
	--log-vehicle \
	--out-dir ../out_debug/session_001
```

Key CLI options:
- `--fault <name>` – choose which fault to inject. Names are case-insensitive and accept variations like `camera_blackout` or the fully qualified class path (`faults.camera_blackout.CameraBlackoutFault`). Use `--help` to see the auto-generated list of available faults.
- `--fault-severity` – float in `[0,1]` passed to the selected fault implementation.
- `--save-rgb/--save-depth` – store per-frame PNG dumps (clean vs. faulty RGB, and depth buffers).
- `--video-rgb/--video-depth` – emit MP4s using OpenCV or imageio (falls back automatically).
- `--log-vehicle` – record one JSON line per tick with control commands, transforms, and velocity.
- `--time` – run duration in seconds before clean shutdown.

Output directory structure (created on demand):
- `clean_rgb/` and `faulty_rgb/` – frame-by-frame PNGs.
- `depth/` – PNG depth visualizations.
- `video/` – MP4s (`rgb_clean.mp4`, `rgb_faulty.mp4`, `depth.mp4`).
- `vehicle.jsonl` – vehicle telemetry log (when `--log-vehicle` is set).

The async writer backend guarantees every frame/log is enqueued; shutdown waits for the queue to drain before exiting.

## Implementing New Faults
1. Create a new file in `src/faults/` (e.g., `thermal_noise.py`).
2. Subclass `BaseFault`, implement `inject`, `remove`, and any data-path transforms (`apply_to_image`, etc.).
3. Give the class a canonical label with `fault_name = "thermal_noise"` (optional but keeps names tidy).
4. Export any custom constructor parameters you need; they can be forwarded via CLI by extending `inject_and_collect.py`.

The package automatically discovers subclasses on import, so `--fault thermal_noise` will begin working immediately.

## Potential Extensions
- Design targeted evaluations that reproduce real-world sensor misalignments (e.g., bumped or tilted cameras from uneven tire pressure) and quantify the performance gap between CARLA's simulated data and data with considerations to realistic physical fault.
- Fine-tune the autonomy stacks on degraded data and report recovery in closed-loop metrics after fault injection.

## Tips for Experiments
- Combine `--video-*` flags with `--log-vehicle` to align control inputs with the recorded imagery.
- For batch runs, script multiple invocations of `inject_and_collect.py` with different `--fault` values or severity levels.

## Troubleshooting
- `ImportError: agents.navigation.basic_agent` – verify `--carla-root` points to the CARLA install containing `PythonAPI/carla/agents/navigation/basic_agent.py`.
- If CARLA does not shut down cleanly, run `./carla_server.sh stop` again or manually `pkill -f CarlaUE4`.

## License
This repository is distributed for educational and research purposes. Refer to CARLA's own license for simulator usage terms.
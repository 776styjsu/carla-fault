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
- `src/inject_and_collect_pcla.py` – **Main entry point** for running PCLA agents (SimLingo, TransFuser, etc.) with fault injection.
- `src/inject_and_collect.py` – Legacy entry point for standard closed-loop runs (manual or simple agents).
- `src/carla_runner/` – wrappers for spawning vehicles, managing sensors, launching CARLA, and orchestrating faults.
- `src/faults/` – fault base class plus concrete fault implementations. New faults dropped here are auto-discovered.
- `configs/` – YAML sensor layouts (e.g., RGB and depth cameras mounted on the ego vehicle).
- `env/environment.yml` – conda environment spec with CARLA Python API, PCLA dependencies, and required Python packages.
- `external/PCLA/` – submodule containing the PCLA framework and agent implementations.

## Prerequisites
- CARLA simulator installation (e.g., `/opt/carla-0.9.15`).
- Python 3.8 (the provided environment.yml pins this version).
- Conda for environment management.
- GPU drivers/X11 forwarding as required by CARLA (or run in headless mode).

## Environment Setup
```bash
conda env create -f env/environment.yml
conda activate carla-fault-injection
export CARLA_ROOT=/path/to/carla-0.9.15  # optional, used by carla_server.sh and CLI
git submodule update --init --recursive  # initialize PCLA
```

**Note on PCLA Dependencies:**
The `environment.yml` includes dependencies for PCLA agents. However, `torch-scatter` is pinned to a specific CUDA version. If you encounter issues, you may need to reinstall it matching your CUDA version:
```bash
pip uninstall torch-scatter
pip install torch-scatter -f https://data.pyg.org/whl/torch-2.2.0+cu121.html  # Adjust for your torch/cuda version
```

**PCLA Model Weights:**
To use PCLA agents, you must download their pre-trained weights:
```bash
cd external/PCLA
python pcla_functions/download_weights.py
# Or manually download from Hugging Face as described in external/PCLA/README.md
cd ../..
```

**LMDrive Agent Setup:**
If you plan to use the `LMDrive` agent, additional setup steps are required (uninstalling `timm`, setting up `vision_encoder` and `LAVIS`). Please refer to `external/PCLA/README.md` for details.

## Running the Simulator
Use the helper script to start/stop CARLA. It records the server PID in `src/.carla_server.pid` for easy shutdown.

```bash
cd src
./carla_server.sh start 2000 interactive  # or headless
# ... later ...
./carla_server.sh stop
```

**Note:** Starting the CARLA server via the script can sometimes be inconsistent. It is **recommended to launch CARLA manually** in a separate terminal and ensure it listens on the port you pass to the runner script (default 2000).

```bash
# In a separate terminal
cd $CARLA_ROOT
./CarlaUE4.sh -carla-rpc-port=2000
```

## Running an Experiment
The primary entry point is `inject_and_collect_pcla.py`. This script wraps the PCLA framework to evaluate state-of-the-art agents (like TransFuser, SimLingo, etc.) with fault injection.

```bash
cd src
python inject_and_collect_pcla.py \
	--carla-root "$CARLA_ROOT" \
	--port 2000 \
	--time 60 \
	--agent tf_tf \
	--fault camera_blackout \
	--fault-severity 1.0 \
	--save-rgb \
	--video-rgb \
	--log-vehicle \
	--out-dir ../out_debug_pcla/session_001
```

Key CLI options:
- `--agent <name>` – The PCLA agent ID. Currently tested and working agents: `tf_tf` (TransFuser), `if_if` (InterFuser), and `simlingo_simlingo` (SimLingo). See `external/PCLA/README.md` for the full list.
- `--fault <name>` – choose which fault to inject (e.g., `camera_blackout`).
- `--fault-severity` – float in `[0,1]`.
- `--save-rgb/--save-depth` – store per-frame PNG dumps.
- `--video-rgb/--video-depth` – emit MP4s.
- `--log-vehicle` – record vehicle telemetry.
- `--sync` – Forces synchronous mode (default: True), recommended for PCLA.
- `--route-file <path>` – Path to save/load the generated route XML (default: `temp_route.xml`).

Output directory structure:
- `clean_rgb/` and `faulty_rgb/` – frame-by-frame PNGs.
- `depth/` – PNG depth visualizations.
- `video/` – MP4s (`rgb_clean.mp4`, `rgb_faulty.mp4`, `depth.mp4`).
- `vehicle.jsonl` – vehicle telemetry log.

The async writer backend guarantees every frame/log is enqueued; shutdown waits for the queue to drain before exiting.

## Running with Basic Agents (Legacy)
For simple agents (BasicAgent) or manual control without the PCLA framework, use `inject_and_collect.py`.

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
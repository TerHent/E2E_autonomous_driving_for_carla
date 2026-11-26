# E2E Autonomous Driving for CARLA

Group project for E2E autonomous driving in CARLA (`0.9.16`), combining long-term route planning, an LLM-based tactical layer, and short-term control.

## Repo Layout

- `src/schemas.py` — Shared data contracts (`SceneState`, `RouteWindow`, `EgoState`, `ActorState`, `ScenarioCase`).
- `src/scenarios.py` — Offline tactical test scenarios (`data/scenarios/*.json`).
- `scripts/run_carla_scenarios.py` — Runs scenarios on CARLA `0.9.16` in 20 Hz synchronous mode and logs `SceneState` `.jsonl` (+ optional `.mp4` video).
- `tests/` — Unit tests (run without CARLA).

## Offline Development (Mac / Laptop, No CARLA Needed)

```bash
python3 -m unittest discover -s tests -v
python3 -m src.scenarios --export data/scenarios
```

## Running on the CARLA Linux Server (`sim5`)

### 1. One-time virtual environment setup

```bash
python3 -m venv .venv
.venv/bin/pip install carla==0.9.16 imageio imageio-ffmpeg numpy
```

*(If `pip` cannot find a `carla==0.9.16` wheel for your Python version, install the local wheel from `~/Carla/CARLA_0.9.16/PythonAPI/carla/dist/carla-0.9.16-*.whl`.)*

### 2. Check if CARLA is already running

Before starting CARLA on the shared server, check whether port `2000` is already in use:

```bash
ss -tlnp | grep 2000 || ps aux | grep CarlaUE4
```

### 3. Start CARLA & run scenarios (2 terminals or `tmux`)

**Terminal 1** (only if CARLA is not already running — wait ~5–10s for it to listen on port `2000`):

```bash
~/Carla/CARLA_0.9.16/CarlaUE4.sh -RenderOffScreen
```

**Terminal 2** (always use `.venv/bin/python`, not system `python3`):

```bash
git pull
.venv/bin/python scripts/run_carla_scenarios.py --scenario all --record
```

Outputs are written to `recordings/<scenario>.jsonl` and `recordings/<scenario>.mp4` (gitignored).

> **Shared server etiquette:** Press `Ctrl+C` in Terminal 1 to stop `CarlaUE4.sh` when you are finished so it does not hog the GPU.

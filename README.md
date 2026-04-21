# Open-RMF Warehouse Thesis Repro Pack

This repository is a **reproducibility pack** for the warehouse map experiments used in the thesis (Chapter~6). It ships **only** the thesis-specific artefacts; it does **not** vendor the Open-RMF stack.

The following **four** navigation-map worlds, their Gazebo/Rviz launch wiring, fleet YAML, and compose-task JSONs are **not** part of the stock `open-rmf/rmf_demos` release at the pinned tags—they were added for this work:

| Map folder (`rmf_demos_maps/maps/…`) | Role in the thesis |
|--------------------------------------|---------------------|
| `warehouse` | Original-scale **baseline** (bidirectional corridors) |
| `warehouse_perf` | Original-scale **directed-loop** layout |
| `warehouse_extension` | Expanded baseline (higher load, more robots) |
| `warehouse_perf_extension` | Expanded directed-loop layout |

**Contents of this repo**

- `overlays/rmf_demos/` — drop-in files for `rmf_demos`, `rmf_demos_maps`, and `rmf_demos_gz` (maps, launch XML, `config/warehouse/tinyRobot_config.yaml`, task JSONs under `launch/include/warehouse/tasks/`)
- `scripts/warehouse_benchmark.py` — automated Gazebo launch + compose-task dispatch + JSON metrics (supports `--campaign four-warehouse-layouts`)
- `scripts/rmf_traffic_log_stats.py` — optional offline keyword stats on fleet-adapter logs
- `tools/apply_overlay.py` — copies the overlay into your local `rmf_ws/src/.../rmf_demos` tree

**License:** Apache License 2.0 (see `LICENSE`).

---

## Upstream versions (as referenced in the thesis)

- `open-rmf/rmf`: tag `release-jazzy-240617` (commit `6aeae8db7256ef10fae785adae2e430f90f3f27e`)
- `open-rmf/rmf_demos`: tag `2.0.3` (commit `9c6eb30295c5716abbb0d30f0d8ddace348ee244`)

Follow the official Open-RMF installation guide for **ROS 2 Jazzy** (Gazebo Harmonic / `gz sim`, dependencies, `rosdep`, etc.) before relying on the steps below. This README assumes a working RMF workspace can already build upstream packages.

---

## 1) Create a workspace and clone upstream

```bash
mkdir -p ~/rmf_ws/src
cd ~/rmf_ws/src

git clone https://github.com/open-rmf/rmf_demos.git -b 2.0.3
git clone https://github.com/open-rmf/rmf.git -b release-jazzy-240617
```

If your checkout lives under `~/rmf_ws/src/demonstrations/rmf_demos` (common in metarepos), that layout is supported as well.

---

## 2) Clone this repro pack (anywhere)

```bash
git clone https://github.com/<your-fork>/openrmf-warehouse-repro.git
cd openrmf-warehouse-repro
```

---

## 3) Apply the overlay

This copies thesis maps and launch files **into** your `rmf_demos` source tree (next to upstream packages).

```bash
python3 tools/apply_overlay.py --rmf-ws ~/rmf_ws
```

The script looks for **`~/rmf_ws/src/rmf_demos`** or **`~/rmf_ws/src/demonstrations/rmf_demos`** and merges `overlays/rmf_demos/` on top of it.

---

## 4) Build the workspace

```bash
cd ~/rmf_ws
rosdep install --from-paths src --ignore-src -r -y   # if not already satisfied
colcon build
source install/setup.bash
```

`rmf_demos_maps` runs `building_map_generator` at build time and installs generated assets (including `maps/<world>/nav_graphs/0.yaml`). **Build before** running the benchmark or the fleet adapter will not find the nav graph.

---

## 5) Quick manual check (optional)

Terminal A (simulation + RMF core for one world):

```bash
source ~/rmf_ws/install/setup.bash
ros2 launch rmf_demos_gz warehouse.launch.xml
```

Use `warehouse_perf.launch.xml`, `warehouse_extension.launch.xml`, or `warehouse_perf_extension.launch.xml` to sanity-check the other three worlds.

Patrol-style launch files (`*_patrol.launch.xml`) match the staggered dispatch used in the benchmark’s extension maps.

---

## 6) Reproduce the thesis benchmark (recommended)

From a sourced workspace, run the **four-layout campaign** (same world names as `ros2 launch rmf_demos_gz <world>.launch.xml`):

```bash
source ~/rmf_ws/install/setup.bash
python3 /path/to/openrmf-warehouse-repro/scripts/warehouse_benchmark.py \
  --campaign four-warehouse-layouts \
  --results-dir ~/benchmark_results/live_campaign \
  --print-quartiles
```

Defaults for this campaign: **`--repeats 10`**, **`--tasks-per-robot 1`** (single compose round per robot, consistent with the thesis tables unless you override).

Aggregated output:

- `<results-dir>/warehouse_benchmark_results.json` — per-run rows plus `worlds[].summary` (mean, std, median, quartiles).

### Run a single world

```bash
python3 /path/to/openrmf-warehouse-repro/scripts/warehouse_benchmark.py \
  --world warehouse_perf \
  --repeats 10 \
  --tasks-per-robot 1 \
  --results-dir ~/benchmark_results/single_world
```

Valid `--world` values: `warehouse`, `warehouse_perf`, `warehouse_extension`, `warehouse_perf_extension`.

---

## 7) Optional: offline log keyword counting

```bash
python3 /path/to/openrmf-warehouse-repro/scripts/rmf_traffic_log_stats.py /tmp/fleet_adapter.log
```

---

## Troubleshooting

- **Overlay not found:** ensure `rmf_demos` exists under `~/rmf_ws/src` or `~/rmf_ws/src/demonstrations/`.
- **Missing nav graph / fleet adapter errors:** run `colcon build` after applying the overlay so `rmf_demos_maps` generates and installs `maps/<world>/nav_graphs/0.yaml`.
- **Gazebo or ROS packages missing:** complete the upstream Open-RMF / ROS 2 Jazzy dependency install first; this repo only adds map and launch content.

---

## 中文摘要

本仓库为论文仓库场景实验的 **overlay 复现包**：在按官方文档装好 ROS 2 Jazzy 与 Open-RMF 工作空间后，将本仓库 `overlays` 覆盖到本地 `rmf_demos` 源码树，`colcon build` 生成导航图，再运行 `warehouse_benchmark.py` 的 `--campaign four-warehouse-layouts` 即可批量复现四种布局的基准测试与 JSON 汇总。

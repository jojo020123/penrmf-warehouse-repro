# Open-RMF Warehouse Repro Pack

This repository is a **reproducibility pack** for the Open-RMF simulation experiments
(`warehouse` / `warehouse_perf`) reported in the thesis. It does **not** bundle upstream
Open-RMF source code; instead, it provides:

- **Overlay artefacts**: maps (`*.building.yaml` + reference PNG), launch files, fleet config, task JSONs
- **Benchmark scripts**: task injection + metric aggregation (JSON outputs)
- **One-step overlay installer**: copies the overlay into your local `rmf_ws/src/` checkout

License: Apache License 2.0 (see `LICENSE`).

## Upstream versions (as referenced in the thesis)

- `open-rmf/rmf`: tag `release-jazzy-240617` (commit `6aeae8db7256ef10fae785adae2e430f90f3f27e`)
- `open-rmf/rmf_demos`: tag `2.0.3` (commit `9c6eb30295c5716abbb0d30f0d8ddace348ee244`)

## Quick reproduction (recommended workflow)

### 1) Create a workspace and clone upstream repositories

```bash
mkdir -p ~/rmf_ws/src
cd ~/rmf_ws/src

# Upstream demos (worlds / launch files)
git clone https://github.com/open-rmf/rmf_demos.git -b 2.0.3

# Upstream rmf (ROS 2 Jazzy release)
git clone https://github.com/open-rmf/rmf.git -b release-jazzy-240617
```

> If you already have your own `rmf_ws`, you may skip this step, but make sure the versions
> match those used in the thesis.

### 2) Apply the overlay (copy this pack into `rmf_demos`)

```bash
cd /path/to/openrmf-warehouse-repro
python3 tools/apply_overlay.py --rmf-ws ~/rmf_ws
```

### 3) Build

```bash
cd ~/rmf_ws
colcon build
source install/setup.bash
```

### 4) Run the benchmark (generate JSONs for the tables)

```bash
python3 /path/to/openrmf-warehouse-repro/scripts/warehouse_benchmark.py \
  --world warehouse \
  --repeats 20 \
  --tasks-per-robot 34 \
  --results-dir benchmark_results/warehouse

python3 /path/to/openrmf-warehouse-repro/scripts/warehouse_benchmark.py \
  --world warehouse_perf \
  --repeats 20 \
  --tasks-per-robot 34 \
  --results-dir benchmark_results/warehouse_perf
```

The aggregated output file is:

- `<results-dir>/warehouse_benchmark_results.json`

### 5) (Optional) Offline log keyword counting

```bash
python3 /path/to/openrmf-warehouse-repro/scripts/rmf_traffic_log_stats.py /tmp/fleet_adapter.log
```

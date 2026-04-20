# Open-RMF Warehouse Repro Pack

本仓库用于**复现论文中 warehouse / warehouse_perf 的 Open-RMF 仿真实验**。它不包含 Open-RMF 上游源码本体，而是提供：

- **overlay 工件**：地图(`*.building.yaml` + 底图 PNG)、launch、fleet config、task JSON
- **benchmark 脚本**：任务注入 + 指标统计（输出 JSON）
- **一键安装脚本**：把 overlay 覆盖到你 clone 下来的 `rmf_ws/src/` 中

许可协议：Apache License 2.0（见 `LICENSE`）。

## 上游版本（论文中引用）

- `open-rmf/rmf`: tag `release-jazzy-240617` (commit `6aeae8db7256ef10fae785adae2e430f90f3f27e`)
- `open-rmf/rmf_demos`: tag `2.0.3` (commit `9c6eb30295c5716abbb0d30f0d8ddace348ee244`)

## 快速复现（推荐工作流）

### 1) 创建工作区并拉取上游

```bash
mkdir -p ~/rmf_ws/src
cd ~/rmf_ws/src

# 上游 demos（世界/launch）
git clone https://github.com/open-rmf/rmf_demos.git -b 2.0.3

# 上游 rmf（Jazzy release）
git clone https://github.com/open-rmf/rmf.git -b release-jazzy-240617
```

> 如果你已经有自己的 `rmf_ws`，也可以跳过此步骤，但请确保版本与论文一致。

### 2) 应用 overlay（把本仓库内容覆盖到 rmf_demos）

```bash
cd /path/to/openrmf-warehouse-repro
python3 tools/apply_overlay.py --rmf-ws ~/rmf_ws
```

### 3) 编译

```bash
cd ~/rmf_ws
colcon build
source install/setup.bash
```

### 4) 跑 benchmark（生成表格所需 JSON）

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

输出汇总文件为：

- `<results-dir>/warehouse_benchmark_results.json`

### 5)（可选）离线统计终端日志计数

```bash
python3 /path/to/openrmf-warehouse-repro/scripts/rmf_traffic_log_stats.py /tmp/fleet_adapter.log
```

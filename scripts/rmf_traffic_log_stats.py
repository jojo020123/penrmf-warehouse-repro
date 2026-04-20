#!/usr/bin/env python3
"""从 fleet_adapter 终端日志统计 RMF 交通压力指标（离线）。

用法:
  # 从保存的日志文件
  python3 rmf_traffic_log_stats.py /tmp/fleet_adapter.log

  # 从管道
  ros2 launch ... 2>&1 | tee /tmp/run.log
  python3 rmf_traffic_log_stats.py /tmp/run.log

指标说明（数值越小通常越好，需与任务完成率一起看）:
  waiting_for_traffic   : 进入「等待路权」的次数
  excessively_delayed   : 「交通依赖过久」触发重规划的次数
  replanning_requested  : 显式重规划请求次数
  collision_avoid_stop  : Gazebo slotcar 因避碰停车的次数（若日志含该行）

同一任务多次重试会让上述计数升高；优化地图后对比同一测试脚本下的计数即可。
"""

from __future__ import annotations

import re
import sys
from collections import Counter


PATTERNS = [
    ("waiting_for_traffic", re.compile(r"waiting for traffic", re.I)),
    ("excessively_delayed", re.compile(r"traffic dependency is excessively delayed", re.I)),
    ("replanning_requested", re.compile(r"Replanning requested for", re.I)),
    ("collision_avoid_stop", re.compile(r"Stopping \[.*\] to avoid a collision", re.I)),
]


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    path = sys.argv[1]
    try:
        text = open(path, encoding="utf-8", errors="replace").read()
    except OSError as e:
        print(f"无法读取文件: {e}", file=sys.stderr)
        return 1

    counts = Counter()
    for name, rx in PATTERNS:
        counts[name] = len(rx.findall(text))

    print("=== RMF 交通相关日志统计 ===")
    for k in ("waiting_for_traffic", "excessively_delayed", "replanning_requested", "collision_avoid_stop"):
        print(f"  {k:22s}  {counts[k]}")
    print("来源:", path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

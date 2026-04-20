#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
from pathlib import Path


def copy_tree(src: Path, dst: Path) -> None:
    if not src.exists():
        raise FileNotFoundError(src)
    for p in src.rglob("*"):
        rel = p.relative_to(src)
        target = dst / rel
        if p.is_dir():
            target.mkdir(parents=True, exist_ok=True)
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(p, target)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Apply overlay artefacts into an rmf_ws checkout."
    )
    parser.add_argument(
        "--rmf-ws",
        type=Path,
        required=True,
        help="Path to rmf_ws (workspace root).",
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    overlays = repo_root / "overlays"
    rmf_ws = args.rmf_ws.expanduser().resolve()
    src_dir = rmf_ws / "src"

    if not src_dir.exists():
        raise SystemExit(f"rmf_ws/src not found: {src_dir}")

    # This pack targets rmf_demos overlay under rmf_ws/src/rmf_demos
    # Users may have different layouts (e.g., demonstrations/rmf_demos). Handle both.
    candidates = [
        src_dir / "rmf_demos",
        src_dir / "demonstrations" / "rmf_demos",
    ]
    rmf_demos_root = next((c for c in candidates if c.exists()), None)
    if rmf_demos_root is None:
        raise SystemExit(
            "Cannot find rmf_demos under rmf_ws/src. Expected one of:\n"
            f"  - {candidates[0]}\n"
            f"  - {candidates[1]}\n"
        )

    # Copy overlay into rmf_demos tree
    overlay_rmf_demos = overlays / "rmf_demos"
    copy_tree(overlay_rmf_demos, rmf_demos_root)
    print(f"Applied overlay to: {rmf_demos_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


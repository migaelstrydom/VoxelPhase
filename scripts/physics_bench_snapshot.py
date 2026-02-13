#!/usr/bin/env python3
"""Create a named snapshot of physics benchmark outputs."""

from __future__ import annotations

import argparse
import json
import shutil
from datetime import datetime, timezone
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Snapshot target/physics_bench outputs to a named folder"
    )
    parser.add_argument(
        "--source-dir",
        default="target/physics_bench",
        help="Directory containing current benchmark outputs",
    )
    parser.add_argument(
        "--snapshots-dir",
        default="target/physics_bench_snapshots",
        help="Directory where snapshots are stored",
    )
    parser.add_argument(
        "--name",
        default=None,
        help="Snapshot name (default: UTC timestamp)",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing snapshot with the same name",
    )
    return parser.parse_args()


def default_snapshot_name() -> str:
    now = datetime.now(timezone.utc)
    return now.strftime("%Y%m%dT%H%M%SZ")


def main() -> int:
    args = parse_args()
    source_dir = Path(args.source_dir)
    snapshots_dir = Path(args.snapshots_dir)
    snapshot_name = args.name or default_snapshot_name()
    snapshot_dir = snapshots_dir / snapshot_name

    if not source_dir.exists():
        print(f"Source directory does not exist: {source_dir}")
        return 1
    if not source_dir.is_dir():
        print(f"Source path is not a directory: {source_dir}")
        return 1

    files = sorted(p for p in source_dir.iterdir() if p.is_file())
    if not files:
        print(f"Source directory is empty: {source_dir}")
        return 1

    snapshots_dir.mkdir(parents=True, exist_ok=True)
    if snapshot_dir.exists():
        if not args.overwrite:
            print(
                f"Snapshot already exists: {snapshot_dir}\n"
                "Use --overwrite to replace it."
            )
            return 1
        shutil.rmtree(snapshot_dir)

    shutil.copytree(source_dir, snapshot_dir)

    metadata = {
        "snapshot_name": snapshot_name,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "source_dir": str(source_dir),
        "file_count": len(files),
        "files": [p.name for p in files],
    }
    metadata_path = snapshot_dir / "snapshot_meta.json"
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")

    print(f"Created snapshot: {snapshot_dir}")
    print(f"Copied {len(files)} files from {source_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

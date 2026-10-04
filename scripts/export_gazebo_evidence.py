#!/usr/bin/env python3
"""Export a Gazebo trial batch as an offline, checksum-verified audit package."""

import argparse
from pathlib import Path

from gazebo_evidence_package import export_batch


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch", type=Path, required=True, help="batch directory containing summary.json")
    parser.add_argument("--output", type=Path, required=True, help="output tar package path")
    parser.add_argument("--acceptance", type=Path,
                        default=root / "src/robot_panda_gz_sim/config/acceptance.json")
    args = parser.parse_args()
    result = export_batch(args.batch, args.output, args.acceptance)
    print(f"archive={result['archive']}")
    print(f"archive_bytes={result['archive_bytes']}")
    print(f"archive_sha256={result['archive_sha256']}")
    print(f"run_count={result['manifest']['run_count']}")
    print(f"data_rows={result['manifest']['data_row_count']}")
    print(f"roundtrip_matches={result['manifest']['export_recompute_matches_raw']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

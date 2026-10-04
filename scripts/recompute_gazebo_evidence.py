#!/usr/bin/env python3
"""Recompute Gazebo physical audits from a packaged JSONL gzip archive offline."""

import argparse
import json
from pathlib import Path

from gazebo_evidence_package import recompute_archive


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    args = parser.parse_args()
    result = recompute_archive(args.archive)
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if result["all_recomputations_match"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

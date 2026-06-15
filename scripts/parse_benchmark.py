#!/usr/bin/env python3
"""Average the per-frame metrics in benchmark.log (written by the --autobench harness).

Usage: python scripts/parse_benchmark.py [path]   (default: benchmark.log)

Each line is comma-separated key=value pairs, with two bracketed sub-sections:
  [sim breakdown] ...   -> keys prefixed sb_
  [steering] ...        -> keys prefixed st_
(so the duplicate "signals" key in both sub-sections doesn't collide).
Prints the mean of every numeric field across all captured frames.
"""
import re
import sys

path = sys.argv[1] if len(sys.argv) > 1 else "benchmark.log"
lines = [ln.strip() for ln in open(path, encoding="utf-8") if ln.strip()]

sums: dict[str, float] = {}
counts: dict[str, int] = {}
order: list[str] = []

for line in lines:
    parts = re.split(r"\[sim breakdown\]|\[steering\]", line)
    markers = re.findall(r"\[sim breakdown\]|\[steering\]", line)
    prefixes = [""] + ["sb_" if "sim" in m else "st_" for m in markers]
    for prefix, part in zip(prefixes, parts):
        for k, v in re.findall(r"(\w+)=(-?[\d.]+)", part):
            if k == "ts":
                continue
            key = prefix + k
            if key not in sums:
                sums[key] = 0.0
                counts[key] = 0
                order.append(key)
            sums[key] += float(v)
            counts[key] += 1

print(f"frames: {len(lines)}")
for k in order:
    print(f"  {k:16s} = {sums[k] / counts[k]:8.2f}")

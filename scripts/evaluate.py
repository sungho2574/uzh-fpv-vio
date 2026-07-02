#!/usr/bin/env python3
"""
Evaluate VINS-Mono / VINS-Mono+LC results against UZH-FPV ground truth.

Runs on a Python >=3.10 host (NOT inside the ROS Kinetic container, which is
stuck on Python 2.7). Requires: evo, numpy, pandas, matplotlib.

    python3 -m venv .venv && source .venv/bin/activate
    pip install evo numpy pandas matplotlib

Usage:
    python3 scripts/evaluate.py \
        --no-loop-csv output/no_loop/vins_result_no_loop.csv \
        --loop-csv output/loop/vins_result_loop.csv \
        --gt data/indoor_forward_9_davis_with_gt/groundtruth.txt \
        --out results

What it does:
  1. Converts each vins_result_*.csv (VINS-Mono's native format:
     `t_ns,tx,ty,tz,qw,qx,qy,qz,vx,vy,vz`, no header, scalar-first quaternion)
     into TUM format (`t tx ty tz qx qy qz qw`, space-delimited) alongside the
     ground truth, which is already TUM format.
  2. Runs `evo_traj` to produce a combined, GT-aligned 3D trajectory plot.
  3. Runs `evo_ape` and `evo_rpe` (SE(3)/Umeyama-aligned, translation part)
     for each variant against ground truth, saving per-run result zips/plots.
  4. Collects APE/RPE RMSE/mean/median/std into results/summary.csv.
"""
import argparse
import json
import subprocess
import sys
import zipfile
from pathlib import Path

import pandas as pd

VINS_CSV_COLUMNS = ["t_ns", "tx", "ty", "tz", "qw", "qx", "qy", "qz", "vx", "vy", "vz"]


def vins_csv_to_tum(csv_path: Path, tum_path: Path) -> None:
    """Convert VINS-Mono's native output CSV to a TUM trajectory file."""
    df = pd.read_csv(csv_path, header=None, names=VINS_CSV_COLUMNS)
    tum = pd.DataFrame({
        "t": df["t_ns"] / 1e9,
        "tx": df["tx"],
        "ty": df["ty"],
        "tz": df["tz"],
        "qx": df["qx"],
        "qy": df["qy"],
        "qz": df["qz"],
        "qw": df["qw"],
    })
    tum_path.parent.mkdir(parents=True, exist_ok=True)
    tum.to_csv(tum_path, sep=" ", header=False, index=False)
    print(f"wrote {tum_path} ({len(tum)} poses)")


def run(cmd: list[str]) -> None:
    print("+", " ".join(str(c) for c in cmd))
    subprocess.run(cmd, check=True)


def read_stats_from_zip(zip_path: Path) -> dict:
    with zipfile.ZipFile(zip_path) as zf:
        with zf.open("stats.json") as f:
            return json.load(f)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-loop-csv", type=Path, required=True, help="output/no_loop/vins_result_no_loop.csv")
    ap.add_argument("--loop-csv", type=Path, required=True, help="output/loop/vins_result_loop.csv")
    ap.add_argument("--gt", type=Path, required=True, help="groundtruth.txt (already TUM format)")
    ap.add_argument("--out", type=Path, default=Path("results"), help="output directory")
    ap.add_argument("--t-max-diff", type=float, default=0.02, help="max timestamp diff (s) for GT association")
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    tum_dir = args.out / "tum"
    tum_dir.mkdir(parents=True, exist_ok=True)

    no_loop_tum = tum_dir / "vins_mono_no_loop.tum"
    loop_tum = tum_dir / "vins_mono_loop.tum"
    vins_csv_to_tum(args.no_loop_csv, no_loop_tum)
    vins_csv_to_tum(args.loop_csv, loop_tum)

    # 1. Combined 3D trajectory plot (GT + both estimates, all GT-aligned).
    run([
        "evo_traj", "tum", str(no_loop_tum), str(loop_tum),
        "--ref", str(args.gt),
        "-a", "--sync",
        "--plot_mode", "xyz",
        "--save_plot", str(args.out / "trajectory_3d.png"),
    ])

    # 2. Per-variant APE / RPE against ground truth.
    variants = {"vins_mono": no_loop_tum, "vins_mono_lc": loop_tum}
    rows = []
    for name, est_tum in variants.items():
        row = {"variant": name}
        for metric in ("ape", "rpe"):
            zip_path = args.out / f"{name}_{metric}.zip"
            plot_path = args.out / f"{name}_{metric}.png"
            run([
                f"evo_{metric}", "tum", str(args.gt), str(est_tum),
                "-r", "trans_part",
                "-a",
                "--t_max_diff", str(args.t_max_diff),
                "--save_results", str(zip_path),
                "--save_plot", str(plot_path),
            ])
            stats = read_stats_from_zip(zip_path)
            for key in ("rmse", "mean", "median", "std", "min", "max"):
                row[f"{metric}_{key}"] = stats.get(key)
        rows.append(row)

    summary = pd.DataFrame(rows)
    summary_path = args.out / "summary.csv"
    summary.to_csv(summary_path, index=False)
    print(f"\nwrote {summary_path}:")
    print(summary.to_string(index=False))
    print(
        "\nNote: ground truth only overlaps the estimated trajectories for "
        "~28.8s in the middle of the ~77.6s flight (mocap volume entry/exit); "
        "error stats above are computed only over that overlapping window."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

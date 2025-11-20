#!/usr/bin/env python3

"""
Quick helper to visualize X/Y trajectories from the EKF CSV logs.

Example:
    python3 plot_square_path_csv.py square_path_ekf_test_20251111_153024.csv \\
        --source filtered --output square_plot.png
"""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot an X/Y trajectory from a square_path_ekf CSV log.",
    )
    parser.add_argument(
        "csv_path",
        type=Path,
        help="Path to the CSV file produced by square_path_ekf_tester.",
    )
    parser.add_argument(
        "--source",
        choices=("filtered", "raw"),
        default="filtered",
        help="Which data columns to plot when --x-column/--y-column are not provided.",
    )
    parser.add_argument(
        "--x-column",
        help="Override the column to use for the X axis.",
    )
    parser.add_argument(
        "--y-column",
        help="Override the column to use for the Y axis.",
    )
    parser.add_argument(
        "--title",
        default="Square Path Trajectory",
        help="Title for the plot.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional path to save the figure. If omitted, the plot window is shown.",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=180,
        help="Figure DPI when saving to --output (default: 180).",
    )
    parser.add_argument(
        "--no-equal-aspect",
        action="store_true",
        help="Disable enforcing equal aspect ratio on the axes.",
    )
    return parser.parse_args()


def _to_float(value):
    if value is None:
        return None

    value = value.strip()
    if not value:
        return None

    try:
        return float(value)
    except ValueError:
        return None


def load_xy(csv_path: Path, x_column: str, y_column: str):
    with csv_path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        missing = [col for col in (x_column, y_column) if col not in reader.fieldnames]
        if missing:
            raise ValueError(
                f"Missing columns in CSV: {', '.join(missing)}. "
                f"Available columns: {', '.join(reader.fieldnames or [])}",
            )

        xs, ys = [], []
        for row in reader:
            x_val = _to_float(row.get(x_column))
            y_val = _to_float(row.get(y_column))
            if x_val is None or y_val is None:
                continue
            xs.append(x_val)
            ys.append(y_val)

    if not xs:
        raise ValueError(
            f"No numeric samples found for columns '{x_column}'/'{y_column}'.",
        )

    return xs, ys


def main():
    args = parse_args()

    csv_path = args.csv_path.expanduser()
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    x_column = args.x_column or f"{args.source}_x"
    y_column = args.y_column or f"{args.source}_y"

    xs, ys = load_xy(csv_path, x_column, y_column)

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(xs, ys, linewidth=2, color="#0077cc", label=f"{x_column} vs {y_column}")
    ax.scatter(xs[0], ys[0], color="green", s=60, zorder=5, label="Start")
    ax.scatter(xs[-1], ys[-1], color="red", s=60, zorder=5, label="End")
    ax.set_xlabel(f"{x_column} (m)")
    ax.set_ylabel(f"{y_column} (m)")
    ax.set_title(args.title)
    ax.grid(True, alpha=0.3)
    if not args.no_equal_aspect:
        ax.set_aspect("equal", adjustable="datalim")
    ax.legend()

    if args.output:
        output_path = args.output.expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=args.dpi, bbox_inches="tight")
        print(f"Saved plot to {output_path}")
    else:
        plt.show()


if __name__ == "__main__":
    main()

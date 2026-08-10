#!/usr/bin/env python3
"""Evaluate raw, rigid-aligned and RTAB-Map trajectories against local INS."""

import argparse
import csv
import math
import os
import sys

import numpy as np
import yaml


FIELDS = ("timestamp", "x", "y", "z", "qx", "qy", "qz", "qw")


def load_trajectory(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        missing = [field for field in FIELDS if field not in (reader.fieldnames or [])]
        if missing:
            raise ValueError("%s misses columns: %s" % (path, ", ".join(missing)))
        for row in reader:
            try:
                values = [float(row[field]) for field in FIELDS]
            except (TypeError, ValueError):
                continue
            norm = np.linalg.norm(values[4:8])
            if (values[0] <= 0.0 or
                    any(not math.isfinite(value) for value in values) or norm < 1.0e-9):
                continue
            values[4:8] = [value / norm for value in values[4:8]]
            rows.append(values)
    if not rows:
        raise ValueError("%s contains no valid samples" % path)
    data = np.asarray(rows, dtype=np.float64)
    data = data[np.argsort(data[:, 0], kind="stable")]
    _, reverse_indices = np.unique(data[::-1, 0], return_index=True)
    return data[np.sort(data.shape[0] - 1 - reverse_indices)]


def quaternion_yaw(data):
    x, y, z, w = data[:, 4], data[:, 5], data[:, 6], data[:, 7]
    return np.unwrap(np.arctan2(2.0 * (w * z + x * y),
                                1.0 - 2.0 * (y * y + z * z)))


def interpolate_ins(ins, timestamps):
    positions = np.column_stack([
        np.interp(timestamps, ins[:, 0], ins[:, column])
        for column in (1, 2, 3)
    ])
    yaw = np.interp(timestamps, ins[:, 0], quaternion_yaw(ins))
    return positions, yaw


def angle_error(first, second):
    return np.arctan2(np.sin(first - second), np.cos(first - second))


def rmse_xy(errors):
    return float(np.sqrt(np.mean(np.sum(errors * errors, axis=1))))


def rmse_scalar(errors):
    return float(np.sqrt(np.mean(np.square(errors))))


def rpe_xy(source, target, distance):
    if len(source) < 2:
        return float("nan"), 0
    cumulative = np.concatenate((
        [0.0], np.cumsum(np.linalg.norm(np.diff(target[:, :2], axis=0), axis=1))))
    errors = []
    for first in range(len(source) - 1):
        second = int(np.searchsorted(cumulative, cumulative[first] + distance))
        if second >= len(source):
            break
        source_delta = source[second, :2] - source[first, :2]
        target_delta = target[second, :2] - target[first, :2]
        errors.append(np.linalg.norm(source_delta - target_delta))
    if not errors:
        return float("nan"), 0
    return float(np.sqrt(np.mean(np.square(errors)))), len(errors)


def evaluate(name, trajectory, ins, offset, origin):
    shifted = trajectory[:, 0] + offset
    valid = (shifted >= ins[0, 0]) & (shifted <= ins[-1, 0])
    source = trajectory[valid]
    shifted = shifted[valid]
    if len(source) < 3:
        raise ValueError("%s has fewer than three INS-matched samples" % name)
    target_absolute, target_yaw = interpolate_ins(ins, shifted)
    target = target_absolute - origin
    source_yaw = quaternion_yaw(source)
    heading = angle_error(source_yaw, target_yaw)
    xy_errors = source[:, 1:3] - target[:, :2]
    z_errors = source[:, 3] - target[:, 2]
    rpe10, rpe10_count = rpe_xy(source[:, 1:4], target, 10.0)
    rpe50, rpe50_count = rpe_xy(source[:, 1:4], target, 50.0)
    return {
        "name": name,
        "source": source,
        "target": target,
        "xy_errors": np.linalg.norm(xy_errors, axis=1),
        "z_errors": z_errors,
        "heading_errors": heading,
        "metrics": {
            "matched_samples": int(len(source)),
            "ape_xy_rmse_m": rmse_xy(xy_errors),
            "ape_z_rmse_m": rmse_scalar(z_errors),
            "rpe_10m_xy_rmse_m": rpe10,
            "rpe_10m_pairs": rpe10_count,
            "rpe_50m_xy_rmse_m": rpe50,
            "rpe_50m_pairs": rpe50_count,
            "heading_rmse_deg": math.degrees(rmse_scalar(heading)),
            "heading_median_abs_deg": math.degrees(float(np.median(np.abs(heading)))),
            "start_xy_error_m": float(np.linalg.norm(xy_errors[0])),
            "end_xy_error_m": float(np.linalg.norm(xy_errors[-1])),
        },
    }


def save_plots(output_dir, ins, origin, evaluations, reference_label):
    cache = os.path.join(
        os.environ.get("TMPDIR", "/tmp"), "rtabmap_bringup_matplotlib")
    os.makedirs(cache, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", cache)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ins_local = ins[:, 1:4] - origin
    figure, axis = plt.subplots(figsize=(9, 8))
    axis.plot(ins_local[:, 0], ins_local[:, 1], label=reference_label, linewidth=1.6)
    styles = {
        "fastlio_raw": ("FAST-LIO raw", "tab:orange"),
        "fastlio_aligned": ("FAST-LIO rigid-aligned", "tab:green"),
        "rtabmap_optimized": ("RTAB-Map optimized", "tab:blue"),
    }
    for result in evaluations:
        label, color = styles[result["name"]]
        source = result["source"]
        axis.plot(source[:, 1], source[:, 2], label=label, color=color, linewidth=1.1)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("X / East (m)")
    axis.set_ylabel("Y / North (m)")
    axis.set_title("INS, raw FAST-LIO, rigid-aligned FAST-LIO and RTAB-Map")
    axis.grid(True, linewidth=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "xy_compare.png"), dpi=180)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(10, 4.5))
    ins_elapsed = ins[:, 0] - ins[0, 0]
    axis.plot(ins_elapsed, ins_local[:, 2], label=reference_label, linewidth=1.3)
    for result in evaluations:
        label, color = styles[result["name"]]
        source = result["source"]
        axis.plot(source[:, 0] - source[0, 0], source[:, 3],
                  label=label, color=color, linewidth=1.0)
    axis.set_xlabel("elapsed time (s)")
    axis.set_ylabel("Z (m)")
    axis.set_title("Z trajectory comparison")
    axis.grid(True, linewidth=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "z_compare.png"), dpi=180)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(10, 4.5))
    for result in evaluations:
        label, color = styles[result["name"]]
        source = result["source"]
        axis.plot(source[:, 0] - source[0, 0], result["xy_errors"],
                  label=label, color=color, linewidth=1.0)
    axis.set_xlabel("trajectory elapsed time (s)")
    axis.set_ylabel("absolute XY error (m)")
    axis.set_title("XY error versus time")
    axis.grid(True, linewidth=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "error_xy_vs_time.png"), dpi=180)
    plt.close(figure)


def finite_document(value):
    if isinstance(value, dict):
        return {key: finite_document(item) for key, item in value.items()}
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ins-csv", required=True)
    parser.add_argument(
        "--ins-reference-csv", default="",
        help="Optional pre-localized body-reference INS trajectory in FAST-LIO time")
    parser.add_argument("--fastlio-raw-csv", required=True)
    parser.add_argument("--fastlio-aligned-csv", required=True)
    parser.add_argument("--rtabmap-optimized-csv", default="")
    parser.add_argument("--alignment-yaml", required=True)
    parser.add_argument("--output-dir", required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    ins = load_trajectory(args.ins_reference_csv or args.ins_csv)
    with open(args.alignment_yaml, "r", encoding="utf-8") as stream:
        alignment = yaml.safe_load(stream)
    alignment_offset = float(alignment["time_offset_sec"])
    origin_document = alignment["ins_translation_origin"]
    if args.ins_reference_csv:
        offset = 0.0
        origin = np.zeros(3, dtype=np.float64)
        reference_label = "INS base local"
        reference_name = "ins_body_local_fastlio_time"
    else:
        offset = alignment_offset
        origin = np.asarray([origin_document[axis] for axis in ("x", "y", "z")],
                            dtype=np.float64)
        reference_label = "INS local"
        reference_name = "raw_ins_translation_localized"
    trajectories = [
        ("fastlio_raw", load_trajectory(args.fastlio_raw_csv)),
        ("fastlio_aligned", load_trajectory(args.fastlio_aligned_csv)),
    ]
    if args.rtabmap_optimized_csv:
        trajectories.append((
            "rtabmap_optimized", load_trajectory(args.rtabmap_optimized_csv)))
    evaluations = [
        evaluate(name, trajectory, ins, offset, origin)
        for name, trajectory in trajectories
    ]
    output_dir = os.path.abspath(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)
    save_plots(output_dir, ins, origin, evaluations, reference_label)
    document = {
        "time_offset_sec": alignment_offset,
        "time_offset_convention": "ins_time = fastlio_time + time_offset_sec",
        "evaluation_reference": reference_name,
        "evaluation_reference_query_offset_sec": offset,
        "ins_translation_origin": {
            "x": float(origin[0]), "y": float(origin[1]), "z": float(origin[2])},
        "trajectories": {
            result["name"]: result["metrics"] for result in evaluations},
    }
    document = finite_document(document)
    with open(os.path.join(output_dir, "trajectory_metrics.yaml"),
              "w", encoding="utf-8") as stream:
        yaml.safe_dump(document, stream, default_flow_style=False, sort_keys=False)
    lines = [
        "time_offset_sec=%.9f" % alignment_offset,
        "time_offset_convention=ins_time = fastlio_time + time_offset_sec",
        "evaluation_reference=%s" % reference_name,
        "evaluation_reference_query_offset_sec=%.9f" % offset,
    ]
    for result in evaluations:
        lines.append("")
        lines.append("[%s]" % result["name"])
        for key, value in result["metrics"].items():
            lines.append("%s=%s" % (key, value))
    report = "\n".join(lines) + "\n"
    with open(os.path.join(output_dir, "alignment_report.txt"),
              "w", encoding="utf-8") as stream:
        stream.write(report)
    sys.stdout.write(report)


if __name__ == "__main__":
    try:
        main()
    except (KeyError, OSError, TypeError, ValueError, yaml.YAMLError) as error:
        print("evaluate_ins_alignment: %s" % error, file=sys.stderr)
        raise SystemExit(2)

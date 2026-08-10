#!/usr/bin/env python3
"""Estimate FAST-LIO-map to translation-localized INS ENU SE(2)+Z alignment."""

import argparse
import csv
import math
import os
import sys

import numpy as np


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
            if values[0] <= 0.0 or any(not math.isfinite(value) for value in values):
                continue
            quaternion_norm = np.linalg.norm(values[4:8])
            if quaternion_norm < 1.0e-9:
                continue
            values[4:8] = [value / quaternion_norm for value in values[4:8]]
            rows.append(values)
    if not rows:
        raise ValueError("%s contains no valid trajectory samples" % path)
    data = np.asarray(rows, dtype=np.float64)
    data = data[np.argsort(data[:, 0], kind="stable")]
    # Keep the final sample for duplicate stamps so interpolation is well-defined.
    _, reversed_indices = np.unique(data[::-1, 0], return_index=True)
    indices = np.sort(data.shape[0] - 1 - reversed_indices)
    return data[indices]


def cumulative_xy_distance(points):
    if len(points) < 2:
        return np.zeros(len(points), dtype=np.float64)
    steps = np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)
    return np.concatenate(([0.0], np.cumsum(steps)))


def trim_fastlio_initialization(trajectory, movement_threshold):
    if len(trajectory) < 3:
        raise ValueError("FAST-LIO trajectory has fewer than three samples")
    displacement = np.linalg.norm(
        trajectory[:, 1:3] - trajectory[0, 1:3], axis=1)
    moving = np.flatnonzero(displacement >= movement_threshold)
    if moving.size == 0:
        raise ValueError("FAST-LIO never moves %.3f m from its initial pose" %
                         movement_threshold)
    # FAST-LIO publishes only after initialization, but trim repeated leading
    # identity poses while retaining one sample immediately before motion.
    start = max(0, int(moving[0]) - 1)
    return trajectory[start:], start


def interpolate_positions(trajectory, timestamps):
    return np.column_stack([
        np.interp(timestamps, trajectory[:, 0], trajectory[:, column])
        for column in (1, 2, 3)
    ])


def interpolate_quaternions(trajectory, timestamps):
    quaternions = trajectory[:, 4:8].copy()
    for index in range(1, len(quaternions)):
        if np.dot(quaternions[index - 1], quaternions[index]) < 0.0:
            quaternions[index] *= -1.0
    interpolated = np.column_stack([
        np.interp(timestamps, trajectory[:, 0], quaternions[:, column])
        for column in range(4)
    ])
    norms = np.linalg.norm(interpolated, axis=1)
    if np.any(norms < 1.0e-9):
        raise ValueError("interpolated INS quaternion is invalid")
    return interpolated / norms[:, None]


def fit_se2(source_xy, target_xy):
    source_mean = np.mean(source_xy, axis=0)
    target_mean = np.mean(target_xy, axis=0)
    source_centered = source_xy - source_mean
    target_centered = target_xy - target_mean
    covariance = source_centered.T @ target_centered
    left, _, right_transpose = np.linalg.svd(covariance)
    rotation = right_transpose.T @ left.T
    if np.linalg.det(rotation) < 0.0:
        right_transpose[-1, :] *= -1.0
        rotation = right_transpose.T @ left.T
    translation = target_mean - rotation @ source_mean
    return rotation, translation


def rmse(errors):
    return float(np.sqrt(np.mean(np.sum(errors * errors, axis=1))))


def scalar_rmse(errors):
    return float(np.sqrt(np.mean(np.square(errors))))


def evaluate_offset(fastlio, ins, offset, ins_origin_stamp):
    shifted_stamps = fastlio[:, 0] + offset
    valid = ((shifted_stamps >= ins[0, 0]) &
             (shifted_stamps <= ins[-1, 0]))
    if np.count_nonzero(valid) < 3:
        return None
    source = fastlio[valid]
    ins_stamps = shifted_stamps[valid]
    target_absolute = interpolate_positions(ins, ins_stamps)
    origin_stamp = ins_origin_stamp + offset
    if origin_stamp < ins[0, 0] or origin_stamp > ins[-1, 0]:
        return None
    origin = interpolate_positions(ins, np.asarray([origin_stamp]))[0]
    origin_orientation = interpolate_quaternions(
        ins, np.asarray([origin_stamp]))[0]
    target = target_absolute - origin
    rotation, translation_xy = fit_se2(source[:, 1:3], target[:, :2])
    aligned_xy = (rotation @ source[:, 1:3].T).T + translation_xy
    tz = float(np.median(target[:, 2] - source[:, 3]))
    aligned_z = source[:, 3] + tz
    return {
        "offset": float(offset),
        "source": source,
        "ins_stamps": ins_stamps,
        "target": target,
        "origin": origin,
        "origin_stamp": float(origin_stamp),
        "origin_orientation": origin_orientation,
        "rotation": rotation,
        "translation_xy": translation_xy,
        "tz": tz,
        "aligned_xy": aligned_xy,
        "aligned_z": aligned_z,
        "xy_rmse_before": rmse(source[:, 1:3] - target[:, :2]),
        "xy_rmse_after": rmse(aligned_xy - target[:, :2]),
        "z_rmse_before": scalar_rmse(source[:, 3] - target[:, 2]),
        "z_rmse_after": scalar_rmse(aligned_z - target[:, 2]),
    }


def write_yaml(path, result, source_count, discarded_count, motion_m, search):
    yaw = math.atan2(result["rotation"][1, 0], result["rotation"][0, 0])
    tx, ty = result["translation_xy"]
    ox, oy, oz = result["origin"]
    oqx, oqy, oqz, oqw = result["origin_orientation"]
    lines = [
        "source_frame: fastlio_map",
        "target_frame: ins_local_odom",
        "time_offset_convention: ins_time = fastlio_time + time_offset_sec",
        "time_offset_sec: %.9f" % result["offset"],
        "yaw_rad: %.12f" % yaw,
        "translation:",
        "  x: %.12f" % tx,
        "  y: %.12f" % ty,
        "  z: %.12f" % result["tz"],
        "ins_translation_origin:",
        "  stamp: %.9f" % result["origin_stamp"],
        "  reference_frame: ins",
        "  x: %.12f" % ox,
        "  y: %.12f" % oy,
        "  z: %.12f" % oz,
        "  orientation:",
        "    x: %.12f" % oqx,
        "    y: %.12f" % oqy,
        "    z: %.12f" % oqz,
        "    w: %.12f" % oqw,
        "metrics:",
        "  matched_samples: %d" % len(result["source"]),
        "  source_samples: %d" % source_count,
        "  discarded_initial_samples: %d" % discarded_count,
        "  accumulated_xy_motion_m: %.6f" % motion_m,
        "  xy_rmse_before: %.9f" % result["xy_rmse_before"],
        "  xy_rmse_after: %.9f" % result["xy_rmse_after"],
        "  z_rmse_before: %.9f" % result["z_rmse_before"],
        "  z_rmse_after: %.9f" % result["z_rmse_after"],
        "search:",
        "  minimum_sec: %.6f" % search[0],
        "  maximum_sec: %.6f" % search[1],
        "  step_sec: %.6f" % search[2],
    ]
    with open(path, "w", encoding="utf-8") as stream:
        stream.write("\n".join(lines) + "\n")


def write_matches(path, result):
    fieldnames = (
        "fastlio_timestamp", "ins_timestamp",
        "fastlio_x", "fastlio_y", "fastlio_z",
        "aligned_x", "aligned_y", "aligned_z",
        "ins_local_x", "ins_local_y", "ins_local_z",
        "xy_error_before", "xy_error_after", "z_error_before", "z_error_after")
    source = result["source"]
    target = result["target"]
    before_xy = np.linalg.norm(source[:, 1:3] - target[:, :2], axis=1)
    after_xy = np.linalg.norm(result["aligned_xy"] - target[:, :2], axis=1)
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(fieldnames)
        for index in range(len(source)):
            writer.writerow((
                source[index, 0], result["ins_stamps"][index],
                source[index, 1], source[index, 2], source[index, 3],
                result["aligned_xy"][index, 0], result["aligned_xy"][index, 1],
                result["aligned_z"][index], target[index, 0], target[index, 1],
                target[index, 2], before_xy[index], after_xy[index],
                source[index, 3] - target[index, 2],
                result["aligned_z"][index] - target[index, 2]))


def save_plots(output_dir, best, offsets, errors):
    matplotlib_cache = os.path.join(
        os.environ.get("TMPDIR", "/tmp"), "rtabmap_bringup_matplotlib")
    os.makedirs(matplotlib_cache, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", matplotlib_cache)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    source = best["source"]
    target = best["target"]

    def xy_plot(filename, fast_xy, fast_label, title):
        figure, axis = plt.subplots(figsize=(8, 7))
        axis.plot(target[:, 0], target[:, 1], label="INS local ENU", linewidth=1.4)
        axis.plot(fast_xy[:, 0], fast_xy[:, 1], label=fast_label, linewidth=1.1)
        axis.scatter(target[0, 0], target[0, 1], s=30, label="matched start")
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("X / East (m)")
        axis.set_ylabel("Y / North (m)")
        axis.set_title(title)
        axis.grid(True, linewidth=0.3)
        axis.legend()
        figure.tight_layout()
        figure.savefig(os.path.join(output_dir, filename), dpi=180)
        plt.close(figure)

    xy_plot("alignment_xy_before.png", source[:, 1:3], "FAST-LIO raw",
            "FAST-LIO raw vs translation-localized INS")
    xy_plot("alignment_xy_after.png", best["aligned_xy"], "FAST-LIO rigid-aligned",
            "FAST-LIO SE(2)-aligned vs translation-localized INS")

    elapsed = source[:, 0] - source[0, 0]
    figure, axis = plt.subplots(figsize=(9, 4))
    axis.plot(elapsed, target[:, 2], label="INS local Z")
    axis.plot(elapsed, source[:, 3], label="FAST-LIO raw Z")
    axis.plot(elapsed, best["aligned_z"], label="FAST-LIO aligned Z")
    axis.set_xlabel("FAST-LIO elapsed time (s)")
    axis.set_ylabel("Z (m)")
    axis.set_title("Z alignment")
    axis.grid(True, linewidth=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "alignment_z.png"), dpi=180)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(9, 4))
    axis.plot(offsets, errors)
    axis.axvline(best["offset"], color="tab:red", linestyle="--",
                 label="best %.3f s" % best["offset"])
    axis.set_xlabel("time offset (s), INS time = FAST-LIO time + offset")
    axis.set_ylabel("post-alignment XY RMSE (m)")
    axis.set_title("Alignment error versus time offset")
    axis.grid(True, linewidth=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "alignment_error_vs_time_offset.png"), dpi=180)
    plt.close(figure)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fastlio-csv", required=True)
    parser.add_argument("--ins-csv", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--time-offset-min", type=float, default=-0.5)
    parser.add_argument("--time-offset-max", type=float, default=0.5)
    parser.add_argument("--time-offset-step", type=float, default=0.01)
    parser.add_argument("--minimum-motion", type=float, default=20.0)
    parser.add_argument("--initial-movement-threshold", type=float, default=0.05)
    return parser.parse_args()


def main():
    args = parse_args()
    if (args.time_offset_step <= 0.0 or
            args.time_offset_max < args.time_offset_min):
        raise ValueError("invalid time-offset search range")
    if args.minimum_motion <= 0.0 or args.initial_movement_threshold <= 0.0:
        raise ValueError("motion thresholds must be positive")
    fastlio_all = load_trajectory(args.fastlio_csv)
    ins = load_trajectory(args.ins_csv)
    fastlio, discarded = trim_fastlio_initialization(
        fastlio_all, args.initial_movement_threshold)
    motion = float(cumulative_xy_distance(fastlio[:, 1:4])[-1])
    if motion < args.minimum_motion:
        raise ValueError(
            "FAST-LIO accumulated XY motion %.3f m is below %.3f m" %
            (motion, args.minimum_motion))

    count = int(round((args.time_offset_max - args.time_offset_min) /
                      args.time_offset_step)) + 1
    candidates = args.time_offset_min + np.arange(count) * args.time_offset_step
    evaluated = []
    for offset in candidates:
        result = evaluate_offset(fastlio, ins, offset, fastlio[0, 0])
        if result is not None:
            evaluated.append(result)
    if not evaluated:
        raise ValueError("no time offset produced valid interpolated samples")
    best = min(evaluated, key=lambda item: item["xy_rmse_after"])

    output_dir = os.path.abspath(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)
    yaml_path = os.path.join(output_dir, "ins_fastlio_alignment.yaml")
    write_yaml(yaml_path, best, len(fastlio_all), discarded, motion,
               (args.time_offset_min, args.time_offset_max, args.time_offset_step))
    write_matches(os.path.join(output_dir, "alignment_matches.csv"), best)
    save_plots(output_dir, best,
               [item["offset"] for item in evaluated],
               [item["xy_rmse_after"] for item in evaluated])

    yaw = math.atan2(best["rotation"][1, 0], best["rotation"][0, 0])
    report = (
        "best_time_offset_sec=%.9f\n"
        "time_offset_convention=ins_time = fastlio_time + time_offset_sec\n"
        "yaw_rad=%.12f\nyaw_deg=%.9f\n"
        "tx=%.12f\nty=%.12f\ntz=%.12f\n"
        "matched_samples=%d\naccumulated_xy_motion_m=%.6f\n"
        "xy_rmse_before=%.9f\nxy_rmse_after=%.9f\n"
        "z_rmse_before=%.9f\nz_rmse_after=%.9f\n" %
        (best["offset"], yaw, math.degrees(yaw),
         best["translation_xy"][0], best["translation_xy"][1], best["tz"],
         len(best["source"]), motion, best["xy_rmse_before"],
         best["xy_rmse_after"], best["z_rmse_before"], best["z_rmse_after"]))
    with open(os.path.join(output_dir, "alignment_report.txt"),
              "w", encoding="utf-8") as stream:
        stream.write(report)
    sys.stdout.write(report)
    sys.stdout.write("alignment_yaml=%s\n" % yaml_path)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError) as error:
        print("estimate_fastlio_ins_alignment: %s" % error, file=sys.stderr)
        raise SystemExit(2)

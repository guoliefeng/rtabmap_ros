#!/usr/bin/env python3
"""Inspect a PCD file and create a compact JSON report and top-down plot."""

import argparse
import json
import os

import numpy as np


TYPE_MAP = {
    ("F", 4): "<f4",
    ("F", 8): "<f8",
    ("I", 1): "i1",
    ("I", 2): "<i2",
    ("I", 4): "<i4",
    ("I", 8): "<i8",
    ("U", 1): "u1",
    ("U", 2): "<u2",
    ("U", 4): "<u4",
    ("U", 8): "<u8",
}


def read_header(path):
    values = {}
    header_size = 0
    line_count = 0
    with open(path, "rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD header has no DATA line")
            header_size += len(line)
            line_count += 1
            text = line.decode("ascii", "strict").strip()
            if not text or text.startswith("#"):
                continue
            key, *items = text.split()
            values[key.upper()] = items
            if key.upper() == "DATA":
                break
    return values, header_size, line_count


def structured_dtype(header):
    fields = header["FIELDS"]
    sizes = [int(value) for value in header["SIZE"]]
    types = header["TYPE"]
    counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError("FIELDS, SIZE, TYPE and COUNT lengths differ")
    description = []
    for name, size, kind, count in zip(fields, sizes, types, counts):
        scalar = TYPE_MAP.get((kind.upper(), size))
        if scalar is None:
            raise ValueError("Unsupported PCD scalar type %s%d" % (kind, size))
        description.append((name, scalar) if count == 1 else (name, scalar, (count,)))
    return np.dtype(description)


def load_xyz(path):
    header, offset, line_count = read_header(path)
    data_kind = header["DATA"][0].lower()
    points = int(header.get("POINTS", [header["WIDTH"][0]])[0])
    dtype = structured_dtype(header)
    if data_kind == "binary":
        records = np.memmap(path, mode="r", dtype=dtype, offset=offset, shape=(points,))
    elif data_kind == "ascii":
        matrix = np.loadtxt(path, skiprows=line_count, max_rows=points)
        records = np.empty(points, dtype=dtype)
        column = 0
        for name in dtype.names:
            count = int(np.prod(dtype[name].shape)) if dtype[name].shape else 1
            records[name] = matrix[:, column] if count == 1 else matrix[:, column:column + count]
            column += count
    else:
        raise ValueError("DATA %s is not supported; use binary or ascii PCD" % data_kind)
    for axis in ("x", "y", "z"):
        if axis not in records.dtype.names:
            raise ValueError("PCD has no '%s' field" % axis)
    xyz = np.column_stack((records["x"], records["y"], records["z"])).astype(np.float64)
    return header, data_kind, xyz


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcd")
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--max-plot-points", type=int, default=750000)
    args = parser.parse_args()

    path = os.path.abspath(args.pcd)
    if not os.path.isfile(path):
        parser.error("PCD does not exist: %s" % path)
    output_dir = os.path.abspath(args.output_dir or os.path.dirname(path))
    os.makedirs(output_dir, exist_ok=True)

    header, data_kind, xyz = load_xyz(path)
    finite_mask = np.isfinite(xyz).all(axis=1)
    finite = xyz[finite_mask]
    if finite.size == 0:
        raise ValueError("PCD contains no finite XYZ points")
    quantiles = np.percentile(finite, [1, 50, 99], axis=0)
    minimum = finite.min(axis=0)
    maximum = finite.max(axis=0)
    report = {
        "pcd": path,
        "size_bytes": os.path.getsize(path),
        "data": data_kind,
        "fields": header["FIELDS"],
        "declared_points": int(header.get("POINTS", [len(xyz)])[0]),
        "finite_points": int(len(finite)),
        "nonfinite_points": int(len(xyz) - len(finite)),
        "bounds_m": {axis: [float(minimum[index]), float(maximum[index])]
                     for index, axis in enumerate("xyz")},
        "extent_m": {axis: float(maximum[index] - minimum[index])
                     for index, axis in enumerate("xyz")},
        "percentiles_m": {
            label: {axis: float(quantiles[row, index]) for index, axis in enumerate("xyz")}
            for row, label in enumerate(("p01", "p50", "p99"))
        },
    }
    report_path = os.path.join(output_dir, "pcd_report.json")
    with open(report_path, "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
        stream.write("\n")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        step = max(1, int(np.ceil(len(finite) / max(1, args.max_plot_points))))
        sample = finite[::step]
        figure, axis = plt.subplots(figsize=(12, 5))
        points = axis.scatter(sample[:, 0], sample[:, 1], c=sample[:, 2], s=0.15,
                              cmap="viridis", rasterized=True)
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("X (m)")
        axis.set_ylabel("Y (m)")
        axis.set_title("Yangpu GPS multi-session cloud (top view)")
        axis.grid(True, linewidth=0.2)
        figure.colorbar(points, ax=axis, label="Z (m)")
        figure.tight_layout()
        figure.savefig(os.path.join(output_dir, "pcd_topdown.png"), dpi=180)
        plt.close(figure)
    except ImportError:
        report["plot_warning"] = "matplotlib is unavailable"

    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()

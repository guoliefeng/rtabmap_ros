#!/usr/bin/env python3
"""Create a compact, offline quality report for an RTAB-Map SQLite database."""

import argparse
import collections
import csv
import os
import re
import sqlite3
import struct
import sys
import zlib


LINK_TYPES = {
    0: "Neighbor",
    1: "Global Closure",
    2: "Local Space Closure",
    3: "Local Time Closure",
    4: "User Closure",
    5: "Virtual Closure",
    6: "Neighbor Merged",
    7: "Pose Prior",
    8: "Landmark",
    9: "Gravity",
}


def parse_pose(blob):
    if blob is None or len(blob) != 48:
        return None
    values = struct.unpack("<12f", blob)
    return values[3], values[7], values[11]


def transform_translation(blob):
    parsed = parse_pose(blob)
    if parsed is None:
        return None
    return parsed + ((parsed[0] ** 2 + parsed[1] ** 2 + parsed[2] ** 2) ** 0.5,)


def parse_statistics(blob):
    if not blob:
        return {}
    try:
        text = zlib.decompress(blob).decode("utf-8", "replace")
    except (zlib.error, UnicodeDecodeError):
        return {}
    result = {}
    for item in text.split(";"):
        if ":" not in item:
            continue
        key, value = item.split(":", 1)
        try:
            result[key] = float(value)
        except ValueError:
            continue
    return result


def extent(points):
    if not points:
        return None
    return {
        "x": (min(point[1] for point in points), max(point[1] for point in points)),
        "y": (min(point[2] for point in points), max(point[2] for point in points)),
        "z": (min(point[3] for point in points), max(point[3] for point in points)),
    }


def trajectory_length(points):
    total = 0.0
    for first, second in zip(points, points[1:]):
        total += ((second[1] - first[1]) ** 2 + (second[2] - first[2]) ** 2 +
                  (second[3] - first[3]) ** 2) ** 0.5
    return total


def load_fastlio_trajectory(path):
    if not path:
        return []
    points = []
    with open(path, newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            try:
                points.append((float(row["stamp"]), float(row["x"]), float(row["y"]), float(row["z"])))
            except (KeyError, TypeError, ValueError):
                continue
    return sorted(points)


def save_plots(output_dir, raw_points, optimized_points, map_ids, fastlio_points):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return ["WARN: matplotlib is unavailable; PNG plots were not created"]

    messages = []

    def save_xy(path, series, title):
        figure, axis = plt.subplots(figsize=(8, 7))
        for label, points, color in series:
            if points:
                axis.plot([point[1] for point in points], [point[2] for point in points],
                          label=label, linewidth=1.0, color=color)
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("X (m)")
        axis.set_ylabel("Y (m)")
        axis.set_title(title)
        axis.grid(True, linewidth=0.3)
        if any(points for _, points, _ in series):
            axis.legend()
        figure.tight_layout()
        figure.savefig(path, dpi=160)
        plt.close(figure)

    def save_z(path, series, title):
        figure, axis = plt.subplots(figsize=(9, 4))
        for label, points, color in series:
            if points:
                axis.plot([point[0] for point in points], [point[3] for point in points],
                          label=label, linewidth=1.0, color=color)
        axis.set_xlabel("Stamp (s)")
        axis.set_ylabel("Z (m)")
        axis.set_title(title)
        axis.grid(True, linewidth=0.3)
        if any(points for _, points, _ in series):
            axis.legend()
        figure.tight_layout()
        figure.savefig(path, dpi=160)
        plt.close(figure)

    save_xy(os.path.join(output_dir, "trajectory_xy.png"),
            [("RTAB-Map raw odometry", raw_points, "tab:gray"),
             ("RTAB-Map optimized", optimized_points, "tab:blue")],
            "RTAB-Map raw and optimized trajectory")
    save_z(os.path.join(output_dir, "trajectory_z.png"),
           [("RTAB-Map raw odometry", raw_points, "tab:gray"),
            ("RTAB-Map optimized", optimized_points, "tab:blue")],
           "RTAB-Map raw and optimized Z")
    if map_ids:
        figure, axis = plt.subplots(figsize=(9, 4))
        axis.step([item[0] for item in map_ids], [item[1] for item in map_ids], where="mid")
        axis.set_xlabel("Node id")
        axis.set_ylabel("map_id")
        axis.set_title("RTAB-Map session timeline")
        axis.grid(True, linewidth=0.3)
        figure.tight_layout()
        figure.savefig(os.path.join(output_dir, "map_id_timeline.png"), dpi=160)
        plt.close(figure)
    if fastlio_points:
        save_xy(os.path.join(output_dir, "fastlio_xy.png"),
                [("FAST-LIO raw odometry", fastlio_points, "tab:orange")],
                "FAST-LIO raw trajectory")
        save_xy(os.path.join(output_dir, "rtabmap_xy.png"),
                [("RTAB-Map optimized", optimized_points, "tab:blue")],
                "RTAB-Map optimized trajectory")
        save_xy(os.path.join(output_dir, "fastlio_vs_rtabmap_xy.png"),
                [("FAST-LIO raw odometry", fastlio_points, "tab:orange"),
                 ("RTAB-Map optimized", optimized_points, "tab:blue")],
                "FAST-LIO vs RTAB-Map optimized trajectory")
        save_z(os.path.join(output_dir, "fastlio_vs_rtabmap_z.png"),
               [("FAST-LIO raw odometry", fastlio_points, "tab:orange"),
                ("RTAB-Map optimized", optimized_points, "tab:blue")],
               "FAST-LIO vs RTAB-Map optimized Z")
    return messages


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("database", help="Path to rtabmap.db")
    parser.add_argument("--output-dir", default="", help="Default: database directory")
    parser.add_argument("--fastlio-trajectory", default="", help="CSV written by check_fastlio_sync.py")
    args = parser.parse_args()

    database = os.path.abspath(args.database)
    if not os.path.isfile(database):
        parser.error("database does not exist: %s" % database)
    output_dir = os.path.abspath(args.output_dir or os.path.dirname(database))
    os.makedirs(output_dir, exist_ok=True)

    connection = sqlite3.connect("file:%s?mode=ro" % database, uri=True)
    cursor = connection.cursor()
    stored_node_count = cursor.execute("select count(*) from Node").fetchone()[0]
    discarded_node_count = cursor.execute("select count(*) from Node where weight = -9").fetchone()[0]
    # A weight of -9 is RTAB-Map's internal marker for a signature removed by
    # rehearsal. It remains in the database for bookkeeping, but it is not a
    # node in the graph used for localization, optimization, or map export.
    node_rows = cursor.execute(
        "select id, map_id, stamp, pose from Node where weight > -9 order by id").fetchall()
    raw_points = []
    map_ids = []
    for node_id, map_id, stamp, pose in node_rows:
        parsed = parse_pose(pose)
        map_ids.append((node_id, map_id))
        if parsed is not None and stamp is not None:
            raw_points.append((float(stamp), parsed[0], parsed[1], parsed[2]))

    map_counts = collections.Counter(map_id for _, map_id in map_ids)
    link_counts = dict(cursor.execute(
        "select l.type, count(*) from Link l "
        "join Node a on a.id=l.from_id join Node b on b.id=l.to_id "
        "where a.weight > -9 and b.weight > -9 group by l.type"))
    link_rows = cursor.execute(
        "select l.from_id, l.to_id, l.type, l.transform from Link l "
        "join Node a on a.id=l.from_id join Node b on b.id=l.to_id "
        "where l.from_id > 0 and l.to_id > 0 and l.from_id != l.to_id "
        "and a.weight > -9 and b.weight > -9").fetchall()
    node_map_ids = {node_id: map_id for node_id, map_id, _, _ in node_rows}
    unique_links = {}
    for first, second, link_type, transform in link_rows:
        key = (min(first, second), max(first, second), link_type)
        unique_links.setdefault(key, transform)
    cross_map_links = []
    for (first, second, link_type), transform in sorted(unique_links.items()):
        first_map = node_map_ids.get(first)
        second_map = node_map_ids.get(second)
        if first_map is not None and second_map is not None and first_map != second_map:
            cross_map_links.append((first, second, first_map, second_map, link_type,
                                    transform_translation(transform)))
    adjacency = {node_id: set() for node_id, _, _, _ in node_rows}
    for first, second, _, _ in link_rows:
        adjacency.setdefault(first, set()).add(second)
        adjacency.setdefault(second, set()).add(first)
    components = []
    unseen = set(adjacency)
    while unseen:
        root = unseen.pop()
        component = {root}
        frontier = [root]
        while frontier:
            node = frontier.pop()
            for neighbor in adjacency.get(node, ()):
                if neighbor in unseen:
                    unseen.remove(neighbor)
                    component.add(neighbor)
                    frontier.append(neighbor)
        components.append(component)
    largest_component = max((len(component) for component in components), default=0)

    optimized_points = []
    for node_id, stamp, data in cursor.execute(
            "select s.id, s.stamp, s.data from Statistics s join Node n on n.id=s.id "
            "where n.weight > -9 order by s.id"):
        values = parse_statistics(data)
        keys = ("Loop/MapToBase_x/m", "Loop/MapToBase_y/m", "Loop/MapToBase_z/m")
        if stamp is not None and all(key in values for key in keys):
            optimized_points.append((float(stamp), values[keys[0]], values[keys[1]], values[keys[2]]))

    table_counts = {}
    for table in ("Node", "Data", "Link", "Feature", "Word", "GlobalDescriptor", "Statistics"):
        try:
            table_counts[table] = cursor.execute("select count(*) from %s" % table).fetchone()[0]
        except sqlite3.Error:
            table_counts[table] = None
    connection.close()

    fastlio_points = load_fastlio_trajectory(args.fastlio_trajectory)
    messages = save_plots(output_dir, raw_points, optimized_points, map_ids, fastlio_points)
    report_lines = [
        "# RTAB-Map database report",
        "",
        "Database: %s" % database,
        "Size: %d bytes" % os.path.getsize(database),
        "",
        "## Counts",
    ]
    for table in ("Node", "Data", "Link", "Statistics", "Feature", "Word", "GlobalDescriptor"):
        report_lines.append("- %s: %s" % (table, table_counts[table] if table_counts[table] is not None else "table missing"))
    report_lines.append("- Node records used by the active map: %d" % len(node_rows))
    report_lines.append("- Rehearsal-merged historical node records (weight=-9): %d" % discarded_node_count)
    if stored_node_count != len(node_rows) + discarded_node_count:
        report_lines.append("- Other inactive node records: %d" % (
            stored_node_count - len(node_rows) - discarded_node_count))
    report_lines.extend(["", "## Maps", "- map_id count: %d" % len(map_counts)])
    for map_id, count in sorted(map_counts.items()):
        report_lines.append("- map_id %s: %d nodes" % (map_id, count))
    report_lines.append("- single-node maps: %d" % sum(count == 1 for count in map_counts.values()))
    report_lines.extend(["", "## Connectivity"])
    report_lines.append("- connected components: %d" % len(components))
    report_lines.append("- largest component: %d nodes (%.2f%%)" % (
        largest_component, 100.0 * largest_component / len(node_rows) if node_rows else 0.0))
    report_lines.extend(["", "## Link types"])
    for link_type, count in sorted(link_counts.items()):
        report_lines.append("- %s (%d): %d" % (LINK_TYPES.get(link_type, "Unknown"), link_type, count))
    report_lines.extend(["", "## Cross-session constraints"])
    report_lines.append("- unique cross-map edges: %d" % len(cross_map_links))
    report_lines.append("- unique cross-map closure edges: %d" % sum(
        link[4] not in (0, 6) for link in cross_map_links))
    if cross_map_links:
        cross_map_pairs = collections.Counter(
            tuple(sorted((link[2], link[3]))) for link in cross_map_links)
        for map_pair, count in sorted(cross_map_pairs.items()):
            report_lines.append("- map_id %d <-> %d: %d unique edges" %
                                (map_pair[0], map_pair[1], count))
        for first, second, first_map, second_map, link_type, translation in cross_map_links[:20]:
            transform_text = "transform unavailable"
            if translation is not None:
                transform_text = "t=(%.3f, %.3f, %.3f) m, norm=%.3f m" % translation
            report_lines.append(
                "- node %d (map %d) <-> node %d (map %d): %s (%d), %s" %
                (first, first_map, second, second_map,
                 LINK_TYPES.get(link_type, "Unknown"), link_type, transform_text))
    report_lines.extend(["", "## Trajectories"])
    for label, points in (("RTAB-Map raw odometry", raw_points),
                          ("RTAB-Map optimized (Statistics MapToBase)", optimized_points),
                          ("FAST-LIO raw odometry", fastlio_points)):
        bounds = extent(points)
        if bounds:
            report_lines.append("- %s: %d points, length %.3f m, X=%s, Y=%s, Z=%s" % (
                label, len(points), trajectory_length(points), bounds["x"], bounds["y"], bounds["z"]))
        else:
            report_lines.append("- %s: unavailable" % label)
    report_lines.extend(["", "## Generated files"])
    for name in ("trajectory_xy.png", "trajectory_z.png", "map_id_timeline.png",
                 "fastlio_xy.png", "rtabmap_xy.png", "fastlio_vs_rtabmap_xy.png",
                 "fastlio_vs_rtabmap_z.png"):
        if os.path.isfile(os.path.join(output_dir, name)):
            report_lines.append("- %s" % name)
    report_lines.extend(messages)
    report_path = os.path.join(output_dir, "db_report.txt")
    with open(report_path, "w", encoding="utf-8") as stream:
        stream.write("\n".join(report_lines) + "\n")
    print("\n".join(report_lines))
    print("Wrote %s" % report_path)


if __name__ == "__main__":
    main()

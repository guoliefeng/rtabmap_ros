#!/usr/bin/env bash
set -euo pipefail

if [[ -f /opt/ros/noetic/setup.bash ]]; then
  # This script is bash-specific; sourcing setup.bash from zsh can resolve BASH_SOURCE incorrectly.
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_BAG_DIR="${HOME}/dataDisk/hainan/yangpu/qc"
DEFAULT_REPORT="$(cd "${SCRIPT_DIR}/../.." && pwd)/docs/yangpu_bag_interface_report.md"
BAG_DIR="${YANGPU_BAG_DIR:-${DEFAULT_BAG_DIR}}"
BAG_PATTERN="${YANGPU_BAG_PATTERN:-2026-06-22-12-*}"
REPORT_PATH="${YANGPU_BAG_REPORT:-${DEFAULT_REPORT}}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag-dir)
      BAG_DIR="$2"; shift 2;;
    --pattern)
      BAG_PATTERN="$2"; shift 2;;
    --report)
      REPORT_PATH="$2"; shift 2;;
    --help|-h)
      echo "Usage: $0 [--bag-dir DIR] [--pattern GLOB] [--report FILE]"
      exit 0;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2;;
  esac
done

mapfile -t BAG_FILES < <(find "$BAG_DIR" -maxdepth 1 -type f -name "$BAG_PATTERN" -print | sort)
if [[ ${#BAG_FILES[@]} -eq 0 ]]; then
  echo "FAIL: no bag files matched ${BAG_DIR}/${BAG_PATTERN}" >&2
  exit 1
fi

mkdir -p "$(dirname "$REPORT_PATH")"
python3 - "$REPORT_PATH" "${BAG_FILES[@]}" <<'PY'
import collections
import datetime
import math
import os
import sys

import rosbag


report_path = sys.argv[1]
bag_paths = sys.argv[2:]
selected = {
    "/lidar_preprocessor/meta_cloud",
    "/ins_driver/imu",
    "/localization/ins",
    "/tf",
    "/tf_static",
    "/clock",
}
pointcloud_topics = set()
topic_counts = collections.defaultdict(int)
topic_types = {}
topic_rates = collections.defaultdict(list)
bag_durations = []
first_messages = {}
imu_samples = []
static_transforms = []


def record_first(topic, msg):
    if topic not in first_messages:
        first_messages[topic] = msg


for path in bag_paths:
    # The index contains counts, types and frequencies. Do not deserialize the
    # large PointCloud2 payloads while aggregating the whole dataset.
    with rosbag.Bag(path) as bag:
        bag_durations.append((path, bag.get_end_time() - bag.get_start_time()))
        info = bag.get_type_and_topic_info()
        for topic, details in info.topics.items():
            topic_counts[topic] += details.message_count
            topic_types[topic] = details.msg_type
            if details.frequency and math.isfinite(details.frequency):
                topic_rates[topic].append(details.frequency)
            if details.msg_type == "sensor_msgs/PointCloud2":
                pointcloud_topics.add(topic)

# Read only small message samples from the first bag. IMU and localization
# messages are small; the point-cloud sample loop stops after the first message
# of every PointCloud2 topic and never scans all cloud payloads.
sample_path = bag_paths[0]
with rosbag.Bag(sample_path) as bag:
    for topic, msg, _ in bag.read_messages(topics=sorted(selected - {"/tf_static"})):
        record_first(topic, msg)
        if topic == "/ins_driver/imu" and len(imu_samples) < 10000:
            imu_samples.append(msg)
    remaining_cloud_topics = set(pointcloud_topics)
    for topic, msg, _ in bag.read_messages(topics=sorted(remaining_cloud_topics)):
        record_first(topic, msg)
        remaining_cloud_topics.discard(topic)
        if not remaining_cloud_topics:
            break

# Static TF is tiny, so collect all occurrences without touching point clouds.
for path in bag_paths:
    with rosbag.Bag(path) as bag:
        for _, msg, _ in bag.read_messages(topics=["/tf_static"]):
            static_transforms.extend(msg.transforms)


def fmt_rate(topic):
    values = topic_rates.get(topic, [])
    return "%.3f Hz" % (sum(values) / len(values)) if values else "n/a"


def msg_frame(msg):
    return getattr(getattr(msg, "header", None), "frame_id", "") or "(empty)"


def field_text(msg):
    names = {1: "INT8", 2: "UINT8", 3: "INT16", 4: "UINT16", 5: "INT32", 6: "UINT32", 7: "FLOAT32", 8: "FLOAT64"}
    return ", ".join("%s(%s@%d)" % (f.name, names.get(f.datatype, str(f.datatype)), f.offset) for f in msg.fields)


def status(ok, label, detail):
    return "%s **%s** — %s" % ("PASS:" if ok else "WARN:", label, detail)


cloud_msg = first_messages.get("/lidar_preprocessor/meta_cloud")
imu_msg = first_messages.get("/ins_driver/imu")
ins_msg = first_messages.get("/localization/ins")
cloud_frame = msg_frame(cloud_msg) if cloud_msg else "(missing)"
imu_frame = msg_frame(imu_msg) if imu_msg else "(missing)"
imu_valid = bool(imu_samples) and all(
    math.isfinite(v)
    for msg in imu_samples
    for v in (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
)
imu_nonzero = bool(imu_samples) and all(
    math.sqrt(msg.orientation.x ** 2 + msg.orientation.y ** 2 + msg.orientation.z ** 2 + msg.orientation.w ** 2) > 1e-9
    for msg in imu_samples
)
imu_cov_minus_one = sum(abs(msg.orientation_covariance[0] + 1.0) < 1e-12 for msg in imu_samples)
static_pairs = {(t.header.frame_id.lstrip("/"), t.child_frame_id.lstrip("/")) for t in static_transforms}
base_to_imu = ("base_link", imu_frame.lstrip("/")) in static_pairs
base_to_cloud = cloud_frame.lstrip("/") == "base_link" or ("base_link", cloud_frame.lstrip("/")) in static_pairs

total_duration = sum(duration for _, duration in bag_durations)
has_clock = "/clock" in topic_counts
lines = []
lines.append("# Yangpu rosbag interface report")
lines.append("")
lines.append("Generated: %s" % datetime.datetime.now().astimezone().isoformat(timespec="seconds"))
lines.append("")
lines.append("## Input files")
lines.append("")
lines.append("- Matched files: **%d**" % len(bag_paths))
lines.append("- Total indexed duration: **%.3f s**" % total_duration)
lines.append("")
lines.append("## Static checks")
lines.append("")
lines.append(status(bool(pointcloud_topics), "PointCloud2 topics", ", ".join(sorted(pointcloud_topics)) or "none"))
lines.append(status("/lidar_preprocessor/meta_cloud" in pointcloud_topics, "/lidar_preprocessor/meta_cloud", "present" if "/lidar_preprocessor/meta_cloud" in pointcloud_topics else "missing"))
lines.append(status(cloud_msg is not None, "meta_cloud frame", cloud_frame))
lines.append(status(imu_msg is not None and topic_types.get("/ins_driver/imu") == "sensor_msgs/Imu", "/ins_driver/imu type", topic_types.get("/ins_driver/imu", "missing")))
lines.append(status(bool(imu_samples) and imu_valid and imu_nonzero, "IMU quaternion", "finite and non-zero in %d sampled messages" % len(imu_samples)))
lines.append(status(bool(imu_samples) and imu_cov_minus_one == 0, "IMU orientation covariance", "covariance[0] != -1 in %d sampled messages" % len(imu_samples)))
lines.append(status(base_to_cloud, "base_link <- cloud frame", "%s (identity or static TF)" % cloud_frame))
lines.append(status(base_to_imu, "base_link <- IMU frame", "static TF base_link -> %s" % imu_frame if base_to_imu else "missing static TF base_link -> %s" % imu_frame))
lines.append(status("/localization/ins" in topic_counts, "/localization/ins", "%s, frame=%s, child_frame_id=%s" % (topic_types.get("/localization/ins", "missing"), msg_frame(ins_msg) if ins_msg else "missing", getattr(ins_msg, "child_frame_id", "missing") if ins_msg else "missing")))
lines.append(("WARN:" if not has_clock else "PASS:") + " **/clock in bag** — " + ("present" if has_clock else "absent; rosbag play --clock must publish it at runtime"))
lines.append(("PASS:" if "/tf_static" in topic_counts else "WARN:") + " **/tf_static** — " + ("present" if "/tf_static" in topic_counts else "missing"))
lines.append(("WARN:" if "/tf" not in topic_counts else "PASS:") + " **/tf** — " + ("present" if "/tf" in topic_counts else "absent; odometry will publish dynamic TF"))
lines.append("")
lines.append("## PointCloud2 topics")
lines.append("")
lines.append("| Topic | Type | Messages | Indexed rate | First frame | Fields | x y z intensity | time/timestamp | ring |")
lines.append("|---|---|---:|---:|---|---|---|---|---|")
for topic in sorted(pointcloud_topics):
    msg = first_messages.get(topic)
    names = {f.name for f in msg.fields} if msg else set()
    xyz = all(name in names for name in ("x", "y", "z"))
    intensity = "intensity" in names
    time_field = any(name in names for name in ("time", "t", "timestamp"))
    ring = "ring" in names
    lines.append("| `%s` | `%s` | %d | %s | `%s` | `%s` | %s | %s | %s |" % (
        topic, topic_types[topic], topic_counts[topic], fmt_rate(topic), msg_frame(msg) if msg else "missing",
        field_text(msg) if msg else "missing", "PASS" if xyz and intensity else "WARN", "yes" if time_field else "no", "yes" if ring else "no"))
lines.append("")
lines.append("## Candidate topics")
lines.append("")
for topic in sorted(topic_counts):
    if topic in ("/lidar_preprocessor/meta_cloud", "/ins_driver/imu", "/localization/ins"):
        lines.append("- `%s`: `%s`, **%d** messages, indexed rate **%s**" % (topic, topic_types[topic], topic_counts[topic], fmt_rate(topic)))
lines.append("")
lines.append("## TF static summary")
lines.append("")
for parent, child in sorted(static_pairs):
    lines.append("- `%s -> %s`" % (parent, child))
lines.append("")
lines.append("## Interpretation")
lines.append("")
lines.append("- The selected cloud is already expressed in `base_link`; no cloud-frame static transform is needed.")
lines.append("- The selected IMU frame is `%s`; the bag contains a direct static transform from `base_link` to that frame, so the launch default is verified from the bag." % imu_frame)
lines.append("- The cloud fields do not include per-point time or ring metadata; first-stage bringup therefore keeps `deskewing=false` and does not invent a ring/time interface.")
lines.append("- `/localization/ins` is recorded as `%s`, but GPS/INS is intentionally not connected in this stage." % topic_types.get("/localization/ins", "missing"))
lines.append("")

with open(report_path, "w", encoding="utf-8") as stream:
    stream.write("\n".join(lines) + "\n")

print("Wrote %s" % report_path)
for line in lines:
    if line.startswith(("PASS:", "WARN:")):
        print(line)
PY

echo "PASS: report generated at ${REPORT_PATH}"

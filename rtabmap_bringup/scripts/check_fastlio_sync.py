#!/usr/bin/env python3
"""Measure FAST-LIO odometry/body-cloud timestamp agreement using wall time."""

import argparse
import bisect
import csv
import json
import math
import statistics
import sys
import time

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage


class SyncChecker:
    def __init__(self):
        self.odom_stamps = []
        self.cloud_stamps = []
        self.trajectory = []
        self.tf_stamps = []
        self.backend_tf_stamps = []

    def odom_callback(self, msg):
        stamp = msg.header.stamp.to_sec()
        if stamp <= 0.0:
            return
        self.odom_stamps.append(stamp)
        pose = msg.pose.pose
        self.trajectory.append((stamp, pose.position.x, pose.position.y, pose.position.z,
                                pose.orientation.x, pose.orientation.y,
                                pose.orientation.z, pose.orientation.w))

    def cloud_callback(self, msg):
        stamp = msg.header.stamp.to_sec()
        if stamp > 0.0:
            self.cloud_stamps.append(stamp)

    def tf_callback(self, msg, frames):
        parent_frame, child_frame, map_frame, backend_odom_frame = frames
        for transform in msg.transforms:
            if (transform.header.frame_id == parent_frame and
                    transform.child_frame_id == child_frame):
                stamp = transform.header.stamp.to_sec()
                if stamp > 0.0:
                    self.tf_stamps.append(stamp)
            if (map_frame and transform.header.frame_id == map_frame and
                    transform.child_frame_id == (backend_odom_frame or parent_frame)):
                stamp = transform.header.stamp.to_sec()
                if stamp > 0.0:
                    self.backend_tf_stamps.append(stamp)


def topic_rate(stamps):
    ordered = sorted(set(stamps))
    if len(ordered) < 2 or ordered[-1] <= ordered[0]:
        return 0.0
    return float(len(ordered) - 1) / (ordered[-1] - ordered[0])


def nearest_differences(cloud_stamps, odom_stamps, max_match_sec):
    ordered_odom = sorted(odom_stamps)
    differences = []
    if not ordered_odom:
        return differences
    for cloud_stamp in sorted(cloud_stamps):
        index = bisect.bisect_left(ordered_odom, cloud_stamp)
        candidates = []
        if index < len(ordered_odom):
            candidates.append(ordered_odom[index])
        if index > 0:
            candidates.append(ordered_odom[index - 1])
        if candidates:
            difference = min((cloud_stamp - candidate for candidate in candidates), key=abs)
            if abs(difference) <= max_match_sec:
                differences.append(difference)
    return differences


def write_trajectory(path, trajectory):
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("stamp", "x", "y", "z", "qx", "qy", "qz", "qw"))
        writer.writerows(trajectory)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--odom-topic", default="/fast_lio_ns/loc_result")
    parser.add_argument("--cloud-topic", default="/fast_lio_ns/cloud_registered_body")
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--odom-frame", default="map")
    parser.add_argument("--base-frame", default="base_link_fast_lio")
    parser.add_argument("--map-frame", default="",
                        help="Optional RTAB-Map root frame to verify map_frame -> odom_frame TF.")
    parser.add_argument("--backend-odom-frame", default="",
                        help="Optional RTAB-Map odom frame when it differs from FAST-LIO's frame.")
    parser.add_argument("--duration", type=float, default=15.0,
                        help="Wall-clock sampling duration in seconds.")
    parser.add_argument("--max-match-ms", type=float, default=50.0)
    parser.add_argument("--output-json", default="")
    parser.add_argument("--trajectory-csv", default="")
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    if args.duration <= 0.0 or args.max_match_ms <= 0.0:
        parser.error("duration and max-match-ms must be positive")

    rospy.init_node("check_fastlio_sync", anonymous=True)
    checker = SyncChecker()
    rospy.Subscriber(args.odom_topic, Odometry, checker.odom_callback, queue_size=200)
    rospy.Subscriber(args.cloud_topic, PointCloud2, checker.cloud_callback, queue_size=200)
    rospy.Subscriber(args.tf_topic, TFMessage, checker.tf_callback,
                     callback_args=(args.odom_frame, args.base_frame, args.map_frame,
                                    args.backend_odom_frame), queue_size=200)

    deadline = time.monotonic() + args.duration
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        time.sleep(0.05)

    differences = nearest_differences(checker.cloud_stamps, checker.odom_stamps,
                                      args.max_match_ms / 1000.0)
    abs_ms = [abs(value) * 1000.0 for value in differences]
    result = {
        "odom_topic": args.odom_topic,
        "cloud_topic": args.cloud_topic,
        "sample_duration_wall_sec": args.duration,
        "odom_messages": len(checker.odom_stamps),
        "cloud_messages": len(checker.cloud_stamps),
        "odom_rate_hz": topic_rate(checker.odom_stamps),
        "cloud_rate_hz": topic_rate(checker.cloud_stamps),
        "matched_pairs": len(differences),
        "unmatched_cloud_messages": len(checker.cloud_stamps) - len(differences),
        "tf_topic": args.tf_topic,
        "fastlio_tf_parent_frame": args.odom_frame,
        "fastlio_tf_child_frame": args.base_frame,
        "fastlio_tf_messages": len(checker.tf_stamps),
        "fastlio_tf_rate_hz": topic_rate(checker.tf_stamps),
        "rtabmap_tf_parent_frame": args.map_frame,
        "rtabmap_tf_child_frame": ((args.backend_odom_frame or args.odom_frame)
                                    if args.map_frame else ""),
        "rtabmap_tf_messages": len(checker.backend_tf_stamps),
        "rtabmap_tf_rate_hz": topic_rate(checker.backend_tf_stamps),
    }
    if abs_ms:
        result.update({
            "delta_mean_ms": statistics.mean(abs_ms),
            "delta_median_ms": statistics.median(abs_ms),
            "delta_max_ms": max(abs_ms),
            "delta_signed_mean_ms": statistics.mean(value * 1000.0 for value in differences),
            "exact_stamp_pairs": sum(value <= 1e-6 for value in abs_ms),
        })
        if result["delta_max_ms"] <= 1e-3:
            result["sync_recommendation"] = "approx_sync=false (all matched header stamps are equal within 1 microsecond)"
        elif result["delta_max_ms"] <= 20.0:
            result["sync_recommendation"] = (
                "approx_sync=true; measured max interval is %.6f s" %
                (result["delta_max_ms"] / 1000.0))
        else:
            result["sync_recommendation"] = (
                "FAIL: measured maximum timestamp gap %.3f ms exceeds the 20 ms initial limit" %
                result["delta_max_ms"])
    else:
        result["sync_recommendation"] = "FAIL: no odometry/cloud timestamp pairs were sampled"

    print("FAST-LIO timestamp synchronization")
    for key in ("odom_topic", "cloud_topic", "odom_messages", "cloud_messages", "odom_rate_hz",
                "cloud_rate_hz", "matched_pairs", "unmatched_cloud_messages", "tf_topic",
                "fastlio_tf_parent_frame", "fastlio_tf_child_frame", "fastlio_tf_messages",
                "fastlio_tf_rate_hz", "rtabmap_tf_parent_frame", "rtabmap_tf_child_frame",
                "rtabmap_tf_messages", "rtabmap_tf_rate_hz", "delta_mean_ms",
                "delta_median_ms", "delta_max_ms", "exact_stamp_pairs", "sync_recommendation"):
        if key in result:
            print("%s: %s" % (key, result[key]))

    if args.trajectory_csv:
        write_trajectory(args.trajectory_csv, checker.trajectory)
        print("trajectory_csv: %s" % args.trajectory_csv)
    if args.output_json:
        with open(args.output_json, "w", encoding="utf-8") as stream:
            json.dump(result, stream, indent=2, sort_keys=True)
            stream.write("\n")
        print("output_json: %s" % args.output_json)

    if not differences or result.get("delta_max_ms", math.inf) > 20.0:
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())

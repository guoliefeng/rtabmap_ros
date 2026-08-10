#!/usr/bin/env python3
"""Record FAST-LIO and INS odometry independently without timestamp pairing."""

import csv
import math
import os
import threading

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


FIELDS = ("timestamp", "x", "y", "z", "qx", "qy", "qz", "qw")


class CsvTrajectoryWriter:
    def __init__(self, path, label):
        self.path = path
        self.label = label
        self.stream = open(path, "w", newline="", encoding="utf-8", buffering=1)
        self.writer = csv.DictWriter(self.stream, fieldnames=FIELDS)
        self.writer.writeheader()
        self.count = 0
        self.frame_id = None
        self.child_frame_id = None
        self.lock = threading.Lock()

    @staticmethod
    def _values(message):
        pose = message.pose.pose
        values = (
            message.header.stamp.to_sec(),
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if values[0] <= 0.0 or any(not math.isfinite(value) for value in values):
            return None
        quaternion_norm = math.sqrt(sum(value * value for value in values[4:]))
        if quaternion_norm < 1.0e-9:
            return None
        return values

    def callback(self, message):
        values = self._values(message)
        if values is None:
            rospy.logwarn_throttle(5.0, "Dropping invalid %s odometry sample", self.label)
            return
        with self.lock:
            self.writer.writerow(dict(zip(FIELDS, values)))
            self.count += 1
            if self.frame_id is None:
                self.frame_id = message.header.frame_id
                self.child_frame_id = getattr(message, "child_frame_id", "")
                rospy.loginfo(
                    "%s trajectory frame_id='%s' child_frame_id='%s'",
                    self.label, self.frame_id, self.child_frame_id)
            if self.count % 500 == 0:
                self.stream.flush()

    def close(self):
        with self.lock:
            if not self.stream.closed:
                self.stream.flush()
                self.stream.close()


class TrajectoryCollector:
    def __init__(self):
        output_dir = os.path.abspath(rospy.get_param("~output_dir", ""))
        if not output_dir:
            raise ValueError("~output_dir must not be empty")
        os.makedirs(output_dir, exist_ok=True)
        fastlio_topic = rospy.get_param(
            "~fastlio_topic", "/fast_lio_ns/loc_result")
        ins_topic = rospy.get_param("~ins_topic", "/localization/ins")
        aligned_topic = rospy.get_param("~aligned_topic", "")
        prior_topic = rospy.get_param("~prior_topic", "")
        fastlio_filename = rospy.get_param("~fastlio_filename", "fastlio.csv")
        ins_filename = rospy.get_param("~ins_filename", "ins.csv")
        aligned_filename = rospy.get_param(
            "~aligned_filename", "fastlio_aligned.csv")
        prior_filename = rospy.get_param(
            "~prior_filename", "ins_body_local.csv")
        if not fastlio_topic or not ins_topic:
            raise ValueError("trajectory topics must not be empty")

        self.fastlio = CsvTrajectoryWriter(
            os.path.join(output_dir, fastlio_filename), "FAST-LIO")
        self.ins = CsvTrajectoryWriter(
            os.path.join(output_dir, ins_filename), "INS")
        self.aligned = None
        self.prior = None
        self.fastlio_subscriber = rospy.Subscriber(
            fastlio_topic, Odometry, self.fastlio.callback, queue_size=1000)
        self.ins_subscriber = rospy.Subscriber(
            ins_topic, Odometry, self.ins.callback, queue_size=5000)
        self.aligned_subscriber = None
        self.prior_subscriber = None
        if aligned_topic:
            self.aligned = CsvTrajectoryWriter(
                os.path.join(output_dir, aligned_filename), "FAST-LIO aligned")
            self.aligned_subscriber = rospy.Subscriber(
                aligned_topic, Odometry, self.aligned.callback, queue_size=1000)
        if prior_topic:
            self.prior = CsvTrajectoryWriter(
                os.path.join(output_dir, prior_filename), "INS body local")
            self.prior_subscriber = rospy.Subscriber(
                prior_topic, PoseWithCovarianceStamped,
                self.prior.callback, queue_size=5000)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "Collecting independent trajectories: %s -> %s, %s -> %s",
            fastlio_topic, self.fastlio.path, ins_topic, self.ins.path)

    def shutdown(self):
        self.fastlio.close()
        self.ins.close()
        if self.aligned is not None:
            self.aligned.close()
        if self.prior is not None:
            self.prior.close()
        rospy.loginfo(
            "Trajectory collection complete: FAST-LIO=%d INS=%d aligned=%d prior=%d",
            self.fastlio.count, self.ins.count,
            self.aligned.count if self.aligned is not None else 0,
            self.prior.count if self.prior is not None else 0)


if __name__ == "__main__":
    rospy.init_node("collect_fastlio_ins_trajectory")
    try:
        TrajectoryCollector()
    except (OSError, ValueError) as error:
        rospy.logfatal("Cannot start trajectory collector: %s", error)
        raise SystemExit(2)
    rospy.spin()

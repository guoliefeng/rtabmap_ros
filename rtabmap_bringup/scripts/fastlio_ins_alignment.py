#!/usr/bin/env python3
"""Apply a calibrated yaw+translation transform to FAST-LIO odometry."""

import copy
import math
import os

import numpy as np
import rospy
import yaml
from nav_msgs.msg import Odometry


class FastlioInsAlignment:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic", "/fast_lio_ns/loc_result")
        self.output_topic = rospy.get_param(
            "~output_topic", "/fast_lio_ns/loc_result_ins_aligned")
        self.alignment_file = os.path.abspath(
            rospy.get_param("~alignment_file", ""))
        self.expected_input_frame = rospy.get_param(
            "~expected_input_frame", "map")
        self.expected_child_frame = rospy.get_param(
            "~expected_child_frame", "base_link_fast_lio")
        self.strict_frames = bool(rospy.get_param("~strict_frames", True))
        self.linear_stddev = float(rospy.get_param("~linear_stddev", 0.05))
        self.angular_stddev = float(rospy.get_param(
            "~angular_stddev", math.radians(1.0)))
        if not self.input_topic or not self.output_topic or not self.alignment_file:
            raise ValueError("input_topic, output_topic and alignment_file are required")
        if not os.path.isfile(self.alignment_file):
            raise ValueError("alignment file does not exist: %s" % self.alignment_file)
        if (not math.isfinite(self.linear_stddev) or self.linear_stddev <= 0.0 or
                not math.isfinite(self.angular_stddev) or self.angular_stddev <= 0.0):
            raise ValueError("fallback standard deviations must be finite and positive")

        with open(self.alignment_file, "r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        if not isinstance(document, dict):
            raise ValueError("alignment YAML root must be a mapping")
        self.output_frame = str(document.get("target_frame", "ins_local_odom"))
        self.time_offset_sec = float(document.get("time_offset_sec", 0.0))
        self.yaw = float(document["yaw_rad"])
        translation = document.get("translation")
        if not isinstance(translation, dict):
            raise ValueError("alignment YAML translation must be a mapping")
        self.translation = np.asarray([
            float(translation[axis]) for axis in ("x", "y", "z")],
            dtype=np.float64)
        values = [self.time_offset_sec, self.yaw] + self.translation.tolist()
        if not self.output_frame or any(not math.isfinite(value) for value in values):
            raise ValueError("alignment YAML contains invalid values")

        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        self.rotation = np.asarray([
            [cosine, -sine, 0.0],
            [sine, cosine, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        self.covariance_rotation = np.zeros((6, 6), dtype=np.float64)
        self.covariance_rotation[:3, :3] = self.rotation
        self.covariance_rotation[3:, 3:] = self.rotation
        self.yaw_quaternion = np.asarray(
            [0.0, 0.0, math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)])
        self.fallback_covariance = np.zeros((6, 6), dtype=np.float64)
        self.fallback_covariance[0, 0] = self.linear_stddev ** 2
        self.fallback_covariance[1, 1] = self.linear_stddev ** 2
        self.fallback_covariance[2, 2] = self.linear_stddev ** 2
        self.fallback_covariance[3, 3] = self.angular_stddev ** 2
        self.fallback_covariance[4, 4] = self.angular_stddev ** 2
        self.fallback_covariance[5, 5] = self.angular_stddev ** 2

        self.publisher = rospy.Publisher(
            self.output_topic, Odometry, queue_size=200)
        self.subscriber = rospy.Subscriber(
            self.input_topic, Odometry, self.callback, queue_size=500)
        self.received = 0
        self.published = 0
        self.fallback_pose_covariances = 0
        self.fallback_twist_covariances = 0
        rospy.loginfo(
            "FAST-LIO fixed INS alignment: %s [%s] -> %s [%s], "
            "yaw=%.9f rad (%.6f deg), t=(%.6f, %.6f, %.6f) m, "
            "calibrated time offset=%.3f s (stamp is not changed)",
            self.input_topic, self.expected_input_frame,
            self.output_topic, self.output_frame,
            self.yaw, math.degrees(self.yaw),
            self.translation[0], self.translation[1], self.translation[2],
            self.time_offset_sec)

    @staticmethod
    def quaternion_multiply(left, right):
        lx, ly, lz, lw = left
        rx, ry, rz, rw = right
        return np.asarray((
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ), dtype=np.float64)

    @staticmethod
    def normalized_quaternion(message):
        values = np.asarray([
            message.x, message.y, message.z, message.w], dtype=np.float64)
        norm = np.linalg.norm(values)
        if not np.all(np.isfinite(values)) or norm < 1.0e-9:
            return None
        return values / norm

    @staticmethod
    def valid_covariance(values):
        covariance = np.asarray(values, dtype=np.float64).reshape((6, 6))
        if not np.all(np.isfinite(covariance)):
            return None
        if np.max(np.abs(covariance)) < 1.0e-15:
            return None
        if np.any(np.diag(covariance) < 0.0):
            return None
        return covariance

    def pose_covariance(self, values):
        covariance = self.valid_covariance(values)
        if covariance is None:
            self.fallback_pose_covariances += 1
            covariance = self.fallback_covariance
        rotated = (self.covariance_rotation @ covariance @
                   self.covariance_rotation.T)
        return ((rotated + rotated.T) * 0.5).reshape(36).tolist()

    def twist_covariance(self, values):
        # Odometry twist is expressed in child_frame_id, which is unchanged by
        # the world-frame yaw alignment. Preserve valid input covariance.
        covariance = self.valid_covariance(values)
        if covariance is None:
            self.fallback_twist_covariances += 1
            covariance = self.fallback_covariance
        return covariance.reshape(36).tolist()

    def callback(self, message):
        self.received += 1
        if self.strict_frames:
            if message.header.frame_id != self.expected_input_frame:
                rospy.logerr_throttle(
                    5.0, "Dropping FAST-LIO odometry with frame_id='%s', expected '%s'",
                    message.header.frame_id, self.expected_input_frame)
                return
            if message.child_frame_id != self.expected_child_frame:
                rospy.logerr_throttle(
                    5.0, "Dropping FAST-LIO odometry with child_frame_id='%s', expected '%s'",
                    message.child_frame_id, self.expected_child_frame)
                return
        source_position = np.asarray([
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        ], dtype=np.float64)
        source_orientation = self.normalized_quaternion(
            message.pose.pose.orientation)
        if not np.all(np.isfinite(source_position)) or source_orientation is None:
            rospy.logwarn_throttle(5.0, "Dropping invalid FAST-LIO odometry pose")
            return

        output = Odometry()
        output.header = copy.deepcopy(message.header)
        # Keep the FAST-LIO sensor stamp exactly. The calibrated time offset is
        # for cross-sensor evaluation, not a license to retimestamp odometry.
        output.header.frame_id = self.output_frame
        output.child_frame_id = message.child_frame_id
        aligned_position = self.rotation @ source_position + self.translation
        output.pose.pose.position.x = aligned_position[0]
        output.pose.pose.position.y = aligned_position[1]
        output.pose.pose.position.z = aligned_position[2]
        aligned_orientation = self.quaternion_multiply(
            self.yaw_quaternion, source_orientation)
        aligned_orientation /= np.linalg.norm(aligned_orientation)
        (output.pose.pose.orientation.x, output.pose.pose.orientation.y,
         output.pose.pose.orientation.z, output.pose.pose.orientation.w) = \
            aligned_orientation.tolist()
        output.pose.covariance = self.pose_covariance(message.pose.covariance)
        output.twist.twist = copy.deepcopy(message.twist.twist)
        output.twist.covariance = self.twist_covariance(message.twist.covariance)
        self.publisher.publish(output)
        self.published += 1
        rospy.loginfo_throttle(
            30.0, "FAST-LIO INS alignment published %d/%d messages; "
            "fallback covariance pose=%d twist=%d",
            self.published, self.received,
            self.fallback_pose_covariances, self.fallback_twist_covariances)


if __name__ == "__main__":
    rospy.init_node("fastlio_ins_alignment")
    try:
        FastlioInsAlignment()
    except (KeyError, OSError, TypeError, ValueError, yaml.YAMLError) as error:
        rospy.logfatal("Cannot start FAST-LIO INS alignment: %s", error)
        raise SystemExit(2)
    rospy.spin()

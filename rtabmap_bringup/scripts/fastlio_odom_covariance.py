#!/usr/bin/env python3
"""Prepare FAST-LIO odometry for RTAB-Map covariance and session alignment."""

import copy
import math
import threading
from collections import deque

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class FastlioOdomCovariance:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/fast_lio_ns/loc_result")
        self.output_topic = rospy.get_param(
            "~output_topic", "/fast_lio_ns/loc_result_rtabmap")
        self.linear_stddev = float(rospy.get_param("~linear_stddev", 0.05))
        self.angular_stddev = float(rospy.get_param(
            "~angular_stddev", math.radians(1.0)))
        self.align_to_global_pose = bool(rospy.get_param(
            "~align_to_global_pose", False))
        self.global_pose_topic = rospy.get_param(
            "~global_pose_topic", "/rtabmap/global_pose")
        self.aligned_frame_id = rospy.get_param(
            "~aligned_frame_id", "gps_local_odom")
        self.alignment_tolerance = float(rospy.get_param(
            "~alignment_tolerance", 0.05))
        self.min_alignment_translation = float(rospy.get_param(
            "~min_alignment_translation", 0.05))
        self.publish_delay_sec = float(rospy.get_param(
            "~publish_delay_sec", 0.0))
        if not self.input_topic or not self.output_topic:
            raise ValueError("input_topic and output_topic must be non-empty")
        if not math.isfinite(self.linear_stddev) or self.linear_stddev <= 0.0:
            raise ValueError("linear_stddev must be finite and positive")
        if not math.isfinite(self.angular_stddev) or self.angular_stddev <= 0.0:
            raise ValueError("angular_stddev must be finite and positive")
        if self.align_to_global_pose and (not self.global_pose_topic or
                                          not self.aligned_frame_id):
            raise ValueError("global_pose_topic and aligned_frame_id must be non-empty")
        if (not math.isfinite(self.alignment_tolerance) or
                self.alignment_tolerance <= 0.0):
            raise ValueError("alignment_tolerance must be finite and positive")
        if (not math.isfinite(self.min_alignment_translation) or
                self.min_alignment_translation < 0.0):
            raise ValueError("min_alignment_translation must be finite and non-negative")
        if not math.isfinite(self.publish_delay_sec) or self.publish_delay_sec < 0.0:
            raise ValueError("publish_delay_sec must be finite and non-negative")

        self.publisher = rospy.Publisher(self.output_topic, Odometry, queue_size=100)
        self.subscriber = rospy.Subscriber(
            self.input_topic, Odometry, self.odom_callback, queue_size=200)
        self.global_pose_subscriber = None
        self.lock = threading.Lock()
        self.pending_odometry = deque(maxlen=200)
        self.global_poses = deque(maxlen=1000)
        self.alignment = None
        if self.align_to_global_pose:
            self.global_pose_subscriber = rospy.Subscriber(
                self.global_pose_topic,
                PoseWithCovarianceStamped,
                self.global_pose_callback,
                queue_size=200,
            )
        self.count = 0
        rospy.loginfo(
            "FAST-LIO RTAB covariance adapter: %s -> %s, linear stddev=%.4f m, "
            "angular stddev=%.6f rad, align_to_global_pose=%s, aligned_frame='%s', "
            "publish_delay=%.3f s",
            self.input_topic,
            self.output_topic,
            self.linear_stddev,
            self.angular_stddev,
            self.align_to_global_pose,
            self.aligned_frame_id,
            self.publish_delay_sec,
        )

    @staticmethod
    def _normalized_quaternion(pose):
        quaternion = pose.orientation
        values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        norm = math.sqrt(sum(value * value for value in values))
        if any(not math.isfinite(value) for value in values) or norm < 1.0e-9:
            return None
        return tuple(value / norm for value in values)

    @staticmethod
    def _quaternion_conjugate(quaternion):
        return (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])

    @staticmethod
    def _quaternion_multiply(left, right):
        lx, ly, lz, lw = left
        rx, ry, rz, rw = right
        return (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )

    @staticmethod
    def _rotate(quaternion, vector):
        pure = (vector[0], vector[1], vector[2], 0.0)
        rotated = FastlioOdomCovariance._quaternion_multiply(
            FastlioOdomCovariance._quaternion_multiply(quaternion, pure),
            FastlioOdomCovariance._quaternion_conjugate(quaternion),
        )
        return rotated[:3]

    @staticmethod
    def _pose_tuple(pose):
        quaternion = FastlioOdomCovariance._normalized_quaternion(pose)
        if quaternion is None:
            return None
        position = (pose.position.x, pose.position.y, pose.position.z)
        if any(not math.isfinite(value) for value in position):
            return None
        return position, quaternion

    @staticmethod
    def _alignment_transform(global_pose, odometry_pose):
        global_transform = FastlioOdomCovariance._pose_tuple(global_pose)
        odometry_transform = FastlioOdomCovariance._pose_tuple(odometry_pose)
        if global_transform is None or odometry_transform is None:
            return None
        global_position, global_orientation = global_transform
        odometry_position, odometry_orientation = odometry_transform
        orientation = FastlioOdomCovariance._quaternion_multiply(
            global_orientation,
            FastlioOdomCovariance._quaternion_conjugate(odometry_orientation),
        )
        rotated_position = FastlioOdomCovariance._rotate(
            orientation, odometry_position)
        position = tuple(global_position[index] - rotated_position[index]
                         for index in range(3))
        return position, orientation

    def _try_alignment_locked(self):
        if self.alignment is not None or not self.global_poses:
            return []
        candidate_odometry = None
        for odometry in self.pending_odometry:
            pose = odometry.pose.pose.position
            if math.sqrt(pose.x * pose.x + pose.y * pose.y + pose.z * pose.z) >= \
                    self.min_alignment_translation:
                candidate_odometry = odometry
                break
        if candidate_odometry is None:
            return []
        stamp = candidate_odometry.header.stamp.to_sec()
        candidate_global = min(
            self.global_poses,
            key=lambda item: abs(item.header.stamp.to_sec() - stamp),
        )
        delta = abs(candidate_global.header.stamp.to_sec() - stamp)
        if delta > self.alignment_tolerance:
            return []
        self.alignment = self._alignment_transform(
            candidate_global.pose.pose, candidate_odometry.pose.pose)
        if self.alignment is None:
            rospy.logwarn_throttle(5.0, "Cannot align invalid FAST-LIO/GPS poses")
            return []
        position, orientation = self.alignment
        rospy.loginfo(
            "Aligned FAST-LIO odometry to GPS-local frame at stamp %.6f "
            "(delta=%.3f ms): translation=(%.3f, %.3f, %.3f), "
            "quaternion=(%.5f, %.5f, %.5f, %.5f)",
            stamp,
            delta * 1000.0,
            position[0], position[1], position[2],
            orientation[0], orientation[1], orientation[2], orientation[3],
        )
        pending = list(self.pending_odometry)
        self.pending_odometry.clear()
        self.global_poses.clear()
        return pending

    def global_pose_callback(self, message):
        pending = []
        with self.lock:
            if self.alignment is None:
                self.global_poses.append(copy.deepcopy(message))
                pending = self._try_alignment_locked()
        for odometry in pending:
            self._publish(odometry)

    def odom_callback(self, message):
        if not self.align_to_global_pose:
            self._publish(message)
            return
        pending = []
        publish_current = False
        with self.lock:
            if self.alignment is None:
                self.pending_odometry.append(copy.deepcopy(message))
                pending = self._try_alignment_locked()
            else:
                publish_current = True
        for odometry in pending:
            self._publish(odometry)
        if publish_current:
            self._publish(message)

    def _publish(self, message):
        output = Odometry()
        output.header = copy.deepcopy(message.header)
        output.child_frame_id = message.child_frame_id
        output.pose.pose = copy.deepcopy(message.pose.pose)
        output.twist.twist = copy.deepcopy(message.twist.twist)

        if self.alignment is not None:
            position, orientation = self.alignment
            source = self._pose_tuple(message.pose.pose)
            if source is None:
                rospy.logwarn_throttle(5.0, "Dropping invalid FAST-LIO odometry pose")
                return
            source_position, source_orientation = source
            rotated = self._rotate(orientation, source_position)
            output.pose.pose.position.x = position[0] + rotated[0]
            output.pose.pose.position.y = position[1] + rotated[1]
            output.pose.pose.position.z = position[2] + rotated[2]
            output_orientation = self._quaternion_multiply(
                orientation, source_orientation)
            (output.pose.pose.orientation.x, output.pose.pose.orientation.y,
             output.pose.pose.orientation.z, output.pose.pose.orientation.w) = \
                output_orientation
            # Deliberately use a distinct frame with no TF. CoreWrapper then
            # consumes this aligned Odometry pose instead of replacing it with
            # FAST-LIO's original map->base TF lookup.
            output.header.frame_id = self.aligned_frame_id

        covariance = [0.0] * 36
        linear_variance = self.linear_stddev ** 2
        angular_variance = self.angular_stddev ** 2
        for index in (0, 7, 14):
            covariance[index] = linear_variance
        for index in (21, 28, 35):
            covariance[index] = angular_variance
        output.pose.covariance = covariance
        # CoreWrapper prefers twist covariance when it is valid. Use the same
        # conservative edge uncertainty so it cannot fall back to FAST-LIO's
        # tiny instantaneous EKF state covariance.
        output.twist.covariance = list(covariance)
        if self.publish_delay_sec > 0.0:
            # Delay only delivery, never the ROS header timestamp. This lets
            # CoreWrapper's high-rate global-pose callback fill its timestamp
            # buffer before ExactTime releases the matching odom/cloud pair.
            timer = threading.Timer(self.publish_delay_sec, self._emit, (output,))
            timer.daemon = True
            timer.start()
        else:
            self._emit(output)

    def _emit(self, output):
        if rospy.is_shutdown():
            return
        self.publisher.publish(output)
        self.count += 1
        rospy.loginfo_throttle(
            30.0, "FAST-LIO RTAB covariance adapter published %d messages", self.count)


if __name__ == "__main__":
    rospy.init_node("fastlio_odom_covariance")
    try:
        FastlioOdomCovariance()
    except (TypeError, ValueError) as error:
        rospy.logfatal("Invalid FAST-LIO covariance adapter configuration: %s", error)
        raise SystemExit(2)
    rospy.spin()

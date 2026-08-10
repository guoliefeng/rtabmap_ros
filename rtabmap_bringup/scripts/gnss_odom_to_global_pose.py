#!/usr/bin/env python3
"""Adapt global GNSS odometry to RTAB-Map's covariance-aware pose priors."""

import copy
import json
import math
import os
import tempfile
from collections import deque

import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class GnssOdomToGlobalPose:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/localization/gnss_odom")
        self.output_topic = rospy.get_param("~output_topic", "/rtabmap/global_pose")
        self.output_frame_id = rospy.get_param("~output_frame_id", "base_link_fast_lio")
        self.expected_child_frame = rospy.get_param("~expected_child_frame", "base_link")
        self.strict_child_frame = bool(rospy.get_param("~strict_child_frame", True))
        self.diagonalize_covariance = bool(rospy.get_param("~diagonalize_covariance", True))
        self.allow_zero_covariance = bool(rospy.get_param("~allow_zero_covariance", True))
        self.covariance_scale = float(rospy.get_param("~covariance_scale", 1.0))
        self.pose_delay_sec = float(rospy.get_param("~pose_delay_sec", 3.0))
        self.position_only = bool(rospy.get_param("~position_only", True))
        self.local_origin = bool(rospy.get_param("~local_origin", False))
        self.local_origin_mode = str(
            rospy.get_param("~local_origin_mode", "se3")).strip().lower()
        self.origin_file = str(rospy.get_param("~origin_file", ""))
        self.use_alignment_time_offset = bool(
            rospy.get_param("~use_alignment_time_offset", False))
        self.output_stamp_offset_sec = float(
            rospy.get_param("~output_stamp_offset_sec", 0.0))
        self.apply_pose_reference_extrinsic = bool(
            rospy.get_param("~apply_pose_reference_extrinsic", False))
        self.body_reference_frame = str(
            rospy.get_param("~body_reference_frame", "base_link"))
        self.pose_reference_frame = str(
            rospy.get_param("~pose_reference_frame", "ins"))
        self.min_position_stddev = self._vector_param(
            "~min_position_stddev", [0.20, 0.20, 0.50])
        self.min_orientation_stddev = self._vector_param(
            "~min_orientation_stddev", [0.0349066, 0.0349066, 0.0174533])
        self.fallback_position_stddev = self._vector_param(
            "~fallback_position_stddev", [2.0, 2.0, 3.0])
        self.fallback_orientation_stddev = self._vector_param(
            "~fallback_orientation_stddev", [0.35, 0.35, 0.35])
        self.max_position_stddev = float(rospy.get_param("~max_position_stddev", 20.0))

        if not self.input_topic or not self.output_topic or not self.output_frame_id:
            raise ValueError("input_topic, output_topic and output_frame_id must be non-empty")
        if not math.isfinite(self.covariance_scale) or self.covariance_scale <= 0.0:
            raise ValueError("covariance_scale must be finite and positive")
        if not math.isfinite(self.max_position_stddev) or self.max_position_stddev <= 0.0:
            raise ValueError("max_position_stddev must be finite and positive")
        if not math.isfinite(self.pose_delay_sec) or self.pose_delay_sec < 0.0:
            raise ValueError("pose_delay_sec must be finite and non-negative")
        if not math.isfinite(self.output_stamp_offset_sec):
            raise ValueError("output_stamp_offset_sec must be finite")
        if self.local_origin_mode not in ("se3", "translation_only"):
            raise ValueError("local_origin_mode must be 'se3' or 'translation_only'")
        if (self.apply_pose_reference_extrinsic and
                (not self.body_reference_frame or not self.pose_reference_frame)):
            raise ValueError(
                "body_reference_frame and pose_reference_frame must be non-empty "
                "when pose-reference compensation is enabled")

        self.received = 0
        self.published = 0
        self.fallback_covariance_count = 0
        self.pending = deque()
        self.pose_reference_extrinsic = None
        self.origin_in_pose_reference = False
        self.tf_buffer = None
        self.tf_listener = None
        if self.apply_pose_reference_extrinsic:
            self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.origin = self._load_origin() if self.local_origin and self.origin_file else None
        self.publisher = rospy.Publisher(
            self.output_topic, PoseWithCovarianceStamped, queue_size=20)
        self.subscriber = rospy.Subscriber(
            self.input_topic, Odometry, self.callback, queue_size=100)
        rospy.loginfo(
            "GNSS global-pose adapter: %s -> %s, RTAB sensor frame='%s', "
            "expected child='%s', covariance floors position=%s m orientation=%s rad, "
            "position_only=%s, local_origin=%s (%s), origin_file='%s', delay=%.3f s, "
            "pose_reference_compensation=%s (%s <- %s), output_stamp_offset=%.3f s",
            self.input_topic,
            self.output_topic,
            self.output_frame_id,
            self.expected_child_frame,
            self.min_position_stddev,
            self.min_orientation_stddev,
            self.position_only,
            self.local_origin,
            self.local_origin_mode,
            self.origin_file,
            self.pose_delay_sec,
            self.apply_pose_reference_extrinsic,
            self.body_reference_frame,
            self.pose_reference_frame,
            self.output_stamp_offset_sec,
        )

    @staticmethod
    def _vector_param(name, default):
        values = rospy.get_param(name, default)
        if not isinstance(values, list) or len(values) != 3:
            raise ValueError("%s must be a three-element list" % name)
        result = [float(value) for value in values]
        if any(not math.isfinite(value) or value <= 0.0 for value in result):
            raise ValueError("%s values must be finite and positive" % name)
        return result

    @staticmethod
    def _pose_is_finite(pose):
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        return all(math.isfinite(value) for value in values)

    @staticmethod
    def _normalize_quaternion(pose):
        quaternion = pose.orientation
        norm = math.sqrt(
            quaternion.x * quaternion.x + quaternion.y * quaternion.y +
            quaternion.z * quaternion.z + quaternion.w * quaternion.w)
        if not math.isfinite(norm) or norm < 1.0e-9:
            return False
        quaternion.x /= norm
        quaternion.y /= norm
        quaternion.z /= norm
        quaternion.w /= norm
        return True

    @staticmethod
    def _normalize_quaternion_values(values):
        if not isinstance(values, (list, tuple)) or len(values) != 4:
            raise ValueError("origin orientation must have four elements")
        result = [float(value) for value in values]
        norm = math.sqrt(sum(value * value for value in result))
        if any(not math.isfinite(value) for value in result) or norm < 1.0e-9:
            raise ValueError("origin orientation is not a finite quaternion")
        return tuple(value / norm for value in result)

    def _load_origin(self):
        if not os.path.exists(self.origin_file):
            return None
        with open(self.origin_file, "r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        if not isinstance(document, dict):
            raise ValueError("origin file must contain a mapping")
        alignment_origin = document.get("ins_translation_origin")
        if isinstance(alignment_origin, dict):
            position = [alignment_origin.get(axis) for axis in ("x", "y", "z")]
            orientation_document = alignment_origin.get("orientation")
            if not isinstance(orientation_document, dict):
                raise ValueError("alignment INS origin must contain orientation")
            orientation_values = [orientation_document.get(axis)
                                  for axis in ("x", "y", "z", "w")]
            self.origin_in_pose_reference = True
            if self.use_alignment_time_offset:
                alignment_offset = float(document.get("time_offset_sec"))
                if not math.isfinite(alignment_offset):
                    raise ValueError("alignment time_offset_sec must be finite")
                # Alignment convention: INS time = FAST-LIO time + offset.
                # Therefore an INS sample corresponds to output/FAST-LIO time
                # INS time - offset.
                self.output_stamp_offset_sec = -alignment_offset
        else:
            position = document.get("position")
            orientation_values = document.get("orientation")
        if not isinstance(position, list) or len(position) != 3:
            raise ValueError("origin position must have three elements")
        position = tuple(float(value) for value in position)
        if any(not math.isfinite(value) for value in position):
            raise ValueError("origin position contains a non-finite value")
        orientation = self._normalize_quaternion_values(orientation_values)
        rospy.loginfo("Loaded local origin from %s (pose_reference=%s)",
                      self.origin_file, self.origin_in_pose_reference)
        return position, orientation

    def _save_origin(self, message, position, orientation):
        if not self.origin_file:
            return
        directory = os.path.dirname(os.path.abspath(self.origin_file))
        os.makedirs(directory, exist_ok=True)
        document = {
            "format_version": 1,
            "source_frame_id": message.header.frame_id,
            "source_child_frame_id": message.child_frame_id,
            "stamp": message.header.stamp.to_sec(),
            "position": list(position),
            "orientation": list(orientation),
        }
        descriptor, temporary_path = tempfile.mkstemp(
            prefix=".gnss_origin_", suffix=".json", dir=directory, text=True)
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                json.dump(document, stream, indent=2, sort_keys=True)
                stream.write("\n")
            os.replace(temporary_path, self.origin_file)
        except Exception:
            try:
                os.unlink(temporary_path)
            except OSError:
                pass
            raise
        rospy.loginfo("Saved GNSS local origin to %s", self.origin_file)

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
    def _rotation_matrix(quaternion):
        x, y, z, w = quaternion
        return (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
             2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
             2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
             1.0 - 2.0 * (x * x + y * y)),
        )

    @staticmethod
    def _matrix_vector(matrix, vector):
        return tuple(sum(matrix[row][column] * vector[column]
                         for column in range(3)) for row in range(3))

    @staticmethod
    def _rotate_covariance(source, rotation):
        # First-order frame change for [x, y, z, rx, ry, rz]. The GNSS node
        # publishes a conventional 6x6 pose covariance in the ENU frame.
        transform = [[0.0] * 6 for _ in range(6)]
        for block in (0, 3):
            for row in range(3):
                for column in range(3):
                    transform[block + row][block + column] = rotation[row][column]
        covariance = [[float(source[row * 6 + column]) for column in range(6)]
                      for row in range(6)]
        intermediate = [[sum(transform[row][index] * covariance[index][column]
                             for index in range(6))
                         for column in range(6)] for row in range(6)]
        rotated = [[sum(intermediate[row][index] * transform[column][index]
                        for index in range(6))
                    for column in range(6)] for row in range(6)]
        return [rotated[row][column] for row in range(6) for column in range(6)]

    def _to_local_pose(self, message, pose):
        if not self.local_origin:
            return None
        source_position = (pose.position.x, pose.position.y, pose.position.z)
        source_orientation = (
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)
        if self.origin is None:
            self.origin = (source_position, source_orientation)
            self._save_origin(message, source_position, source_orientation)
        origin_position, origin_orientation = self.origin
        delta = tuple(source_position[index] - origin_position[index]
                      for index in range(3))
        if self.local_origin_mode == "translation_only":
            # Preserve the global/ENU axis directions. In particular, do not
            # rotate positions by the inverse first-pose vehicle attitude.
            pose.position.x, pose.position.y, pose.position.z = delta
            return None
        inverse_orientation = self._quaternion_conjugate(origin_orientation)
        rotation = self._rotation_matrix(inverse_orientation)
        local_position = self._matrix_vector(rotation, delta)
        local_orientation = self._normalize_quaternion_values(
            self._quaternion_multiply(inverse_orientation, source_orientation))
        pose.position.x, pose.position.y, pose.position.z = local_position
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = local_orientation
        return rotation

    def _get_pose_reference_extrinsic(self):
        if not self.apply_pose_reference_extrinsic:
            return None
        if self.pose_reference_extrinsic is not None:
            return self.pose_reference_extrinsic
        try:
            transform = self.tf_buffer.lookup_transform(
                self.body_reference_frame,
                self.pose_reference_frame,
                rospy.Time(0),
                rospy.Duration(0.0),
            ).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as error:
            rospy.logwarn_throttle(
                5.0, "Waiting for pose-reference TF %s <- %s: %s",
                self.body_reference_frame, self.pose_reference_frame, error)
            return None
        translation = (
            float(transform.translation.x),
            float(transform.translation.y),
            float(transform.translation.z),
        )
        orientation = self._normalize_quaternion_values((
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ))
        if any(not math.isfinite(value) for value in translation):
            rospy.logerr("Pose-reference TF has a non-finite translation")
            return None
        self.pose_reference_extrinsic = translation, orientation
        rospy.loginfo(
            "Cached pose-reference TF %s <- %s: translation=%s",
            self.body_reference_frame, self.pose_reference_frame, translation)
        return self.pose_reference_extrinsic

    def _pose_reference_to_body(self, pose):
        """Convert T_world_reference to T_world_body using T_body_reference."""
        extrinsic = self._get_pose_reference_extrinsic()
        if self.apply_pose_reference_extrinsic and extrinsic is None:
            return False
        if extrinsic is None:
            return True
        translation_body_reference, orientation_body_reference = extrinsic
        if self.origin is not None and self.origin_in_pose_reference:
            self.origin = self._reference_pose_to_body_values(
                self.origin[0], self.origin[1],
                translation_body_reference, orientation_body_reference)
            self.origin_in_pose_reference = False
            rospy.loginfo("Converted loaded INS origin to %s with static extrinsic",
                          self.body_reference_frame)
        position_world_reference = (
            pose.position.x, pose.position.y, pose.position.z)
        orientation_world_reference = (
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)
        position_world_body, orientation_world_body = self._reference_pose_to_body_values(
            position_world_reference, orientation_world_reference,
            translation_body_reference, orientation_body_reference)
        pose.position.x, pose.position.y, pose.position.z = position_world_body
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = orientation_world_body
        return True

    def _reference_pose_to_body_values(
            self, position_world_reference, orientation_world_reference,
            translation_body_reference, orientation_body_reference):
        orientation_world_body = self._normalize_quaternion_values(
            self._quaternion_multiply(
                orientation_world_reference,
                self._quaternion_conjugate(orientation_body_reference)))
        rotation_world_body = self._rotation_matrix(orientation_world_body)
        lever_world = self._matrix_vector(
            rotation_world_body, translation_body_reference)
        position_world_body = tuple(
            position_world_reference[index] - lever_world[index]
            for index in range(3))
        return position_world_body, orientation_world_body

    def _covariance(self, source, rotation=None):
        source = list(source)
        if rotation is not None:
            source = self._rotate_covariance(source, rotation)
        diagonal_indices = (0, 7, 14, 21, 28, 35)
        source_diagonal = [source[index] for index in diagonal_indices]
        has_reported_covariance = any(
            math.isfinite(value) and value > 0.0 for value in source_diagonal)
        if not has_reported_covariance:
            if not self.allow_zero_covariance:
                return None
            self.fallback_covariance_count += 1

        output = [0.0] * 36
        if has_reported_covariance and not self.diagonalize_covariance:
            for index, value in enumerate(source):
                if math.isfinite(value):
                    output[index] = value * self.covariance_scale

        floors = self.min_position_stddev + self.min_orientation_stddev
        fallbacks = self.fallback_position_stddev + self.fallback_orientation_stddev
        for axis, index in enumerate(diagonal_indices):
            value = source[index] if has_reported_covariance else fallbacks[axis] ** 2
            if not math.isfinite(value) or value <= 0.0:
                value = fallbacks[axis] ** 2
            value *= self.covariance_scale
            output[index] = max(value, floors[axis] ** 2)

        # RTAB-Map/GTSAM interprets an angular covariance >=9999 as a
        # position-only XYZ prior. This avoids treating sub-degree dual-antenna
        # attitude as a rigid LiDAR-map orientation constraint.
        if self.position_only:
            for index in diagonal_indices[3:]:
                output[index] = 10000.0

        if any(math.sqrt(output[index]) > self.max_position_stddev
               for index in diagonal_indices[:3]):
            return None
        return output

    def callback(self, message):
        self.received += 1
        if message.header.stamp.is_zero():
            rospy.logwarn_throttle(5.0, "Dropping GNSS odometry with zero timestamp")
            return
        if (self.expected_child_frame and
                message.child_frame_id != self.expected_child_frame):
            text = "GNSS child_frame_id='%s', expected '%s'" % (
                message.child_frame_id, self.expected_child_frame)
            if self.strict_child_frame:
                rospy.logwarn_throttle(5.0, "Dropping pose: %s", text)
                return
            rospy.logwarn_throttle(5.0, "%s; applying configured identity frame alias", text)
        if not self._pose_is_finite(message.pose.pose):
            rospy.logwarn_throttle(5.0, "Dropping non-finite GNSS pose")
            return

        output = PoseWithCovarianceStamped()
        output.header.stamp = (
            message.header.stamp +
            rospy.Duration.from_sec(self.output_stamp_offset_sec))
        # CoreWrapper uses this field as the measured sensor/body frame when it
        # converts the global pose to frame_id. It is intentionally not the ENU
        # world frame from Odometry.header.frame_id.
        output.header.frame_id = self.output_frame_id
        output.pose.pose = copy.deepcopy(message.pose.pose)
        if not self._normalize_quaternion(output.pose.pose):
            rospy.logwarn_throttle(5.0, "Dropping GNSS pose with invalid quaternion")
            return
        if not self._pose_reference_to_body(output.pose.pose):
            return
        rotation = self._to_local_pose(message, output.pose.pose)
        covariance = self._covariance(message.pose.covariance, rotation)
        if covariance is None:
            rospy.logwarn_throttle(5.0, "Dropping GNSS pose with invalid or excessive covariance")
            return
        output.pose.covariance = covariance
        self.pending.append(output)
        cutoff = output.header.stamp - rospy.Duration.from_sec(self.pose_delay_sec)
        while self.pending and self.pending[0].header.stamp <= cutoff:
            self.publisher.publish(self.pending.popleft())
            self.published += 1
        rospy.loginfo_throttle(
            30.0,
            "GNSS global-pose adapter published %d/%d poses (fallback covariance: %d)",
            self.published,
            self.received,
            self.fallback_covariance_count,
        )


if __name__ == "__main__":
    rospy.init_node("gnss_odom_to_global_pose")
    try:
        GnssOdomToGlobalPose()
    except (TypeError, ValueError) as error:
        rospy.logfatal("Invalid GNSS global-pose adapter configuration: %s", error)
        raise SystemExit(2)
    rospy.spin()

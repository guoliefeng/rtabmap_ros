#!/usr/bin/env python3
import math
import sys
import time

import rospy
import tf2_ros


def frame_name(value):
    return value.strip().lstrip("/")


def check_transform(buffer, target, source):
    target = frame_name(target)
    source = frame_name(source)
    if target == source:
        return True, "identity"
    try:
        transform = buffer.lookup_transform(target, source, rospy.Time(0), rospy.Duration(0.2))
        t = transform.transform.translation
        q = transform.transform.rotation
        q_norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if not math.isfinite(q_norm) or q_norm < 1e-9:
            return False, "invalid quaternion"
        return True, "xyz=(%.6f, %.6f, %.6f), q=(%.6f, %.6f, %.6f, %.6f)" % (
            t.x, t.y, t.z, q.x, q.y, q.z, q.w)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
        return False, str(exc)


def main():
    rospy.init_node("tf_preflight", anonymous=False)
    base = frame_name(rospy.get_param("~base_frame", "base_link"))
    cloud = frame_name(rospy.get_param("~cloud_frame", "base_link"))
    imu = frame_name(rospy.get_param("~imu_frame", "chcnav_msg_parser"))
    timeout = float(rospy.get_param("~timeout", 30.0))
    strict = bool(rospy.get_param("~strict", False))

    buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
    listener = tf2_ros.TransformListener(buffer)
    del listener
    deadline = time.monotonic() + max(0.0, timeout)
    last_errors = {}
    cloud_ok = False
    imu_ok = False
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        cloud_ok, cloud_detail = check_transform(buffer, base, cloud)
        imu_ok, imu_detail = check_transform(buffer, base, imu)
        if cloud_ok and imu_ok:
            rospy.loginfo("TF PASS: %s <- %s (%s)", base, cloud, cloud_detail)
            rospy.loginfo("TF PASS: %s <- %s (%s)", base, imu, imu_detail)
            return 0
        last_errors = {"cloud": cloud_detail, "imu": imu_detail}
        time.sleep(0.2)

    if not cloud_ok:
        rospy.logerr("Missing transform: %s <- %s (%s)", base, cloud, last_errors.get("cloud", "timeout"))
    if not imu_ok:
        rospy.logerr("Missing transform: %s <- %s (%s)", base, imu, last_errors.get("imu", "timeout"))
    if strict:
        return 2
    rospy.logwarn("TF preflight finished with warnings; mapping nodes were left running (strict=false).")
    return 0


if __name__ == "__main__":
    sys.exit(main())

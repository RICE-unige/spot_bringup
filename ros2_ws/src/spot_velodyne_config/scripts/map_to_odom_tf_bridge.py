#!/usr/bin/env python3
"""Publish map -> odom by aligning Spot odom with the SLAM base frame."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def _normalize_quat(q):
    norm = math.sqrt(sum(v * v for v in q))
    if norm == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(v / norm for v in q)


def _quat_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def _quat_multiply_raw(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_multiply(a, b):
    return _normalize_quat(_quat_multiply_raw(a, b))


def _quat_rotate(q, v):
    qv = (v[0], v[1], v[2], 0.0)
    rotated = _quat_multiply_raw(_quat_multiply_raw(q, qv), _quat_conjugate(q))
    return (rotated[0], rotated[1], rotated[2])


def _transform_to_tuple(transform):
    t = transform.transform.translation
    r = transform.transform.rotation
    return (t.x, t.y, t.z), _normalize_quat((r.x, r.y, r.z, r.w))


def _invert(transform_tuple):
    t, q = transform_tuple
    qi = _quat_conjugate(q)
    ti = _quat_rotate(qi, (-t[0], -t[1], -t[2]))
    return ti, qi


def _compose(a, b):
    ta, qa = a
    tb, qb = b
    rb = _quat_rotate(qa, tb)
    return (ta[0] + rb[0], ta[1] + rb[1], ta[2] + rb[2]), _quat_multiply(qa, qb)


def _clamp(value, low, high):
    return max(low, min(high, value))


def _translation_delta(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _quat_angle(a, b):
    dot = abs(sum(a[i] * b[i] for i in range(4)))
    return 2.0 * math.acos(_clamp(dot, -1.0, 1.0))


def _quat_to_rpy(q):
    x, y, z, w = _normalize_quat(q)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _quat_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return _normalize_quat(
        (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
    )


def _quat_slerp(a, b, alpha):
    alpha = _clamp(alpha, 0.0, 1.0)
    a = _normalize_quat(a)
    b = _normalize_quat(b)
    dot = sum(a[i] * b[i] for i in range(4))
    if dot < 0.0:
        b = tuple(-value for value in b)
        dot = -dot
    dot = _clamp(dot, -1.0, 1.0)

    if dot > 0.9995:
        return _normalize_quat(tuple(a[i] + alpha * (b[i] - a[i]) for i in range(4)))

    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * alpha
    sin_theta = math.sin(theta)
    scale_a = math.cos(theta) - dot * sin_theta / sin_theta_0
    scale_b = sin_theta / sin_theta_0
    return _normalize_quat(tuple(scale_a * a[i] + scale_b * b[i] for i in range(4)))


class MapToOdomTfBridge(Node):
    def __init__(self):
        super().__init__("map_to_odom_tf_bridge")
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.slam_base_frame = self.declare_parameter("slam_base_frame", "slam_body").value
        self.spot_odom_frame = self.declare_parameter("spot_odom_frame", "odom").value
        self.spot_base_frame = self.declare_parameter("spot_base_frame", "body").value
        self.publish_parent_frame = self.declare_parameter("publish_parent_frame", self.map_frame).value
        self.publish_child_frame = self.declare_parameter("publish_child_frame", self.spot_odom_frame).value
        self.lookup_timeout_sec = float(self.declare_parameter("lookup_timeout_sec", 0.2).value)
        self.sync_odom_to_slam_stamp = bool(self.declare_parameter("sync_odom_to_slam_stamp", True).value)
        self.stamp_with_slam_time = bool(self.declare_parameter("stamp_with_slam_time", True).value)
        publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 20.0).value)
        self.translation_filter_alpha = _clamp(
            float(self.declare_parameter("translation_filter_alpha", 1.0).value), 0.0, 1.0
        )
        self.rotation_filter_alpha = _clamp(
            float(self.declare_parameter("rotation_filter_alpha", 1.0).value), 0.0, 1.0
        )
        self.translation_deadband_m = max(0.0, float(self.declare_parameter("translation_deadband_m", 0.0).value))
        self.rotation_deadband_rad = max(0.0, float(self.declare_parameter("rotation_deadband_rad", 0.0).value))
        self.yaw_only = bool(self.declare_parameter("yaw_only", False).value)
        self.lock_z = bool(self.declare_parameter("lock_z", False).value)
        self.translation_z_offset_m = float(self.declare_parameter("translation_z_offset_m", 0.0).value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.warn_count = 0
        self.filtered_map_to_odom = None
        self.locked_z = None
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_transform)
        self.get_logger().info(
            "Publishing {} -> {} from {} -> {} and {} -> {}; sync_odom_to_slam_stamp={}, stamp_with_slam_time={}, filter alpha t={:.3f} r={:.3f}, deadband t={:.4f}m r={:.5f}rad, yaw_only={}, lock_z={}, z_offset={:.3f}m".format(
                self.publish_parent_frame,
                self.publish_child_frame,
                self.map_frame,
                self.slam_base_frame,
                self.spot_odom_frame,
                self.spot_base_frame,
                self.sync_odom_to_slam_stamp,
                self.stamp_with_slam_time,
                self.translation_filter_alpha,
                self.rotation_filter_alpha,
                self.translation_deadband_m,
                self.rotation_deadband_rad,
                self.yaw_only,
                self.lock_z,
                self.translation_z_offset_m,
            )
        )

    def constrain_transform(self, raw_transform):
        translation, rotation = raw_transform

        if self.lock_z:
            if self.locked_z is None:
                self.locked_z = translation[2]
                self.get_logger().info(f"Locked map -> odom z at {self.locked_z:.3f}m before offset")
            translation = (translation[0], translation[1], self.locked_z)

        if self.translation_z_offset_m != 0.0:
            translation = (translation[0], translation[1], translation[2] + self.translation_z_offset_m)

        if self.yaw_only:
            _, _, yaw = _quat_to_rpy(rotation)
            rotation = _quat_from_rpy(0.0, 0.0, yaw)

        return translation, rotation

    def filter_transform(self, raw_transform):
        if self.filtered_map_to_odom is None:
            self.filtered_map_to_odom = raw_transform
            return raw_transform

        previous_translation, previous_rotation = self.filtered_map_to_odom
        raw_translation, raw_rotation = raw_transform
        translation_delta = _translation_delta(previous_translation, raw_translation)
        rotation_delta = _quat_angle(previous_rotation, raw_rotation)

        if translation_delta <= self.translation_deadband_m:
            translation = previous_translation
        else:
            alpha = self.translation_filter_alpha
            translation = tuple(
                previous_translation[i] + alpha * (raw_translation[i] - previous_translation[i]) for i in range(3)
            )

        if rotation_delta <= self.rotation_deadband_rad:
            rotation = previous_rotation
        else:
            rotation = _quat_slerp(previous_rotation, raw_rotation, self.rotation_filter_alpha)

        self.filtered_map_to_odom = translation, rotation
        return self.filtered_map_to_odom

    def publish_transform(self):
        try:
            timeout = Duration(seconds=self.lookup_timeout_sec)
            map_to_slam_base = self.tf_buffer.lookup_transform(
                self.map_frame, self.slam_base_frame, Time(), timeout
            )
            lookup_time = Time()
            if self.sync_odom_to_slam_stamp:
                lookup_time = Time.from_msg(map_to_slam_base.header.stamp)
            odom_to_spot_base = self.tf_buffer.lookup_transform(
                self.spot_odom_frame, self.spot_base_frame, lookup_time, timeout
            )
        except TransformException as exc:
            self.warn_count += 1
            if self.warn_count == 1 or self.warn_count % 100 == 0:
                self.get_logger().warn(f"Waiting for TF inputs: {exc}")
            return

        self.warn_count = 0
        map_to_odom = _compose(_transform_to_tuple(map_to_slam_base), _invert(_transform_to_tuple(odom_to_spot_base)))
        map_to_odom = self.constrain_transform(map_to_odom)
        map_to_odom = self.filter_transform(map_to_odom)

        msg = TransformStamped()
        if self.stamp_with_slam_time:
            msg.header.stamp = map_to_slam_base.header.stamp
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.publish_parent_frame
        msg.child_frame_id = self.publish_child_frame
        msg.transform.translation.x = map_to_odom[0][0]
        msg.transform.translation.y = map_to_odom[0][1]
        msg.transform.translation.z = map_to_odom[0][2]
        msg.transform.rotation.x = map_to_odom[1][0]
        msg.transform.rotation.y = map_to_odom[1][1]
        msg.transform.rotation.z = map_to_odom[1][2]
        msg.transform.rotation.w = map_to_odom[1][3]
        self.broadcaster.sendTransform(msg)


def main():
    rclpy.init()
    node = MapToOdomTfBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Publish map -> odom by fusing slam_toolbox 2D pose with MRG height."""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


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


def _pose_to_tuple(pose_msg):
    p = pose_msg.pose.pose.position
    r = pose_msg.pose.pose.orientation
    return (p.x, p.y, p.z), _normalize_quat((r.x, r.y, r.z, r.w))


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


class HybridMapToOdomTfBridge(Node):
    def __init__(self):
        super().__init__("hybrid_map_to_odom_tf_bridge")
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.slam_base_frame = self.declare_parameter("slam_base_frame", "slam_body").value
        self.spot_odom_frame = self.declare_parameter("spot_odom_frame", "odom").value
        self.spot_base_frame = self.declare_parameter("spot_base_frame", "body").value
        self.publish_parent_frame = self.declare_parameter("publish_parent_frame", self.map_frame).value
        self.publish_child_frame = self.declare_parameter("publish_child_frame", self.spot_odom_frame).value
        self.slam_toolbox_pose_topic = self.declare_parameter(
            "slam_toolbox_pose_topic", "/slam_toolbox/pose"
        ).value
        self.slam_toolbox_base_frame = self.declare_parameter(
            "slam_toolbox_base_frame", "slam_toolbox_velodyne"
        ).value
        self.lookup_timeout_sec = float(self.declare_parameter("lookup_timeout_sec", 0.2).value)
        self.pose_timeout_sec = float(self.declare_parameter("pose_timeout_sec", 2.0).value)
        self.sync_odom_to_pose_stamp = _as_bool(self.declare_parameter("sync_odom_to_pose_stamp", False).value)
        self.stamp_with_pose_time = _as_bool(self.declare_parameter("stamp_with_pose_time", False).value)
        self.use_slam_toolbox_xy_yaw = _as_bool(
            self.declare_parameter("use_slam_toolbox_xy_yaw", True).value
        )
        self.use_slam_toolbox_tf = _as_bool(self.declare_parameter("use_slam_toolbox_tf", True).value)
        self.use_mrg_z = _as_bool(self.declare_parameter("use_mrg_z", True).value)
        self.use_mrg_roll_pitch = _as_bool(self.declare_parameter("use_mrg_roll_pitch", False).value)
        self.fallback_to_mrg = _as_bool(self.declare_parameter("fallback_to_mrg_if_pose_missing", True).value)
        self.translation_z_offset_m = float(self.declare_parameter("translation_z_offset_m", 0.0).value)
        publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 20.0).value)
        self.translation_filter_alpha = _clamp(
            float(self.declare_parameter("translation_filter_alpha", 0.8).value), 0.0, 1.0
        )
        self.rotation_filter_alpha = _clamp(
            float(self.declare_parameter("rotation_filter_alpha", 0.8).value), 0.0, 1.0
        )
        self.translation_deadband_m = max(0.0, float(self.declare_parameter("translation_deadband_m", 0.005).value))
        self.rotation_deadband_rad = max(0.0, float(self.declare_parameter("rotation_deadband_rad", 0.005).value))

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.latest_pose = None
        self.latest_pose_received_at = None
        self.filtered_map_to_odom = None
        self.warn_count = 0
        self.pose_warn_count = 0
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.slam_toolbox_pose_topic,
            self.pose_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_transform)

        self.get_logger().info(
            "Publishing {} -> {} from slam_toolbox XY/yaw topic {} and MRG {} -> {} Z; "
            "slam_toolbox TF base={}; Spot odom input is {} -> {}; MRG roll/pitch={}, fallback_to_mrg={}, "
            "filter alpha t={:.3f} r={:.3f}, deadband t={:.4f}m r={:.5f}rad".format(
                self.publish_parent_frame,
                self.publish_child_frame,
                self.slam_toolbox_pose_topic,
                self.map_frame,
                self.slam_base_frame,
                self.slam_toolbox_base_frame if self.use_slam_toolbox_tf else "disabled",
                self.spot_odom_frame,
                self.spot_base_frame,
                self.use_mrg_roll_pitch,
                self.fallback_to_mrg,
                self.translation_filter_alpha,
                self.rotation_filter_alpha,
                self.translation_deadband_m,
                self.rotation_deadband_rad,
            )
        )

    def pose_callback(self, msg):
        self.latest_pose = msg
        self.latest_pose_received_at = self.get_clock().now()

    def pose_is_fresh(self):
        if self.latest_pose is None or self.latest_pose_received_at is None:
            return False
        age = (self.get_clock().now() - self.latest_pose_received_at).nanoseconds * 1e-9
        return age <= self.pose_timeout_sec

    def hybrid_map_to_base(self, mrg_map_to_base, slam_toolbox_map_to_base=None, slam_toolbox_stamp=None):
        if slam_toolbox_map_to_base is not None:
            slam_translation, slam_rotation = slam_toolbox_map_to_base
            self.pose_warn_count = 0
        elif self.use_slam_toolbox_xy_yaw and self.pose_is_fresh():
            slam_translation, slam_rotation = _pose_to_tuple(self.latest_pose)
            slam_toolbox_stamp = self.latest_pose.header.stamp
            self.pose_warn_count = 0
        else:
            self.pose_warn_count += 1
            if not self.fallback_to_mrg:
                return None, None
            if self.pose_warn_count == 1 or self.pose_warn_count % 100 == 0:
                self.get_logger().warn("Using MRG pose while waiting for a fresh slam_toolbox pose or TF")
            return mrg_map_to_base, None

        mrg_translation, mrg_rotation = mrg_map_to_base
        _, _, slam_yaw = _quat_to_rpy(slam_rotation)

        roll = 0.0
        pitch = 0.0
        if self.use_mrg_roll_pitch:
            roll, pitch, _ = _quat_to_rpy(mrg_rotation)

        z = mrg_translation[2] if self.use_mrg_z else slam_translation[2]
        translation = (
            slam_translation[0],
            slam_translation[1],
            z + self.translation_z_offset_m,
        )
        rotation = _quat_from_rpy(roll, pitch, slam_yaw)
        return (translation, rotation), slam_toolbox_stamp

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
            mrg_map_to_base_msg = self.tf_buffer.lookup_transform(
                self.map_frame, self.slam_base_frame, Time(), timeout
            )
        except TransformException as exc:
            self.warn_count += 1
            if self.warn_count == 1 or self.warn_count % 100 == 0:
                self.get_logger().warn(f"Waiting for MRG TF input: {exc}")
            return

        mrg_map_to_base = _transform_to_tuple(mrg_map_to_base_msg)
        slam_toolbox_map_to_base = None
        slam_toolbox_stamp = None
        if self.use_slam_toolbox_xy_yaw and self.use_slam_toolbox_tf:
            try:
                slam_toolbox_tf = self.tf_buffer.lookup_transform(
                    self.map_frame, self.slam_toolbox_base_frame, Time(), Duration(seconds=self.lookup_timeout_sec)
                )
                slam_toolbox_map_to_base = _transform_to_tuple(slam_toolbox_tf)
                slam_toolbox_stamp = slam_toolbox_tf.header.stamp
            except TransformException:
                pass

        map_to_base, pose_stamp = self.hybrid_map_to_base(
            mrg_map_to_base,
            slam_toolbox_map_to_base,
            slam_toolbox_stamp,
        )
        if map_to_base is None:
            return

        try:
            timeout = Duration(seconds=self.lookup_timeout_sec)
            lookup_time = Time()
            if self.sync_odom_to_pose_stamp and pose_stamp is not None:
                lookup_time = Time.from_msg(pose_stamp)
            odom_to_spot_base = self.tf_buffer.lookup_transform(
                self.spot_odom_frame, self.spot_base_frame, lookup_time, timeout
            )
        except TransformException as exc:
            self.warn_count += 1
            if self.warn_count == 1 or self.warn_count % 100 == 0:
                self.get_logger().warn(f"Waiting for Spot odom TF input: {exc}")
            return

        self.warn_count = 0
        map_to_odom = _compose(map_to_base, _invert(_transform_to_tuple(odom_to_spot_base)))
        map_to_odom = self.filter_transform(map_to_odom)

        msg = TransformStamped()
        if self.stamp_with_pose_time and pose_stamp is not None:
            msg.header.stamp = pose_stamp
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
    node = HybridMapToOdomTfBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Publish a LIO-SAM IMU topic from Boston Dynamics Spot SDK state."""

import math
import os
import threading
import time

import grpc
import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from bosdyn.client import create_standard_sdk
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client.robot_state import RobotStateClient, RobotStateStreamingClient


def _normalize_quat(q):
    norm = math.sqrt(sum(v * v for v in q))
    if norm == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(v / norm for v in q)


def _quat_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def _quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_rotate(q, v):
    rotated = _quat_multiply(_quat_multiply(q, (v[0], v[1], v[2], 0.0)), _quat_conjugate(q))
    return (rotated[0], rotated[1], rotated[2])


def _vector3(proto):
    return (proto.x, proto.y, proto.z)


def _seconds_from_timestamp(timestamp):
    return float(timestamp.seconds) + float(timestamp.nanos) * 1e-9


def _normalize_time(seconds, nanos):
    while nanos < 0:
        seconds -= 1
        nanos += 1000000000
    while nanos >= 1000000000:
        seconds += 1
        nanos -= 1000000000
    return int(seconds), int(nanos)


class SpotSdkImuPublisher(Node):
    def __init__(self):
        super().__init__("spot_sdk_imu_publisher")

        self.hostname = self.declare_parameter("hostname", os.getenv("SPOT_HOSTNAME", "192.168.80.3")).value
        self.username = self.declare_parameter("username", os.getenv("SPOT_USERNAME", "user")).value
        self.password_file = self.declare_parameter(
            "password_file", os.getenv("SPOT_PASSWORD_FILE", "/run/spot-secrets/spot_password.txt")
        ).value
        self.password = self.declare_parameter("password", os.getenv("SPOT_PASSWORD", "")).value
        self.topic = self.declare_parameter("topic", os.getenv("LIO_SAM_IMU_TOPIC", "/lio_sam/imu")).value
        self.frame_id = self.declare_parameter("frame_id", os.getenv("SPOT_SDK_IMU_FRAME", "liosam_body")).value
        self.source_mode = self.declare_parameter(
            "source_mode", os.getenv("SPOT_SDK_IMU_SOURCE_MODE", "auto")
        ).value
        self.fallback_rate_hz = float(
            self.declare_parameter("fallback_rate_hz", float(os.getenv("SPOT_SDK_IMU_FALLBACK_RATE_HZ", "100.0"))).value
        )
        self.fallback_use_velocity_derivative = bool(
            self.declare_parameter(
                "fallback_use_velocity_derivative",
                os.getenv("SPOT_SDK_IMU_FALLBACK_USE_VELOCITY_DERIVATIVE", "True").lower() == "true",
            ).value
        )
        self.fallback_accel_filter_coeff = float(
            self.declare_parameter(
                "fallback_accel_filter_coeff",
                float(os.getenv("SPOT_SDK_IMU_FALLBACK_ACCEL_FILTER_COEFF", "0.3")),
            ).value
        )
        self.startup_delay_sec = float(
            self.declare_parameter(
                "startup_delay_sec",
                float(os.getenv("SPOT_SDK_IMU_STARTUP_DELAY_SEC", "0.0")),
            ).value
        )
        self.add_gravity = bool(
            self.declare_parameter(
                "add_gravity", os.getenv("SPOT_SDK_IMU_ADD_GRAVITY", "True").lower() == "true"
            ).value
        )
        self.gravity = float(self.declare_parameter("gravity", float(os.getenv("SPOT_SDK_IMU_GRAVITY", "9.80511"))).value)

        self.imu_pub = self.create_publisher(Imu, self.topic, 200)
        self.status_pub = self.create_publisher(String, "/lio_sam/imu_status", 10)
        self.robot = None
        self.state_client = None
        self.streaming_client = None
        self.previous_fallback_time = None
        self.previous_linear_velocity_odom = None
        self.filtered_linear_acceleration_body = None
        self.stop_event = threading.Event()
        self.worker = threading.Thread(target=self.run, daemon=True)

        self.get_logger().info(
            f"Publishing Spot SDK IMU to {self.topic} in frame {self.frame_id} using source_mode={self.source_mode}"
        )
        self.worker.start()

    def destroy_node(self):
        self.stop_event.set()
        if self.worker.is_alive():
            self.worker.join(timeout=2.0)
        return super().destroy_node()

    def run(self):
        while not self.stop_event.is_set() and rclpy.ok():
            try:
                self.connect()
                if self.source_mode in ("auto", "stream"):
                    if self.try_streaming_imu():
                        return
                    if self.source_mode == "stream":
                        self.get_logger().error("Spot SDK inertial stream stopped and fallback is disabled.")
                        return
                self.run_kinematic_fallback()
            except Exception as exc:
                self.get_logger().error(f"Spot SDK IMU publisher connection failed: {exc}")
                self.publish_status(f"error: {exc}")
                self.robot = None
                self.state_client = None
                self.streaming_client = None
                self.stop_event.wait(5.0)

    def connect(self):
        password = self.password
        if not password and self.password_file:
            with open(self.password_file, "r", encoding="utf-8") as password_handle:
                password = password_handle.read().strip()
        if not password:
            raise RuntimeError("No Spot password available from SPOT_PASSWORD or SPOT_PASSWORD_FILE")

        sdk = create_standard_sdk("spot-lio-sam-imu", [RobotStateStreamingClient])
        self.robot = sdk.create_robot(self.hostname)
        self.robot.authenticate(self.username, password)
        self.robot.time_sync.wait_for_sync(timeout_sec=10)
        self.state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        self.streaming_client = self.robot.ensure_client(RobotStateStreamingClient.default_service_name)
        self.get_logger().info("Connected to Spot SDK and established time sync.")

    def robot_time_to_ros_time(self, robot_timestamp):
        clock_skew = self.robot.time_sync.endpoint.clock_skew
        seconds = robot_timestamp.seconds - clock_skew.seconds
        nanos = robot_timestamp.nanos - clock_skew.nanos
        seconds, nanos = _normalize_time(seconds, nanos)
        return Time(sec=seconds, nanosec=nanos)

    def add_gravity_if_requested(self, q_odom_link, acceleration_link):
        if not self.add_gravity:
            return acceleration_link
        gravity_link = _quat_rotate(_quat_conjugate(q_odom_link), (0.0, 0.0, self.gravity))
        return (
            acceleration_link[0] + gravity_link[0],
            acceleration_link[1] + gravity_link[1],
            acceleration_link[2] + gravity_link[2],
        )

    def make_imu_msg(self, stamp, q_odom_link, angular_velocity_link, acceleration_link):
        q_odom_link = _normalize_quat(q_odom_link)
        acceleration_link = self.add_gravity_if_requested(q_odom_link, acceleration_link)

        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.orientation.x = q_odom_link[0]
        msg.orientation.y = q_odom_link[1]
        msg.orientation.z = q_odom_link[2]
        msg.orientation.w = q_odom_link[3]
        msg.angular_velocity.x = angular_velocity_link[0]
        msg.angular_velocity.y = angular_velocity_link[1]
        msg.angular_velocity.z = angular_velocity_link[2]
        msg.linear_acceleration.x = acceleration_link[0]
        msg.linear_acceleration.y = acceleration_link[1]
        msg.linear_acceleration.z = acceleration_link[2]
        msg.orientation_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
        msg.angular_velocity_covariance = [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.0004]
        msg.linear_acceleration_covariance = [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]
        return msg

    def try_streaming_imu(self):
        self.publish_status("starting Spot inertial stream")
        try:
            saw_packet = False
            for response in self.streaming_client.get_robot_state_stream():
                if self.stop_event.is_set():
                    return True
                inertial_state = response.inertial_state
                if not saw_packet and inertial_state.packets:
                    saw_packet = True
                    self.get_logger().info(
                        "Using Spot SDK inertial stream: "
                        f"{len(inertial_state.packets)} packets/update, "
                        f"packet_rate={inertial_state.packet_rate}, "
                        f"mounting_link={inertial_state.mounting_link_name}"
                    )
                    self.publish_status("using Spot SDK inertial stream")
                for packet in inertial_state.packets:
                    q = packet.odom_rot_link
                    imu_msg = self.make_imu_msg(
                        self.robot_time_to_ros_time(packet.timestamp),
                        (q.x, q.y, q.z, q.w),
                        _vector3(packet.angular_velocity_rt_odom_in_link_frame),
                        _vector3(packet.acceleration_rt_odom_in_link_frame),
                    )
                    self.imu_pub.publish(imu_msg)
            return True
        except grpc.RpcError as exc:
            if exc.code() == grpc.StatusCode.PERMISSION_DENIED and self.source_mode == "auto":
                self.get_logger().warn(
                    "Spot inertial stream is unavailable: permission denied. "
                    "Falling back to SDK kinematic-state IMU approximation."
                )
                self.publish_status("fallback: inertial stream permission denied")
                return False
            raise

    def run_kinematic_fallback(self):
        self.get_logger().warn(
            "Using SDK kinematic-state IMU fallback. This is adequate for wiring tests, "
            "but LIO-SAM should use the licensed Spot inertial stream or an external IMU for field data."
        )
        self.publish_status("using SDK kinematic fallback")
        period = 1.0 / max(self.fallback_rate_hz, 1.0)
        startup_deadline = time.monotonic() + max(self.startup_delay_sec, 0.0)
        warming_up = self.startup_delay_sec > 0.0

        while not self.stop_event.is_set() and rclpy.ok():
            loop_start = time.monotonic()
            try:
                state = self.state_client.get_robot_state()
                now = time.monotonic()
                if now < startup_deadline:
                    self.publish_fallback_imu(state, publish=False)
                else:
                    if warming_up:
                        self.get_logger().info("Fallback IMU warm-up complete; publishing /lio_sam/imu.")
                        warming_up = False
                    self.publish_fallback_imu(state)
            except Exception as exc:
                self.get_logger().warn(f"Failed to publish fallback IMU sample: {exc}")
            elapsed = time.monotonic() - loop_start
            time.sleep(max(0.0, period - elapsed))

    def publish_fallback_imu(self, robot_state, publish=True):
        kinematic_state = robot_state.kinematic_state
        odom_tform_body = get_a_tform_b(kinematic_state.transforms_snapshot, ODOM_FRAME_NAME, BODY_FRAME_NAME)
        q = odom_tform_body.rot
        q_odom_body = _normalize_quat((q.x, q.y, q.z, q.w))
        velocity = kinematic_state.velocity_of_body_in_odom
        linear_velocity_odom = _vector3(velocity.linear)
        angular_velocity_odom = _vector3(velocity.angular)

        stamp = self.get_clock().now().to_msg()
        current_time = time.monotonic()
        if self.previous_fallback_time is None:
            acceleration_odom = (0.0, 0.0, 0.0)
        elif self.fallback_use_velocity_derivative:
            dt = max(current_time - self.previous_fallback_time, 1e-3)
            acceleration_odom = (
                (linear_velocity_odom[0] - self.previous_linear_velocity_odom[0]) / dt,
                (linear_velocity_odom[1] - self.previous_linear_velocity_odom[1]) / dt,
                (linear_velocity_odom[2] - self.previous_linear_velocity_odom[2]) / dt,
            )
        else:
            acceleration_odom = (0.0, 0.0, 0.0)
        self.previous_fallback_time = current_time
        self.previous_linear_velocity_odom = linear_velocity_odom

        q_body_odom = _quat_conjugate(q_odom_body)
        angular_velocity_body = _quat_rotate(q_body_odom, angular_velocity_odom)
        acceleration_body = _quat_rotate(q_body_odom, acceleration_odom)
        if self.fallback_use_velocity_derivative:
            coeff = min(max(self.fallback_accel_filter_coeff, 0.0), 1.0)
            if self.filtered_linear_acceleration_body is None:
                self.filtered_linear_acceleration_body = acceleration_body
            else:
                previous = self.filtered_linear_acceleration_body
                self.filtered_linear_acceleration_body = (
                    coeff * acceleration_body[0] + (1.0 - coeff) * previous[0],
                    coeff * acceleration_body[1] + (1.0 - coeff) * previous[1],
                    coeff * acceleration_body[2] + (1.0 - coeff) * previous[2],
                )
            acceleration_body = self.filtered_linear_acceleration_body
        else:
            self.filtered_linear_acceleration_body = None
        if publish:
            self.imu_pub.publish(self.make_imu_msg(stamp, q_odom_body, angular_velocity_body, acceleration_body))

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = SpotSdkImuPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

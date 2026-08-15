#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import math
from typing import Optional, Tuple, List

import cv2
import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

# TF2
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)


class QrTracker(Node):
    """Tracks a QR tag in camera images and performs fine-positioning near a Nav2 goal."""

    def __init__(self) -> None:
        super().__init__("qr_tracker_node")

        # ---------- Parameters ----------
        self._declare_params()

        # ---------- State ----------
        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()

        self.last_seen_stamp = self.get_clock().now()
        self.lost_timeout = Duration(seconds=1.0)

        self.success_active: bool = False
        self.last_goal_pose: Optional[PoseStamped] = None

        # Publish done only on change
        self._finepos_done_state: Optional[bool] = None

        # ---------- QoS ----------
        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.image_queue_size,
        )
        flag_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        goal_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---------- TF ----------
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------- Interfaces ----------
        self.image_sub = self.create_subscription(Image, self.camera_topic, self.image_cb, cam_qos)
        self.flag_sub = self.create_subscription(Bool, self.success_flag_topic, self.success_flag_cb, flag_qos)
        self.goal_sub = self.create_subscription(PoseStamped, self.goal_pose_topic, self.goal_pose_cb, goal_qos)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.done_pub = self.create_publisher(Bool, self.done_topic, 10)

        # Safety timer (stop/clear done if disabled)
        self.create_timer(0.2, self.safety_timer_cb)

        self.get_logger().info(
            f"qr_tracker: camera={self.camera_topic}, cmd_vel={self.cmd_vel_topic}, "
            f"start_flag={self.success_flag_topic}, goal_pose={self.goal_pose_topic}, "
            f"done_topic={self.done_topic}, base_frame={self.base_frame_id}"
        )

    # ==================== Parameters ====================

    def _declare_params(self) -> None:
        # Topics
        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("success_flag_topic", "/nav2/success_flag")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("done_topic", "/finepositioning/done")
        self.declare_parameter("image_queue_size", 5)

        # TF & goal-proximity stop
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("stop_distance_thresh_m", 0.15)

        # QR measurement model
        self.declare_parameter("qr_size_m", 0.16)     # physical edge length of QR (meters)
        self.declare_parameter("fx", 0.0)             # focal length (pixels); if 0, derive from HFOV
        self.declare_parameter("camera_hfov_deg", 70.0)

        # Motion control gains/limits
        self.declare_parameter("target_distance_m", 0.7)
        self.declare_parameter("linear_kp", 0.8)
        self.declare_parameter("angular_kp", 0.006)   # per pixel
        self.declare_parameter("max_linear_x", 0.25)
        self.declare_parameter("max_angular_z", 0.6)
        self.declare_parameter("search_angular_z", 0.2)
        self.declare_parameter("invert_x", False)

        # "Done" decision thresholds
        self.declare_parameter("done_yaw_thresh_deg", 5.0)
        self.declare_parameter("done_range_tolerance_m", 0.05)

        # Read
        gp = self.get_parameter
        self.camera_topic = gp("camera_topic").value
        self.cmd_vel_topic = gp("cmd_vel_topic").value
        self.success_flag_topic = gp("success_flag_topic").value
        self.goal_pose_topic = gp("goal_pose_topic").value
        self.done_topic = gp("done_topic").value
        self.image_queue_size = int(gp("image_queue_size").value)

        self.base_frame_id = gp("base_frame_id").value
        self.stop_distance_thresh_m = float(gp("stop_distance_thresh_m").value)

        self.qr_size_m = float(gp("qr_size_m").value)
        self.fx_param = float(gp("fx").value)
        self.camera_hfov_deg = float(gp("camera_hfov_deg").value)

        self.target_distance_m = float(gp("target_distance_m").value)
        self.linear_kp = float(gp("linear_kp").value)
        self.angular_kp = float(gp("angular_kp").value)
        self.max_linear_x = float(gp("max_linear_x").value)
        self.max_angular_z = float(gp("max_angular_z").value)
        self.search_angular_z = float(gp("search_angular_z").value)
        self.invert_x = bool(gp("invert_x").value)

        self.done_yaw_thresh_deg = float(gp("done_yaw_thresh_deg").value)
        self.done_range_tolerance_m = float(gp("done_range_tolerance_m").value)

    # ==================== Subscriptions ====================

    def success_flag_cb(self, msg: Bool) -> None:
        prev = self.success_active
        self.success_active = bool(msg.data)
        if self.success_active and not prev:
            self.get_logger().info("Nav2 success_flag=True → QR tracking ENABLED.")
            self._set_done(False)  # reset when (re)entering fine positioning
        elif not self.success_active and prev:
            self.get_logger().info("Nav2 success_flag=False → QR tracking DISABLED; stopping and clearing done.")
            self._publish_stop()
            self._set_done(False)

    def goal_pose_cb(self, msg: PoseStamped) -> None:
        self.last_goal_pose = msg  # keep most recent

    def image_cb(self, msg: Image) -> None:
        # Gate by Nav2 flag
        if not self.success_active:
            return

        # Estimate from image and TF
        qr_visible, bearing_rad, z_est, twist = self._compute_control_from_qr(msg)
        near_goal = self._near_goal_threshold()

        # Unified DONE decision
        done_ok = self._compute_done(qr_visible, bearing_rad, z_est, near_goal)
        if done_ok:
            self._publish_stop()
            self._set_done(True)
            self.get_logger().info("[DONE] Fine positioning complete.")
            return

        # Not done → act
        self._set_done(False)

        if qr_visible:
            # --- NEW LOG: robot position w.r.t. QR (camera frame) ---
            # lateral x (right +) and forward z using simple pinhole geometry
            x_lateral = z_est * math.tan(bearing_rad)
            bearing_deg = math.degrees(bearing_rad)
            delta_z = z_est - self.target_distance_m
            self.get_logger().info(
                f"[QR SEEN] pos_cam: x={x_lateral:.3f} m, z={z_est:.3f} m | "
                f"bearing={bearing_deg:.2f}° | Δz={delta_z:+.3f} m"
            )

            # Keep adjusting even near_goal (distance/heading gates handled inside control)
            self.cmd_pub.publish(twist)
            self.last_seen_stamp = self.get_clock().now()
            if near_goal:
                self._info_throttle(0.5, f"[FINE-ADJUST] bearing={bearing_deg:.2f}°, z={z_est:.2f}m")
            else:
                self._info_throttle(0.5, f"[APPROACH] bearing={bearing_deg:.2f}°, z={z_est:.2f}m")
            return

        # QR lost → search
        elapsed = (self.get_clock().now() - self.last_seen_stamp).nanoseconds * 1e-9
        self._warn_throttle(1.0, f"[SEARCH] QR not visible for {elapsed:.2f}s ...")

        if (self.get_clock().now() - self.last_seen_stamp) > self.lost_timeout:
            if abs(self.search_angular_z) > 1e-6:
                spin = Twist()
                spin.angular.z = self._clip(self.search_angular_z, -self.max_angular_z, self.max_angular_z)
                self.cmd_pub.publish(spin)
                self._warn_throttle(1.0, f"[SEARCH] spinning: {spin.angular.z:.3f} rad/s")
            else:
                self._publish_stop()
        else:
            self._publish_stop()

    # ==================== Core logic ====================

    def _measure_qr(self, img_msg: Image) -> Tuple[bool, float, float]:
        """Returns (qr_visible, bearing_rad, z_est_m)."""
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        h, w = cv_img.shape[:2]

        fx = self.fx_param
        if fx <= 0.0:
            hfov = math.radians(self.camera_hfov_deg)
            fx = (w / 2.0) / math.tan(hfov / 2.0)

        # Robustly handle OpenCV API differences
        points = self._detect_multi_points(cv_img)
        best = self._pick_best_qr(points)
        if best is None:
            return False, 0.0, 0.0

        cx, _, px_size = self._polygon_center_and_size(best)
        x_err_px = cx - (w / 2.0)
        bearing_rad = math.atan2(x_err_px, fx)  # sign: +right, -left

        z_est = (fx * self.qr_size_m) / max(px_size, 1e-6)
        return True, bearing_rad, z_est

    def _compute_control_from_qr(self, img_msg: Image) -> Tuple[bool, float, float, Twist]:
        """Returns (qr_visible, bearing_rad, z_est_m, twist_cmd)."""
        twist = Twist()
        qr_visible, bearing_rad, z_est = self._measure_qr(img_msg)
        if not qr_visible:
            return False, 0.0, 0.0, twist

        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        w = cv_img.shape[1]
        fx = self.fx_param if self.fx_param > 0.0 else (w / 2.0) / math.tan(math.radians(self.camera_hfov_deg) / 2.0)

        # Recompute pixel error for P-control
        x_err_px = math.tan(bearing_rad) * fx

        ang_cmd = -self.angular_kp * x_err_px
        ang_cmd = self._clip(ang_cmd, -self.max_angular_z, self.max_angular_z)

        dist_err = z_est - self.target_distance_m
        lin_cmd = self.linear_kp * dist_err
        if self.invert_x:
            lin_cmd = -lin_cmd
        lin_cmd = self._clip(lin_cmd, -self.max_linear_x, self.max_linear_x)

        # Prioritize heading when off by >10°
        if abs(bearing_rad) > math.radians(10.0):
            lin_cmd *= 0.3

        twist.linear.x = lin_cmd
        twist.angular.z = ang_cmd
        return True, bearing_rad, z_est, twist

    def _compute_done(self, qr_visible: bool, bearing_rad: float, z_est: float, near_goal: bool) -> bool:
        """
        Conditions (all True):
          1) success_active
          2) near_goal (TF distance <= stop_distance_thresh_m)
          3) qr_visible
          4) |bearing| <= done_yaw_thresh_deg
          5) |z_est - target_distance_m| <= done_range_tolerance_m
        """
        if not (self.success_active and near_goal and qr_visible):
            return False
        yaw_ok = abs(bearing_rad) <= math.radians(self.done_yaw_thresh_deg)
        range_ok = abs(z_est - self.target_distance_m) <= self.done_range_tolerance_m
        return yaw_ok and range_ok

    # ==================== TF / Proximity ====================

    def _near_goal_threshold(self) -> bool:
        """True if planar distance(base → goal) <= stop_distance_thresh_m, in goal frame."""
        if self.last_goal_pose is None:
            return False

        goal_frame = self.last_goal_pose.header.frame_id or "map"
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame=goal_frame,
                source_frame=self.base_frame_id,
                time=Time(),  # latest
                timeout=Duration(seconds=0.05),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return False

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y

        gx = self.last_goal_pose.pose.position.x
        gy = self.last_goal_pose.pose.position.y

        dist = math.hypot(gx - rx, gy - ry)
        return dist <= self.stop_distance_thresh_m

    # ==================== Utils ====================

    def _detect_multi_points(self, cv_img) -> Optional[np.ndarray]:
        """
        Calls OpenCV's QRCodeDetector.detectAndDecodeMulti robustly across versions.
        Returns points (N x 4 x 2) or None.
        """
        try:
            # OpenCV >= 4.5 typical: retval, decoded_info, points, straight_qrcode
            _, _, points, _ = self.detector.detectAndDecodeMulti(cv_img)
            return points
        except Exception:
            # Fallback: some builds return 3-tuple
            try:
                _, points, _ = self.detector.detectAndDecodeMulti(cv_img)  # type: ignore[misc]
                return points
            except Exception:
                return None

    @staticmethod
    def _polygon_center_and_size(poly: np.ndarray) -> Tuple[float, float, float]:
        poly = np.array(poly, dtype=np.float32).reshape(4, 2)
        cx = float(np.mean(poly[:, 0]))
        cy = float(np.mean(poly[:, 1]))
        edges: List[float] = []
        for i in range(4):
            j = (i + 1) % 4
            edges.append(float(np.linalg.norm(poly[i] - poly[j])))
        return cx, cy, float(np.mean(edges))

    @staticmethod
    def _pick_best_qr(points: Optional[np.ndarray]) -> Optional[np.ndarray]:
        if points is None or len(points) == 0:
            return None
        best_idx, best_sz = -1, -1.0
        for i, poly in enumerate(points):
            poly = np.array(poly, dtype=np.float32).reshape(4, 2)
            edges: List[float] = []
            for j in range(4):
                k = (j + 1) % 4
                edges.append(float(np.linalg.norm(poly[j] - poly[k])))
            sz = float(np.mean(edges))
            if sz > best_sz:
                best_sz, best_idx = sz, i
        return np.array(points[best_idx], dtype=np.float32).reshape(4, 2)

    @staticmethod
    def _clip(x: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, x))

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _set_done(self, state: bool) -> None:
        """Publish /finepositioning/done only when the state changes."""
        if self._finepos_done_state is None or self._finepos_done_state != state:
            self.done_pub.publish(Bool(data=state))
            self._finepos_done_state = state
            self._info_throttle(0.5, f"finepositioning/done → {state}")

    # ---------- Throttled logging helpers ----------

    def _info_throttle(self, period_s: float, msg: str) -> None:
        self.get_logger().info(msg, throttle_duration_sec=period_s)  # rclpy >= 1.1

    def _warn_throttle(self, period_s: float, msg: str) -> None:
        self.get_logger().warn(msg, throttle_duration_sec=period_s)

    # ==================== Node lifecycle ====================

    def safety_timer_cb(self) -> None:
        """Clear 'done' when disabled. (Stopping is handled in other places.)"""
        if not self.success_active:
            self._set_done(False)


def main() -> None:
    rclpy.init()
    node = QrTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

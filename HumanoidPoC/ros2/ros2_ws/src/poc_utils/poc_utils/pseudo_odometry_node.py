#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import collections
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

import tf_transformations


class PseudoOdometryNode(Node):
    def __init__(self):
        super().__init__("pseudo_odometry_node")

        # ---- 設定パラメータ ----
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        if use_sim_time:
            self.get_logger().info("Using simulation time (use_sim_time=True)")

        # 平滑化窓長
        self.declare_parameter('avg_window', 5)
        self.avg_window = int(self.get_parameter('avg_window').value)
        self.avg_window = max(1, self.avg_window)
        self.get_logger().info(f"Delta smoothing window size: {self.avg_window}")

        # ---- TF関連 ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # フレーム名
        self.map_frame = "map"
        self.odom_frame = "odom"
        self.base_frame = "base_link"

        # 状態
        self.use_real_map_tf = False
        self.prev_map_to_odom = None
        self.odom_T = np.eye(4)
        self.delta_buf = collections.deque(maxlen=self.avg_window)

        # 起動直後：ツリー確立（恒等）
        self.publish_identity_tf(self.map_frame, self.odom_frame)
        self.publish_identity_tf(self.odom_frame, self.base_frame)

        # タイマー
        self.timer = self.create_timer(0.5, self.timer_callback)  # 20 Hz
        self.get_logger().info("PseudoOdometryNode started.")

    # ------------- 基本ユーティリティ -------------
    def publish_identity_tf(self, parent, child):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def tf_to_matrix(self, transform):
        t = transform.translation
        q = transform.rotation
        T = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0, 3], T[1, 3], T[2, 3] = t.x, t.y, t.z
        return T

    def matrix_to_tf(self, T):
        trans = [T[0, 3], T[1, 3], T[2, 3]]
        quat = tf_transformations.quaternion_from_matrix(T)  # [x,y,z,w]
        return trans, quat

    def publish_tf(self, T, parent, child):
        trans, quat = self.matrix_to_tf(T)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(trans[0])
        t.transform.translation.y = float(trans[1])
        t.transform.translation.z = float(trans[2])
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(t)

    def publish_odom_msg(self, T):
        trans, quat = self.matrix_to_tf(T)
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = float(trans[0])
        msg.pose.pose.position.y = float(trans[1])
        msg.pose.pose.position.z = float(trans[2])
        msg.pose.pose.orientation.x = float(quat[0])
        msg.pose.pose.orientation.y = float(quat[1])
        msg.pose.pose.orientation.z = float(quat[2])
        msg.pose.pose.orientation.w = float(quat[3])
        # 速度は未設定（必要なら追加）
        self.odom_pub.publish(msg)

    # ------------- 平滑化（移動平均） -------------
    def average_transforms(self, transforms):
        """ΔT行列の配列を、並進は算術平均、回転は四元数平均で平滑化"""
        if not transforms:
            return np.eye(4)
        if len(transforms) == 1:
            return transforms[0]

        # 平均並進
        translations = np.array([T[0:3, 3] for T in transforms])
        mean_t = np.mean(translations, axis=0)

        # 回転行列 -> 四元数（[x,y,z,w]）へ
        quats = []
        for T in transforms:
            q = tf_transformations.quaternion_from_matrix(T)
            quats.append(np.array(q, dtype=np.float64))

        # 四元数の符号揃え（最初の四元数に対して内積が負なら反転）
        ref = quats[0]
        aligned = []
        for q in quats:
            if np.dot(q, ref) < 0.0:
                aligned.append(-q)
            else:
                aligned.append(q)
        aligned = np.stack(aligned, axis=0)

        # 平均→正規化
        mean_q = np.mean(aligned, axis=0)
        norm = np.linalg.norm(mean_q)
        if norm < 1e-12:
            mean_q = ref  # 退避
        else:
            mean_q = mean_q / norm

        # 四元数→回転行列
        R4 = tf_transformations.quaternion_matrix(mean_q)  # 4x4
        mean_R = R4[0:3, 0:3]

        # 平均変換ΔT
        T_avg = np.eye(4)
        T_avg[0:3, 0:3] = mean_R
        T_avg[0:3, 3] = mean_t
        return T_avg

    # ------------- メインループ -------------
    def timer_callback(self):
        # map→odom を試す
        try:
            tf_map_odom = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.odom_frame,
                Time(),  # 最新
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            if not self.use_real_map_tf:
                self.use_real_map_tf = True
                self.get_logger().info("Detected map→odom TF — switched to real updates.")
        except Exception:
            # まだmap→odomが無い：恒等でツリー維持（Nav2/hdlの初期化待ち）
            if not self.use_real_map_tf:
                self.publish_identity_tf(self.map_frame, self.odom_frame)
                self.publish_tf(self.odom_T, self.odom_frame, self.base_frame)
                self.publish_odom_msg(self.odom_T)
            return

        # 実運用モード：差分→平滑→積算
        if self.prev_map_to_odom is None:
            self.prev_map_to_odom = tf_map_odom
            return

        current_T = self.tf_to_matrix(tf_map_odom.transform)
        prev_T = self.tf_to_matrix(self.prev_map_to_odom.transform)
        delta_T = np.linalg.inv(prev_T) @ current_T

        # ΔT をバッファし、移動平均
        self.delta_buf.append(delta_T)
        smoothed_delta = self.average_transforms(list(self.delta_buf))

        # odom→base_link を更新（積算）
        self.odom_T = self.odom_T @ smoothed_delta

        # 出力
        self.publish_tf(self.odom_T, self.odom_frame, self.base_frame)
        self.publish_odom_msg(self.odom_T)

        # 次回比較用
        self.prev_map_to_odom = tf_map_odom


def main(args=None):
    rclpy.init(args=args)
    node = PseudoOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

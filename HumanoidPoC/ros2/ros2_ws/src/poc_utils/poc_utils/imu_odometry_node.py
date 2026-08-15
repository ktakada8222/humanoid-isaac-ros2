#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# import math
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Quaternion, Twist, TransformStamped
# from tf_transformations import quaternion_from_euler
# from tf2_ros import TransformBroadcaster

# class ImuOdometryNode(Node):
#     def __init__(self):
#         super().__init__('imu_odometry_node')

#         # ===== パラメータ =====
#         self.declare_parameter('odom_frame', 'odom')
#         self.declare_parameter('base_frame', 'base_link')
#         self.declare_parameter('topic_name', '/imu/data')
#         self.declare_parameter('odom_topic', '/odom')
#         # self.declare_parameter('use_sim_time', False)

#         self.odom_frame = self.get_parameter('odom_frame').value
#         self.base_frame = self.get_parameter('base_frame').value
#         imu_topic = self.get_parameter('topic_name').value
#         odom_topic = self.get_parameter('odom_topic').value

#         # ===== パブリッシャ / サブスクライバ =====
#         self.sub_imu = self.create_subscription(Imu, imu_topic, self.imu_callback, 10)
#         self.pub_odom = self.create_publisher(Odometry, odom_topic, 10)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # ===== 内部状態 =====
#         self.last_time = None
#         self.x = 0.0
#         self.y = 0.0
#         self.z = 0.0
#         self.vx = 0.0
#         self.vy = 0.0
#         self.vz = 0.0
#         self.roll = 0.0
#         self.pitch = 0.0
#         self.yaw = 0.0

#         self.get_logger().info('✅ IMU Odometry Node with TF started.')

#     def imu_callback(self, msg: Imu):
#         # --- 時間計算 ---
#         current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
#         if self.last_time is None:
#             self.last_time = current_time
#             return

#         dt = current_time - self.last_time
#         if dt <= 0:
#             return
#         self.last_time = current_time

#         # --- 角速度による姿勢更新 ---
#         self.roll += msg.angular_velocity.x * dt
#         self.pitch += msg.angular_velocity.y * dt
#         self.yaw += msg.angular_velocity.z * dt

#         # --- 加速度を重力補正して座標変換 ---
#         ax = msg.linear_acceleration.x
#         ay = msg.linear_acceleration.y
#         az = msg.linear_acceleration.z - 9.81  # 重力除去

#         cr, sr = math.cos(self.roll), math.sin(self.roll)
#         cp, sp = math.cos(self.pitch), math.sin(self.pitch)
#         cy, sy = math.cos(self.yaw), math.sin(self.yaw)

#         ax_w = ax * (cp * cy) + ay * (sr * sp * cy - cr * sy) + az * (cr * sp * cy + sr * sy)
#         ay_w = ax * (cp * sy) + ay * (sr * sp * sy + cr * cy) + az * (cr * sp * sy - sr * cy)
#         az_w = ax * (-sp) + ay * (sr * cp) + az * (cr * cp)

#         # --- 速度・位置積分 ---
#         self.vx += ax_w * dt
#         self.vy += ay_w * dt
#         self.vz += az_w * dt

#         self.x += self.vx * dt
#         self.y += self.vy * dt
#         self.z += self.vz * dt

#         # --- オドメトリメッセージ生成 ---
#         quat = quaternion_from_euler(self.roll, self.pitch, self.yaw)
#         odom_msg = Odometry()
#         odom_msg.header.stamp = msg.header.stamp
#         odom_msg.header.frame_id = self.odom_frame
#         odom_msg.child_frame_id = self.base_frame

#         odom_msg.pose.pose.position.x = self.x
#         odom_msg.pose.pose.position.y = self.y
#         odom_msg.pose.pose.position.z = self.z
#         odom_msg.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

#         odom_msg.twist.twist.linear.x = self.vx
#         odom_msg.twist.twist.linear.y = self.vy
#         odom_msg.twist.twist.linear.z = self.vz
#         odom_msg.twist.twist.angular = msg.angular_velocity

#         self.pub_odom.publish(odom_msg)

#         # --- TF 送信 (odom -> base_link) ---
#         t = TransformStamped()
#         t.header.stamp = msg.header.stamp
#         t.header.frame_id = self.odom_frame
#         t.child_frame_id = self.base_frame
#         t.transform.translation.x = self.x
#         t.transform.translation.y = self.y
#         t.transform.translation.z = self.z
#         t.transform.rotation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
#         self.tf_broadcaster.sendTransform(t)


# def main(args=None):
#     rclpy.init(args=args)
#     node = ImuOdometryNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

# ========= Quaternion utilities (no external deps) =========
def q_normalize(q):
    n = math.sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3])
    if n == 0.0:
        return [1.0, 0.0, 0.0, 0.0]
    return [q[0]/n, q[1]/n, q[2]/n, q[3]/n]

def q_multiply(q, r):
    # (w,x,y,z) * (w,x,y,z)
    return [
        q[0]*r[0] - q[1]*r[1] - q[2]*r[2] - q[3]*r[3],
        q[0]*r[1] + q[1]*r[0] + q[2]*r[3] - q[3]*r[2],
        q[0]*r[2] - q[1]*r[3] + q[2]*r[0] + q[3]*r[1],
        q[0]*r[3] + q[1]*r[2] - q[2]*r[1] + q[3]*r[0],
    ]

def q_from_axis_angle(axis, angle):
    ax, ay, az = axis
    n = math.sqrt(ax*ax+ay*ay+az*az)
    if n == 0.0:
        return [1.0, 0.0, 0.0, 0.0]
    s = math.sin(angle/2.0)/n
    return [math.cos(angle/2.0), ax*s, ay*s, az*s]

def q_from_euler(roll, pitch, yaw):
    cr = math.cos(roll/2.0);  sr = math.sin(roll/2.0)
    cp = math.cos(pitch/2.0); sp = math.sin(pitch/2.0)
    cy = math.cos(yaw/2.0);   sy = math.sin(yaw/2.0)
    # Z * Y * R
    return [
        cy*cp*cr + sy*sp*sr,
        cy*cp*sr - sy*sp*cr,
        sy*cp*sr + cy*sp*cr,
        sy*cp*cr - cy*sp*sr,
    ]

def q_to_euler(q):
    w, x, y, z = q
    # roll (x-axis rotation)
    t0 = +2.0 * (w*x + y*z)
    t1 = +1.0 - 2.0 * (x*x + y*y)
    roll = math.atan2(t0, t1)
    # pitch (y-axis)
    t2 = +2.0 * (w*y - z*x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    # yaw (z-axis)
    t3 = +2.0 * (w*z + x*y)
    t4 = +1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

def q_rotate_vec(q, v):
    # rotate v (3,) by q (w,x,y,z)
    w, x, y, z = q
    # q * (0,v) * q^{-1}
    # use optimized formula:
    # t = 2 * cross(q_xyz, v)
    # v' = v + w * t + cross(q_xyz, t)
    qv = np.array([x, y, z])
    t = 2.0 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)

def q_slerp(q0, q1, t):
    # simple slerp; t in [0,1]
    w0,x0,y0,z0 = q0
    w1,x1,y1,z1 = q1
    dot = w0*w1 + x0*x1 + y0*y1 + z0*z1
    if dot < 0.0:
        w1,x1,y1,z1 = -w1,-x1,-y1,-z1
        dot = -dot
    if dot > 0.9995:
        # nearly linear
        w = w0 + t*(w1-w0)
        x = x0 + t*(x1-x0)
        y = y0 + t*(y1-y0)
        z = z0 + t*(z1-z0)
        return q_normalize([w,x,y,z])
    theta_0 = math.acos(dot)
    sin_0 = math.sin(theta_0)
    theta = theta_0 * t
    sin_t = math.sin(theta)
    s0 = math.cos(theta) - dot * sin_t / sin_0
    s1 = sin_t / sin_0
    return [s0*w0 + s1*w1, s0*x0 + s1*x1, s0*y0 + s1*y1, s0*z0 + s1*z1]

# ============================================================

class ImuOdomAdv(Node):
    def __init__(self):
        super().__init__("imu_odometry_adv")

        # ---- Parameters ----
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("gravity", 9.81)
        self.declare_parameter("alpha_complementary", 0.98)  # 近いほどジャイロ優先
        self.declare_parameter("calib_duration", 2.0)        # [s] 初期静止キャリブ時間
        self.declare_parameter("gyro_thresh", 0.03)          # [rad/s] 静止判定
        self.declare_parameter("acc_thresh", 0.2)            # [m/s^2] |‖a‖-g|
        self.declare_parameter("stationary_min_duration", 0.2)  # [s]
        self.declare_parameter("bias_ema_tau", 2.0)          # [s] 静止中のバイアス追従の時定数
        self.declare_parameter("max_vel_on_noise", 0.0)      # [m/s] ノイズ漏れ防止のクリップ
        self.declare_parameter("two_d_mode", False)          # 2D拘束
        self.declare_parameter("publish_tf", True)

        self.imu_topic  = self.get_parameter("imu_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.g          = float(self.get_parameter("gravity").value)
        self.alpha      = float(self.get_parameter("alpha_complementary").value)
        self.calib_T    = float(self.get_parameter("calib_duration").value)
        self.gyro_th    = float(self.get_parameter("gyro_thresh").value)
        self.acc_th     = float(self.get_parameter("acc_thresh").value)
        self.stat_Tmin  = float(self.get_parameter("stationary_min_duration").value)
        self.bias_tau   = float(self.get_parameter("bias_ema_tau").value)
        self.max_vclip  = float(self.get_parameter("max_vel_on_noise").value)
        self.two_d      = bool(self.get_parameter("two_d_mode").value)
        self.do_tf      = bool(self.get_parameter("publish_tf").value)

        # ---- IO ----
        self.sub = self.create_subscription(Imu, self.imu_topic, self.cb_imu, 100)
        self.pub = self.create_publisher(Odometry, self.odom_topic, 50)
        self.tf_pub = TransformBroadcaster(self) if self.do_tf else None

        # ---- States ----
        self.t_last = None
        self.calib_done = False
        self.calib_start_time = None
        self.gyro_buf = []
        self.acc_buf  = []

        self.gyro_bias = np.zeros(3, dtype=float)
        # 可変バイアス（静止時にEMA追従）
        self.gyro_bias_run = np.zeros(3, dtype=float)

        # 姿勢：ワールド→ボディの四元数（ROSの慣例に合わせ w,x,y,z）
        # 初期は「後で」加速度から求める
        self.q = [1.0, 0.0, 0.0, 0.0]

        # 速度・位置（World frame）
        self.v = np.zeros(3, dtype=float)
        self.p = np.zeros(3, dtype=float)

        # 静止判定
        self.sta_acc_time = 0.0
        self.is_stationary = False

        self.get_logger().info("IMU Odometry ADV started (bias, stationary, complementary).")

    def cb_imu(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t_last is None:
            self.t_last = t
            self.calib_start_time = t
            # 初回はバッファに入れて終了
            self.gyro_buf.append(np.array([msg.angular_velocity.x,
                                           msg.angular_velocity.y,
                                           msg.angular_velocity.z], dtype=float))
            self.acc_buf.append(np.array([msg.linear_acceleration.x,
                                          msg.linear_acceleration.y,
                                          msg.linear_acceleration.z], dtype=float))
            return

        dt = t - self.t_last
        if dt <= 0.0:
            return
        self.t_last = t

        gyro = np.array([msg.angular_velocity.x,
                         msg.angular_velocity.y,
                         msg.angular_velocity.z], dtype=float)
        acc  = np.array([msg.linear_acceleration.x,
                         msg.linear_acceleration.y,
                         msg.linear_acceleration.z], dtype=float)

        # -------- 初期キャリブレーション --------
        if not self.calib_done:
            self.gyro_buf.append(gyro)
            self.acc_buf.append(acc)
            if (t - self.calib_start_time) >= self.calib_T:
                gyro_mean = np.mean(self.gyro_buf, axis=0)
                acc_mean  = np.mean(self.acc_buf, axis=0)
                self.gyro_bias = gyro_mean.copy()
                self.gyro_bias_run = gyro_mean.copy()

                # 重力方向から初期姿勢（roll, pitch）を推定（yaw=0）
                ax, ay, az = acc_mean.tolist()
                roll  = math.atan2(ay, az)
                pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
                yaw   = 0.0
                self.q = q_normalize(q_from_euler(roll, pitch, yaw))

                self.calib_done = True
                self.get_logger().info(
                    f"[Calib] gyro_bias={self.gyro_bias}, init rpy={roll:.3f},{pitch:.3f},{yaw:.3f}"
                )
            else:
                return  # キャリブ期間中は更新しない

        # -------- 静止検知（しきい値判定を積算）--------
        gyro_err = np.linalg.norm(gyro - self.gyro_bias_run)
        acc_norm = np.linalg.norm(acc)
        is_still = (gyro_err < self.gyro_th) and (abs(acc_norm - self.g) < self.acc_th)

        if is_still:
            self.sta_acc_time += dt
        else:
            self.sta_acc_time = 0.0

        self.is_stationary = (self.sta_acc_time >= self.stat_Tmin)

        # -------- 姿勢更新：ジャイロ積分 + 加速度補正(roll/pitch) --------
        # ジャイロからバイアスを引く（ランタイム追従）
        omega = gyro - self.gyro_bias_run
        # クォータニオン微分：dq = 0.5 * q ⊗ (0, omega) * dt
        dq = [0.0, omega[0], omega[1], omega[2]]
        dq = q_multiply(self.q, dq)
        self.q = q_normalize([self.q[0] + 0.5 * dq[0] * dt,
                              self.q[1] + 0.5 * dq[1] * dt,
                              self.q[2] + 0.5 * dq[2] * dt,
                              self.q[3] + 0.5 * dq[3] * dt])

        # 加速度から roll/pitch を推定して補正（コンプリメンタリ）
        if 0.5*self.g < acc_norm < 1.5*self.g:  # 極端な加速度時は信用しない
            ax, ay, az = acc.tolist()
            roll_acc  = math.atan2(ay, az)
            pitch_acc = math.atan2(-ax, math.sqrt(ay*ay + az*az))
            # yaw は現在の推定を維持
            _, _, yaw = q_to_euler(self.q)
            q_acc = q_from_euler(roll_acc, pitch_acc, yaw)
            self.q = q_slerp(self.q, q_acc, 1.0 - self.alpha)

        # 2Dモードなら roll/pitch,z を拘束
        if self.two_d:
            _, _, yaw = q_to_euler(self.q)
            self.q = q_normalize(q_from_euler(0.0, 0.0, yaw))

        # -------- 重力除去 → 加速度をWorldへ → 速度・位置積分 --------
        a_world = q_rotate_vec(self.q, acc) - np.array([0.0, 0.0, self.g])

        if self.is_stationary:
            # 速度をゼロ固定（ZUPT）
            self.v[:] = 0.0
            # バイアスを静止中のみEMAで追従（温度ドリフト等）
            if self.bias_tau > 1e-3:
                k = dt / (self.bias_tau + dt)
                self.gyro_bias_run = (1.0 - k) * self.gyro_bias_run + k * gyro
            # 2Dなら高さも固定
            if self.two_d:
                self.p[2] = 0.0
        else:
            self.v += a_world * dt
            if self.max_vclip > 0.0:
                # ノイズで微小に流れるのを抑えるオプション
                self.v = np.clip(self.v, -self.max_vclip, self.max_vclip)
            self.p += self.v * dt

        # 2Dモード拘束
        if self.two_d:
            self.v[2] = 0.0

        # -------- Publish Odometry + TF --------
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        odom.pose.pose.position.x = float(self.p[0])
        odom.pose.pose.position.y = float(self.p[1])
        odom.pose.pose.position.z = float(self.p[2])
        odom.pose.pose.orientation = Quaternion(
            x=float(self.q[1]), y=float(self.q[2]), z=float(self.q[3]), w=float(self.q[0])
        )

        odom.twist.twist.linear.x = float(self.v[0])
        odom.twist.twist.linear.y = float(self.v[1])
        odom.twist.twist.linear.z = float(self.v[2])
        # 角速度はバイアス除去後をそのまま
        odom.twist.twist.angular.x = float(omega[0])
        odom.twist.twist.angular.y = float(omega[1])
        odom.twist.twist.angular.z = float(omega[2])

        # ざっくりした共分散（Nav2/robot_localization用に）
        big = 1e3
        pcov = np.zeros((6,6), dtype=float)
        pcov[0,0] = pcov[1,1] = (0.03 if self.is_stationary else 0.3)
        pcov[2,2] = (0.05 if self.two_d else 0.3)
        pcov[3,3] = pcov[4,4] = pcov[5,5] = (0.02 if self.is_stationary else 0.2)
        odom.pose.covariance = pcov.reshape(-1).tolist()

        tcov = np.zeros((6,6), dtype=float)
        tcov[0,0] = tcov[1,1] = (0.02 if self.is_stationary else 0.2)
        tcov[2,2] = (0.02 if self.two_d else 0.2)
        tcov[3,3] = tcov[4,4] = tcov[5,5] = (0.02 if self.is_stationary else 0.2)
        odom.twist.covariance = tcov.reshape(-1).tolist()

        self.pub.publish(odom)

        if self.do_tf and self.tf_pub is not None:
            tmsg = TransformStamped()
            tmsg.header.stamp = msg.header.stamp
            tmsg.header.frame_id = self.odom_frame
            tmsg.child_frame_id  = self.base_frame
            tmsg.transform.translation.x = float(self.p[0])
            tmsg.transform.translation.y = float(self.p[1])
            tmsg.transform.translation.z = float(self.p[2])
            tmsg.transform.rotation = odom.pose.pose.orientation
            self.tf_pub.sendTransform(tmsg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuOdomAdv()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

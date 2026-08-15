import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
try:
    from tf2_ros import TransformBroadcaster
except ImportError:
    # tf2_ros(tf2_ros_py) が未ビルドの環境向けフォールバック。
    # tf2_msgs/TFMessage を /tf に直接 publish する（TransformBroadcaster 相当）。
    from tf2_msgs.msg import TFMessage

    class TransformBroadcaster:
        def __init__(self, node, qos=10):
            self._pub = node.create_publisher(TFMessage, "/tf", qos)

        def sendTransform(self, transform):
            tfs = transform if isinstance(transform, (list, tuple)) else [transform]
            self._pub.publish(TFMessage(transforms=list(tfs)))
import gym


def _to_np(x):
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    return np.asarray(x)


class IsaacRosOdometryWrapper(gym.Wrapper):
    """
    IsaacLab の観測を nav_msgs/Odometry として publish するラッパー
    - pose: robot_pos, root_quat_w
    - twist: base_lin_vel, base_ang_vel
    - TF: odom -> base_link
    """

    def __init__(self, env: gym.Env, *,
                 base_frame="base_link",
                 odom_frame="odom",
                 odom_topic="/odom",
                 use_sim_time=True,
                 qos_depth=5):
        super().__init__(env)

        if not rclpy.ok():
            rclpy.init(args=None)
            self._shutdown_on_close = True
        else:
            self._shutdown_on_close = False

        self.node = Node("isaac_lab_odom_pub")
        if use_sim_time:
            self.node.set_parameters([rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.pub_odom = self.node.create_publisher(Odometry, odom_topic, qos_depth)
        self.tf_broadcaster = TransformBroadcaster(self.node)

        self.base_frame = base_frame
        self.odom_frame = odom_frame

    # def _now(self):
    #     rclpy.spin_once(self.node, timeout_sec=0.0)
    #     now_ros = self.node.get_clock().now()
    #     if now_ros.nanoseconds == 0:
    #         return Clock(clock_type=ClockType.SYSTEM_TIME).now().to_msg()
    #     return now_ros.to_msg()
    def _now(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        now_ros = self.node.get_clock().now()
        # use_sim_time 運用では /clock 未受信時に壁時計を出さない（GLIM/TF を汚染するため）。
        if now_ros.nanoseconds == 0:
            return None
        return now_ros.to_msg()

    def _publish(self, obs):
        stamp = self._now()
        if stamp is None:   # sim時間が未準備 → 壁時計を出さずスキップ
            return

        pos = _to_np(obs["odom"]["position"])[0]          # (x,y,z)
        quat_wxyz = _to_np(obs["odom"]["orientation"])[0] # (w,x,y,z)
        lin_vel = _to_np(obs["odom"]["linear_velocity"])[0]
        ang_vel = _to_np(obs["odom"]["angular_velocity"])[0]

        # Odometry msg
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        msg.pose.pose.position.x = float(pos[0])
        msg.pose.pose.position.y = float(pos[1])
        msg.pose.pose.position.z = float(pos[2])
        msg.pose.pose.orientation = Quaternion(
            x=float(quat_wxyz[1]),  # ROSは(x,y,z,w)順
            y=float(quat_wxyz[2]),
            z=float(quat_wxyz[3]),
            w=float(quat_wxyz[0])
        )

        msg.twist.twist.linear = Vector3(
            x=float(lin_vel[0]), y=float(lin_vel[1]), z=float(lin_vel[2])
        )
        msg.twist.twist.angular = Vector3(
            x=float(ang_vel[0]), y=float(ang_vel[1]), z=float(ang_vel[2])
        )

        self.pub_odom.publish(msg)

        # TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        self._publish(obs)
        return obs, info

    def step(self, action):
        obs, rew, term, trunc, info = self.env.step(action)
        self._publish(obs)
        return obs, rew, term, trunc, info

    def close(self):
        super().close()
        try:
            self.node.destroy_node()
        except Exception:
            pass
        if self._shutdown_on_close and rclpy.ok():
            rclpy.shutdown()

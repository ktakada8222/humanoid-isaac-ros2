# ros_cmdvel_wrapper.py
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import gym


class _RosCmdVelNode(Node):
    def __init__(self, topic: str = "/cmd_vel", use_sim_time: bool = False):
        super().__init__("isaac_lab_cmdvel")
        if use_sim_time:
            self.set_parameters([rclpy.parameter.Parameter(
                'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True
            )])
        self.command = np.zeros(3, dtype=np.float32)  # [vx, vy, wz]
        self.subscription = self.create_subscription(
            Twist,
            topic,
            self.listener_callback,
            10
        )

    def listener_callback(self, msg: Twist):
        # linear.x を前進後退, linear.y を横移動, angular.z を旋回に割り当て
        self.command = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32)


class IsaacRosCmdVelWrapper(gym.Wrapper):
    """ROS2 /cmd_vel を購読して base_command を上書きする Gym ラッパー"""
    def __init__(self, env: gym.Env, *, topic: str = "/cmd_vel", use_sim_time: bool = False):
        super().__init__(env)

        # rclpy 初期化
        if not rclpy.ok():
            rclpy.init(args=None)
            self._shutdown_on_close = True
        else:
            self._shutdown_on_close = False

        self.node = _RosCmdVelNode(topic=topic, use_sim_time=use_sim_time)

    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        return obs, info

    def step(self, action):
        # ROS のコールバックを処理（非ブロッキング）
        rclpy.spin_once(self.node, timeout_sec=0.0)

        # /cmd_vel から受け取ったコマンドを action 生成に使う
        base_command = self.node.command
        # print(base_command)

        # 下流の run_policy に渡すため、info に埋め込むやり方がわかりやすい
        obs, rew, terminated, truncated, info = self.env.step(action)
        info["base_command"] = base_command
        return obs, rew, terminated, truncated, info

    def close(self):
        try:
            super().close()
        finally:
            try:
                self.node.destroy_node()
            except Exception:
                pass
            if self._shutdown_on_close and rclpy.ok():
                rclpy.shutdown()

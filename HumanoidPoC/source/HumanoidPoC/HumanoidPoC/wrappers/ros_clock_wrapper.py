import gym
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock as ClockMsg

class _ClockNode(Node):
    def __init__(self):
        super().__init__("isaac_lab_clock_pub")

class IsaacRosClockWrapper(gym.Wrapper):
    """env.reset()/step() ごとに /clock を publish する Gym ラッパー."""
    def __init__(self, env: gym.Env, dt: float = 1/60.0, qos_depth: int = 5):
        super().__init__(env)

        if not rclpy.ok():
            rclpy.init(args=None)
            self._shutdown_on_close = True
        else:
            self._shutdown_on_close = False

        self.node = _ClockNode()
        self.pub_clock = self.node.create_publisher(ClockMsg, "/clock", qos_depth)

        # 内部シミュレーション時間 [秒]
        self.sim_time = 0.0
        self.dt = dt   # 1 step あたりのシミュレーション時間（既知 or env.cfg.sim.dt）

    def _publish_clock(self):
        clock_msg = ClockMsg()
        clock_msg.clock.sec = int(self.sim_time)
        clock_msg.clock.nanosec = int((self.sim_time - int(self.sim_time)) * 1e9)
        self.pub_clock.publish(clock_msg)

    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        # self.sim_time = 0.0
        self._publish_clock()
        return obs, info

    def step(self, action):
        obs, rew, terminated, truncated, info = self.env.step(action)
        self.sim_time += self.dt
        self._publish_clock()
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

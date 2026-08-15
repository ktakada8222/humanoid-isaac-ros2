from __future__ import annotations
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import gym

class _RosNode(Node):
    def __init__(self, node_name: str, use_sim_time: bool = False):
        super().__init__(node_name)
        if use_sim_time:
            self.set_parameters([rclpy.parameter.Parameter(
                'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True
            )])

class IsaacRosRealsenseWrapper(gym.Wrapper):
    """
    env.reset()/step() 後に obs から Realsense (RGB/Depth) を publish する Gym ラッパー
    """
    def __init__(
        self,
        env: gym.Env,
        *,
        realsense_key: str = "realsense_front",   # obs 内のキー名
        rgb_topic: str = "/camera/color/image_raw",
        depth_topic: str = "/camera/depth/image_raw",
        camera_info_topic: str = "/camera/camera_info",
        frame_id: str = "camera_link",
        width: int = 640,
        height: int = 480,
        fx: float = 300.0,
        fy: float = 300.0,
        cx: float = 320.0,
        cy: float = 240.0,
        use_sim_time: bool = True,
        qos_depth: int = 5,
    ):
        super().__init__(env)
        self.realsense_key = realsense_key
        self.frame_id = frame_id
        self.width = width
        self.height = height
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy

        # ROS2 init
        if not rclpy.ok():
            rclpy.init(args=None)
            self._shutdown_on_close = True
        else:
            self._shutdown_on_close = False

        self.node = _RosNode("isaac_lab_realsense_pub", use_sim_time=use_sim_time)
        self.pub_rgb = self.node.create_publisher(Image, rgb_topic, qos_depth)
        self.pub_depth = self.node.create_publisher(Image, depth_topic, qos_depth)
        self.pub_info = self.node.create_publisher(CameraInfo, camera_info_topic, qos_depth)

        self.bridge = CvBridge()

    def _now(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        now_ros = self.node.get_clock().now()
        if now_ros.nanoseconds == 0:
            return Clock(clock_type=ClockType.SYSTEM_TIME).now().to_msg()
        return now_ros.to_msg()

    def _publish_images(self, obs_dict):
        """obs[realsense_key] から RGB/Depth を publish"""
        if self.realsense_key not in obs_dict:
            return

        data = obs_dict[self.realsense_key]

        # data が dict 形式 {"rgb": (H,W,3), "depth": (H,W)} 想定
        rgb = data.get("rgb", None)
        depth = data.get("depth", None)

        stamp = self._now()

        if rgb is not None:
            # rgb_np = self._to_numpy(rgb)[0]   # (H,W,3), float32 0-1 のはず
            # print(rgb_np.min(), rgb_np.max())
            # if rgb_np.dtype != np.uint8:
            #     rgb_np = ((rgb_np * 0.5 + 0.5) * 255.0).clip(0,255).astype(np.uint8)
            # msg_rgb = self.bridge.cv2_to_imgmsg(rgb_np, encoding="rgb8")
            # msg_rgb.header = Header(stamp=stamp, frame_id=self.frame_id)
            # self.pub_rgb.publish(msg_rgb)

            rgb_np = self._to_numpy(rgb)[0]
            # 実測範囲に基づく min-max 正規化
            min_val, max_val = rgb_np.min(), rgb_np.max()
            rgb_np = (rgb_np - min_val) / (max_val - min_val + 1e-8)
            # ガンマ補正
            rgb_np = np.power(np.clip(rgb_np, 0, 1), 1/2.2)
            rgb_np = (rgb_np * 255).astype(np.uint8)
            msg_rgb = self.bridge.cv2_to_imgmsg(rgb_np, encoding="rgb8")
            msg_rgb.header = Header(stamp=stamp, frame_id=self.frame_id)
            self.pub_rgb.publish(msg_rgb)

        if depth is not None:
            depth_np = self._to_numpy(depth)[0].astype(np.float32)
            if depth_np.ndim == 3 and depth_np.shape[-1] == 1:
                depth_np = depth_np[..., 0]   # (H,W,1) → (H,W)
            msg_depth = self.bridge.cv2_to_imgmsg(depth_np, encoding="32FC1")
            msg_depth.header = Header(stamp=stamp, frame_id=self.frame_id)
            self.pub_depth.publish(msg_depth)

        # CameraInfo
        info = CameraInfo()
        info.header = Header(stamp=stamp, frame_id=self.frame_id)
        info.width = self.width
        info.height = self.height
        info.k = [self.fx, 0.0, self.cx,
                  0.0, self.fy, self.cy,
                  0.0, 0.0, 1.0]
        info.p = [self.fx, 0.0, self.cx, 0.0,
                  0.0, self.fy, self.cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self.pub_info.publish(info)

    def _to_numpy(self, x):
        if hasattr(x, "detach"):
            return x.detach().cpu().numpy()
        return np.asarray(x)

    # ====== Gym API ======
    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        self._publish_images(obs)
        return obs, info

    def step(self, action):
        obs, rew, terminated, truncated, info = self.env.step(action)
        self._publish_images(obs)
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

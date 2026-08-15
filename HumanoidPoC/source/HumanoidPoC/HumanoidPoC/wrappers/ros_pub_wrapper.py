# ros_pub_wrapper.py
from __future__ import annotations
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.clock import Clock, ClockType
from builtin_interfaces.msg import Time as TimeMsg
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
try:
    from tf2_ros import StaticTransformBroadcaster
except ImportError:
    # tf2_ros(tf2_ros_py) が未ビルドの環境向けフォールバック。
    # tf2_msgs/TFMessage を /tf_static に直接 publish する（StaticTransformBroadcaster 相当）。
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

    class StaticTransformBroadcaster:
        def __init__(self, node):
            qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._pub = node.create_publisher(TFMessage, "/tf_static", qos)
            self._tfs = {}

        def sendTransform(self, transform):
            tfs = transform if isinstance(transform, (list, tuple)) else [transform]
            for t in tfs:
                self._tfs[t.child_frame_id] = t   # 子フレームごとに保持して全体を再送（latched）
            self._pub.publish(TFMessage(transforms=list(self._tfs.values())))
from geometry_msgs.msg import Quaternion, Vector3
try:
    from sensor_msgs_py import point_cloud2
except ImportError:
    # sensor_msgs_py が未ビルドの環境向けフォールバック（point_cloud2.create_cloud 相当）。
    # 本ラッパは XYZI(全FLOAT32) のみ使用するため float 値前提で pack する。
    import struct as _struct

    class _PC2:
        _DT = {1: ('b', 1), 2: ('B', 1), 3: ('h', 2), 4: ('H', 2),
               5: ('i', 4), 6: ('I', 4), 7: ('f', 4), 8: ('d', 8)}

        @classmethod
        def _fmt(cls, fields):
            fmt, offset = '<', 0
            for f in sorted(fields, key=lambda f: f.offset):
                if offset < f.offset:
                    fmt += 'x' * (f.offset - offset)
                    offset = f.offset
                ch, sz = cls._DT[f.datatype]
                fmt += f.count * ch
                offset += f.count * sz
            return fmt

        @classmethod
        def create_cloud(cls, header, fields, points):
            st = _struct.Struct(cls._fmt(fields))
            pts = list(points)
            buff = bytearray(st.size * len(pts))
            off = 0
            for p in pts:
                st.pack_into(buff, off, *[float(v) for v in p])
                off += st.size
            return PointCloud2(header=header, height=1, width=len(pts),
                               is_dense=False, is_bigendian=False, fields=fields,
                               point_step=st.size, row_step=st.size * len(pts),
                               data=bytes(buff))

    point_cloud2 = _PC2()
import gym
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Bool

def _reshape_points(arr: np.ndarray) -> np.ndarray:
    """points を (N,3) に揃える（flat 3N にも対応）。"""
    if arr.ndim == 1:
        if arr.size % 3 != 0:
            raise ValueError(f"points length {arr.size} is not divisible by 3")
        return arr.reshape(-1, 3)
    if arr.ndim == 2 and arr.shape[1] >= 3:
        return arr[:, :3]
    raise ValueError(f"Unsupported points shape: {arr.shape}")

def _as_float32(x):
    return np.asarray(x, dtype=np.float32)

class _RosNode(Node):
    def __init__(self, node_name: str, use_sim_time: bool = False):
        super().__init__(node_name)
        if use_sim_time:
            # /clock を使う（Isaac 側で /clock を出している前提）
            self.set_parameters([rclpy.parameter.Parameter(
                'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True
            )])

class IsaacRosPubWrapper(gym.Wrapper):
    """env.step()/reset() 後に、obs から PointCloud2 / IMU を publish する Gym ラッパー."""
    def __init__(
        self,
        env: gym.Env,
        *,
        points_key: str | None = "points",
        imu_key: str | None = "imu",
        base_frame: str = "base_link",
        lidar_frame_id: str = "lidar",
        imu_frame_id: str = "imu_link",
        lidar_tf=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),  # (x,y,z,qx,qy,qz,qw)
        imu_tf=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        point_topic: str = "/points_raw",
        imu_topic: str = "/imu/data",
        use_sim_time: bool = True,
        qos_depth: int = 5,
        # --- 点群フィルタ（地図作成の品質向上用） ---
        drop_zero_points: bool = True,   # (0,0,0) パディング/無返り点を除去
        min_range: float = 0.0,          # この距離未満を除去（0=無効, 例:0.3 で近接ノイズ除去）
        max_range: float = 0.0,          # この距離超を除去（0=無効, 例:100.0）
        min_valid_points: int = 1,       # 有効点がこれ未満ならフレームをスキップ
    ):
        super().__init__(env)
        self.base_frame = base_frame
        self.lidar_frame_id = lidar_frame_id
        self.imu_frame_id = imu_frame_id

        self.points_key = points_key
        self.imu_key = imu_key
        # self.frame_id = frame_id

        # 点群フィルタ設定
        self.drop_zero_points = drop_zero_points
        self.min_range = float(min_range)
        self.max_range = float(max_range)
        self.min_valid_points = int(min_valid_points)

        # rclpy 初期化（多重初期化の安全側）
        if not rclpy.ok():
            rclpy.init(args=None)
            self._shutdown_on_close = True
        else:
            self._shutdown_on_close = False

        self.node = _RosNode("isaac_lab_pub", use_sim_time=use_sim_time)
        self.pub_points = None
        self.pub_imu = None

        # static TF broadcaster
        self.static_tf_broadcaster = StaticTransformBroadcaster(self.node)

        tfs = []
        tfs.append(self._make_tf(self.base_frame, self.lidar_frame_id, lidar_tf))
        tfs.append(self._make_tf(self.base_frame, self.imu_frame_id, imu_tf))
        self.static_tf_broadcaster.sendTransform(tfs)

        self.pub_points_enabled = True  # ← デフォルトで Publish 有効

        # Boolトピック /pub_points を購読してON/OFFを切り替え
        self.sub_pub_points_ctrl = self.node.create_subscription(
            Bool,
            "/pub_points",                 # 外部制御トピック
            self._cb_pub_points_ctrl,      # コールバック関数
            qos_depth
        )

        if self.points_key is not None:
            self.pub_points = self.node.create_publisher(PointCloud2, point_topic, qos_depth)
        if self.imu_key is not None:
            self.pub_imu = self.node.create_publisher(Imu, imu_topic, qos_depth)

    def _cb_pub_points_ctrl(self, msg: Bool):
        """外部からのON/OFF制御コールバック"""
        self.pub_points_enabled = bool(msg.data)
        state = "enabled" if self.pub_points_enabled else "disabled"
        self.node.get_logger().info(f"[IsaacRosPubWrapper] PointCloud publishing {state}.")

    def _make_tf(self, parent: str, child: str, tf_tuple):
        t = TransformStamped()
        # static tf は stamp 0（latched）でOK。sim時間が未準備でも壁時計は使わない。
        t.header.stamp = self._now() or TimeMsg(sec=0, nanosec=0)
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(tf_tuple[0])
        t.transform.translation.y = float(tf_tuple[1])
        t.transform.translation.z = float(tf_tuple[2])
        t.transform.rotation.x = float(tf_tuple[3])
        t.transform.rotation.y = float(tf_tuple[4])
        t.transform.rotation.z = float(tf_tuple[5])
        t.transform.rotation.w = float(tf_tuple[6])
        return t

    # ====== publish helpers ======
    def _now(self):
        # # use_sim_time=True の場合は /clock に同期
        # return self.node.get_clock().now().to_msg()
        # use_sim_time の時は /clock を取り込むために spin_once
        # if getattr(self, "node", None) is not None and self.node is not None:
        #     try:
        #         # 非ブロッキングで1ステップ回す（/clock を処理）
        #         rclpy.spin_once(self.node, timeout_sec=0.0)
        #     except Exception:
        #         pass

        # 現在の ROS Time を取得
        rclpy.spin_once(self.node, timeout_sec=0.0)  # /clock を処理

        now_ros = self.node.get_clock().now()

        # use_sim_time 運用では、/clock 未受信（now==0）のときに system time（壁時計）を
        # 出してはいけない。壁時計スタンプの1メッセージが GLIM の "last" を汚染し、
        # 以降の sim 時間スタンプが全て "timestamp rewind" として破棄される（地図が作れない）。
        # sim 時間がまだ来ていない場合は None を返し、呼び出し側で publish をスキップする。
        if now_ros.nanoseconds == 0:
            return None
        return now_ros.to_msg()

    # def _publish_points(self, points_np: np.ndarray):
    #     pts = _reshape_points(_as_float32(points_np))
    #     header = Header(stamp=self._now(), frame_id=self.lidar_frame_id)
    #     msg = point_cloud2.create_cloud_xyz32(header, pts)
    #     self.pub_points.publish(msg)
    def _publish_points(self, points_np: np.ndarray):
        pts = _reshape_points(_as_float32(points_np))

        # 無効点の除去（地図作成の品質に直結）：
        #   rtx_lidar_sensor 側で max_points まで (0,0,0) ゼロパディングされ、
        #   無返りレイも (0,0,0) になるため、これらを残すと GLIM が
        #   センサ原点に巨大な点塊を受け取りマップが崩壊する。NaN/Inf も除去。
        if pts.size:
            mask = np.isfinite(pts).all(axis=1)
            if self.drop_zero_points:
                mask &= np.any(pts != 0.0, axis=1)        # (0,0,0) を除去
            if self.min_range > 0.0 or self.max_range > 0.0:
                r = np.linalg.norm(pts, axis=1)
                if self.min_range > 0.0:
                    mask &= r >= self.min_range
                if self.max_range > 0.0:
                    mask &= r <= self.max_range
            pts = pts[mask]

        # 有効点が少なすぎるフレームのみスキップ（ログは抑制してスパムを防ぐ）
        if pts.shape[0] < self.min_valid_points:
            self.node.get_logger().warn(
                f"skip publishing point cloud (valid points={pts.shape[0]})",
                throttle_duration_sec=2.0,
            )
            return

        stamp = self._now()
        if stamp is None:   # sim時間が未準備 → 壁時計を出さずスキップ
            return
        header = Header(stamp=stamp, frame_id=self.lidar_frame_id)

        # 強度をダミーで 0 にする
        intensities = np.zeros((pts.shape[0], 1), dtype=np.float32)
        pts_with_i = np.hstack([pts, intensities])  # (N,4) [x,y,z,i]

        # PointFields を定義 (XYZI)
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # PointCloud2 メッセージを生成
        msg = point_cloud2.create_cloud(header, fields, pts_with_i)
        self.pub_points.publish(msg)


    def _publish_imu(self, imu_dict):
        """
        imu_dict 期待:
          {
            "orient": np.ndarray shape (4,) -> [x, y, z, w] or torch->cpu().numpy()
            "ang_vel": np.ndarray shape (3,),
            "lin_acc": np.ndarray shape (3,)
          }
        """
        # トーチ Tensors を許容
        def to_np(x):
            if hasattr(x, "detach"):
                return x.detach().cpu().numpy()
            return np.asarray(x)

        orient = to_np(imu_dict.get("orient", [0, 0, 0, 1])).reshape(-1)
        ang = to_np(imu_dict.get("ang_vel", [0, 0, 0])).reshape(-1)
        acc = to_np(imu_dict.get("lin_acc", [0, 0, 0])).reshape(-1)

        stamp = self._now()
        if stamp is None:   # sim時間が未準備 → 壁時計を出さずスキップ
            return
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.imu_frame_id

        # Isaac Lab の imu_orientation は (w, x, y, z) 順で返す（公式docstring準拠）。
        # ROS の Quaternion は (x, y, z, w) なので並べ替える。
        msg.orientation = Quaternion(x=float(orient[1]), y=float(orient[2]), z=float(orient[3]), w=float(orient[0]))
        msg.angular_velocity = Vector3(x=float(ang[0]), y=float(ang[1]), z=float(ang[2]))
        msg.linear_acceleration = Vector3(x=float(acc[0]), y=float(acc[1]), z=float(acc[2]))

        # 共分散が未計算なら -1（unknown）で埋めるのが通例
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        self.pub_imu.publish(msg)

    # ====== Gym API ======
    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        self._maybe_publish(obs)
        return obs, info

    def step(self, action):
        obs, rew, terminated, truncated, info = self.env.step(action)
        self._maybe_publish(obs)
        return obs, rew, terminated, truncated, info

    def _maybe_publish(self, obs):
        # obs は dict で、各値が torch.Tensor の可能性が高い
        def to_np_first(x):
            # 形状: (num_envs, D) 想定 → 先頭 env を publish
            if hasattr(x, "detach"):
                x = x.detach().cpu().numpy()
            else:
                x = np.asarray(x)
            return x[0] if x.ndim >= 2 else x

        if self.points_key and self.pub_points is not None and self.points_key in obs and self.pub_points_enabled:
            try:
                pts = to_np_first(obs[self.points_key])
                self._publish_points(pts)
            except Exception as e:
                self.node.get_logger().warn(f"publish points failed: {e}")

        if self.imu_key and self.pub_imu is not None and self.imu_key in obs:
            try:
                imu_raw = obs[self.imu_key]
                # imu_raw が dict[torch.Tensor] ならそのまま渡す
                # 環境実装により { 'orient': ..., 'ang_vel': ..., 'lin_acc': ... } or (num_envs, 10) 等もあり得る
                if isinstance(imu_raw, dict):
                    imu_dict = {k: v[0] if (hasattr(v, "shape") and getattr(v, "shape", [0])[0] > 1) else v
                                for k, v in imu_raw.items()}
                else:
                    # もし配列一発で来る設計なら、ここで分解： [qw,qx,qy,qz,wx,wy,wz,ax,ay,az] 等
                    arr = to_np_first(imu_raw).reshape(-1)
                    # ここは環境仕様に合わせて並びを調整してください
                    imu_dict = {
                        "orient": arr[0:4],      # [x,y,z,w] 前提に合わせる
                        "ang_vel": arr[4:7],
                        "lin_acc": arr[7:10],
                    }
                self._publish_imu(imu_dict)
            except Exception as e:
                self.node.get_logger().warn(f"publish imu failed: {e}")

    def close(self):
        try:
            super().close()
        finally:
            # Node 破棄
            try:
                self.node.destroy_node()
            except Exception:
                pass
            if self._shutdown_on_close and rclpy.ok():
                rclpy.shutdown()

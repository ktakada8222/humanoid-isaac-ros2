# policies/sf_tron1a_flat_policy_with_lidar.py
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu

from .sf_tron1a_flat_policy import SFTRON1AFlatTerrainPolicy
from factory_task.sensors.imu_bridge import IsaacImuBridge, ensure_imu_under

from dataclasses import dataclass, replace
from typing import Tuple, Optional

import omni
import omni.usd
import omni.replicator.core as rep

from rclpy.parameter import Parameter

from pxr import Usd, UsdGeom, Gf
import threading

from rosgraph_msgs.msg import Clock
import omni.timeline  # ← 追加

import copy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data


class SimClockPublisher(Node):
    def __init__(self, rate_hz: float = 1000.0):  # ← 1kHz で刻む
        super().__init__("sim_clock_publisher")
        # /clock は BEST_EFFORT / VOLATILE / KEEP_LAST(1)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(Clock, "/clock", qos)
        self._tl = omni.timeline.get_timeline_interface()
        self.timer = self.create_timer(1.0 / rate_hz, self._on_timer)

    def _on_timer(self):
        # シミュレータが再生中のときだけ /clock を流す（停止中は送らない）
        try:
            if hasattr(self._tl, "is_playing") and not self._tl.is_playing():
                return
            # Isaac 版差異: get_current_time() or get_time()
            if hasattr(self._tl, "get_current_time"):
                t = float(self._tl.get_current_time())
            else:
                t = float(self._tl.get_time())
        except Exception:
            return  # タイムライン未初期化時は何もしない

        msg = Clock()
        sec = int(t)
        msg.clock.sec = sec
        msg.clock.nanosec = int((t - sec) * 1e9)
        self.pub.publish(msg)


class PointsRestamper(Node):
    def __init__(self, in_topic: str, out_topic: str):
        super().__init__("points_restamper")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # ← ここを BEST_EFFORT に
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.pub = self.create_publisher(PointCloud2, out_topic, qos)
        self.sub = self.create_subscription(PointCloud2, in_topic, self._cb, qos)
        
        # ★ これを追加：ROS時間に切り替える
        self.set_parameters([Parameter("use_sim_time", value=True)])

        self.get_logger().info(f"[restamp] {in_topic} -> {out_topic}")

    def _cb(self, msg: PointCloud2):
        m = copy.copy(msg)
        m.header.stamp = self.get_clock().now().to_msg()  # /clock が有ればシム時間、無ければ壁時計
        self.pub.publish(m)

# ============================
# 1) Config stays the same
# ============================
@dataclass(frozen=True)
class LidarConfig:
    parent_prim: str = "/World/SF_TRON/base_Link"
    prim_path:   str = "/World/SF_TRON/base_Link/vlp16"
    frame_id:    str = "vlp16"
    topic:       str = "/velodyne/points_raw"
    rate_hz:     float = 10.0
    translation: Tuple[float, float, float] = (0.0, 0.0, 0.20)
    orientation_euler_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    profile:     str = "Example_Rotary"
    
    imu_enabled: bool = True
    imu_topic: Optional[str] = "/velodyne/points_imu"

    def with_overrides(self, **kwargs) -> "LidarConfig":
        return replace(self, **kwargs)


# ============================
# 2) Helpers (non-destructive)
# ============================
_DEF_SENSOR_SUFFIX = "_sensor"  # mesh と同階層にセンサを追加するための接尾語


def _get_world_xf(prim: Usd.Prim) -> Gf.Matrix4d:
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    return cache.GetLocalToWorldTransform(prim)


def _set_local_matrix(prim: Usd.Prim, m: Gf.Matrix4d) -> None:
    x = UsdGeom.Xformable(prim)
    # 既存のxformOpsが無い前提で1つ追加（必要なら上書きロジックに変更）
    op = x.AddTransformOp()
    op.Set(m)


def _ensure_sensor_beside_mesh(cfg: LidarConfig) -> Usd.Prim:
    """Keep mesh at cfg.prim_path, add RTX LiDAR sensor at same pose under the same parent.
    Returns the created/ensured sensor prim.
    """
    stage = omni.usd.get_context().get_stage()
    mesh = stage.GetPrimAtPath(cfg.prim_path)
    if not mesh or not mesh.IsValid():
        raise RuntimeError(f"Mesh prim not found: {cfg.prim_path}")

    parent_path = str(mesh.GetPath().GetParentPath())
    sensor_path = str(mesh.GetPath()) + _DEF_SENSOR_SUFFIX

    sensor = stage.GetPrimAtPath(sensor_path)
    if not sensor or not sensor.IsValid():
        # 正規の RTX LiDAR センサ Prim を新規作成（位置合わせはあとで行列で）
        _, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=sensor_path,
            parent=parent_path,
            config=cfg.profile,
            translation=(0.0, 0.0, 0.0),
            orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        )

    # 親に対するローカル行列 = 親のWorld^-1 * メッシュのWorld
    parent = stage.GetPrimAtPath(parent_path)
    m_world_mesh = _get_world_xf(mesh)
    m_world_parent = _get_world_xf(parent) if parent and parent.IsValid() else Gf.Matrix4d(1.0)
    m_local_sensor = m_world_parent.GetInverse() * m_world_mesh

    # センサのローカル行列をメッシュと同じに設定
    _set_local_matrix(sensor, m_local_sensor)
    return sensor


def _attach_ros2_writer(sensor_prim: Usd.Prim, cfg: LidarConfig):
    # LiDARセンサ Prim から 1x1 RP を作成し、ROS2 Writer を直 attach
    rp = rep.create.render_product(sensor_prim.GetPath(), (1, 1), name="RtxLidarRP")
    writer = rep.writers.get("RtxLidarROS2PublishPointCloud")
    writer.initialize(topicName=cfg.topic, frameId=cfg.frame_id)
    writer.attach([rp])

    # keep references
    return rp, writer


def _ensure_extensions():
    extmgr = omni.kit.app.get_app().get_extension_manager()
    for ext in ("isaacsim.sensors.rtx", "isaacsim.ros2.bridge", "isaacsim.sensors.physics"):  # ★ 追加
        if not extmgr.is_extension_enabled(ext):
            extmgr.set_extension_enabled_immediate(ext, True)


# ============================
# 3) IMU Publisher (Python方式)
# ============================
class ImuPublisher(Node):
    def __init__(self, prim_path, frame_id, topic, rate_hz: float = 200.0):
        super().__init__("imu_publisher")
        self.prim_path = prim_path
        self.frame_id = frame_id
        self.rate_hz = rate_hz

        # GLIM 向けのセンサQoS
        self.publisher = self.create_publisher(Imu, topic, qos_profile_sensor_data)
        self.set_parameters([Parameter("use_sim_time", value=True)])

        # /clock を監視（BEST_EFFORTでOK）
        self._clock_sub = self.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)

        # 前回の状態
        self._last_clock_t = None          # float [s]
        self._last_pos = None              # np.array(3)
        self._last_rot = None              # Gf.Quatd
        self._last_vel = None              # np.array(3)

    def _read_pose(self):
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(self.prim_path)
        m = omni.usd.get_world_transform_matrix(prim)
        p = np.array(m.ExtractTranslation(), dtype=float)
        q = m.ExtractRotation().GetQuat()  # Gf.Quatd
        return p, q

    def _quat_slerp(self, q0: Gf.Quatd, q1: Gf.Quatd, alpha: float) -> Gf.Quatd:
        # Gf には Slerp 関数があるのでそれを使う
        return Gf.Slerp(alpha, q0, q1)

    def _on_clock(self, clk: Clock):
        # 現在のシム時刻（秒）
        t = float(clk.clock.sec) + float(clk.clock.nanosec) * 1e-9

        # 現在の姿勢（フレーム境界の値）
        cur_pos, cur_rot = self._read_pose()

        if self._last_clock_t is None:
            # 初回：1発だけ publish（静止相当）
            self._publish_one(t, cur_pos, cur_rot, np.zeros(3), np.array([0.0,0.0,9.81]))
            self._last_clock_t = t
            self._last_pos = cur_pos
            self._last_rot = cur_rot
            self._last_vel = np.zeros(3)
            return

        dt = t - self._last_clock_t
        if dt <= 0.0:
            # 時間が進んでいなければ何もしない
            return

        # 端点から速度・角速度・加速度を計算
        prev_pos = self._last_pos
        prev_rot = self._last_rot
        prev_vel = self._last_vel if self._last_vel is not None else (cur_pos - prev_pos) / max(dt, 1e-9)

        cur_vel = (cur_pos - prev_pos) / dt
        lin_acc_world = (cur_vel - prev_vel) / dt
        lin_acc_world[2] -= 9.81  # 重力

        # 角速度（ワールド）を端点差分で近似
        dq = cur_rot * prev_rot.GetInverse()
        angle = 2.0 * np.arccos(np.clip(dq.GetReal(), -1.0, 1.0))
        axis = np.array(dq.GetImaginary(), dtype=float)
        if np.linalg.norm(axis) > 1e-12:
            axis = axis / np.linalg.norm(axis)
        omega_world = axis * (angle / dt) if dt > 1e-9 else np.zeros(3)

        # 目標発行数（例：dt=0.05s, rate=200Hz → N=10）
        N = max(int(np.ceil(dt * self.rate_hz)), 1)

        # 区間 [t_prev, t] を N 分割し、各分点で補間して publish
        for i in range(1, N+1):
            alpha = i / float(N)
            ti = self._last_clock_t + alpha * dt
            # 姿勢補間（位置：線形、姿勢：SLERP）
            pi = (1.0 - alpha) * prev_pos + alpha * cur_pos
            qi = self._quat_slerp(prev_rot, cur_rot, alpha)

            # 加速度：区間一定近似（ワールド→ローカルへ）
            rot_inv = qi.GetInverse()
            acc_local = rot_inv.Transform(Gf.Vec3d(*lin_acc_world))

            # 角速度：区間一定近似（ワールド→ローカルへ）
            omega_local = rot_inv.Transform(Gf.Vec3d(*omega_world))

            self._publish_one(ti, pi, qi, omega_local, acc_local)

        # 状態更新
        self._last_clock_t = t
        self._last_pos = cur_pos
        self._last_rot = cur_rot
        self._last_vel = cur_vel

    def _publish_one(self, t_sec: float, pos_np: np.ndarray, quat: Gf.Quatd,
                     omega_local: Gf.Vec3d | np.ndarray,
                     acc_local:  Gf.Vec3d | np.ndarray):
        # ヘッダに「シム時刻」を明示セット（/clock に合わせる）
        sec = int(t_sec)
        nsec = int((t_sec - sec) * 1e9)

        imu = Imu()
        imu.header.stamp.sec = sec
        imu.header.stamp.nanosec = nsec
        imu.header.frame_id = self.frame_id

        # orientation
        imu.orientation.x = quat.GetImaginary()[0]
        imu.orientation.y = quat.GetImaginary()[1]
        imu.orientation.z = quat.GetImaginary()[2]
        imu.orientation.w = quat.GetReal()

        # angular velocity (local)
        imu.angular_velocity.x = float(omega_local[0])
        imu.angular_velocity.y = float(omega_local[1])
        imu.angular_velocity.z = float(omega_local[2])

        # linear acceleration (local)
        imu.linear_acceleration.x = float(acc_local[0])
        imu.linear_acceleration.y = float(acc_local[1])
        imu.linear_acceleration.z = float(acc_local[2])

        self.publisher.publish(imu)


class StableImuPublisher(Node):
    def __init__(self,
                 prim_path: str,
                 frame_id: str = "imu_link",
                 topic: str = "/imu/data_raw",
                 rate_hz: float = 200.0,
                 use_orientation: bool = False,
                 gravity: float = 9.81):
        super().__init__("stable_imu_publisher")
        self.prim_path = prim_path
        self.frame_id = frame_id
        self.rate_hz = float(rate_hz)
        self.gravity = float(gravity)
        self.use_orientation = bool(use_orientation)

        # IMU pub (GLIM想定: sensor QoS)
        self.publisher = self.create_publisher(Imu, topic, qos_profile_sensor_data)
        self.set_parameters([Parameter("use_sim_time", value=True)])

        # /clock を購読（時刻駆動）
        self._clock_sub = self.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)

        # 前回の状態
        self._last_t = None
        self._last_pos = None
        self._last_rot = None
        self._last_vel = None

        # 観測ベースで「IMUローカルの重力方向」を +Z に合わせる 3x3 回転
        self._align_R = None  # np.ndarray shape (3,3)

        # ベクトルが十分大きいときだけ整列を確定させる（静止判定の簡易版）
        self._min_g_norm_for_align = 1.0  # m/s^2

        self.get_logger().info(f"[StableIMU] prim={prim_path}, frame_id={frame_id}, topic={topic}, rate={rate_hz}Hz")

    # ====== USD pose helpers ======
    def _read_pose(self):
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(self.prim_path)
        m = omni.usd.get_world_transform_matrix(prim)
        p = np.array(m.ExtractTranslation(), dtype=float)
        q = m.ExtractRotation().GetQuat()  # Gf.Quatd (world orientation of prim)
        return p, q

    @staticmethod
    def _quat_slerp(q0: Gf.Quatd, q1: Gf.Quatd, alpha: float) -> Gf.Quatd:
        return Gf.Slerp(alpha, q0, q1)

    # ====== Align helper: rotate observed gravity -> +Z ======
    @staticmethod
    def _align_to_plus_z(g_local: np.ndarray) -> np.ndarray:
        # g_local を正規化し、ez=(0,0,1) に回す 3x3 回転行列を返す
        v = g_local / (np.linalg.norm(g_local) + 1e-9)
        ez = np.array([0.0, 0.0, 1.0])
        c = float(np.clip(np.dot(v, ez), -1.0, 1.0))
        if c > 0.9999:
            return np.eye(3)
        if c < -0.9999:
            # 180度回転：任意の直交ベクトルを軸に
            axis = np.array([1.0, 0.0, 0.0]) if abs(v[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
            axis = axis / np.linalg.norm(axis)
            K = np.array([[0, -axis[2], axis[1]],
                          [axis[2],  0,      -axis[0]],
                          [-axis[1], axis[0], 0]])
            return -np.eye(3) + 2.0 * np.outer(axis, axis)
        # 一般：ロドリゲス
        axis = np.cross(v, ez)
        s = np.linalg.norm(axis)
        axis = axis / (s + 1e-9)
        K = np.array([[0, -axis[2], axis[1]],
                      [axis[2], 0, -axis[0]],
                      [-axis[1], axis[0], 0]])
        return np.eye(3) + K * s + K @ K * ((1.0 - c) / (s * s + 1e-9))

    # ====== Main tick from /clock ======
    def _on_clock(self, clk: Clock):
        t = float(clk.clock.sec) + float(clk.clock.nanosec) * 1e-9
        cur_pos, cur_rot = self._read_pose()

        if self._last_t is None:
            # 初回は「静止相当」を1発だけ：角速度0、加速度は +Z に重力
            self._publish_one(t, cur_pos, cur_rot,
                              omega_local=np.zeros(3),
                              acc_local=np.array([0.0, 0.0, self.gravity]))
            self._last_t = t
            self._last_pos = cur_pos
            self._last_rot = cur_rot
            self._last_vel = np.zeros(3)
            return

        dt = t - self._last_t
        if dt <= 0.0:
            return

        # 速度・加速度（ワールド系）。IMUが感じるのは「重力 + 動的加速度」なので +Z に重力を足す。
        prev_pos = self._last_pos
        prev_vel = self._last_vel
        cur_vel = (cur_pos - prev_pos) / dt
        lin_acc_world = (cur_vel - prev_vel) / dt + np.array([0.0, 0.0, self.gravity])

        # 角速度（ワールド系）を端点差分で近似
        prev_rot = self._last_rot
        dq = cur_rot * prev_rot.GetInverse()
        angle = 2.0 * np.arccos(np.clip(dq.GetReal(), -1.0, 1.0))
        axis = np.array(dq.GetImaginary(), dtype=float)
        if np.linalg.norm(axis) > 1e-12:
            axis = axis / np.linalg.norm(axis)
        omega_world = axis * (angle / dt) if dt > 1e-9 else np.zeros(3)

        # ワールド→ローカル（IMUローカル。ただし“IMUローカル”の定義はPrim依存）
        rot_inv = cur_rot.GetInverse()
        acc_local = rot_inv.Transform(Gf.Vec3d(*lin_acc_world))
        omega_local = rot_inv.Transform(Gf.Vec3d(*omega_world))
        acc_local = np.array([acc_local[0], acc_local[1], acc_local[2]])
        omega_local = np.array([omega_local[0], omega_local[1], omega_local[2]])

        # 初回に観測された「ローカル重力方向」を +Z に合わせる回転を確定
        if self._align_R is None:
            gnorm = float(np.linalg.norm(acc_local))
            if gnorm > self._min_g_norm_for_align:
                self._align_R = self._align_to_plus_z(acc_local)

        # 整列を適用（以後、常に“GLIM期待のIMUローカル”で出す）
        if self._align_R is not None:
            acc_local = self._align_R @ acc_local
            omega_local = self._align_R @ omega_local

        # # 目標発行レートに対してオーバーサンプルはしない（まずは1:1で安定性を確認）
        # acc_local  = np.array([-acc_local[0],   acc_local[1],  acc_local[2]])     # アドホクク・実験的調調整 (TODO)
        # omega_local = np.array([-omega_local[0], omega_local[1], omega_local[2]]) # アドホクク・実験的調調整 (TODO)
        self._publish_one(t, cur_pos, cur_rot, omega_local, acc_local)

        # 状態更新
        self._last_t = t
        self._last_pos = cur_pos
        self._last_rot = cur_rot
        self._last_vel = cur_vel

    # ====== Publish one IMU sample ======
    def _publish_one(self, t_sec: float, pos_np: np.ndarray, quat_world: Gf.Quatd,
                     omega_local: np.ndarray, acc_local: np.ndarray):
        sec = int(t_sec)
        nsec = int((t_sec - sec) * 1e9)

        msg = Imu()
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nsec
        msg.header.frame_id = self.frame_id  # 例: "imu_link"（LiDARと分けるほうが良い）

        # # orientation：まずは使わない（GLIMが角速度・加速度のみ使う想定）
        # if self.use_orientation:
        #     # 必要なら“IMUローカル基準の姿勢”を計算して入れる。
        #     # ここでは簡略化のため world quat をそのまま publish しない方針。
        #     msg.orientation.x = 0.0
        #     msg.orientation.y = 0.0
        #     msg.orientation.z = 0.0
        #     msg.orientation.w = 1.0
        # else:
        #     msg.orientation.x = 0.0
        #     msg.orientation.y = 0.0
        #     msg.orientation.z = 0.0
        #     msg.orientation.w = 1.0

        # # angular velocity (rad/s) / linear acceleration (m/s^2)
        # msg.angular_velocity.x = float(omega_local[0])
        # msg.angular_velocity.y = float(omega_local[1])
        # msg.angular_velocity.z = float(omega_local[2])

        # msg.linear_acceleration.x = float(acc_local[0])
        # msg.linear_acceleration.y = float(acc_local[1])
        # msg.linear_acceleration.z = float(acc_local[2])

        msg.linear_acceleration.z = float(9.8)

        self.publisher.publish(msg)


# ===============================================
# 3) Main class (minimal edits to your existing one)
# ===============================================
class SFTRON1AFlatTerrainPolicyWithLiDAR(SFTRON1AFlatTerrainPolicy):
    def __init__(self, *args, lidar_cfg: Optional[LidarConfig] = None, **kwargs):
        super().__init__(*args, **kwargs)
        self._lidar: LidarConfig = lidar_cfg or LidarConfig()
        self._keep_alive = ()  # hold rp/writers
        self._imu_thread = None

        if not rclpy.ok():
            rclpy.init(args=None)

        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._clock_node = SimClockPublisher(rate_hz=1000.0)   # /clock を配信
        self._executor.add_node(self._clock_node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    def set_lidar_config(self, **overrides) -> None:
        self._lidar = self._lidar.with_overrides(**overrides)

    def add_vlp16_like_rtx_lidar_and_ros_pub(
        self,
        *,
        parent_prim: Optional[str] = None,
        lidar_prim: Optional[str] = None,
        frame_id: Optional[str] = None,
        topic_name: Optional[str] = None,
        hz: Optional[float] = None,
        translation: Optional[Tuple[float, float, float]] = None,
        orientation_euler_deg: Optional[Tuple[float, float, float]] = None,
        config: Optional[str] = None,
    ) -> bool:
        # Merge overrides
        # ov = {}
        # if parent_prim is not None: ov["parent_prim"] = parent_prim
        # if lidar_prim  is not None: ov["prim_path"] = lidar_prim
        # if frame_id    is not None: ov["frame_id"] = frame_id
        # if topic_name  is not None: ov["topic"] = topic_name
        # if hz          is not None: ov["rate_hz"] = float(hz)
        # if translation is not None: ov["translation"] = translation
        # if orientation_euler_deg is not None: ov["orientation_euler_deg"] = orientation_euler_deg
        # if config     is not None: ov["profile"] = config
        # self._lidar = self._lidar.with_overrides(**ov)

        # Enable required extensions
        _ensure_extensions()

        # Non-destructive: keep mesh, add sensor beside it, same pose
        sensor_prim = _ensure_sensor_beside_mesh(self._lidar)

        # Replicator writer
        rp, pc_writer = _attach_ros2_writer(sensor_prim, self._lidar)

        # --- PointCloud2 restamp → *_sync に流す ---
        restamper = PointsRestamper(
            in_topic="/velodyne/points_raw",
            out_topic="/velodyne/points"          # ←GLIM が読んでいる従来名
        )

        self._executor.add_node(restamper)

        # --- IMU Publisher ---
        imu_node = None
        if self._lidar.imu_enabled:
            # imu_topic = self._lidar.imu_topic or (self._lidar.topic + "_imu")
            # # imu_node = ImuPublisher(
            # #     prim_path=self._lidar.prim_path,
            # #     frame_id=self._lidar.frame_id,
            # #     topic=imu_topic,
            # # )
            # imu_node = StableImuPublisher(
            #     prim_path=self._lidar.prim_path,
            #     frame_id=self._lidar.frame_id,            # IMU専用フレーム名を推奨（vlp16と共有しない）
            #     topic=imu_topic,
            #     rate_hz=200.0,
            #     use_orientation=False,          # まずは False で安定させる
            #     gravity=9.81
            # )
            # # ★ IMU はシム時間に追従
            # imu_node.set_parameters([Parameter("use_sim_time", value=True)])
            # # ★ Executor に登録（/clock と同じプロセスでOK）
            # self._executor.add_node(imu_node)

            # 既存の update() ループも併用可（publish自体はspin不要だが、今回は合わせて継続）
            # def imu_spin():
            #     rate = 1.0 / self._lidar.rate_hz
            #     while rclpy.ok():
            #         imu_node.update()
            #         time.sleep(rate)
            # self._imu_thread = threading.Thread(target=imu_spin, daemon=True)
            # self._imu_thread.start()

            # imu_prim = ensure_imu_beside_sensor(sensor_prim, imu_name_suffix="_imu", freq_hz=200.0)

            # # 公式IMU→ROS2 橋渡しノードを起動
            # imu_topic = self._lidar.imu_topic or (self._lidar.topic + "_imu")
            # imu_node = IsaacImuBridge(
            #     imu_prim_path=str(imu_prim.GetPath()),
            #     topic=imu_topic,
            #     frame_id="vlp16_imu",       # ★ IMUはLiDARとは別frameにするのが安全
            #     hz=200.0,
            #     read_gravity=True           # GLIMの期待に合わせて True/False を選択
            # )
            # imu_node.set_parameters([Parameter("use_sim_time", value=True)])
            # self._executor.add_node(imu_node)
            # imu_prim = ensure_imu_beside_sensor(sensor_prim, imu_name_suffix="_imu", freq_hz=200.0)

            # imu_topic = self._lidar.imu_topic or (self._lidar.topic + "_imu")
            # imu_node = IsaacImuBridge(
            #     imu_prim_path=str(imu_prim.GetPath()),
            #     topic=imu_topic,
            #     frame_id="vlp16_imu",     # ← LiDAR と別名
            #     hz=200.0,
            #     read_gravity=True
            # )
            # imu_node.set_parameters([Parameter("use_sim_time", value=True)])
            # self._executor.add_node(imu_node)

            imu_path = ensure_imu_under("/World/SF_TRON/base_Link/vlp16", "Imu_Sensor_imu", 200.0)

            imu_topic = self._lidar.imu_topic or (self._lidar.topic + "_imu")
            imu_node = IsaacImuBridge(
                imu_prim_path=imu_path,
                topic=imu_topic,
                frame_id="vlp16_imu",     # LiDARの "vlp16" とは別名
                hz=200.0,
                read_gravity=True
            )
            self._executor.add_node(imu_node)

            # imu_path = ensure_imu_beside_sensor(sensor_prim, imu_name_suffix="_imu", freq_hz=200.0)

            # imu_topic = self._lidar.imu_topic or (self._lidar.topic + "_imu")
            # imu_node = IsaacImuBridge(
            #     imu_prim_path=imu_path,   # ← GetPath() ではなく そのまま渡す
            #     topic=imu_topic,
            #     frame_id="vlp16_imu",
            #     hz=200.0,
            #     read_gravity=True
            # )
            # # imu_node.set_parameters([Parameter("use_sim_time", value=True)])
            # self._executor.add_node(imu_node)

            print(f"[IMU] created/ensured at: {imu_path}")
            # print(f"[IMU] parent rigid: {parent_prim.GetPath()}")

        self._keep_alive = (rp, pc_writer, imu_node, restamper)

        # self._keep_alive = (rp, pc_writer, imu_node, restamper)
        print(f"[LiDAR+IMU/ROS2] LiDAR topic={self._lidar.topic}, IMU topic={imu_node.pub.topic_name if imu_node else 'disabled'}")
        return True

# # factory_task/sensors/imu_bridge.py
# from __future__ import annotations
# import numpy as np
# import omni
# import omni.usd
# import omni.kit.commands
# from pxr import Usd, UsdGeom, Gf
# from rclpy.node import Node
# from rclpy.parameter import Parameter
# from sensor_msgs.msg import Imu
# from rclpy.qos import qos_profile_sensor_data

# # Isaac 純正 IMU 読み出しIF
# from isaacsim.sensors.physics import _sensor

# # LiDARの隣にIMUセンサPrimを生成（姿勢はLiDARセンサPrimに一致）
# def ensure_imu_beside_sensor(sensor_like_prim: Usd.Prim,
#                              imu_name_suffix: str = "_imu",
#                              freq_hz: float = 200.0,
#                              lin_filter: int = 5,
#                              ang_filter: int = 5,
#                              ori_filter: int = 5) -> Usd.Prim:
#     stage = omni.usd.get_context().get_stage()
#     if not sensor_like_prim or not sensor_like_prim.IsValid():
#         raise RuntimeError("sensor_like_prim is invalid")

#     parent_path = str(sensor_like_prim.GetPath().GetParentPath())
#     base_name = sensor_like_prim.GetPath().name
#     imu_name = base_name.replace("_sensor", "") + imu_name_suffix
#     imu_path = f"{parent_path}/{imu_name}"

#     imu_prim = stage.GetPrimAtPath(imu_path)
#     if not imu_prim or not imu_prim.IsValid():
#         omni.kit.commands.execute(
#             "IsaacSensorCreateImuSensor",
#             path=imu_name,
#             parent=parent_path,
#             sensor_period=1.0/float(freq_hz),
#             linear_acceleration_filter_size=int(lin_filter),
#             angular_velocity_filter_size=int(ang_filter),
#             orientation_filter_size=int(ori_filter),
#             translation=Gf.Vec3d(0, 0, 0),
#             orientation=Gf.Quatd(1, 0, 0, 0),
#         )
#         imu_prim = stage.GetPrimAtPath(imu_path)

#     # 親に対するローカル行列を LiDARセンサPrim と一致させる
#     m_world_sensor = omni.usd.get_world_transform_matrix(sensor_like_prim)
#     parent_prim    = stage.GetPrimAtPath(parent_path)
#     m_world_parent = omni.usd.get_world_transform_matrix(parent_prim) if parent_prim and parent_prim.IsValid() else Gf.Matrix4d(1.0)
#     m_local_imu    = m_world_parent.GetInverse() * m_world_sensor

#     x = UsdGeom.Xformable(imu_prim)
#     if not any(x.GetOrderedXformOps()):
#         op = x.AddTransformOp()
#         op.Set(m_local_imu)
#     else:
#         for op in x.GetOrderedXformOps():
#             if op.GetOpType() == UsdGeom.XformOp.TypeTransform:
#                 op.Set(m_local_imu)
#                 break

#     return imu_prim


# class IsaacImuBridge(Node):
#     """
#     公式IMUセンサのローカルx/y/z値を、そのままROS2 /imu/data_raw へpublishするだけの橋渡し。
#     - read_gravity=True: 静止で±9.81が一軸に出る（重力込み）
#       read_gravity=False: 静止で0（重力除去）
#     """
#     def __init__(self,
#                  imu_prim_path: str,
#                  topic: str = "/imu/data_raw",
#                  frame_id: str = "vlp16_imu",
#                  hz: float = 200.0,
#                  read_gravity: bool = True):
#         super().__init__("isaac_imu_bridge")
#         self.imu_path = imu_prim_path
#         self.frame_id = frame_id
#         self.read_gravity = bool(read_gravity)

#         self.pub = self.create_publisher(Imu, topic, qos_profile_sensor_data)
#         self.set_parameters([Parameter("use_sim_time", value=True)])

#         self._iface = _sensor.acquire_imu_sensor_interface()
#         self._timer = self.create_timer(1.0/float(hz), self._tick)

#         self.get_logger().info(
#             f"[IsaacImuBridge] imu={self.imu_path}, topic={topic}, frame={frame_id}, read_gravity={read_gravity}"
#         )

#     def _tick(self):
#         r = self._iface.get_sensor_reading(self.imu_path,
#                                            use_latest_data=True,
#                                            read_gravity=self.read_gravity)
#         if not r.is_valid:
#             return

#         msg = Imu()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = self.frame_id

#         # 公式IMUは“IMU Prim のローカル軸”で出す。軸入替え/反転はここでは絶対にしない。
#         msg.linear_acceleration.x = float(r.lin_acc_x)
#         msg.linear_acceleration.y = float(r.lin_acc_y)
#         msg.linear_acceleration.z = float(r.lin_acc_z)

#         msg.angular_velocity.x = float(r.ang_vel_x)
#         msg.angular_velocity.y = float(r.ang_vel_y)
#         msg.angular_velocity.z = float(r.ang_vel_z)

#         # orientationは未使用なら単位クォータニオンでOK
#         msg.orientation.w = 1.0
#         msg.orientation.x = 0.0
#         msg.orientation.y = 0.0
#         msg.orientation.z = 0.0

#         self.pub.publish(msg)

# factory_task/sensors/imu_bridge.py
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from rclpy.parameter import Parameter

from pxr import Usd, UsdGeom, Gf
import omni
import omni.usd
from omni.kit import commands as kit_commands  
from pxr import Usd, UsdGeom, Gf
from rclpy.exceptions import ParameterAlreadyDeclaredException

# Isaac IMU 低レベル I/F
from isaacsim.sensors.physics import _sensor as imu_low

from pxr import Usd, UsdGeom, Gf, Sdf, UsdPhysics, PhysxSchema
import omni

from pxr import Usd, UsdGeom, Sdf, Gf
import omni

def ensure_imu_under(parent_path_str: str, imu_token: str = "Imu_Sensor_imu",
                     freq_hz: float = 200.0) -> str:
    stage = omni.usd.get_context().get_stage()
    parent_path = Sdf.Path(parent_path_str)              # /World/.../base_Link/vlp16
    if not stage.GetPrimAtPath(parent_path):
        raise RuntimeError(f"parent prim not found: {parent_path}")

    imu_path = parent_path.AppendChild(imu_token)        # ← 子名にスラッシュ禁止
    imu_prim = stage.GetPrimAtPath(imu_path)

    if not imu_prim or not imu_prim.IsValid():
        ok, _ = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path=str(imu_path),                          # 絶対パスのみ
            sensor_period=1.0/float(freq_hz),
            translation=Gf.Vec3d(0,0,0),
            orientation=Gf.Quatd(1,0,0,0),
            linear_acceleration_filter_size=10,
            angular_velocity_filter_size=10,
            orientation_filter_size=10,
        )
        if not ok:
            raise RuntimeError(f"IMU create failed: {imu_path}")
        imu_prim = stage.GetPrimAtPath(imu_path)
        if not imu_prim or not imu_prim.IsValid():
            raise RuntimeError(f"IMU prim invalid after create: {imu_path}")

    # 親(vlp16)と同じローカル姿勢にしたいなら、ここは不要（ローカル=0姿勢でOK）
    print(f"[IMU] ensured at: {str(imu_path)}")
    return str(imu_path)

def _get_world_xf(prim: Usd.Prim) -> Gf.Matrix4d:
    cache = UsdGeom.XformCache()
    return cache.GetLocalToWorldTransform(prim)

def _find_rigid_parent(prim: Usd.Prim) -> Usd.Prim:
    """上方に遡って RigidBody を探す。UsdPhysics / PhysxSchema の両方に対応。"""
    stage = prim.GetStage()
    p = prim
    while p and p.IsValid() and str(p.GetPath()) != "/":
        try:
            # Isaac Sim では UsdPhysics.RigidBodyAPI が基本
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                # 有効フラグが存在していればなお良し（無くても applied 判定だけで十分）
                attr = UsdPhysics.RigidBodyAPI(p).GetRigidBodyEnabledAttr()
                if not attr or attr.IsValid() is False or attr.Get() is True:
                    return p
        except Exception:
            pass

        try:
            # 旧/補助的: PhysxSchema 側の API が付いている場合も拾う
            if p.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                return p
        except Exception:
            pass

        p = stage.GetPrimAtPath(p.GetPath().GetParentPath())

    return None  # 見つからない場合

def ensure_imu_beside_sensor(sensor_prim, imu_name_suffix="_imu", freq_hz=200.0) -> str:
    stage = omni.usd.get_context().get_stage()

    parent_prim = sensor_prim.GetParent()
    parent_path = str(parent_prim.GetPath())
    imu_name    = f"Imu_Sensor{imu_name_suffix}"
    imu_path    = str(Sdf.Path(parent_path).AppendChild(imu_name))  # ← 絶対パス

    # 既存チェック
    imu_prim = stage.GetPrimAtPath(imu_path)
    if not imu_prim or not imu_prim.IsValid():
        # ★ ここでは "IsaacSensorCreateImuSensor" だけを呼ぶ（parentは渡さない）
        ok, _ = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path=imu_path,
            sensor_period=1.0/float(freq_hz),
            translation=Gf.Vec3d(0,0,0),
            orientation=Gf.Quatd(1,0,0,0),
            linear_acceleration_filter_size=10,
            angular_velocity_filter_size=10,
            orientation_filter_size=10,
        )
        if not ok:
            raise RuntimeError(f"IMU create failed: {imu_path}")
        imu_prim = stage.GetPrimAtPath(imu_path)
        if not imu_prim or not imu_prim.IsValid():
            raise RuntimeError(f"IMU prim invalid after create: {imu_path}")

    # センサ（LiDAR）と同じローカル姿勢に合わせる
    cache = UsdGeom.XformCache()
    m_world_sensor = cache.GetLocalToWorldTransform(sensor_prim)
    m_world_parent = cache.GetLocalToWorldTransform(parent_prim)
    m_local_imu    = m_world_parent.GetInverse() * m_world_sensor
    UsdGeom.Xformable(imu_prim).AddTransformOp().Set(m_local_imu)

    print(f"[IMU] ensured at: {imu_path}")
    return imu_path  # ← 以後は「文字列パス」を返す

class IsaacImuBridge(Node):
    def __init__(self, imu_prim_path: str, topic: str, frame_id: str, hz: float = 200.0, read_gravity: bool = True):
        super().__init__("isaac_imu_bridge")

        # ★ use_sim_time は「宣言済みならスキップ」
        try:
            if not self.has_parameter("use_sim_time"):
                self.declare_parameter("use_sim_time", True)
        except ParameterAlreadyDeclaredException:
            pass

        self.imu_path = imu_prim_path
        self.frame_id = frame_id
        self.read_gravity = bool(read_gravity)

        # Publisher: センサQoS（BestEffort/Volatile）
        self.pub = self.create_publisher(Imu, topic, qos_profile_sensor_data)

        # Isaac IMU インターフェイス
        self._imu_if = imu_low.acquire_imu_sensor_interface()

        # タイマー駆動でポーリング
        period = 1.0/float(hz)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(f"[IsaacImuBridge] publishing {topic} @ {hz:.1f} Hz, frame_id={frame_id}, gravity={self.read_gravity}")

    def _tick(self):
        # PLAY 中で無いと常に invalid になる点に注意
        r = self._imu_if.get_sensor_reading(self.imu_path, use_latest_data=True, read_gravity=self.read_gravity)
        
        if not r.is_valid:
            if not hasattr(self, "_last_warn") or (self.get_clock().now().nanoseconds - getattr(self, "_last_warn", 0)) > 1e9:
                self.get_logger().warn(f"IMU read invalid at {self.imu_path} (timeline playing? rigid parent?)")
                self._last_warn = self.get_clock().now().nanoseconds
            return
    

        now = self.get_clock().now().to_msg()
        msg = Imu()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id

        # 角速度 [rad/s]
        msg.angular_velocity.x = float(r.ang_vel_x)
        msg.angular_velocity.y = float(r.ang_vel_y)
        msg.angular_velocity.z = float(r.ang_vel_z)

        # 加速度 [m/s^2]（read_gravity=True なら重力込み。False なら動的のみ）
        msg.linear_acceleration.x = float(r.lin_acc_x)
        msg.linear_acceleration.y = float(r.lin_acc_y)
        msg.linear_acceleration.z = float(r.lin_acc_z)

        # orientation は使わない（必要なら r.orientation をクォータニオンで詰める）
        msg.orientation.w = 1.0

        self.pub.publish(msg)

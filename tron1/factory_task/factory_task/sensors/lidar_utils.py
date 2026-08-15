# # sensors/lidar_utils.py
# import math
# import numpy as np
# from typing import Tuple
# from pxr import Gf, Sdf, UsdGeom
# import omni
# import omni.kit.commands
# from omni.isaac.core.utils.prims import define_prim
# from omni.isaac.sensor import _sensor

# def create_rtx_lidar(
#     parent_prim: str,
#     lidar_prim: str,
#     translation: Tuple[float, float, float] = (0.0, 0.0, 1.0),
#     orientation_euler_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0),
#     hz: float = 10.0,
#     h_fov_deg: float = 360.0,
#     v_fov_min_deg: float = -7.0,
#     v_fov_max_deg: float = 52.0,
#     horiz_res: int = 2048,
#     vert_res: int = 10,
#     enable_intensity: bool = True,
# ):
#     stage = omni.usd.get_context().get_stage()
#     if not stage.GetPrimAtPath(parent_prim).IsValid():
#         define_prim(parent_prim, "Xform")

#     if not stage.GetPrimAtPath(lidar_prim).IsValid():
#         omni.kit.commands.execute(
#             "IsaacSensorCreateRtxLidar",
#             path=lidar_prim,
#             parent=parent_prim,
#         )

#     xform = UsdGeom.Xformable(stage.GetPrimAtPath(lidar_prim))
#     t = Gf.Vec3d(*translation)
#     rx, ry, rz = [math.radians(v) for v in orientation_euler_deg]
#     # ZYX順のオイラー→クォータニオン（簡便式）
#     q = Gf.Quatd(
#         math.cos(rz/2)*math.cos(ry/2)*math.cos(rx/2) + math.sin(rz/2)*math.sin(ry/2)*math.sin(rx/2),
#         Gf.Vec3d(
#             math.cos(rz/2)*math.cos(ry/2)*math.sin(rx/2) - math.sin(rz/2)*math.sin(ry/2)*math.cos(rx/2),
#             math.cos(rz/2)*math.sin(ry/2)*math.cos(rx/2) + math.sin(rz/2)*math.cos(ry/2)*math.sin(rx/2),
#             math.sin(rz/2)*math.cos(ry/2)*math.cos(rx/2) - math.cos(rz/2)*math.sin(ry/2)*math.sin(rx/2),
#         ),
#     )
#     xform.ClearXformOpOrder()
#     xform.AddTranslateOp().Set(t)
#     xform.AddOrientOp().Set(q)

#     prim = stage.GetPrimAtPath(lidar_prim)

#     def _set_attr(name, val):
#         attr = prim.CreateAttribute(name, Sdf.ValueTypeNames.GetTypeFromValue(val))
#         attr.Set(val)

#     # バージョンで属性名が多少違うことがあります。必要に応じてGUIで確認してください。
#     _set_attr("rtxSensor:rotationRate", float(hz))
#     _set_attr("rtxSensor:horizontalFov", float(math.radians(h_fov_deg)))
#     _set_attr("rtxSensor:verticalFov", float(math.radians(v_fov_max_deg - v_fov_min_deg)))
#     _set_attr("rtxSensor:verticalFovLower", float(math.radians(v_fov_min_deg)))
#     _set_attr("rtxSensor:horizontalResolution", int(horiz_res))
#     _set_attr("rtxSensor:verticalResolution", int(vert_res))
#     _set_attr("rtxSensor:enableIntensity", bool(enable_intensity))
#     _set_attr("rtxSensor:highLod", True)

# def get_latest_point_cloud(lidar_prim_path: str) -> np.ndarray:
#     """RTX LiDARの最新点群（N,3）を返す。データなしなら空配列。"""
#     iface = _sensor.acquire_lidar_sensor_interface()
#     pts = iface.get_point_cloud_data(lidar_prim_path)
#     if pts is None:
#         return np.empty((0, 3), dtype=np.float32)
#     return np.asarray(pts, dtype=np.float32)

# sensors/lidar_utils.py (Isaac Sim 5.0)
import math
from typing import Tuple
from pxr import Gf
import omni
import omni.kit.commands
import numpy as np
from omni.isaac.core.utils.prims import define_prim  # 5.0でも利用可

def create_rtx_lidar(
    parent_prim: str,
    lidar_prim: str,
    translation: Tuple[float, float, float] = (0.0, 0.0, 1.0),
    orientation_euler_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    config: str = "Example_Rotary",  # 5.0公式チュートリアルの既定プロファイル
):
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(parent_prim).IsValid():
        define_prim(parent_prim, "Xform")

    # 既存なら何もしない（再入安全）
    if not stage.GetPrimAtPath(lidar_prim).IsValid():
        # 公式サンプルと同じ作成コマンド（IsaacSensorCreateRtxLidar）
        # config は Example_Rotary / Example_Solid_State 等（VLP-16相当なら後で回転/垂直分解能を調整）
        _, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=lidar_prim,
            parent=parent_prim,
            config=config,
            translation=(float(translation[0]), float(translation[1]), float(translation[2])),
            orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # 回転は後でXformで与える
        )
    else:
        sensor = stage.GetPrimAtPath(lidar_prim)

    # 姿勢（XformCommonAPI ではなく、Translate/RotateXYZ属性で安全に）
    from pxr import UsdGeom, Gf as _Gf
    xform = UsdGeom.XformCommonAPI(sensor)
    xform.SetTranslate(_Gf.Vec3d(*[float(v) for v in translation]))

    rx, ry, rz = [float(a) for a in orientation_euler_deg]
    try:
        xform.SetRotateXYZ(_Gf.Vec3d(rx, ry, rz))
    except AttributeError:
        from pxr import UsdGeom as _UG
        try:
            xform.SetRotate(_Gf.Vec3f(rx, ry, rz), _UG.XformCommonAPI.RotationOrderXYZ)
        except Exception:
            xform.SetRotate(_UG.XformCommonAPI.RotationOrderXYZ, _Gf.Vec3d(rx, ry, rz))

    return lidar_prim  # パスを返す


# sensors/lidar_utils.py
# import math
# import numpy as np
# from typing import Tuple
# from pxr import Gf, Sdf, UsdGeom
# import omni
# import omni.kit.commands
# from omni.isaac.core.utils.prims import define_prim
# from omni.isaac.range_sensor import _range_sensor as _rtx


# def create_rtx_lidar(
#     parent_prim: str,
#     lidar_prim: str,
#     translation: Tuple[float, float, float] = (0.0, 0.0, 1.0),
#     orientation_euler_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0),
#     hz: float = 10.0,
#     h_fov_deg: float = 360.0,
#     v_fov_min_deg: float = -15.0,
#     v_fov_max_deg: float = +15.0,
#     horiz_res: int = 1875,
#     vert_res: int = 16,
#     enable_intensity: bool = True,
# ):
#     stage = omni.usd.get_context().get_stage()

#     # 親を必ず用意
#     if not stage.GetPrimAtPath(parent_prim).IsValid():
#         define_prim(parent_prim, "Xform")

#     # LiDAR本体（RTX Rotary LiDAR）を作成（未作成なら）
#     if not stage.GetPrimAtPath(lidar_prim).IsValid():
#         omni.kit.commands.execute(
#             "IsaacSensorCreateRtxLidar",
#             path=lidar_prim,
#             parent=parent_prim,
#         )

#     prim = stage.GetPrimAtPath(lidar_prim)
#     if not prim or not prim.IsValid():
#         raise RuntimeError(f"Failed to create lidar prim at '{lidar_prim}'")

#     # --- 姿勢設定は XformCommonAPI で統一 ---
#     xform_api = UsdGeom.XformCommonAPI(prim)

#     # Translate: ビルドにより Vec3d を要求するため Vec3d 固定
#     xform_api.SetTranslate(Gf.Vec3d(float(translation[0]),
#                                     float(translation[1]),
#                                     float(translation[2])))

#     # Rotate: 環境差を吸収
#     rx, ry, rz = (float(orientation_euler_deg[0]),
#                   float(orientation_euler_deg[1]),
#                   float(orientation_euler_deg[2]))
#     # まずは「Vec3f, RotationOrder」の順で試す（あなたのログの型）
#     try:
#         xform_api.SetRotate(Gf.Vec3f(rx, ry, rz),
#                             UsdGeom.XformCommonAPI.RotationOrderXYZ)
#     except Exception:
#         # 代替：RotationOrder が先、かつ Vec3d を要求する派生ビルド用
#         xform_api.SetRotate(UsdGeom.XformCommonAPI.RotationOrderXYZ,
#                             Gf.Vec3d(rx, ry, rz))

#     # --- RTX LiDAR のプロパティ設定 ---
#     # 角度系はラジアン指定の属性が多い
#     h_fov_rad = float(math.radians(h_fov_deg))
#     v_span_deg = float(v_fov_max_deg - v_fov_min_deg)
#     v_fov_rad = float(math.radians(v_span_deg))
#     v_low_rad = float(math.radians(v_fov_min_deg))

#     def _set_attr(name, val, sdf_type):
#         attr = prim.GetAttribute(name)
#         if not attr.IsValid():
#             attr = prim.CreateAttribute(name, sdf_type)
#         attr.Set(val)

#     # 代表的なプロパティ（環境により属性名が微妙に違う場合あり）
#     _set_attr("rtxSensor:rotationRate",         float(hz),              Sdf.ValueTypeNames.Float)
#     _set_attr("rtxSensor:horizontalFov",        h_fov_rad,              Sdf.ValueTypeNames.Float)
#     _set_attr("rtxSensor:verticalFov",          v_fov_rad,              Sdf.ValueTypeNames.Float)
#     _set_attr("rtxSensor:verticalFovLower",     v_low_rad,              Sdf.ValueTypeNames.Float)
#     _set_attr("rtxSensor:horizontalResolution", int(horiz_res),         Sdf.ValueTypeNames.Int)
#     _set_attr("rtxSensor:verticalResolution",   int(vert_res),          Sdf.ValueTypeNames.Int)
#     _set_attr("rtxSensor:enableIntensity",      bool(enable_intensity), Sdf.ValueTypeNames.Bool)
#     _set_attr("rtxSensor:highLod",              True,                   Sdf.ValueTypeNames.Bool)


def get_latest_point_cloud(lidar_prim_path: str) -> np.ndarray:
    """RTX LiDARの最新点群（N,3）を返す。データなしなら空配列。"""
    # 変更点②: RTX LiDAR のIFを取得
    iface = _rtx.acquire_lidar_sensor_interface()

    # 代表的な戻り値パターンに両対応（バージョン差吸収）
    # A) 直接 (N,3) のPython配列/NumPy互換
    pts = iface.get_point_cloud_data(lidar_prim_path)
    if pts is None:
        return np.empty((0, 3), dtype=np.float32)

    # B) まれに辞書/タプルで {points, intensities} 等を返すビルドがある
    #    その場合も points を優先採用
    if isinstance(pts, dict):
        pts = pts.get("points", None)
        if pts is None:
            return np.empty((0, 3), dtype=np.float32)
    elif isinstance(pts, (list, tuple)) and len(pts) > 0 and not isinstance(pts[0], (float, int)):
        # tuple(points, intensities, timestamps, ...) に緩く対応
        pts = pts[0]

    pts = np.asarray(pts, dtype=np.float32)
    # shape が (N, 4) などの場合は xyz のみ使用（w, intensity は別途拡張可能）
    if pts.ndim == 2 and pts.shape[1] >= 3:
        return pts[:, :3].copy()
    return np.empty((0, 3), dtype=np.float32)
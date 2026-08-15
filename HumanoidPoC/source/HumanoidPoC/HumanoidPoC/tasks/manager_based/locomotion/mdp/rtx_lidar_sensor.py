from __future__ import annotations

import math
import torch
from typing import TYPE_CHECKING

import isaaclab.utils.math as math_utils
from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers.manager_base import ManagerTermBase
from isaaclab.managers.manager_term_cfg import ObservationTermCfg
from isaaclab.sensors import Camera, ContactSensor, Imu, RayCaster, RayCasterCamera, TiledCamera

from isaacsim.sensors.rtx import LidarRtx

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv, ManagerBasedRLEnv

import numpy as np
import omni.replicator.core as rep

from HumanoidPoC.tasks.manager_based.locomotion import mdp

import omni.kit.commands
from pxr import Gf
import omni.replicator.core as rep
import os


def attach_rtx_lidar(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    parent_link="base_Link",
    lidar_name="vlp16",
    config_file_name="Example_Rotary",
    translation=(0.0, 0.0, 0.2),
    orientation=(1.0, 0.0, 0.0, 0.0),
    debug_vis: bool = False,   
):
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    if env_ids is None:
        env_ids = list(range(env.num_envs))

    for i in env_ids:
        parent_path = f"/World/envs/env_{i}/Robot/{parent_link}"
        parent_prim = stage.GetPrimAtPath(parent_path)

        if not parent_prim.IsValid():
            print(f"[attach_rtx_lidar] ERROR: parent prim {parent_path} not found → fallbackでWorld直下に生成される可能性あり")
            continue

        lidar_path = f"{parent_path}/{lidar_name}"

        # scanRate は 10Hz（5Hz に下げても改善せず・むしろ低下したため戻した。
        # 律速はレートではなく1スキャンのレイ数=描画の重さ側だったため、機種を32chに軽量化）。
        sensor_attributes = {'omni:sensor:Core:scanRateBaseHz': 10}

        # LidarRtx で作成（内部で Annotator attach もやってくれる）
        sensor = LidarRtx(
            prim_path=lidar_path,
            translation=np.array(translation),
            orientation=np.array(orientation),
            config_file_name=config_file_name,
            **sensor_attributes,  # 追加属性もここで指定可
        )

        sensor.attach_annotator("IsaacCreateRTXLidarScanBuffer")
        sensor.initialize()
        sensor.enable_visualization()

        actual_path = sensor.prim.GetPath().pathString
        if actual_path != lidar_path:
            print(f"[attach_rtx_lidar] WARNING: expected {lidar_path} but actually created at {actual_path}")
            if parent_prim.IsValid():
                print(f"[attach_rtx_lidar] parent prim {parent_path} exists but is type={parent_prim.GetTypeName()}, so LidarRtx couldn't attach as child.")
            else:
                print(f"[attach_rtx_lidar] parent prim {parent_path} not valid at creation time.")

        else:
            print(f"[attach_rtx_lidar] SUCCESS: created at {actual_path}")

        # --- Debug 可視化 ---
        if debug_vis:
            from pxr import UsdGeom, Gf

            debug_path = f"{lidar_path}_debug"
            sphere = UsdGeom.Sphere.Define(stage, debug_path)
            sphere.AddTranslateOp().Set(Gf.Vec3f(*translation))
            sphere.GetRadiusAttr().Set(0.01)  # 半径1cmの球
            sphere.GetDisplayColorAttr().Set([(1.0, 0.0, 0.0)])  # 赤色

            print(f"[attach_rtx_lidar] DEBUG VIS: added red sphere at {debug_path}")

        # グローバルに保持して ObsTerm 側から参照できるようにする
        if not hasattr(mdp, "RTX_LIDAR_SENSORS"):
            mdp.RTX_LIDAR_SENSORS = {}
        mdp.RTX_LIDAR_SENSORS[i] = sensor

        print("annotators:", sensor.get_annotators().keys())

def rtx_lidar_point_cloud(env, sensor_cfg=None, max_points: int = 32768, yaw_offset: float = 0.0):
    """RTX LiDARから点群を取得し、torch.Tensor (B, max_points*3) に整形して返す."""

    device = env.device if hasattr(env, "device") else torch.device("cpu")
    B = getattr(env, "num_envs", 1)

    pcs = []
    for i in range(B):
        # attach_rtx_lidar 内で mdp.RTX_LIDAR_SENSORS[i] に LidarRtx を格納してある想定
        sensor = mdp.RTX_LIDAR_SENSORS.get(i) if hasattr(mdp, "RTX_LIDAR_SENSORS") else None

        if sensor is None:
            pcs.append(torch.zeros((max_points, 3), device=device))
            continue

        # 最新フレームを取得
        frame = sensor.get_current_frame()
        if "IsaacCreateRTXLidarScanBuffer" not in frame:
            pcs.append(torch.zeros((max_points, 3), device=device))
            continue

        data = frame["IsaacCreateRTXLidarScanBuffer"]
        raw = data.get("data", None)

        if raw is None or len(raw) == 0:
            pcs.append(torch.zeros((max_points, 3), device=device))
            continue

        # --- structured array or 普通の array を判別 ---
        if isinstance(raw, np.ndarray) and raw.dtype.names:
            # structured array → フィールド "x","y","z" をまとめる
            pts = np.stack([raw["x"], raw["y"], raw["z"]], axis=-1).astype(np.float32)
        else:
            # 通常の ndarray (N,3)
            pts = np.asarray(raw, dtype=np.float32)

        # --- yaw_offset を適用 ---
        if yaw_offset != 0.0:
            c, s = np.cos(yaw_offset), np.sin(yaw_offset)
            R = np.array([[c, -s, 0],
                          [s,  c, 0],
                          [0,  0, 1]], dtype=np.float32)
            pts = pts @ R.T

        # --- サンプリング / パディングで max_points に揃える ---
        N = len(pts)
        if N > max_points:
            idx = np.linspace(0, N - 1, num=max_points, dtype=np.int64)
            pts = pts[idx]
        elif N < max_points:
            pad = np.zeros((max_points - N, 3), dtype=np.float32)
            pts = np.vstack([pts, pad])

        pcs.append(torch.from_numpy(pts).to(device))

    # (B, max_points, 3) → (B, max_points*3)
    pc = torch.stack(pcs, dim=0)

    return pc.reshape(B, max_points * 3)




# def attach_rtx_lidar(
#     env: ManagerBasedEnv,
#     env_ids: torch.Tensor | None,
#     parent_link: str = "base_Link",       # ← 外部指定できるようにする
#     lidar_name: str = "vlp16",            # ← 外部指定できるようにする
#     lidar_config: str = "Example_Rotary", # ← config も外部指定可
#     translation=(0.0, 0.0, 0.2),
#     orientation=Gf.Quatd(1, 0, 0, 0),
# ):
#     """
#     RTX LiDAR をロボットの指定リンクにアタッチする。
#     EventTerm (mode="startup") から呼び出される想定。
#     """

#     import omni.usd
#     stage = omni.usd.get_context().get_stage()

#     if env_ids is None:
#         env_ids = list(range(env.num_envs))

#     for i in env_ids:
#         parent_path = f"/World/envs/env_{i}/Robot/{parent_link}"
#         if not stage.GetPrimAtPath(parent_path).IsValid():
#             print(f"[attach_rtx_lidar] ERROR: {parent_path} not found on stage.")
#             continue

#         lidar_path = f"{lidar_name}"

#         _, sensor = omni.kit.commands.execute(
#             "IsaacSensorCreateRtxLidar",
#             path=lidar_path,
#             parent=parent_path,
#             config=lidar_config,
#             translation=translation,
#             orientation=orientation,
#         )

#         print("sensor")
#         print(sensor)

#         if sensor is None:
#             print(f"[attach_rtx_lidar] Failed to create LiDAR for env_{i}")
#             continue

#         # Replicator annotator
#         render_product = rep.create.render_product(sensor.GetPath(), [1, 1])
#         mdp.RTX_LIDAR_ANNOTATOR = rep.AnnotatorRegistry.get_annotator("IsaacCreateRTXLidarScanBuffer")
#         mdp.RTX_LIDAR_ANNOTATOR.attach(render_product)

#         writer = rep.writers.get("RtxLidarDebugDrawPointCloudBuffer")
#         writer.attach(render_product)

#         print(f"[attach_rtx_lidar] RTX LiDAR attached at {lidar_path}")
        
    # """
    # RTX LiDAR をロボットの base_Link にアタッチする。
    # EventTerm (mode="startup") から呼び出される想定。
    # """

    # lidar_config = "Example_Rotary"

    # import omni.usd
    # stage = omni.usd.get_context().get_stage()

    # # env_ids が渡されなければ、全 env に対して適用
    # if env_ids is None:
    #     env_ids = list(range(env.num_envs))

    # for i in env_ids:
    #     parent_path = f"/World/envs/env_{i}/Robot/base_Link"

    #     if not stage.GetPrimAtPath(parent_path).IsValid():
    #         print(f"[attach_rtx_lidar] ERROR: {parent_path} not found on stage.")
    #         continue

    #     lidar_path = f"/World/envs/env_{i}/Robot/vlp16"

    #     # LiDAR を作成
    #     _, sensor = omni.kit.commands.execute(
    #         "IsaacSensorCreateRtxLidar",
    #         path=lidar_path,          # フルパスでOK
    #         parent=parent_path,       # base_Link にアタッチ
    #         config=lidar_config,
    #         translation=(0.0, 0.0, 0.2),  # base_Link からの相対位置
    #         orientation=Gf.Quatd(1, 0, 0, 0),
    #     )

    #     if sensor is None:
    #         print(f"[attach_rtx_lidar] Failed to create LiDAR for env_{i}")
    #         continue

    #     # Replicator annotator を接続（点群を Python 側から読めるようにする）
    #     render_product = rep.create.render_product(sensor.GetPath(), [1, 1])
    #     mdp.RTX_LIDAR_ANNOTATOR = rep.AnnotatorRegistry.get_annotator("IsaacCreateRTXLidarScanBuffer")
    #     mdp.RTX_LIDAR_ANNOTATOR.attach(render_product)

    #     # デバッグ描画（必要なければ削除可）
    #     writer = rep.writers.get("RtxLidarDebugDrawPointCloudBuffer")
    #     writer.attach(render_product)

    #     print(f"[attach_rtx_lidar] RTX LiDAR attached at {lidar_path}")

# def rtx_lidar_point_cloud(env, sensor_cfg=None, max_points: int = 32768, yaw_offset: float = 0.0):

#     print("aaaaaaaaaaaaaaaaaaaaaaaa")

#     device = env.device if hasattr(env, "device") else torch.device("cpu")
#     B = getattr(env, "num_envs", 1)

#     print("bbbbbbbbbbbbbbbbbbbbbb")

#     if mdp.RTX_LIDAR_ANNOTATOR is None:
#         return torch.zeros((B, max_points * 3), device=device)

#     print("ccccccccccccccccccccc")

#     data = mdp.RTX_LIDAR_ANNOTATOR.get_data()
#     if data is None or "data" not in data:
#         return torch.zeros((B, max_points * 3), device=device)

#     print("dddddddddddddddddddd")
#     print(data)

#     pts = np.asarray(data["data"], dtype=np.float32)

#     if pts.ndim != 2 or pts.shape[1] != 3:
#         return torch.zeros((B, max_points * 3), device=device)
    
#     print("eeeeeeeeeeeeeeeeeee")


#     pc = torch.from_numpy(pts).to(device).unsqueeze(0)

#     N = pc.shape[1]
#     # M = min(max_points, N)
#     # pc = pc[:, :M, :]
#     # if M < max_points:
#     #     pad = torch.zeros((1, max_points - M, 3), device=device, dtype=pc.dtype)
#     #     pc = torch.cat([pc, pad], dim=1)

#     # return pc.reshape(B, max_points * 3)

#     # --- ここがポイント：等間隔サンプリングで全周を保持 ---
#     if N > max_points:
#         # 0..N-1 を max_points 分だけ等間隔に抽出
#         idx = np.linspace(0, N - 1, num=max_points, dtype=np.int64)
#         pts = pts[idx]  # (max_points, 3)
#     else:
#         # N < max_points のときは後でパディング
#         pass

#     pc = torch.from_numpy(pts).to(device).unsqueeze(0)  # (1, M', 3)
#     Mprime = pc.shape[1]

#     # パディングして固定長に
#     if Mprime < max_points:
#         pad = torch.zeros((1, max_points - Mprime, 3), device=device, dtype=pc.dtype)
#         pc = torch.cat([pc, pad], dim=1)  # (1, max_points, 3)
    
#     print(pc)

#     return pc.reshape(B, max_points * 3)
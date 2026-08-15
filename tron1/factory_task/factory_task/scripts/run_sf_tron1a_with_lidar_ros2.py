# scripts/run_sf_tron1a_with_lidar_ros2.py
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from time import sleep
import omni
ext_mgr = omni.kit.app.get_app().get_extension_manager()
needed = ["isaacsim.core.nodes", "isaacsim.ros2.bridge", "isaacsim.sensors.rtx", "omni.graph.action"]
for ext in needed:
    if not ext_mgr.is_extension_enabled(ext):
        ext_mgr.set_extension_enabled(ext, True)
        print(f"[INFO] enabled: {ext}")

# 拡張ロードを待つ（数フレーム）
for _ in range(3):
    simulation_app.update()
    sleep(0.05)

import os
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.simulation_context import SimulationContext

# ★ 既存ポリシーの“継承版”を使う
from factory_task.policies.sf_tron1a_flat_policy_with_lidar import (
    SFTRON1AFlatTerrainPolicyWithLiDAR,
)
# ★ initializeはベースクラス経由で呼ぶ（既存スクリプトに合わせる）
from factory_task.controllers.encoder_policy_controller_onnx import (
    EncoderPolicyControllerONNX,
)

from factory_task.scripts.keyboard_controller import KeyboardController

# ---- World ----
world = World(stage_units_in_meters=1.0, physics_dt=1.0/200.0, rendering_dt=8.0/200.0)
world.scene.add_default_ground_plane(
    z_position=0.0,
    name="default_ground_plane",
    prim_path="/World/defaultGroundPlane",
    static_friction=0.2, dynamic_friction=0.2, restitution=0.01,
)

# ---- 追加の工場モデルを読み込み（例） ----
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
factory_usd_path = os.path.join(parent_dir, "usd", "factory_scaled.usd")

stage = omni.usd.get_context().get_stage()
if os.path.exists(factory_usd_path):
    # すでに読み込まれていない場合だけ追加
    root_layer = stage.GetRootLayer()
    if factory_usd_path not in root_layer.subLayerPaths:
        root_layer.subLayerPaths.append(factory_usd_path)
    print(f"[RUN] Loaded additional USD: {factory_usd_path}")
else:
    print(f"[RUN] factory.usd not found: {factory_usd_path}")

# ---- ロボット（LiDAR付き継承クラス） ----
sf_tron1a = SFTRON1AFlatTerrainPolicyWithLiDAR(
    prim_path="/World/SF_TRON",
    name="SF_TRON",
    # 必要なら usd_path を明示（クラス実装に合わせて）
    # usd_path="/path/to/SF_TRON1A.usd",
    position=np.array([0.0, 0.0, 1.05], dtype=np.float32),

    # ★ LiDAR関連のデフォルト（必要に応じて上書き可能）
    # lidar_parent_prim="/World/SF_TRON",
    # lidar_prim_path="/World/SF_TRON/Lidar",
    # lidar_frame_id="lidar_link",
    # lidar_topic="/lidar/points",
    # lidar_rate_hz=10.0,
)
# sf_tron1a.set_lidar_config(topic="/velodyne/points_raw", rate_hz=20.0)

# ---- キーボード操作（例：前進・左右旋回）----
keyboard_controller = KeyboardController()


# ---- 物理ステップで方策forward ----
def on_physics_step(dt: float):
    sf_tron1a.forward(dt, keyboard_controller.get_command())


# ---- 実行フロー ----
world.reset()
world.play()

# 1フレーム進めてから SimulationView を取得
simulation_app.update()
sim_view = SimulationContext.instance().physics_sim_view
if sim_view is None:
    for _ in range(3):
        simulation_app.update()
        sim_view = SimulationContext.instance().physics_sim_view
        if sim_view is not None:
            break
if sim_view is None:
    raise RuntimeError("physics_sim_view を取得できませんでした。")

# ★ 既存と同じ “ベースクラス経由” initialize（重要）
EncoderPolicyControllerONNX.initialize(sf_tron1a, physics_sim_view=sim_view, set_articulation_props=False)

# ロボットの姿勢初期化
sf_tron1a.post_reset()
sf_tron1a.robot.set_joints_default_state(sf_tron1a.default_pos)

# ★ ここで LiDAR を生成し、ROS2 PointCloud2 の発行を開始
#    継承クラスに用意したメソッドを呼ぶ

sf_tron1a.add_vlp16_like_rtx_lidar_and_ros_pub(lidar_prim="/World/SF_TRON/base_Link/vlp16")
# sf_tron1a.add_vlp16_like_rtx_lidar_and_ros_pub(
#     parent_prim="/World/SF_TRON/base_Link",
#     lidar_prim="/World/SF_TRON/base_Link/vlp16",
#     frame_id="vlp16_link",
#     topic_name="/vlp16/points",
#     hz=10.0
# )

# sf_tron1a.add_livox_like_rtx_lidar_and_ros_pub(
#     parent_prim="/World/SF_TRON/base_link",          # ← 直近の親
#     lidar_prim="/World/SF_TRON/base_link/mid360",    # ← 子
#     frame_id="mid360",
#     topic_name="/lidar/points",
#     hz=10.0,
#     h_fov_deg=360.0,
#     v_fov_min_deg=-7.0,
#     v_fov_max_deg=52.0,
#     horiz_res=2048,
#     vert_res=10,
#     translation=(0.0, 0.0, 0.15),   # base_link からの相対マウント位置に合わせて調整
#     orientation_euler_deg=(0.0, 0.0, 0.0),
# )

# 物理コールバック登録
world.add_physics_callback("physics_step", on_physics_step)

try:
    while simulation_app.is_running():
        simulation_app.update()
finally:
    if world.physics_callback_exists("physics_step"):
        world.remove_physics_callback("physics_step")
    # LiDARのROSノード終了（継承クラスに用意）
    if hasattr(sf_tron1a, "shutdown_ros"):
        sf_tron1a.shutdown_ros()
    simulation_app.close()

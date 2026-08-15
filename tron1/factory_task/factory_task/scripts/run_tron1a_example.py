from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import os
import numpy as np
import omni
from omni.isaac.core.simulation_context import SimulationContext
from omni.isaac.core import World
from isaacsim.storage.native import get_assets_root_path
from factory_task.controllers.encoder_policy_controller_onnx import EncoderPolicyControllerONNX
from factory_task.policies.sf_tron1a_flat_policy import SFTRON1AFlatTerrainPolicy
from factory_task.scripts.keyboard_controller import KeyboardController

world = World(stage_units_in_meters=1.0, physics_dt=1.0/200.0, rendering_dt=8.0/200.0)
world.scene.add_default_ground_plane(
    z_position=0.0, name="default_ground_plane", prim_path="/World/defaultGroundPlane",
    static_friction=0.2, dynamic_friction=0.2, restitution=0.01,
)
# World の stage を取得
stage = omni.usd.get_context().get_stage()

# 実行中のスクリプトと同じ階層の usd/factory.usdz を指定
current_dir = os.path.dirname(os.path.abspath(__file__))
factory_usd_path = os.path.join(current_dir, "../usd", "factory_scaled.usd")

# factory.usd をステージに追加
if os.path.exists(factory_usd_path):
    stage.GetRootLayer().subLayerPaths.append(factory_usd_path)
    print(f"Loaded additional USD: {factory_usd_path}")
else:
    print(f"factory.usd not found: {factory_usd_path}")

assets_root = get_assets_root_path()
sf_tron1a = SFTRON1AFlatTerrainPolicy(
    prim_path="/World/SF_TRON", name="SF_TRON",
    position=np.array([0.0, 0.0, 1.05], dtype=np.float32),
)

# keyboardからTRON1Aを操作する
keyboard_controller = KeyboardController()


def on_physics_step(dt: float):
    sf_tron1a.forward(dt, keyboard_controller.get_command())


# --- 実行フロー ---
world.reset()
world.play()

# 1フレーム進めてから SimulationView を取得
# 4回トライする
for _ in range(4):
    simulation_app.update()
    sim_view = SimulationContext.instance().physics_sim_view
    if sim_view is not None:
        break
if sim_view is None:
    raise RuntimeError("physics_sim_view を取得できませんでした。")

# ★ ベースクラス経由で初期化（ここがポイント）
EncoderPolicyControllerONNX.initialize(
    sf_tron1a, physics_sim_view=sim_view, set_articulation_props=False)

sf_tron1a.post_reset()
sf_tron1a.robot.set_joints_default_state(sf_tron1a.default_pos)

# 初期化完了後にコールバック登録
world.add_physics_callback("physics_step", on_physics_step)

while simulation_app.is_running():
    simulation_app.update()

if world.physics_callback_exists("physics_step"):
    world.remove_physics_callback("physics_step")
simulation_app.close()

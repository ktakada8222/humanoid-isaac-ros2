# run_humanoid_example.py
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import carb, omni, omni.appwindow
from omni.isaac.core import World
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.policy.examples.robots.h1 import H1FlatTerrainPolicy
from isaacsim.robot.policy.examples.controllers.policy_controller import PolicyController  # ★追加
from omni.isaac.core.simulation_context import SimulationContext

world = World(stage_units_in_meters=1.0, physics_dt=1.0/200.0, rendering_dt=8.0/200.0)

world.scene.add_default_ground_plane(
    z_position=0.0, name="default_ground_plane", prim_path="/World/defaultGroundPlane",
    static_friction=0.2, dynamic_friction=0.2, restitution=0.01,
)

assets_root = get_assets_root_path()
h1 = H1FlatTerrainPolicy(
    prim_path="/World/H1", name="H1",
    usd_path=assets_root + "/Isaac/Robots/Unitree/H1/h1.usd",
    position=np.array([0.0, 0.0, 1.05], dtype=np.float32),
)

base_command = np.array([0.0, 0.0, 0.0], dtype=np.float32)
keymap = {
    "NUMPAD_8": np.array([0.75, 0.0, 0.0], dtype=np.float32),
    "UP":       np.array([0.75, 0.0, 0.0], dtype=np.float32),
    "NUMPAD_4": np.array([0.0, 0.0, 0.75], dtype=np.float32),
    "LEFT":     np.array([0.0, 0.0, 0.75], dtype=np.float32),
    "NUMPAD_6": np.array([0.0, 0.0, -0.75], dtype=np.float32),
    "RIGHT":    np.array([0.0, 0.0, -0.75], dtype=np.float32),
}
appwindow = omni.appwindow.get_default_app_window()
input_iface = carb.input.acquire_input_interface()
keyboard = appwindow.get_keyboard()

def on_key(event, *args, **kwargs):
    global base_command
    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        if event.input.name in keymap:
            base_command += keymap[event.input.name]
    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        if event.input.name in keymap:
            base_command -= keymap[event.input.name]
    return True

sub_keyboard = input_iface.subscribe_to_keyboard_events(keyboard, on_key)

def on_physics_step(dt: float):
    h1.forward(dt, base_command)

# --- 実行フロー ---
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

# ★ ベースクラス経由で初期化（ここがポイント）
PolicyController.initialize(h1, physics_sim_view=sim_view, set_articulation_props=False)

h1.post_reset()
h1.robot.set_joints_default_state(h1.default_pos)

# 初期化完了後にコールバック登録
world.add_physics_callback("physics_step", on_physics_step)

while simulation_app.is_running():
    simulation_app.update()

if world.physics_callback_exists("physics_step"):
    world.remove_physics_callback("physics_step")
simulation_app.close()

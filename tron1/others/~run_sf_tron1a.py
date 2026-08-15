# # run_sf_tron1a.py  (pip 版 Isaac Sim 用)

# # ★ここを修正：pip 版の SimulationApp を使う
# from isaacsim import SimulationApp
# simulation_app = SimulationApp({"headless": False})  # GUIあり。ヘッドレスは True

# import sys
# import numpy as np
# from pathlib import Path

# # SimulationApp 生成後に omni/isaac を import する
# from omni.isaac.core import World
# import omni

# # ← ここをあなたの sf_tron1a.py があるフォルダに合わせる
# BASE_DIR = Path("/home/tron/humanoid-limx-oli-isaac-ros2/tron1/isaacsim/robot.policy").resolve()
# sys.path.append(str(BASE_DIR))

# from sf_tron1a import SFTRON1AFlatTerrainPolicy  # あなたのクラス

# # World 作成
# world = World(stage_units_in_meters=1.0)

# # ロボット & policy 初期化
# ctrl = SFTRON1AFlatTerrainPolicy(prim_path="/World/Robot")
# ctrl.initialize()

# world.reset()
# world.play()

# # 毎フレーム forward を呼ぶ
# app = omni.kit.app.get_app()
# def on_update(e):
#     dt = world.get_physics_dt()
#     ctrl.forward(dt, np.array([0.0, 0.0, 0.0], dtype=np.float32))  # 必要ならコマンド入力

# _sub = app.get_update_event_stream().create_subscription_to_pop(on_update)

# # メインループ
# while simulation_app.is_running():
#     simulation_app.update()

# simulation_app.close()

# run_h1.py  —— pip 版 Isaac Sim 用ランナー（最小）
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})  # GUIあり。ヘッドレス実行なら True

import sys
import numpy as np
from pathlib import Path
from omni.isaac.core import World
import omni
import os

# ← h1.py を置いたフォルダに合わせて変更（絶対パス推奨）
current_dir = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = Path(os.path.abspath(os.path.join(current_dir, "../isaacsim/robot.policy"))).resolve() # humanoid-limx-oli-isaac-ros2/tron1/isaacsim/robot.policy
sys.path.append(str(BASE_DIR))

# h1.py 内のクラスを読み込み
from h1 import H1FlatTerrainPolicy

# World 作成
world = World(stage_units_in_meters=1.0)

# コントローラ作成＆初期化（USD/pt/yaml は h1.py 側が get_assets_root_path で解決）
ctrl = H1FlatTerrainPolicy(prim_path="/World/Robot")
ctrl.initialize()

# 再生開始
world.reset()
world.play()

# 毎フレーム、ポリシーの forward を呼ぶ（ここではコマンド=0）
app = omni.kit.app.get_app()
def on_update(e):
    dt = world.get_physics_dt()
    ctrl.forward(dt, np.array([0.0, 0.0, 0.0], dtype=np.float32))  # (v_x, v_y, w_z)

_sub = app.get_update_event_stream().create_subscription_to_pop(on_update)

# メインループ
while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()

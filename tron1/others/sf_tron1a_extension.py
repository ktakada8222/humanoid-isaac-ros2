# sf_tron1a_extension.py
# Copyright (c) 2024
# MIT-like: you may adapt in your project

import os
import sys
from typing import Optional

import numpy as np
import omni.ext
import omni
from isaacsim.core import World
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.examples.interactive.base_sample import BaseSampleUITemplate


# ---- Example 実体：LOAD/RESET の本体 ----
class SFTron1AExample:
    """
    Humanoid: SF_TRON1A example.
    - LOAD:  ワールド生成 + あなたのコントローラ起動
    - RESET: 物理だけリセット
    """
    def __init__(self, prim_path: str = "/World/Robot"):
        self.prim_path = prim_path
        self.world: Optional[World] = None
        self.ctrl = None
        self._sub = None

        # sf_tron1a.py を同ディレクトリ or 拡張直下に置いている前提で import パスを通す
        this_dir = os.path.dirname(os.path.abspath(__file__))
        if this_dir not in sys.path:
            sys.path.append(this_dir)

    def load_world(self):
        if self.world is not None:
            # すでに起動済みなら何もしない（必要ならここでRESETでもOK）
            return

        # あなたのポリシー・コントローラを読み込み
        from sf_tron1a import SFTRON1AFlatTerrainPolicy  # ← あなたが作ったクラス

        # ワールド生成
        self.world = World(stage_units_in_meters=1.0)

        # コントローラ作成（USD/pt/yaml は sf_tron1a.py 内で相対パス解決される想定）
        self.ctrl = SFTRON1AFlatTerrainPolicy(prim_path=self.prim_path)
        self.ctrl.initialize()

        # 再生
        self.world.reset()
        self.world.play()

        app = omni.kit.app.get_app()

        def on_update(e):
            if not self.world:
                return
            dt = self.world.get_physics_dt()
            # 必要ならここでキーボード入力などから command を与える
            cmd = np.array([0.0, 0.0, 0.0], dtype=np.float32)
            self.ctrl.forward(dt, cmd)

        # 毎フレーム呼ばれるコールバックを登録
        self._sub = app.get_update_event_stream().create_subscription_to_pop(on_update)

    def reset_world(self):
        if self.world:
            self.world.reset()

    def stop_world(self):
        # 必要に応じて停止/後片付け
        if self._sub:
            self._sub = None
        self.world = None
        self.ctrl = None


# ---- Extension：Examples ブラウザにカードを登録 ----
class SFTron1AExampleExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self.example_name = "Humanoid: SF_TRON1A"
        self.category = "Policy"

        overview = (
            "This Example shows an SF_TRON1A humanoid running a flat terrain policy "
            "trained in Isaac Lab (encoder + policy)."
            "\n\nPress 'Open in IDE' to view/edit the source code."
        )

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": self.example_name,
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/isaac_lab_tutorials/tutorial_policy_deployment.html",
            "overview": overview,
            "sample": SFTron1AExample(),  # ← LOAD/RESET を持つオブジェクト
        }

        ui_handle = BaseSampleUITemplate(**ui_kwargs)

        # Examples ブラウザに登録（左パネルにカードが出る）
        get_browser_instance().register_example(
            name=self.example_name,
            execute_entrypoint=ui_handle.build_window,  # パネル生成
            ui_hook=ui_handle.build_ui,                 # UI（LOAD/RESET）を貼る
            category=self.category,
        )

    def on_shutdown(self):
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)

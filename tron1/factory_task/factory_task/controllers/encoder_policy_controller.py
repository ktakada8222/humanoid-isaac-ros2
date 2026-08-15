# encoder_policy_controller.py
from __future__ import annotations
import io
from pathlib import Path
from typing import Optional

import carb
import numpy as np
import omni
import torch
from isaacsim.core.api.controllers.base_controller import BaseController
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.prims import define_prim, get_prim_at_path

# 既存PolicyControllerが使っているヘルパ
from isaacsim.robot.policy.examples.controllers.config_loader import (
    get_articulation_props,
    get_physics_properties,
    get_robot_joint_properties,
    parse_env_config,
)

class EncoderPolicyController(BaseController):
    """
    エンコーダ(encoder.pt)→ポリシー(policy.pt)の二段推論をサポートするコントローラ。
    - encoder.pt / policy.pt は TorchScript (torch.jit.load 可能) を想定
    - env.yaml から物理設定や関節ゲイン等を読み込み
    """

    def __init__(
        self,
        name: str,
        prim_path: str,
        root_path: Optional[str] = None,
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        prim = get_prim_at_path(prim_path)
        if not prim.IsValid():
            prim = define_prim(prim_path, "Xform")
            if usd_path:
                prim.GetReferences().AddReference(usd_path)
            else:
                carb.log_error("unable to add robot usd, usd_path not provided")

        if root_path is None:
            self.robot = SingleArticulation(prim_path=prim_path, name=name, position=position, orientation=orientation)
        else:
            self.robot = SingleArticulation(prim_path=root_path, name=name, position=position, orientation=orientation)

        # モデル格納
        self.encoder = None
        self.policy  = None
        self.policy_env_params = None

        # 物理プロパティ
        self._decimation = 1
        self._dt = 1.0 / 60.0
        self.render_interval = 1

        # 既定関節姿勢など
        self.default_pos = None
        self.default_vel = None

    # ---------- ローダ ----------
    def _jit_load_local_or_nucleus(self, file_path: str | Path):
        """
        TorchScript (.pt) を読み込む。
        - ローカルパス: torch.jit.load で直接
        - Nucleus など omni.client 対応パス: read_file → BytesIO → jit.load
        """
        p = str(file_path)
        try:
            # まずはローカルとして試す
            return torch.jit.load(p, map_location="cpu")
        except Exception:
            # omni.client 経由で読む（Nucleus等）
            result, _, content = omni.client.read_file(p)
            if result != omni.client.Result.OK:
                raise FileNotFoundError(f"Failed to read model: {p} (omni.client result={result})")
            buf = io.BytesIO(memoryview(content).tobytes())
            return torch.jit.load(buf, map_location="cpu")

    def load_policy(
        self,
        policy_file_path: str | Path,
        policy_encoder_file_path: Optional[str | Path],
        policy_env_path: str | Path,
    ) -> None:
        """
        2段構成のポリシーをロード。
        Args:
            policy_file_path:   policy .pt (TorchScript)
            policy_encoder_file_path: encoder .pt (TorchScript) / Noneなら単段
            policy_env_path:    env.yaml
        """
        # TorchScript 読み込み
        self.policy = self._jit_load_local_or_nucleus(policy_file_path)
        if policy_encoder_file_path is not None:
            self.encoder = self._jit_load_local_or_nucleus(policy_encoder_file_path)

        # env.yaml 読み込み（物理設定・関節ゲイン等）
        self.policy_env_params = parse_env_config(str(policy_env_path))
        self._decimation, self._dt, self.render_interval = get_physics_properties(self.policy_env_params)

    def initialize(
        self,
        physics_sim_view: omni.physics.tensors.SimulationView = None,
        effort_modes: str = "force",
        control_mode: str = "position",
        set_gains: bool = True,
        set_limits: bool = True,
        set_articulation_props: bool = True,
    ) -> None:
        self.robot.initialize(physics_sim_view=physics_sim_view)
        self.robot.get_articulation_controller().set_effort_modes(effort_modes)
        self.robot.get_articulation_controller().switch_control_mode(control_mode)

        max_effort, max_vel, stiffness, damping, self.default_pos, self.default_vel = get_robot_joint_properties(
            self.policy_env_params, self.robot.dof_names
        )
        if set_gains:
            self.robot._articulation_view.set_gains(stiffness, damping)
        if set_limits:
            self.robot._articulation_view.set_max_efforts(max_effort)
            self.robot._articulation_view.set_max_joint_velocities(max_vel)
        if set_articulation_props:
            self._set_articulation_props()

    def _set_articulation_props(self) -> None:
        articulation_prop = get_articulation_props(self.policy_env_params)
        pos_iter = articulation_prop.get("solver_position_iteration_count")
        vel_iter = articulation_prop.get("solver_velocity_iteration_count")
        stab_th = articulation_prop.get("stabilization_threshold")
        self_coll = articulation_prop.get("enabled_self_collisions")
        sleep_th = articulation_prop.get("sleep_threshold")
        if pos_iter not in [None, float("inf")]:
            self.robot.set_solver_position_iteration_count(pos_iter)
        if vel_iter not in [None, float("inf")]:
            self.robot.set_solver_velocity_iteration_count(vel_iter)
        if stab_th not in [None, float("inf")]:
            self.robot.set_stabilization_threshold(stab_th)
        if isinstance(self_coll, bool):
            self.robot.set_enabled_self_collisions(self_coll)
        if sleep_th not in [None, float("inf")]:
            self.robot.set_sleep_threshold(sleep_th)

    # ---------- 推論 ----------
    def _compute_action(self, obs: np.ndarray) -> np.ndarray:
        """
        obs -> (encoder?) -> policy -> action
        どちらも TorchScript 前提（forwardを1バッチのテンソルで受ける）。
        """
        with torch.no_grad():
            x = torch.from_numpy(obs).view(1, -1).float()
            if self.encoder is not None:
                x = self.encoder(x)
            action = self.policy(x).detach().view(-1).cpu().numpy()
        return action

    def _compute_observation(self) -> NotImplementedError:
        """
        Computes the observation. Not implemented.
        """

        raise NotImplementedError(
            "Compute observation need to be implemented, expects np.ndarray in the structure specified by env yaml"
        )

    def forward(self) -> NotImplementedError:
        """
        Forwards the controller. Not implemented.
        """
        raise NotImplementedError(
            "Forward needs to be implemented to compute and apply robot control from observations"
        )

    def post_reset(self) -> None:
        """
        Called after the controller is reset.
        """
        self.robot.post_reset()


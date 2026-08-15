# encoder_policy_controller_onnx.py
from typing import Optional, Tuple, List
import io
import numpy as np
import onnxruntime as ort
import carb
import omni
from isaacsim.core.api.controllers.base_controller import BaseController
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from omni.physx import get_physx_simulation_interface

# 既存の helpers を流用（元の PolicyController が使っているやつ）
from isaacsim.robot.policy.examples.controllers.config_loader import (
    get_articulation_props, get_physics_properties, get_robot_joint_properties, parse_env_config
)

class _OnnxRunner:
    def __init__(self, onnx_path: str):
        so = ort.SessionOptions()
        # メモ: 必要なら最適化レベル等を設定
        self.sess = ort.InferenceSession(onnx_path, sess_options=so, providers=["CPUExecutionProvider"])
        self.in_names  = [i.name for i in self.sess.get_inputs()]
        self.out_names = [o.name for o in self.sess.get_outputs()]
        # 期待される入力シェイプ/ランクを保持（例: ['mlp_input'] が shape=[obs_dim] または [None, obs_dim]）
        self.in_shapes = [i.shape for i in self.sess.get_inputs()]
        self.in_ranks  = [len(i.shape) for i in self.sess.get_inputs()]
        carb.log_info(f"[ONNX] loaded: {onnx_path} | inputs={self.in_names} outputs={self.out_names}")

    def __call__(self, x: np.ndarray) -> np.ndarray:
        # # x shape: (B, obs_dim)
        # feed = {self.in_names[0]: x.astype(np.float32)}
        x = np.asarray(x, dtype=np.float32)
        # 期待ランクに応じて整形（1D or 2D）
        exp_rank = self.in_ranks[0]
        if exp_rank == 1:
            # モデルは [obs_dim] を期待
            x = x.reshape(-1)
        elif exp_rank == 2:
            # モデルは [B, obs_dim] を期待
            if x.ndim == 1:
                x = x.reshape(1, -1)
            elif x.ndim == 2:
                pass  # そのまま
            else:
                raise ValueError(f"Unexpected input ndim {x.ndim} for 2D model input")
        else:
            # 想定外（ほぼ無い）はログだけ
            carb.log_warn(f"[ONNX] unusual expected rank {exp_rank}, passing x.ndim={x.ndim}")
        feed = {self.in_names[0]: x}
        y = self.sess.run(self.out_names, feed)[0]
        return np.asarray(y)

class EncoderPolicyControllerONNX(BaseController):
    """
    Encoder + Policy を ONNX で推論するコントローラ。
    - encoder_onnx が None の場合は policy に観測を直接入力
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
                prim.GetReferences().AddReference(str(usd_path))
            else:
                carb.log_error("unable to add robot usd, usd_path not provided")

        self.robot = SingleArticulation(
            prim_path=(root_path or prim_path),
            name=name,
            position=position,
            orientation=orientation,
        )

        self.encoder = None
        self.policy = None
        self.policy_env_params = None
        self.obs_mean = None
        self.obs_std = None

    def load_policy_onnx(
        self,
        policy_onnx_path: str,
        policy_env_path: str,
        encoder_onnx_path: Optional[str] = None,
        obs_norm_stats: Optional[dict] = None,
    ) -> None:
        """ONNXモデルをロード。encoder があれば encoder→policy の二段推論。"""
        self.policy = _OnnxRunner(policy_onnx_path)
        self.encoder = _OnnxRunner(encoder_onnx_path) if encoder_onnx_path else None

        self.policy_env_params = parse_env_config(policy_env_path)
        self._decimation, self._dt, self.render_interval = get_physics_properties(self.policy_env_params)

        # 例：env から取得できない場合は固定で落とし込む
        self.obs_dim = int(self.policy_env_params.get("obs_dim", 36))        # 3n+12 に合わせる
        self.hist_len = int(self.policy_env_params.get("obs_history", 10))   # 例：10
        import numpy as np
        self._obs_hist = np.zeros((self.hist_len, self.obs_dim), dtype=np.float32)

        if obs_norm_stats:
            import numpy as np
            self.obs_mean = np.asarray(obs_norm_stats.get("mean"), dtype=np.float32)
            self.obs_std  = np.asarray(obs_norm_stats.get("std"),  dtype=np.float32)

    def _push_obs(self, obs: np.ndarray):
        # obs: shape (obs_dim,)
        self._obs_hist[:-1] = self._obs_hist[1:]
        self._obs_hist[-1]  = obs.astype(np.float32)

    def _encoder_input(self) -> np.ndarray:
        # モデルの期待に合わせて渡す。形状は実際の onnx 入力 rank/shape を見て整形
        if self.encoder is None:
            return None
        # 例：rank=2 で [H,D] を期待
        return self._obs_hist.copy()

    # === PolicyController 互換: post_reset を用意 ===
    def post_reset(self) -> None:
        """Reset internal controller/robot state after world reset/play."""
        # ロボット側（PhysXビュー等）のリセット
        try:
            self.robot.post_reset()
        except Exception:
            pass
        # コントローラ内部状態のリセット（必要に応じて上位クラスで上書きOK）
        # 典型的に使うメンバは存在チェックしてから初期化
        if hasattr(self, "_policy_counter"):
            self._policy_counter = 0
        if hasattr(self, "action"):
            try:
                # 既知のヒト型は 19 DOF。無ければ長さを推定してゼロ化
                dof = len(getattr(self, "action"))
                self.action = np.zeros(dof, dtype=np.float32)
            except Exception:
                self.action = np.zeros(0, dtype=np.float32)
        if hasattr(self, "_previous_action"):
            try:
                dof = len(getattr(self, "_previous_action"))
                self._previous_action = np.zeros(dof, dtype=np.float32)
            except Exception:
                self._previous_action = np.zeros(0, dtype=np.float32)

    # === PolicyController 互換: _compute_action を提供（呼び名を合わせる） ===
    def _compute_action(self, obs: np.ndarray) -> np.ndarray:
        """Keep PolicyController-compatible API."""
        return self.compute_action_from_obs(obs)

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

        get_physx_simulation_interface().flush_changes()

        self.robot.get_articulation_controller().switch_control_mode(control_mode)

        max_effort, max_vel, stiffness, damping, self.default_pos, self.default_vel = get_robot_joint_properties(
            self.policy_env_params, self.robot.dof_names
        )
        if set_gains:
            self.robot._articulation_view.set_gains(stiffness, damping)
        if set_limits:
            self.robot._articulation_view.set_max_efforts(max_effort)

            get_physx_simulation_interface().flush_changes()

            self.robot._articulation_view.set_max_joint_velocities(max_vel)
        if set_articulation_props:
            self._set_articulation_props()

    def _set_articulation_props(self) -> None:
        articulation_prop = get_articulation_props(self.policy_env_params)
        spic = articulation_prop.get("solver_position_iteration_count")
        svic = articulation_prop.get("solver_velocity_iteration_count")
        stab = articulation_prop.get("stabilization_threshold")
        self_coll = articulation_prop.get("enabled_self_collisions")
        sleep = articulation_prop.get("sleep_threshold")

        if spic not in [None, float("inf")]: self.robot.set_solver_position_iteration_count(spic)
        if svic not in [None, float("inf")]: self.robot.set_solver_velocity_iteration_count(svic)
        if stab not in [None, float("inf")]: self.robot.set_stabilization_threshold(stab)
        if isinstance(self_coll, bool):      self.robot.set_enabled_self_collisions(self_coll)
        if sleep not in [None, float("inf")]:self.robot.set_sleep_threshold(sleep)

    # def compute_action_from_obs(self, obs_np: np.ndarray) -> np.ndarray:
    #     # x = obs_np.astype(np.float32).reshape(1, -1)
    #     x = obs_np.astype(np.float32)
    #     if self.obs_mean is not None and self.obs_std is not None:
    #         x = (x - self.obs_mean) / (self.obs_std + 1e-8)
    #     if self.encoder is not None:
    #         z = self.encoder(x)
    #         print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
    #         act = self.policy(z)
    #     else:
    #         act = self.policy(x)
    #     # return act.reshape(-1)
    #     return np.asarray(act).reshape(-1)
    # def compute_action_from_obs(self, obs_np: np.ndarray, commands_5: np.ndarray = None) -> np.ndarray:
    #     x = obs_np.astype(np.float32)

    #     if self.obs_mean is not None and self.obs_std is not None:
    #         x = (x - self.obs_mean) / (self.obs_std + 1e-8)

    #     # 履歴に積む（encoder 用）
    #     if hasattr(self, "_obs_hist") and x.shape[0] == self.obs_dim:
    #         self._push_obs(x)

    #     if self.encoder is not None:
    #         enc_in = self._encoder_input()        # shape (H,D) or flat
    #         z = self.encoder(enc_in)
    #         # ★★★ ここが重要： z + 現フレーム obs(x) + commands(5) を連結して policy に渡す ★★★
    #         if commands_5 is None:
    #             # コマンドが無い場合はゼロ（長さ5）
    #             commands_5 = np.zeros(5, dtype=np.float32)
    #         pol_in = np.concatenate([z.reshape(-1), x.reshape(-1), commands_5.astype(np.float32).reshape(-1)], axis=0)
    #         act = self.policy(pol_in)
    #     else:
    #         # encoder なし設計の policy（= 履歴360を直接入力する など）の場合に合わせてここを切り替える
    #         act = self.policy(x)

    #     return np.asarray(act).reshape(-1)

    def compute_action_from_obs(self, obs_policy_35: np.ndarray, commands_3: np.ndarray = None, heading: float = 0.0,
                                obs_encoder_36: np.ndarray = None) -> np.ndarray:
        # ---- policy 用の 35D ----
        x = obs_encoder_36.astype(np.float32)
        # x = obs_policy_35.astype(np.float32)

        if self.obs_mean is not None and self.obs_std is not None:
            x = (x - self.obs_mean) / (self.obs_std + 1e-8)

        # ---- encoder 用の 36D を履歴に積む ----
        if obs_encoder_36 is not None:
            enc_obs = obs_encoder_36.astype(np.float32)
            assert enc_obs.shape[0] == self.obs_dim, f"encoder obs dim {enc_obs.shape[0]} != {self.obs_dim}"
            self._push_obs(enc_obs)

        # commands_4 = [vx, vy, wz, heading]
        # if commands_3 is None:
        #     commands_3 = np.zeros(3, dtype=np.float32)
        # cmd4 = np.array([commands_3[0], commands_3[1], commands_3[2], heading], dtype=np.float32)

        if self.encoder is not None:
            z = self.encoder(self._encoder_input())   # -> 3 次元
            pol_in = np.concatenate([z.reshape(-1), x.reshape(-1), commands_3], axis=0)  # 3 + 35 + 4 = 42
            act = self.policy(pol_in)
        else:
            act = self.policy(x)

        return np.asarray(act).reshape(-1)
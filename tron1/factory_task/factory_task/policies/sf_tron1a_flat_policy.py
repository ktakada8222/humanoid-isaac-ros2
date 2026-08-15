# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from typing import Optional
from pathlib import Path
import math
import numpy as np
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.core.utils.types import ArticulationAction
from factory_task.controllers.encoder_policy_controller_onnx import EncoderPolicyControllerONNX


class SFTRON1AFlatTerrainPolicy(EncoderPolicyControllerONNX):
    """The SF_TRON1A Humanoid running Flat Terrain Policy Locomotion Policy"""

    def __init__(
        self,
        prim_path: str,
        root_path: Optional[str] = None,
        name: str = "sf_tron1a",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        """
        Initialize H1 robot and import flat terrain policy.

        Args:
            prim_path (str) -- prim path of the robot on the stage
            root_path (Optional[str]): The path to the articulation root of the robot
            name (str) -- name of the quadruped
            usd_path (str) -- robot usd filepath in the directory
            position (np.ndarray) -- position of the robot
            orientation (np.ndarray) -- orientation of the robot

        """

        base_dir = Path(__file__).resolve().parent
        parent_dir = base_dir.parent
        usd_path = (parent_dir / "usd" / "SF_TRON1A.usd").resolve()
        if not usd_path.exists():
            raise FileNotFoundError(f"USD not found: {usd_path}")
        usd_path = usd_path.as_posix()

        # モデルと設定（存在するファイル名に合わせて調整）
        policy_onnx = (parent_dir / "models" / "policy.onnx").resolve()
        encoder_onnx = (parent_dir / "models" / "encoder.onnx").resolve()  # 無いなら後で None に
        env_yaml = (parent_dir / "config" / "env.yaml").resolve()      # ある方を使う

        if not policy_onnx.exists():
            raise FileNotFoundError(f"Policy ONNX not found: {policy_onnx}")
        if not encoder_onnx.exists():
            encoder_onnx = None  # エンコーダ分離なし
        if not env_yaml.exists():
            raise FileNotFoundError(f"Env YAML not found: {env_yaml}")

        super().__init__(name, prim_path, root_path, usd_path, position, orientation)

        # 観測正規化統計があれば渡す（無ければ None でOK）
        obs_norm_stats = None
        self.load_policy_onnx(
            policy_onnx_path=str(policy_onnx),
            policy_env_path=str(env_yaml),
            encoder_onnx_path=str(encoder_onnx) if encoder_onnx else None,
            obs_norm_stats=obs_norm_stats,
        )
        
        self._action_scale = 0.25    
        self._previous_action = np.zeros(8)    # TODO CHECK
        self._policy_counter = 0

        self._gait_resample_T = 5.0  # resampling_time_range=(5.0, 5.0)
        self._gait_freq_range = (0.8, 1.6)  # frequencies=(0.8, 1.6)
        self._gait_offset_const = 0.5       # offsets=(0.5, 0.5)
        self._gait_duration_const = 0.5     # durations=(0.5, 0.5)

        self._gait_swing_height = 0.15  # モデルに合わせて  TODO

        # 内部状態
        self._gait_time_acc = 0.0           # 前回サンプルからの経過時間
        self._gait_phase = 0.0              # [0,1) の位相
        self._gait_frequency = np.random.uniform(*self._gait_freq_range)
        self._gait_offset = self._gait_offset_const
        self._gait_duration = self._gait_duration_const

        # 学習時スケール（PolicyCfgで指定していたもの）
        self._scale_base_ang_vel = 0.25
        self._scale_joint_vel = 0.05

    # ---- 追加：gaitコマンドの再サンプル ----
    def _resample_gait_command(self):
        self._gait_frequency = np.random.uniform(*self._gait_freq_range)
        self._gait_offset = self._gait_offset_const
        self._gait_duration = self._gait_duration_const
        self._gait_time_acc = 0.0  # 経過時間はリセット（位相は継続でもOK）

    # ---- 追加：位相の更新＆必要なら再サンプル ----
    def _update_gait_state(self, dt: float):
        # 位相を進める： phase(t+dt) = (phase + dt * frequency) % 1
        self._gait_phase = (self._gait_phase + dt * self._gait_frequency) % 1.0

        # リサンプル判定
        self._gait_time_acc += dt
        if self._gait_time_acc >= self._gait_resample_T:
            self._resample_gait_command()

    # ---- 追加：Labの get_gait_phase 相当（sin/cos）----
    def _get_gait_phase_obs(self) -> np.ndarray:
        phase = self._gait_phase
        return np.array([math.sin(2.0 * math.pi * phase), math.cos(2.0 * math.pi * phase)], dtype=np.float32)

    def _get_gait_command_obs(self) -> np.ndarray:
        # エンコーダ用の仕様に合わせて 4 要素を返せるようにしておく
        return np.array([self._gait_frequency, self._gait_offset, self._gait_duration, self._gait_swing_height], dtype=np.float32)

    def _compute_observation_for_policy(self) -> np.ndarray:
        """
        policy用の現フレーム観測ベクトル（35D = 3+3+n+n+n+2+3, n=8想定）
        - gait_command は 3 要素 [frequency, offset, duration]
        """
        # --- base in body frame ---
        ang_vel_I = self.robot.get_angular_velocity()          # (3,)
        _, q_IB = self.robot.get_world_pose()
        R_IB = quat_to_rot_matrix(q_IB)
        R_BI = R_IB.T
        ang_vel_b = np.matmul(R_BI, ang_vel_I)
        gravity_b = np.matmul(R_BI, np.array([0.0, 0.0, -1.0]))

        # --- joints ---
        q = self.robot.get_joint_positions()                 # (n,)
        dq = self.robot.get_joint_velocities()                # (n,)
        # dof = q.shape[0]

        joint_pos_rel = (q - self.default_pos).astype(np.float32)

        # スケール
        base_ang_vel_scaled = ang_vel_b.astype(np.float32) * self._scale_base_ang_vel
        proj_gravity = gravity_b.astype(np.float32)
        joint_vel_scaled = dq.astype(np.float32) * self._scale_joint_vel

        last_action = np.asarray(self._previous_action, dtype=np.float32)

        # # last_action（長さをDOFに合わせる）
        # if hasattr(self, "_previous_action") and self._previous_action is not None and len(self._previous_action) >= dof:
        #     last_action = np.asarray(self._previous_action[:dof], dtype=np.float32)
        # else:
        #     last_action = np.zeros(dof, dtype=np.float32)

        # gait
        phase = self._gait_phase
        gait_phase = np.array([math.sin(2.0 * math.pi * phase), math.cos(2.0 * math.pi * phase)], dtype=np.float32)
        gait_cmd = np.array([self._gait_frequency, self._gait_offset, self._gait_duration], dtype=np.float32)  # 3要素

        # 連結
        obs = np.concatenate([
            base_ang_vel_scaled,   # (3,)
            proj_gravity,          # (3,)
            joint_pos_rel,         # (n,)
            joint_vel_scaled,      # (n,)
            last_action,           # (n,)
            gait_phase,            # (2,)
            gait_cmd,              # (4,)
        ], axis=0).astype(np.float32)

        return obs

    def _compute_observation_for_encoder(self) -> np.ndarray:
        """
        encoder用の現フレーム観測ベクトル（36D = 3+3+n+n+n+2+4, n=8想定）
        - gait_command は 4 要素 [frequency, offset, duration, swing_height]
        """
        # --- base in body frame ---
        ang_vel_I = self.robot.get_angular_velocity()
        _, q_IB = self.robot.get_world_pose()
        R_IB = quat_to_rot_matrix(q_IB)
        R_BI = R_IB.T
        ang_vel_b = R_BI @ ang_vel_I
        gravity_b = R_BI @ np.array([0.0, 0.0, -1.0], dtype=np.float32)

        # --- joints ---
        q = self.robot.get_joint_positions()
        dq = self.robot.get_joint_velocities()
        # dof = q.shape[0]

        joint_pos_rel = (q - self.default_pos).astype(np.float32)

        base_ang_vel_scaled = ang_vel_b.astype(np.float32) * self._scale_base_ang_vel
        proj_gravity = gravity_b.astype(np.float32)
        joint_vel_scaled = dq.astype(np.float32) * self._scale_joint_vel

        last_action = np.asarray(self._previous_action, dtype=np.float32)

        phase = self._gait_phase
        gait_phase = np.array([math.sin(2.0 * math.pi * phase), math.cos(2.0 * math.pi * phase)], dtype=np.float32)
        gait_cmd4 = np.array([self._gait_frequency, self._gait_offset, self._gait_duration, self._gait_swing_height],
                             dtype=np.float32)  # 4要素

        obs = np.concatenate([
            base_ang_vel_scaled,   # (3,)
            proj_gravity,          # (3,)
            joint_pos_rel,         # (n,)
            joint_vel_scaled,      # (n,)
            last_action,           # (n,)
            gait_phase,            # (2,)
            gait_cmd4,             # (4,)
        ], axis=0).astype(np.float32)

        return obs

    def forward(self, dt, command):
        # 位相更新
        self._update_gait_state(dt)

        if self._policy_counter % self._decimation == 0:
            # 現フレーム観測（policy/encoder 両方作る）
            obs_policy_35 = self._compute_observation_for_policy()
            obs_encoder_36 = self._compute_observation_for_encoder()

            # commands: [vx, vy, wz]（env側の生成コマンドに相当）
            if command is None:
                vx, vy, wz = 0.0, 0.0, 0.0
            else:
                vx, vy, wz = command

            heading = 0.0  # 必要ならここで設定

            # encoder_policy_controller_onnx.py の compute_action_from_obs 拡張版に合わせて渡す
            self.action = self.compute_action_from_obs(
                obs_policy_35,
                np.array([vx, vy, wz], dtype=np.float32),
                heading,
                obs_encoder_36=obs_encoder_36
            )

            self._previous_action = np.asarray(self.action, dtype=np.float32).copy()

        # 関節コマンド反映
        action_cmd = ArticulationAction(joint_positions=self.default_pos + (self.action * self._action_scale))
        self.robot.apply_action(action_cmd)
        # print(action_cmd)
        self._policy_counter += 1

    def initialize(self):
        """
        Overloads the default initialize function to use default articulation root properties in the USD
        """
        return super().initialize(set_articulation_props=False)

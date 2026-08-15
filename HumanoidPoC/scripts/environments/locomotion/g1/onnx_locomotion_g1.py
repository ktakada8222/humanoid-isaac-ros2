"""Script to play a checkpoint if an RL agent from RSL-RL."""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

# local imports
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cli_args  # isort: skip

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default="Isaac-Velocity-Rough-RtxLidar-G1-v0", help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument("--checkpoint_path", type=str, default=None, help="Relative path to checkpoint file.")

# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# RTX LiDAR はレンダ経路（カメラ）が必須なので、video録画の有無に関わらず enable_cameras は常にON。
# video録画自体はレンダ経路を乱す疑いがあるため、既定OFF（必要時のみ --video）。
args_cli.enable_cameras = True
if args_cli.video:
    args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import os
import torch
import time

import argparse
import os
import numpy as np
import torch
import omni.kit.commands
from pxr import Usd, UsdGeom
import carb, omni, omni.appwindow

from isaaclab.app import AppLauncher
from isaaclab.terrains import TerrainImporterCfg

from isaaclab.envs import ManagerBasedRLEnvCfg,DirectMARLEnv, multi_agent_to_single_agent
from isaaclab.utils.dict import print_dict
from isaaclab_tasks.utils import get_checkpoint_path, parse_env_cfg
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper
# Import extensions to set up environment tasks
import HumanoidPoC.tasks  # noqa: F401
from HumanoidPoC.utils.model_io import load_onnx_model, run_policy
from HumanoidPoC.wrappers.ros_pub_wrapper import IsaacRosPubWrapper
from HumanoidPoC.wrappers.ros_clock_wrapper import IsaacRosClockWrapper
from HumanoidPoC.wrappers.ros_cmdvel_wrapper import IsaacRosCmdVelWrapper
from HumanoidPoC.wrappers.ros_odometry_wrapper import IsaacRosOdometryWrapper
# from HumanoidPoC.wrappers.ros_realsense_pub_wrapper import IsaacRosRealsenseWrapper  # ナビ不要のため無効化
from HumanoidPoC.wrappers.ros_env_control_wrapper import IsaacRosEnvControlWrapper

# ========== キーボード操作 ==========
base_command = np.array([0.0, 0.0, 0.0], dtype=np.float32)
keymap = {
    "NUMPAD_8": np.array([0.75, 0.0, 0.0], dtype=np.float32),
    "UP":       np.array([1.0, 0.0, 0.0], dtype=np.float32),
    "DOWN":       np.array([-1.0, 0.0, 0.0], dtype=np.float32),
    "A": np.array([0.0, 0.0, 1.75], dtype=np.float32),
    "LEFT":     np.array([0.0, 2.0, 0.0], dtype=np.float32),
    "D": np.array([0.0, 0.0, -1.75], dtype=np.float32),
    "RIGHT":    np.array([0.0, -2.0, 0.0], dtype=np.float32),
}

appwindow = omni.appwindow.get_default_app_window()
input_iface = carb.input.acquire_input_interface()
keyboard = appwindow.get_keyboard()

def main():
    """Play with RSL-RL agent."""
    # parse configuration
    env_cfg: ManagerBasedRLEnvCfg = parse_env_cfg(
        task_name=args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs
    )
    env_cfg.sim.device = "cuda:0"   # GPU物理を有効にする

    agent_cfg: RslRlPpoAlgorithmMlpCfg = cli_args.parse_rsl_rl_cfg(args_cli.task, args_cli)

    env_cfg.seed = agent_cfg.seed
    env_cfg.episode_length_s = 2000

    # ここで factory_scaled.usd を読み込む
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # Prevu3D連携: 環境変数 PREVU_SCENE_USD を指定すると、そのスキャンUSD（PLY座標へ
    # 整列済みラッパ）を地形に使い、ロボットを床高(PREVU_SPAWN_Z)にスポーンさせる。
    # 地形・地図(PLY)は座標移動しない（マップ整合を保つ）。未指定なら従来のlightwheel。
    _prevu_usd = os.environ.get("PREVU_SCENE_USD")
    if _prevu_usd:
        factory_usd_path = os.path.abspath(os.path.expanduser(_prevu_usd))
    else:
        factory_usd_path = os.path.normpath(
            # os.path.join(current_dir, "../../../usds/factory_scaled.usd")
            # os.path.join(current_dir, "../../../usds/factory_outside.usd")
            os.path.join(current_dir, "../../../usds/lightwheel_factory_sample.usd")
        )
    env_cfg.scene.terrain = TerrainImporterCfg(
        prim_path="/World/factory",
        terrain_type="usd",
        usd_path=factory_usd_path,
    )
    if _prevu_usd:
        _zspawn = float(os.environ.get("PREVU_SPAWN_Z", "-0.7"))
        _p = list(env_cfg.scene.robot.init_state.pos)
        _p[2] = _zspawn
        env_cfg.scene.robot.init_state.pos = tuple(_p)
        # 床が負z(≈-1.6)にあるため、絶対z基準の base_height 終了判定をスポーン基準に下げる
        # （ポリシー観測は絶対高さを使わないので歩行には影響しない）
        try:
            env_cfg.terminations.base_height.params["minimum_height"] = _zspawn - 0.5
        except Exception as _e:
            print("[Prevu3D] base_height 終了判定の調整に失敗:", _e)
        print(f"[Prevu3D] terrain={factory_usd_path}  spawn_z={_zspawn}  min_h={_zspawn-0.5}")

    env_cfg.scene.height_scanner = None  # ポリシー未使用のため無効化（地形レイキャスト負荷削減）    
    env_cfg.curriculum = None

    env_cfg.commands.base_velocity.debug_vis = False  # コマンドの矢印可視化を無効化
    # env_cfg.events.reset_robot_base.params["pose_range"]["yaw"] = (0, 0)

    # env_cfg.events.randomize_actuator_gains = None
    env_cfg.events.push_robot = None  # 通常時の外乱（後方からのランダム押し）を無効化

    step_dt = env_cfg.sim.dt * env_cfg.decimation

    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)
    env = IsaacRosClockWrapper(env, dt=step_dt)
    env = IsaacRosPubWrapper(
        env,
        points_key="points",      # obs のキー名（あなたの環境だと "points" でOK）
        imu_key="imu",            # 同上
        lidar_frame_id="velodyne",         # ヘッダ frame_id
        imu_frame_id = "imu",
        point_topic="/lidar/points",  # 出したいトピック名に合わせて
        imu_topic="/imu/data",
        lidar_tf=(0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0), 
        imu_tf=(0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0),
        use_sim_time=True,        # /clock 同期（Isaac 側が /clock を publish していれば推奨）
        qos_depth=5,
        # --- 点群フィルタ（地図作成の品質向上）---
        drop_zero_points=True,    # (0,0,0) パディング/無返り点を除去（原点ブロブ防止・必須級）
        min_range=0.0,            # 近接ノイズも除去したいなら 0.3 等に
        max_range=0.0,            # 遠方ノイズを切るなら 80.0 等に
        min_valid_points=50,      # 有効点がこれ未満のフレームはスキップ
    )
    env = IsaacRosCmdVelWrapper(env, topic="/cmd_vel", use_sim_time=True)
    env = IsaacRosOdometryWrapper(env, odom_topic="/odom", use_sim_time=True)
    # realsense_down カメラはナビゲーションに不要かつ描画負荷が高いため無効化
    # env = IsaacRosRealsenseWrapper(
    #     env,
    #     realsense_key="realsense_down",    # あるいは "realsense_down"
    #     rgb_topic="/camera/color/image_raw",
    #     depth_topic="/camera/depth/image_raw",
    #     camera_info_topic="/camera/camera_info",
    #     frame_id="realsense_down_link",
    #     width=640,
    #     height=480,
    #     fx=320.0, fy=320.0, cx=320.0, cy=240.0,
    #     use_sim_time=True,
    # )
    sim = env.unwrapped.sim
    env = IsaacRosEnvControlWrapper(env, sim, use_sim_time=True)

    # reset
    obs, _ = env.reset()

    sim = env.unwrapped.sim   # ManagerBasedRLEnv は sim を持っています

    # カメラビューを設定
    sim.set_camera_view(
        eye=(3.0, 9.0, 5.0),
        target=(-5.0, 0.0, 1.2),
        camera_prim_path="/OmniverseKit_Persp"
    )

    current_dir = os.path.dirname(os.path.abspath(__file__))
    model_dir = os.path.join(current_dir, "models")

    policy_sess, _ = load_onnx_model(
        policy_path=os.path.join(model_dir, "policy.onnx")
    )

    obs_policy = obs["policy"].to('cpu').detach().numpy().copy()[0]
    # obs_encoder = obs['obsHistory'].to('cpu').detach().numpy().copy()[0]

    infos = {}


    # velocity_commands の開始インデックスを調べる
    term_sizes = [15, 15, 15, 145, 145, 145]  # 上の表の shape
    offsets = np.cumsum([0] + term_sizes)     # [0, 15, 30, 45, 190, 335, 480]
    vc_start, vc_end = offsets[2], offsets[3] # → (30, 45)

    while simulation_app.is_running():
        with torch.inference_mode():
            # /cmd_vel から得たコマンドを埋め込む
            if "base_command" in infos:
                cmd = infos["base_command"].astype(np.float32)
                cmd_history = np.tile(cmd, 5)   # (15,) に拡張
                obs_policy[vc_start:vc_end] = cmd_history
                
            # 行動推論
            action_np = policy_sess.run(
                [policy_sess.get_outputs()[0].name],
                {policy_sess.get_inputs()[0].name: obs_policy.reshape(1, -1)},
            )[0]
            action =  torch.from_numpy(action_np).clone().view(1, -1)

        obs, reward, terminated, truncated, infos = env.step(action)
        obs_policy = obs["policy"].to('cpu').detach().numpy().copy()[0]
        # command = obs['commands'].to('cpu').detach().numpy().copy()[0]
        # obs_encoder = obs['obsHistory'].to('cpu').detach().numpy().copy()[0]

    env.close()

if __name__ == "__main__":
    EXPORT_POLICY = True
    # run the main execution
    main()
    # close sim app
    simulation_app.close()
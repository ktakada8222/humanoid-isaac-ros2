# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg, RayCasterCfg, patterns, ImuCfg, CameraCfg
from isaaclab.sim.spawners.sensors import PinholeCameraCfg
from isaaclab.sim import DomeLightCfg, MdlFileCfg, RigidBodyMaterialCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
import HumanoidPoC.tasks.manager_based.locomotion.mdp as poc_mdp
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg, RewardsCfg
import math
import numpy as np

##
# Pre-defined configs
##
from isaaclab_assets import G1_MINIMAL_CFG  # isort: skip

# from isaaclab_tasks.manager_based.locomotion.velocity.config.g1.rough_env_cfg import G1RoughEnvCfg
# from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import ObservationsCfg
# from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import EventCfg

from unitree_rl_lab.tasks.locomotion.robots.g1.g1_29dof.velocity_env_cfg import RobotSceneCfg, RobotEnvCfg, ObservationsCfg, EventCfg

fx, fy = 615.0, 615.0
cx, cy = 320.0, 240.0

K = np.array([
    [fx, 0.0, cx],
    [0.0, fy, cy],
    [0.0, 0.0, 1.0]
])

realsense_cfg = PinholeCameraCfg.from_intrinsic_matrix(
    intrinsic_matrix=K.flatten(),
    width=640,
    height=480,
    clipping_range=(0.1, 10.0),
)

@configclass
class ExtendedRobotSceneCfg(RobotSceneCfg):
    # IMUセンサを追加
    imu = ImuCfg(
        prim_path="{ENV_REGEX_NS}/Robot/torso_link",
        update_period=0.005,     # 200 Hz
        history_length=1,
        debug_vis=False,
    )

    # # ★ 正面カメラ（水平）
    # realsense_front = CameraCfg(
    #     prim_path="{ENV_REGEX_NS}/Robot/torso_link/realsense_front",
    #     # RTXカメラが生成されるよう spawn を指定
    #     spawn=realsense_cfg,          
    #     offset=CameraCfg.OffsetCfg(                    # 相対姿勢（torso_link座標系）
    #         pos=(0.10, 0.0, 0.30),
    #         rot=(1.0, 0.0, 0.0, 0.0),                  # wxyz（水平・前向き）
    #         convention="ros",
    #     ),
    #     width=640, height=480, update_period=0.033,    # 30Hz
    #     data_types=["rgb", "distance_to_image_plane"], # 安定の深度表現
    #     debug_vis=False,
    # )

    # ★ 下向き45°カメラ（ナビゲーションに不要かつ描画負荷が高いため無効化）
    # realsense_down = CameraCfg(
    #     prim_path="{ENV_REGEX_NS}/Robot/torso_link/realsense_down",
    #      spawn=realsense_cfg,
    #     offset=CameraCfg.OffsetCfg(
    #         pos=(0.10, 0.0, 0.40),
    #         rot=(0.270598, -0.653281, 0.653281, -0.270598),
    #         convention="ros",
    #     ),
    #     width=640, height=480, update_period=0.033,
    #     data_types=["rgb", "distance_to_image_plane"],
    #     debug_vis=False,
    # )

class RtxLidarEventsCfg(EventCfg):
    # RTX LiDARの付与
    attach_rtx_lidar = EventTerm(
        func=poc_mdp.attach_rtx_lidar,  # 自作関数にする
        mode="startup",
        params={
            "parent_link": "torso_link",      # 実際のロボットのリンク名
            "lidar_name": "vlp16",           # 任意
            # 実験: 128ch(OS0,約6.5万レイ/スキャン)は描画が重く、env.stepのレンダ枠内で
            # スキャンが完成せず取りこぼし→低レート(~1Hz)の一因。32ch(OS1,約1.6万レイ=1/4)に
            # 軽量化してレート向上を狙う。占有格子マッピングには32ch=十分・GLIMにも有効。
            # 重さで戻したい場合: "OS0_REV7_128ch10hz512res"
            "config_file_name": "OS1_REV6_32ch10hz512res",
            "translation": (0.0, 0.0, 0.5),
            "orientation": (0.0, 0.0, 0.0, 1.0), # (0.7071, 0.0, 0.0, -0.7071),
            "debug_vis": True
        },
    )

class RtxLidarObservationsCfg(ObservationsCfg):

    
    @configclass
    class Vlp16Points(ObsGroup):
        rtx_lidar = ObsTerm(
            func=poc_mdp.rtx_lidar_point_cloud,
            params={"yaw_offset": 0}, # -math.pi/2},
        )
   
    @configclass
    class Imu(ObsGroup):
        orient = ObsTerm(
            func=mdp.imu_orientation,
            params={"asset_cfg": SceneEntityCfg("imu")},
        )
        ang_vel = ObsTerm(
            func=mdp.imu_ang_vel,
            params={"asset_cfg": SceneEntityCfg("imu")},
        )
        lin_acc = ObsTerm(
            func=mdp.imu_lin_acc,
            params={"asset_cfg": SceneEntityCfg("imu")},
        )

    @configclass
    class OdometryObsCfg(ObsGroup):
        """Odometry用の観測"""

        position = ObsTerm(func=poc_mdp.robot_pos)
        orientation = ObsTerm(func=mdp.root_quat_w)      # 四元数を返す関数を用意
        linear_velocity = ObsTerm(func=mdp.base_lin_vel)
        angular_velocity = ObsTerm(func=mdp.base_ang_vel)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False 

    # @configclass
    # class RealsenseFrontObs(ObsGroup):
    #     rgb = ObsTerm(
    #         func=mdp.image,
    #         params={"sensor_cfg": SceneEntityCfg("realsense_front"), "data_type": "rgb"},
    #     )
    #     depth = ObsTerm(
    #         func=mdp.image,
    #         params={"sensor_cfg": SceneEntityCfg("realsense_front"), "data_type": "distance_to_image_plane"},
    #     )
        # def __post_init__(self):
        #     self.enable_corruption = False
        #     self.concatenate_terms = False 

    # realsense_down カメラ無効化に伴い観測グループも無効化
    # @configclass
    # class RealsenseDownObs(ObsGroup):
    #     rgb = ObsTerm(
    #         func=mdp.image,
    #         params={"sensor_cfg": SceneEntityCfg("realsense_down"), "data_type": "rgb"},
    #     )
    #     depth = ObsTerm(
    #         func=mdp.image,
    #         params={"sensor_cfg": SceneEntityCfg("realsense_down"), "data_type": "distance_to_image_plane"},
    #     )
    #     def __post_init__(self):
    #         self.enable_corruption = False
    #         self.concatenate_terms = False

    policy: ObservationsCfg.PolicyCfg = ObservationsCfg.PolicyCfg()
    points: Vlp16Points = Vlp16Points()
    imu: Imu = Imu()
    odom: OdometryObsCfg = OdometryObsCfg()
    # realsense_front: RealsenseFrontObs = RealsenseFrontObs()
    # realsense_down: RealsenseDownObs = RealsenseDownObs()

@configclass
class G1RoughRtxLidarEnvCfg(RobotEnvCfg):
    scene: ExtendedRobotSceneCfg = ExtendedRobotSceneCfg(num_envs=4096, env_spacing=2.5)
    events: RtxLidarEventsCfg = RtxLidarEventsCfg()
    observations: RtxLidarObservationsCfg = RtxLidarObservationsCfg()

    # def __post_init__(self):
    #     # === RTX LiDAR の生成を追加 ===
    #     from isaaclab.sim import PhysxCfg
    #     import omni.kit.commands
    #     from pxr import Gf
    #     import omni.replicator.core as rep

    #     # PhysX solver config
    #     self.sim.physx = PhysxCfg(
    #         solver_type=1,
    #         enable_ccd=True,
    #         enable_stabilization=True,
    #         enable_enhanced_determinism=False,
    #         enable_direct_gpu_api=True,   # ← これが必須
    #         gpu_max_rigid_contact_count=524288,
    #         gpu_max_rigid_patch_count=16384,
    #         gpu_found_lost_pairs_capacity=262144,
    #         gpu_found_lost_aggregate_pairs_capacity=262144,
    #         gpu_total_aggregate_pairs_capacity=1048576,
    #         gpu_collision_stack_size=32768,
    #         gpu_heap_capacity=134217728,
    #         gpu_temp_buffer_capacity=16777216,
    #         gpu_max_num_partitions=8,
    #     )
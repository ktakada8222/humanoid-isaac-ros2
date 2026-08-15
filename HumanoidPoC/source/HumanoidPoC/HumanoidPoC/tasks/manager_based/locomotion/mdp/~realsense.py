from __future__ import annotations

import math
import torch
from typing import TYPE_CHECKING

import isaaclab.utils.math as math_utils
from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers.manager_base import ManagerTermBase
from isaaclab.managers.manager_term_cfg import ObservationTermCfg
from isaaclab.sensors import Camera, ContactSensor, Imu, RayCaster, RayCasterCamera, TiledCamera

from isaacsim.sensors.rtx import LidarRtx

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv, ManagerBasedRLEnv

import numpy as np
import omni.replicator.core as rep

from HumanoidPoC.tasks.manager_based.locomotion import mdp

import omni.kit.commands
from pxr import Gf
import omni.replicator.core as rep
import os

def attach_realsense(
    env,
    env_ids=None,
    parent_link="torso_link",
    camera_name="realsense",
    translation=(0.0, 0.0, 0.3),
    orientation=(1.0, 0.0, 0.0, 0.0),
    width=640,
    height=480,
    fov=70.0,
):
    """指定したリンクにRealsenseカメラを付与する"""
    import omni.kit.commands
    from pxr import Gf

    # env_idsがある場合でも共通パスを生成
    # {ENV_REGEX_NS} が置換されるようにベースパスを組み立てる
    parent_prim = f"{env.scene.env_cfg.prim_path}/Robot/{parent_link}"
    cam_prim = f"{parent_prim}/{camera_name}"

    # カメラ生成コマンドを実行
    omni.kit.commands.execute(
        "IsaacSensorCreateCamera",
        path=cam_prim,
        parent=parent_prim,
        translation=Gf.Vec3d(*translation),
        orientation=Gf.Quatd(*orientation),
        resolution=(width, height),
        fov=fov,
        rgb=True,
        depth=True,
    )

    print(f"[attach_realsense] Camera {camera_name} attached to {parent_link}")
    return True

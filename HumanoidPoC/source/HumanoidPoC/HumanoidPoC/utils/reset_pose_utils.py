# reset_pose_utils.py
"""
Utility: IsaacLab-style reset of root pose and velocity.
This function mimics reset_root_state_uniform() but allows direct absolute pose control.
"""

from typing import Iterable, Optional, Tuple
import torch

def reset_root_state_to_pose(
    env,
    pose_xyz: Tuple[float, float, float],
    quat_xyzw: Tuple[float, float, float, float],
    *,
    asset_name: str = "robot",
    env_ids: Optional[Iterable[int]] = None,
    zero_velocity: bool = True,
):
    """Set the root state of the given asset to a specific pose and (optionally) zero velocity."""
    scene = env.unwrapped.scene
    asset = scene[asset_name]
    device = asset.device

    if env_ids is None:
        env_ids_t = torch.tensor([0], dtype=torch.long, device=device)
    else:
        env_ids_t = torch.as_tensor(list(env_ids), dtype=torch.long, device=device)

    # dtypeをfloat32で統一
    origin = scene.env_origins[env_ids_t].to(dtype=torch.float32)
    desired = torch.tensor(pose_xyz, device=device, dtype=torch.float32).view(1, 3).repeat(len(env_ids_t), 1)
    positions = origin + desired

    # Convert ROS xyzw → IsaacLab wxyz
    qx, qy, qz, qw = quat_xyzw
    if (qx, qy, qz, qw) == (0.0, 0.0, 0.0, 0.0):
        qw = 1.0
    orientations = torch.tensor([qw, qx, qy, qz], device=device, dtype=torch.float32).view(1, 4).repeat(len(env_ids_t), 1)

    velocities = torch.zeros((len(env_ids_t), 6), device=device, dtype=torch.float32)
    pose_tensor = torch.cat([positions, orientations], dim=-1).to(dtype=torch.float32)

    asset.write_root_pose_to_sim(pose_tensor, env_ids=env_ids_t)
    asset.write_root_velocity_to_sim(velocities, env_ids=env_ids_t)

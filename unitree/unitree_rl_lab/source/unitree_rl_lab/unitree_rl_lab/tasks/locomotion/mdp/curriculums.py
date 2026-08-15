from __future__ import annotations

import torch
from collections.abc import Sequence
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def _is_update_step(env: ManagerBasedRLEnv, key: str) -> bool:
    """Return True once per episode-length window.

    The original implementation tested ``common_step_counter % max_episode_length == 0``.
    Curricula are only evaluated on the steps where some environment resets, so that exact
    equality is missed most of the time and the command range barely grows. Here we latch on
    the window index instead, which fires on the first curriculum call inside each window.
    """
    cache = getattr(env, "_cmd_level_windows", None)
    if cache is None:
        cache = {}
        env._cmd_level_windows = cache
    window = int(env.common_step_counter // env.max_episode_length)
    if cache.get(key) == window:
        return False
    cache[key] = window
    return True


def lin_vel_cmd_levels(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int],
    reward_term_name: str = "track_lin_vel_xy",
    promote_ratio: float = 0.6,
    demote_ratio: float = 0.3,
    step: float = 0.1,
) -> torch.Tensor:
    command_term = env.command_manager.get_term("base_velocity")
    ranges = command_term.cfg.ranges
    limit_ranges = command_term.cfg.limit_ranges

    reward_term = env.reward_manager.get_term_cfg(reward_term_name)
    reward = torch.mean(env.reward_manager._episode_sums[reward_term_name][env_ids]) / env.max_episode_length_s

    if _is_update_step(env, "lin"):
        if reward > reward_term.weight * promote_ratio:
            delta = step
        elif reward < reward_term.weight * demote_ratio:
            delta = -step
        else:
            delta = 0.0

        if delta != 0.0:
            delta_command = torch.tensor([-delta, delta], device=env.device)
            ranges.lin_vel_x = torch.clamp(
                torch.tensor(ranges.lin_vel_x, device=env.device) + delta_command,
                limit_ranges.lin_vel_x[0],
                limit_ranges.lin_vel_x[1],
            ).tolist()
            ranges.lin_vel_y = torch.clamp(
                torch.tensor(ranges.lin_vel_y, device=env.device) + delta_command,
                limit_ranges.lin_vel_y[0],
                limit_ranges.lin_vel_y[1],
            ).tolist()

    return torch.tensor(ranges.lin_vel_x[1], device=env.device)


def ang_vel_cmd_levels(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int],
    reward_term_name: str = "track_ang_vel_z",
    promote_ratio: float = 0.6,
    demote_ratio: float = 0.3,
    step: float = 0.1,
) -> torch.Tensor:
    command_term = env.command_manager.get_term("base_velocity")
    ranges = command_term.cfg.ranges
    limit_ranges = command_term.cfg.limit_ranges

    reward_term = env.reward_manager.get_term_cfg(reward_term_name)
    reward = torch.mean(env.reward_manager._episode_sums[reward_term_name][env_ids]) / env.max_episode_length_s

    if _is_update_step(env, "ang"):
        if reward > reward_term.weight * promote_ratio:
            delta = step
        elif reward < reward_term.weight * demote_ratio:
            delta = -step
        else:
            delta = 0.0

        if delta != 0.0:
            delta_command = torch.tensor([-delta, delta], device=env.device)
            ranges.ang_vel_z = torch.clamp(
                torch.tensor(ranges.ang_vel_z, device=env.device) + delta_command,
                limit_ranges.ang_vel_z[0],
                limit_ranges.ang_vel_z[1],
            ).tolist()

    return torch.tensor(ranges.ang_vel_z[1], device=env.device)

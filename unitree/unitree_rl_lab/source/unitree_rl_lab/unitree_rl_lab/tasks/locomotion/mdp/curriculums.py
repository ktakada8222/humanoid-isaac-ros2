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


def _shift_range(
    current: tuple[float, float],
    limit: tuple[float, float],
    delta: float,
    min_half_width: float,
) -> list[float]:
    """Widen (delta > 0) or narrow (delta < 0) a symmetric command range.

    The lower bound is kept in [limit_low, -min_half_width] and the upper bound in
    [min_half_width, limit_high], so narrowing can never make low > high (which would make
    ``torch.Tensor.uniform_`` raise) nor collapse the range to a single point.
    """
    low = min(max(current[0] - delta, limit[0]), -min_half_width)
    high = max(min(current[1] + delta, limit[1]), min_half_width)
    return [float(low), float(high)]


def lin_vel_cmd_levels(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int],
    reward_term_name: str = "track_lin_vel_xy",
    promote_ratio: float = 0.6,
    demote_ratio: float = 0.3,
    step: float = 0.1,
    min_half_width: float = 0.1,
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
            ranges.lin_vel_x = _shift_range(ranges.lin_vel_x, limit_ranges.lin_vel_x, delta, min_half_width)
            ranges.lin_vel_y = _shift_range(ranges.lin_vel_y, limit_ranges.lin_vel_y, delta, min_half_width)

    return torch.tensor(ranges.lin_vel_x[1], device=env.device)


def ang_vel_cmd_levels(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int],
    reward_term_name: str = "track_ang_vel_z",
    promote_ratio: float = 0.6,
    demote_ratio: float = 0.3,
    step: float = 0.1,
    min_half_width: float = 0.1,
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
            ranges.ang_vel_z = _shift_range(ranges.ang_vel_z, limit_ranges.ang_vel_z, delta, min_half_width)

    return torch.tensor(ranges.ang_vel_z[1], device=env.device)

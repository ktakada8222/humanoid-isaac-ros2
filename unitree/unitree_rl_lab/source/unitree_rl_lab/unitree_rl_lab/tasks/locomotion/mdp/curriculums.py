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


def _reward_rate_and_survival(
    env: ManagerBasedRLEnv, env_ids: Sequence[int], reward_term_name: str
) -> tuple[torch.Tensor, torch.Tensor]:
    """Mean per-second reward rate and mean survival fraction for the resetting envs.

    ``RewardManager._episode_sums`` only holds what the episode actually accumulated, so
    dividing by ``max_episode_length_s`` conflates "tracked badly" with "terminated early":
    an env that falls after 3 s of a 20 s episode caps out at 0.15 no matter how well it
    tracked, which pins the curriculum at its lowest level forever. Divide by the elapsed
    time instead and report survival separately, so the two failure modes gate independently.

    Safe to read ``episode_length_buf`` here: ``_reset_idx`` runs the curriculum before it
    zeroes that buffer and before ``reward_manager.reset`` clears the episode sums.
    """
    elapsed = env.episode_length_buf[env_ids].float() * env.step_dt
    sums = env.reward_manager._episode_sums[reward_term_name][env_ids]
    rate = torch.mean(sums / elapsed.clamp(min=env.step_dt))
    survival = torch.mean(elapsed) / env.max_episode_length_s
    return rate, survival


def lin_vel_cmd_levels(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int],
    reward_term_name: str = "track_lin_vel_xy",
    promote_ratio: float = 0.6,
    demote_ratio: float = 0.3,
    step: float = 0.1,
    min_half_width: float = 0.1,
    min_survival: float = 0.5,
) -> torch.Tensor:
    command_term = env.command_manager.get_term("base_velocity")
    ranges = command_term.cfg.ranges
    limit_ranges = command_term.cfg.limit_ranges

    reward_term = env.reward_manager.get_term_cfg(reward_term_name)
    rate, survival = _reward_rate_and_survival(env, env_ids, reward_term_name)

    if _is_update_step(env, "lin"):
        if rate > reward_term.weight * promote_ratio and survival > min_survival:
            delta = step
        elif rate < reward_term.weight * demote_ratio:
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
    min_survival: float = 0.5,
) -> torch.Tensor:
    command_term = env.command_manager.get_term("base_velocity")
    ranges = command_term.cfg.ranges
    limit_ranges = command_term.cfg.limit_ranges

    reward_term = env.reward_manager.get_term_cfg(reward_term_name)
    rate, survival = _reward_rate_and_survival(env, env_ids, reward_term_name)

    if _is_update_step(env, "ang"):
        if rate > reward_term.weight * promote_ratio and survival > min_survival:
            delta = step
        elif rate < reward_term.weight * demote_ratio:
            delta = -step
        else:
            delta = 0.0

        if delta != 0.0:
            ranges.ang_vel_z = _shift_range(ranges.ang_vel_z, limit_ranges.ang_vel_z, delta, min_half_width)

    return torch.tensor(ranges.ang_vel_z[1], device=env.device)

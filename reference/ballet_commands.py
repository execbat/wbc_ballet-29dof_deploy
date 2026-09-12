"""Training and UDP variants of one shape-stable 61D ballet command."""

from __future__ import annotations

from dataclasses import dataclass

import torch
from mjlab.envs.manager_based_rl_env import ManagerBasedRlEnv
from mjlab.managers import CommandTermCfg
from mjlab.managers.command_manager import CommandTerm

from wbc_ballet.teleop.protocol import UdpCommandReceiver

# Layout of the internal 61D command tensor: [velocity(3), targets(29), mask(29)].
# 29 = the G1-29DoF joint count, in the robot's canonical joint order (see
# ``wbc_ballet.teleop.protocol.NUM_JOINTS`` for the single source of truth).
_NUM_JOINTS = 29
_TARGET_START = 3
_TARGET_END = _TARGET_START + _NUM_JOINTS  # 32
_MASK_START = _TARGET_END
_MASK_END = _MASK_START + _NUM_JOINTS  # 61


def blend_joint_targets(
    current: torch.Tensor,
    goal: torch.Tensor,
    *,
    target_scale: float,
    target_limit: float,
) -> torch.Tensor:
    """Blend from the current pose to the full-range random target.

    ``target_scale=0`` returns the current normalized joint positions, so the
    first active masks introduce no pose error. ``target_scale=target_limit``
    returns the sampled goal unchanged.
    """
    if target_limit <= 0.0:
        raise ValueError("target_limit must be positive")
    blend = min(max(target_scale / target_limit, 0.0), 1.0)
    return torch.lerp(current, goal, blend)


class BalletCommand(CommandTerm):
    """Random pose/mask/twist generator used during training."""

    cfg: BalletCommandCfg

    def __init__(self, cfg: BalletCommandCfg, env: ManagerBasedRlEnv):
        super().__init__(cfg, env)
        self._command = torch.zeros(self.num_envs, _MASK_END, device=self.device)

    @property
    def command(self) -> torch.Tensor:
        return self._command

    def _resample_command(self, env_ids: torch.Tensor) -> None:
        count = len(env_ids)
        if count == 0:
            return
        robot = self._env.scene["robot"]
        positions = robot.data.joint_pos[env_ids]
        limits = robot.data.joint_pos_limits[env_ids]
        lower, upper = limits[..., 0], limits[..., 1]
        current = (
            2.0 * (positions - lower) / (upper - lower).clamp_min(1.0e-6) - 1.0
        ).clamp(-1.0, 1.0)

        limit = float(self.cfg.target_limit)
        goal = torch.empty_like(current).uniform_(-limit, limit)
        self._command[env_ids, _TARGET_START:_TARGET_END] = blend_joint_targets(
            current,
            goal,
            target_scale=float(self.cfg.target_scale),
            target_limit=limit,
        )
        self._command[env_ids, _MASK_START:_MASK_END] = (
            torch.rand(count, _NUM_JOINTS, device=self.device) < self.cfg.mask_probability
        ).to(self._command.dtype)
        low = torch.tensor(self.cfg.velocity_ranges[0], device=self.device)
        high = torch.tensor(self.cfg.velocity_ranges[1], device=self.device)
        self._command[env_ids, 0:3] = low + torch.rand(count, 3, device=self.device) * (high - low)
        # Explicit stationary and turn-in-place training samples.
        draw = torch.rand(count, device=self.device)
        self._command[env_ids[draw < 0.20], :3] = 0.0
        self._command[env_ids[(draw >= 0.20) & (draw < 0.45)], :2] = 0.0

    def _update_command(self) -> None:
        pass

    def _update_metrics(self) -> None:
        pass


@dataclass(kw_only=True)
class BalletCommandCfg(CommandTermCfg):
    target_scale: float = 1.0
    target_limit: float = 0.8
    mask_probability: float = 0.15
    velocity_ranges: tuple[tuple[float, float, float], tuple[float, float, float]] = (
        (-1.0, -1.0, -1.0),
        (1.0, 1.0, 1.0),
    )

    def build(self, env: ManagerBasedRlEnv) -> BalletCommand:
        return BalletCommand(self, env)


class UdpBalletCommand(BalletCommand):
    """Same 61D tensor as training, sourced from the robot-face UDP stream."""

    cfg: UdpBalletCommandCfg

    def __init__(self, cfg: UdpBalletCommandCfg, env: ManagerBasedRlEnv):
        super().__init__(cfg, env)
        self.receiver = UdpCommandReceiver(cfg.host, cfg.port)

    def compute(self, dt: float) -> None:
        del dt
        command = self.receiver.poll()
        packet = torch.from_numpy(
            __import__("numpy").concatenate((command.velocity, command.targets, command.mask))
        ).to(self.device)
        self._command[:] = packet

    def _resample_command(self, env_ids: torch.Tensor) -> None:
        del env_ids


@dataclass(kw_only=True)
class UdpBalletCommandCfg(BalletCommandCfg):
    host: str = "127.0.0.1"
    port: int = 55001
    resampling_time_range: tuple[float, float] = (1.0e12, 1.0e12)

    def build(self, env: ManagerBasedRlEnv) -> UdpBalletCommand:
        return UdpBalletCommand(self, env)


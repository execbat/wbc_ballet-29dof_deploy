"""Command manager terms for random training and UDP play."""

from wbc_ballet.tasks.ballet.mdp.commands import BalletCommandCfg, UdpBalletCommandCfg
from wbc_ballet.utils.configclass import configclass


@configclass
class BalletCommandsCfg:
    ballet: BalletCommandCfg | None = BalletCommandCfg(
        resampling_time_range=(6.0, 10.0),
        # Curriculum starts at the current normalized pose, then progressively
        # blends toward the full random range below.
        target_scale=0.0,
        # The final target range remains inside the articulation's 0.9
        # soft-limit factor and avoids conflict with joint-limit penalties.
        target_limit=0.8,
        # Curriculum keeps this at zero for the locomotion warm-up and then
        # increases it to the configured final probability.
        mask_probability=0.0,
        velocity_ranges=((-1.0, -1.0, -1.0), (1.0, 1.0, 1.0)),
    )


@configclass
class BalletUdpCommandsCfg(BalletCommandsCfg):
    ballet: BalletCommandCfg | None = UdpBalletCommandCfg(
        host="127.0.0.1",
        port=55001,
    )

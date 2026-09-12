"""Top-level assembly only; manager terms live in sibling cfg modules."""

from dataclasses import field

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.scene import SceneCfg
from mjlab.sim import MujocoCfg, SimulationCfg
from mjlab.utils.nan_guard import NanGuardCfg
from mjlab.viewer import ViewerConfig

from wbc_ballet.utils.configclass import configclass
from wbc_ballet.utils.manager_compat import group_to_dict, observations_to_dict

from .ballet_actions_cfg import BalletActionsCfg
from .ballet_commands_cfg import BalletCommandsCfg, BalletUdpCommandsCfg
from .ballet_curriculum_cfg import (
    BalletCurriculumCfg,
    BalletPlayCurriculumCfg,
    BalletRoughCurriculumCfg,
)
from .ballet_events_cfg import BalletEventsCfg, BalletPlayEventsCfg
from .ballet_metrics_cfg import BalletMetricsCfg
from .ballet_observations_cfg import BalletObservationsCfg, BalletRoughObservationsCfg
from .ballet_rewards_cfg import BalletRewardsCfg
from .ballet_scene_cfg import make_ballet_scene_cfg
from .ballet_terminations_cfg import (
    BalletPlayTerminationsCfg,
    BalletRoughPlayTerminationsCfg,
    BalletRoughTerminationsCfg,
    BalletTerminationsCfg,
)


@configclass
class BalletEnvCfg:
    """Flat training environment assembled from one cfg class per manager."""

    scene: SceneCfg = field(default_factory=lambda: make_ballet_scene_cfg(rough=False, play=False))
    observations: BalletObservationsCfg = BalletObservationsCfg()
    actions: BalletActionsCfg = BalletActionsCfg()
    commands: BalletCommandsCfg = BalletCommandsCfg()
    events: BalletEventsCfg = BalletEventsCfg()
    rewards: BalletRewardsCfg = BalletRewardsCfg()
    terminations: BalletTerminationsCfg = BalletTerminationsCfg()
    curriculum: BalletCurriculumCfg = BalletCurriculumCfg()
    metrics: BalletMetricsCfg = BalletMetricsCfg()

    decimation: int = 4
    episode_length_s: float = 24.0
    sim: SimulationCfg = field(
        default_factory=lambda: SimulationCfg(
            njmax=300,
            contact_sensor_maxmatch=64,
            # Always-on tensor metrics catch every failure without a host sync.
            # Detailed rolling dumps are opt-in with train --enable-nan-guard.
            nan_guard=NanGuardCfg(
                enabled=False,
                buffer_size=32,
                output_dir="logs/nan_dumps",
                max_envs_to_dump=5,
            ),
            mujoco=MujocoCfg(
                timestep=0.005,
                iterations=10,
                ls_iterations=20,
                ccd_iterations=50,
            ),
        )
    )
    viewer: ViewerConfig = field(
        default_factory=lambda: ViewerConfig(
            origin_type=ViewerConfig.OriginType.ASSET_BODY,
            entity_name="robot",
            body_name="torso_link",
            distance=3.0,
            elevation=-5.0,
            azimuth=90.0,
        )
    )

    def to_mjlab_cfg(self) -> ManagerBasedRlEnvCfg:
        return ManagerBasedRlEnvCfg(
            scene=self.scene,
            observations=observations_to_dict(self.observations),
            actions=group_to_dict(self.actions),
            commands=group_to_dict(self.commands),
            events=group_to_dict(self.events),
            rewards=group_to_dict(self.rewards),
            terminations=group_to_dict(self.terminations),
            curriculum=group_to_dict(self.curriculum),
            metrics=group_to_dict(self.metrics),
            sim=self.sim,
            viewer=self.viewer,
            decimation=self.decimation,
            episode_length_s=self.episode_length_s,
        )


@configclass
class BalletRoughEnvCfg(BalletEnvCfg):
    scene: SceneCfg = field(default_factory=lambda: make_ballet_scene_cfg(rough=True, play=False))
    observations: BalletRoughObservationsCfg = BalletRoughObservationsCfg()
    terminations: BalletRoughTerminationsCfg = BalletRoughTerminationsCfg()
    curriculum: BalletRoughCurriculumCfg = BalletRoughCurriculumCfg()

    def __post_init__(self):
        self.sim.nconmax = 70
        self.sim.contact_sensor_maxmatch = 500
        self.sim.mujoco.ccd_iterations = 500


@configclass
class BalletEnvCfg_PLAY(BalletEnvCfg):
    scene: SceneCfg = field(default_factory=lambda: make_ballet_scene_cfg(rough=False, play=True))
    commands: BalletUdpCommandsCfg = BalletUdpCommandsCfg()
    events: BalletPlayEventsCfg = BalletPlayEventsCfg()
    terminations: BalletPlayTerminationsCfg = BalletPlayTerminationsCfg()
    curriculum: BalletPlayCurriculumCfg = BalletPlayCurriculumCfg()
    episode_length_s: float = 1.0e9

    def __post_init__(self):
        self.observations.actor.enable_corruption = False


@configclass
class BalletRoughEnvCfg_PLAY(BalletRoughEnvCfg):
    scene: SceneCfg = field(default_factory=lambda: make_ballet_scene_cfg(rough=True, play=True))
    commands: BalletUdpCommandsCfg = BalletUdpCommandsCfg()
    events: BalletPlayEventsCfg = BalletPlayEventsCfg()
    terminations: BalletRoughPlayTerminationsCfg = BalletRoughPlayTerminationsCfg()
    curriculum: BalletPlayCurriculumCfg = BalletPlayCurriculumCfg()
    episode_length_s: float = 1.0e9

    def __post_init__(self):
        super().__post_init__()
        self.observations.actor.enable_corruption = False


def make_ballet_env_cfg(*, play: bool = False, rough: bool = False) -> ManagerBasedRlEnvCfg:
    cfg_cls = {
        (False, False): BalletEnvCfg,
        (False, True): BalletRoughEnvCfg,
        (True, False): BalletEnvCfg_PLAY,
        (True, True): BalletRoughEnvCfg_PLAY,
    }[(play, rough)]
    return cfg_cls().to_mjlab_cfg()

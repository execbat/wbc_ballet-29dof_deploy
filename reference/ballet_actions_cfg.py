"""Action manager terms for G1 ballet."""

from mjlab.envs.mdp.actions import JointPositionActionCfg

from wbc_ballet.robots.g1 import G1_ACTION_SCALE
from wbc_ballet.utils.configclass import configclass


@configclass
class BalletActionsCfg:
    joint_pos: JointPositionActionCfg | None = JointPositionActionCfg(
        entity_name="robot",
        actuator_names=(".*",),
        scale=G1_ACTION_SCALE,
        use_default_offset=True,
    )

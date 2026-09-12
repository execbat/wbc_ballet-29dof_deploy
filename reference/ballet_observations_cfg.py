"""Observation manager terms; field order is the policy ABI."""

from mjlab.envs import mdp as envs_mdp
from mjlab.managers.observation_manager import ObservationTermCfg as ObsTerm
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.tasks.velocity import mdp
from mjlab.utils.noise import UniformNoiseCfg as Unoise

from wbc_ballet.tasks.ballet import mdp as ballet_mdp
from wbc_ballet.robots.g1.constants import (
    G1_IMU_ANG_VEL_SENSOR,
    G1_IMU_LIN_ACC_SENSOR,
    G1_IMU_LIN_VEL_SENSOR,
)
from wbc_ballet.utils.configclass import configclass
from wbc_ballet.utils.manager_compat import ObsGroup

_TERRAIN_SCAN_MAX_DISTANCE = 5.0


@configclass
class BalletPolicyCfg(ObsGroup):
    # Pelvis-mounted IMU signals. Gyro and velocimeter were already present;
    # the physical accelerometer channel is added explicitly.
    
#    base_lin_vel: ObsTerm | None = ObsTerm( NOT POSSIBLE TO GRASP FROM ROS2 TOPICS ON THE ROBOT
#        func=mdp.builtin_sensor,
#        params={"sensor_name": G1_IMU_LIN_VEL_SENSOR},
#        noise=Unoise(n_min=-0.1, n_max=0.1),
#    )

    base_ang_vel: ObsTerm | None = ObsTerm(
        func=mdp.builtin_sensor,
        params={"sensor_name": G1_IMU_ANG_VEL_SENSOR},
        noise=Unoise(n_min=-0.2, n_max=0.2),
    )
    imu_lin_acc: ObsTerm | None = ObsTerm(
        func=mdp.builtin_sensor,
        params={"sensor_name": G1_IMU_LIN_ACC_SENSOR},
        noise=Unoise(n_min=-0.2, n_max=0.2),
        clip=(-30.0, 30.0),
        scale=0.1,
    )
    projected_gravity: ObsTerm | None = ObsTerm(
        func=mdp.projected_gravity,
        noise=Unoise(n_min=-0.05, n_max=0.05),
    )
    velocity_commands: ObsTerm | None = ObsTerm(func=ballet_mdp.ballet_velocity)
    joint_pos: ObsTerm | None = ObsTerm(
        func=mdp.joint_pos_rel,
        noise=Unoise(n_min=-0.01, n_max=0.01),
    )
    joint_vel: ObsTerm | None = ObsTerm(
        func=mdp.joint_vel_rel,
        noise=Unoise(n_min=-1.5, n_max=1.5),
    )
    actions: ObsTerm | None = ObsTerm(func=mdp.last_action)
    axis_actual_normalized: ObsTerm | None = ObsTerm(func=ballet_mdp.joint_pos_normalized)
    # A target is meaningful only when its mask is active. Hard-gating here
    # prevents inactive UDP slider values from leaking into the policy.
    axis_target_normalized: ObsTerm | None = ObsTerm(func=ballet_mdp.masked_ballet_targets)
    axis_mask: ObsTerm | None = ObsTerm(func=ballet_mdp.ballet_mask)
    height_scan: ObsTerm | None = None

    def __post_init__(self):
        self.enable_corruption = True
        self.concatenate_terms = True
        # Never expose NaN/Inf to RSL-RL. Checking once after concatenation
        # avoids a CUDA synchronization for every individual observation term.
        self.nan_policy = "warn"
        self.nan_check_per_term = False


@configclass
class BalletCriticCfg(BalletPolicyCfg):
    # Privileged observation: computed from the full simulated articulated-body state.
    # It is intentionally unavailable to the actor/deployed policy.
    
    base_lin_vel: ObsTerm | None = ObsTerm(
        func=mdp.builtin_sensor,
        params={"sensor_name": G1_IMU_LIN_VEL_SENSOR},
        noise=Unoise(n_min=-0.1, n_max=0.1),
    )    
    whole_body_com_xy: ObsTerm | None = ObsTerm(func=ballet_mdp.whole_body_com_xy_b)
    support_center_xy: ObsTerm | None = ObsTerm(
        func=ballet_mdp.support_center_xy_b,
        params={
            "feet_cfg": SceneEntityCfg(
                "robot",
                site_names=("left_foot", "right_foot"),
                preserve_order=True,
            ),
            "left_leg_cfg": SceneEntityCfg("robot", joint_names=(r"left_(hip|knee|ankle)_.*",)),
            "right_leg_cfg": SceneEntityCfg("robot", joint_names=(r"right_(hip|knee|ankle)_.*",)),
        },
    )
    foot_height: ObsTerm | None = ObsTerm(
        func=mdp.foot_height, params={"sensor_name": "foot_height_scan"}
    )
    foot_air_time: ObsTerm | None = ObsTerm(
        func=mdp.foot_air_time, params={"sensor_name": "feet_ground_contact"}
    )
    foot_contact: ObsTerm | None = ObsTerm(
        func=mdp.foot_contact, params={"sensor_name": "feet_ground_contact"}
    )
    foot_contact_forces: ObsTerm | None = ObsTerm(
        func=mdp.foot_contact_forces, params={"sensor_name": "feet_ground_contact"}
    )

    def __post_init__(self):
        super().__post_init__()
        self.enable_corruption = False


@configclass
class BalletObservationsCfg:
    actor: BalletPolicyCfg = BalletPolicyCfg()
    critic: BalletCriticCfg = BalletCriticCfg()


@configclass
class BalletRoughPolicyCfg(BalletPolicyCfg):
    height_scan: ObsTerm | None = ObsTerm(
        func=envs_mdp.height_scan,
        params={"sensor_name": "terrain_scan"},
        noise=Unoise(n_min=-0.1, n_max=0.1),
        scale=1 / _TERRAIN_SCAN_MAX_DISTANCE,
    )


@configclass
class BalletRoughCriticCfg(BalletCriticCfg):
    height_scan: ObsTerm | None = ObsTerm(
        func=envs_mdp.height_scan,
        params={"sensor_name": "terrain_scan"},
        scale=1 / _TERRAIN_SCAN_MAX_DISTANCE,
    )


@configclass
class BalletRoughObservationsCfg(BalletObservationsCfg):
    actor: BalletRoughPolicyCfg = BalletRoughPolicyCfg()
    critic: BalletRoughCriticCfg = BalletRoughCriticCfg()

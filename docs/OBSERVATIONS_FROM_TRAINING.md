# Actor and Critic Observations

This document defines the observation ABI for the 29-DoF ballet policy.
The order of terms matters because the concatenated vector is the neural-network input.

## Actor observations — 186D

The actor contains only signals that are available directly from the real Unitree G1
or can be deterministically reconstructed in the deployment runtime.

| # | Observation | Dim | Real-robot source |
|---:|---|---:|---|
| 1 | `imu_gyro` | 3 | IMU gyroscope from `rt/lowstate`: `[wx, wy, wz]`. |
| 2 | `imu_lin_acc` | 3 | IMU accelerometer from `rt/lowstate`: `[ax, ay, az]`. |
| 3 | `projected_gravity` | 3 | Computed from the IMU quaternion/orientation available in `rt/lowstate`. |
| 4 | `velocity_commands` | 3 | Runtime locomotion command `[vx_cmd, vy_cmd, yaw_rate_cmd]`; in this deployment sourced from the training-compatible UDP gamepad packet values 58..60. |
| 5 | `joint_pos` | 29 | Joint position `q` from `rt/lowstate`, represented relative to the default/home pose. |
| 6 | `joint_vel` | 29 | Joint velocity `dq` from `rt/lowstate`. |
| 7 | `actions` | 29 | Previous actor output, stored by the deployment runtime. |
| 8 | `axis_actual_normalized` | 29 | Current joint positions normalized to `[-1, 1]`, computed from `q` and joint limits. |
| 9 | `axis_target_normalized` | 29 | Normalized ballet/WBC joint targets from the training-compatible UDP gamepad packet values 0..28, hard-gated by `axis_mask`. |
| 10 | `axis_mask` | 29 | Binary mask from the training-compatible UDP gamepad packet values 29..57 indicating joints actively controlled by the ballet/WBC target. |
|  | **Total** | **186** | |

Dimension check:

```text
3 + 3 + 3 + 3 + 29 + 29 + 29 + 29 + 29 + 29 = 186
```

The following signals are intentionally **not** actor observations:

- `base_lin_vel`: `rt/lowstate` does not provide base linear velocity directly.
- `whole_body_com_xy`: requires the full articulated-body model/state and is kept privileged.

## Critic observations — 205D

The critic receives all **186D actor observations** plus **19D privileged simulation observations**.
The critic is used only during training, so these additional signals do not have to be available on the real robot.

### Actor observations inherited by the critic — 186D

1. `imu_gyro` — 3
2. `imu_lin_acc` — 3
3. `projected_gravity` — 3
4. `velocity_commands` — 3
5. `joint_pos` — 29
6. `joint_vel` — 29
7. `actions` — 29
8. `axis_actual_normalized` — 29
9. `axis_target_normalized` — 29
10. `axis_mask` — 29

### Additional privileged critic observations — 19D

| # | Observation | Dim | Description |
|---:|---|---:|---|
| 11 | `base_lin_vel` | 3 | Base/pelvis linear velocity available in simulation: `[vx, vy, vz]`. Critic-only because `rt/lowstate` does not provide it directly. |
| 12 | `whole_body_com_xy` | 2 | Whole-body center-of-mass XY relative to the floating base, computed from the full articulated simulation state. |
| 13 | `support_center_xy` | 2 | Mask-aware center of the active foot support region relative to the base. |
| 14 | `foot_height` | 2 | Left/right foot height. |
| 15 | `foot_air_time` | 2 | Left/right foot air time. |
| 16 | `foot_contact` | 2 | Left/right foot contact state. |
| 17 | `foot_contact_forces` | 6 | 3D contact force for each foot: `2 x [Fx, Fy, Fz]`. |
|  | **Privileged subtotal** | **19** | |
|  | **Critic total** | **205** | |

Dimension check:

```text
186 + 3 + 2 + 2 + 2 + 2 + 2 + 6 = 205
```

## Final ABI

```text
Actor:  186D
Critic: 205D
```

`imu_gyro`, `imu_lin_acc`, and `projected_gravity` form the actor's IMU-based body-state input.
`base_lin_vel` and `whole_body_com_xy` are critic-only privileged observations.

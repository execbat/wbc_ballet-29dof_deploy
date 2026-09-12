# Changes made to the supplied deploy repository

This version was produced by comparing:

- `unitrree_g1_deployment-main` — low-level deployment reference;
- `wbc_ballet-29dof-main` — BALLET training/source of truth;
- supplied `wbc_ballet-29dof_deploy-main` — starting deployment repo.

Main corrections:

1. **Use BALLET `game_emulator_run_v1.py`, not v2/flip.**
2. Restored v1 `Speed X` range from the supplied deploy's `[0,1]` back to the
   training value `[-1,1]`.
3. Corrected ONNX observation metadata name `imu_gyro` -> `base_ang_vel` while
   keeping the physical source as `LowState.imu_state.gyroscope`.
4. Updated `reference/` snapshots from the supplied training repo.
5. Kept direct hardware indices `0..28`; did not copy the reference repo's
   policy-specific `joint_ids_map`.
6. Split policy inference (50 Hz) from LowCmd publication (500 Hz).
7. Disabled actor clipping by default because BALLET training does not configure
   RSL-RL `clip_actions`; retained an optional parameter for deliberate use.
8. Added `mode_machine=5` compatibility check and competing `/lowcmd` publisher
   rejection before arm.
9. Changed unused LowCmd motor slots 29..34 to mode 0 rather than enabling all 35.
10. Preserved Unitree's official HG CRC behavior after verifying the reference
    implementation.
11. Strengthened ONNX validation and added a ROS-independent static contract test.
12. Retained explicit enable, pose ramp, stale-state/UDP watchdogs, tilt guard,
    NaN/Inf checks and damping fail-safe from the supplied deploy design.

This repository has been statically validated in the build environment, but it
has **not** been compiled against your robot's exact ROS2/Unitree installation or
run on physical hardware here.

## Automated robot launcher and repository-contained policy

- Added `scripts/run_policy_on_robot.sh` as the recommended robot entry point.
- The launcher detects/sources ROS2 and the environment that provides `unitree_hg`, installs basic build tooling when possible, finds/downloads ONNX Runtime 1.23.2, validates the BALLET source and ONNX ABI, builds in `~/.cache/wbc_ballet_deploy/ros_ws`, checks `/lowstate`, `mode_machine`, `/lowcmd` ownership and UDP port 55001, then starts the policy node.
- `--arm` additionally requires a live valid `game_emulator_run_v1.py` packet before launch and publishes `/ballet/enable=true` only after `/g1_ballet_onnx` appears.
- `policy/policy.onnx` and optional `policy/policy.onnx.data` are now explicitly allowed in Git so normal robot deployment is `git pull --ff-only` followed by the launcher.
- Root `README.md` is now the short operational path; `README_DETAILED.md` contains the long manual procedure and diagnostics.

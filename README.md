# Unitree G1 29DoF — WBC BALLET deploy

Deployment repository for running the **BALLET** policy on a physical Unitree G1 29DoF.

The intended split is:

```text
LAPTOP / CONTROL PC                         UNITREE G1 ONBOARD COMPUTER

 gamepad/game_emulator_run_v1.py
        |
        | UDP :55001
        | targets[29] + mask[29] + velocity[3]
        v
                                  g1_ballet_policy_node
                                    ^             |
                                    |             |
             /lowstate             |             | /lowcmd
       unitree_hg/msg/LowState -----+             +----> unitree_hg/msg/LowCmd
                                                  |
                                         motor_cmd[0..28].q
```

The policy/control loop runs **on the robot**. The game emulator GUI runs **on the external laptop**. Only high-level BALLET commands cross the laptop↔robot network.

## Robot: normal update and launch

The repository is expected to contain the production model as:

```text
policy/policy.onnx
```

`policy/policy.onnx` is intentionally allowed in Git, so there is no manual model copy during a normal robot update.

On the robot:

```bash
git clone git@github.com:execbat/wbc_ballet-29dof_deploy.git
cd wbc_ballet-29dof_deploy
./scripts/run_policy_on_robot.sh --arm
```

That one launcher automatically:

- detects the installed `ROS_DISTRO`;
- finds and sources the ROS environment that provides `unitree_hg`;
- prints the detected ROS and Unitree workspace/prefix;
- installs missing basic build tools when possible;
- finds or downloads the matching ONNX Runtime C/C++ package;
- prepares an isolated Python environment for ONNX validation;
- runs the BALLET source/I/O contract checks;
- validates `policy/policy.onnx` (`186D -> 29D` plus metadata);
- builds `g1_ballet_onnx_deploy` in its own cache workspace;
- checks `/lowstate` and verifies `unitree_hg/msg/LowState`;
- verifies `mode_machine == 5`;
- refuses to continue if another ROS publisher already owns `/lowcmd`;
- checks that UDP `:55001` is free;
- in `--arm` mode, requires a valid live **61-float / 244-byte** packet from `game_emulator_run_v1.py` before starting;
- starts `g1_ballet_policy_node` and enables `/ballet/enable` only after the node is visible.

Every operation is printed as an explicit numbered stage, e.g. `[03/11] Detect and activate ROS2 + Unitree ROS2 environment`.

For a safe node-only start without enabling the policy:

```bash
./scripts/run_policy_on_robot.sh
```

For checks without starting the node:

```bash
./scripts/run_policy_on_robot.sh --check-only
```

## Laptop: game emulator

The laptop must be on a network from which it can reach the robot onboard computer.

From the same repository on the laptop:

```bash
cd <PATH_TO_THIS_REPOSITORY>
git pull --ff-only

export BALLET_ROBOT_IP=<ROBOT_ONBOARD_IP>
export BALLET_UDP_PORT=55001
python3 gamepad/game_emulator_run_v1.py
```

Start the emulator **before** running the robot launcher with `--arm`, and initially keep all joint masks off and `vx=vy=yaw=0`.

The wire packet is exactly:

```text
float32[0:29]   targets[29]
float32[29:58]  mask[29]
float32[58:61]  velocity[vx, vy, yaw]
```

## End-to-end runtime path

```text
Unitree G1
   |
   | /lowstate
   | unitree_hg/msg/LowState
   v
g1_ballet_policy_node
   |
   |-- IMU:
   |     imu_state.gyroscope
   |     imu_state.accelerometer
   |     imu_state.quaternion
   |
   |-- joints:
   |     motor_state[0..28].q
   |     motor_state[0..28].dq
   |
   |-- UDP :55001
   |     from game_emulator_run_v1.py
   |     targets[29] + mask[29] + velocity[3]
   |
   v
186D observation
   |
   v
ONNX
   |
   | action[29]
   v
q_des[i] = default_q[i] + action_scale[i] * action[i]
   |
   v
LowCmd.motor_cmd[i].q = q_des[i]
   |
   | /lowcmd
   | unitree_hg/msg/LowCmd
   v
Unitree low-level motor controller
```

For the complete installation procedure, environment discovery details, manual ROS/DDS diagnostics, ONNX export/validation, topic inspection, troubleshooting and safety checks, see **[README_DETAILED.md](README_DETAILED.md)**.

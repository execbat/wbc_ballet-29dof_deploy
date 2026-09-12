# BALLET end-to-end I/O contract

For the fully expanded signal path, exact 186D offsets, motor mapping, state machine, and live graph checks, see [`END_TO_END_DATA_FLOW.md`](END_TO_END_DATA_FLOW.md).

This is the transport contract that must stay unchanged for the deployed BALLET policy.

```text
Unitree G1 firmware / DDS
        |
        |  ROS2 /lowstate
        |  unitree_hg/msg/LowState
        |  high-frequency stream (official example uses 500 Hz)
        v
+----------------------------+
| g1_ballet_policy_node      |
|                            |
| IMU:                       |
|   imu_state.gyroscope      |----+
|   imu_state.accelerometer  |    |
|   imu_state.quaternion     |    |
|                            |    |
| joints:                    |    |
|   motor_state[0..28].q/dq  |    +--> 186D actor observation
|                            |    |
| UDP :55001                 |    |
| targets[29]                |----+
| mask[29]                   |
| velocity[3]                |
| from game_emulator_run_v1  |
+----------------------------+
        |
        | ONNX output action[29]
        v
q_des[i] = default_q[i] + action_scale[i] * action[i]
        |
        | identity G1 motor mapping i = 0..28
        v
LowCmd.motor_cmd[i].q = q_des[i]
LowCmd.motor_cmd[i].kp/kd = ONNX deployment metadata gains
        |
        | ROS2 /lowcmd
        | unitree_hg/msg/LowCmd
        | published at 500 Hz
        v
Unitree G1 low-level motor controller
```

## Important details

- There is **no separate IMU ROS2 subscription**. On G1, the gyro, accelerometer and quaternion needed by this policy are fields inside `unitree_hg/msg/LowState`.
- `game_emulator_run_v1.py` does **not** publish ROS2 messages. It sends UDP directly to the deploy process. Deployment binds `0.0.0.0:55001` by default.
- UDP wire layout is `[targets(29), mask(29), velocity(vx,vy,yaw)(3)]`.
- The training command tensor is `[velocity(3), targets(29), mask(29)]`; the deploy node deliberately performs that semantic reorder while constructing observations.
- The policy output is **not** sent to 29 separate topics. Unitree accepts the 29 motor position targets as `motor_cmd[0..28].q` fields of one `unitree_hg/msg/LowCmd` message on `/lowcmd`.
- The BALLET training joint order, game emulator order, deploy ABI, and Unitree G1 29DoF motor index order are the same, so deployment uses an identity `i -> i` mapping.

Run before deployment:

```bash
python3 scripts/check_contract.py
python3 scripts/check_io_contract.py
```

On the robot, before arming, also inspect the live graph:

```bash
ros2 topic info /lowstate -v
ros2 topic hz /lowstate
ros2 topic echo --once /lowstate
ros2 topic info /lowcmd -v
```

Expected ROS2 types are:

```text
/lowstate  unitree_hg/msg/LowState
/lowcmd    unitree_hg/msg/LowCmd
```

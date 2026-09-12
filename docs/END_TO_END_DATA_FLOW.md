# BALLET deployment: complete data flow and robot I/O

This document describes the complete runtime path for the **BALLET** policy, from the Unitree G1 sensors and `game_emulator_run_v1.py` to the final motor position targets.

It is intentionally explicit. If any topic name, message type, UDP layout, observation order, joint order, or motor mapping changes, treat that as an ABI change and re-validate the policy before running on hardware.

---

## 1. End-to-end track

```text
CONTROL LAPTOP / PC                                      UNITREE G1 / ROBOT COMPUTER

 gamepad/game_emulator_run_v1.py
        |
        | UDP 50 Hz -> <POLICY_NODE_IP>:55001
        | little-endian float32[61]
        | [ targets[29], mask[29], velocity[vx,vy,yaw][3] ]
        |                                                       Unitree G1
        |                                                          |
        |                                                          | /lowstate
        |                                                          | unitree_hg/msg/LowState
        |                                                          v
        +------------------------------------------------> g1_ballet_policy_node
                                                              |
                                                              |-- IMU from /lowstate
                                                              |     imu_state.gyroscope[0:3]
                                                              |     imu_state.accelerometer[0:3]
                                                              |     imu_state.quaternion[0:4]
                                                              |     imu_state.rpy[0:2]  (safety only)
                                                              |
                                                              |-- joints from /lowstate
                                                              |     motor_state[0..28].q
                                                              |     motor_state[0..28].dq
                                                              |
                                                              |-- game command from UDP :55001
                                                              |     targets[29]
                                                              |     mask[29]
                                                              |     velocity[3]
                                                              |
                                                              v
                                                     186D actor observation
                                                              |
                                                              v
                                                     policy/policy.onnx
                                                              |
                                                              | action[29]
                                                              v
                                              q_raw[i] = default_q[i]
                                                       + action_scale[i] * action[i]
                                                              |
                                                              | mechanical joint-limit guard
                                                              v
                                                         q_des[29]
                                                              |
                                     policy loop 50 Hz --------+--------> latest q_des held
                                                              |
                                                              | lowcmd loop 500 Hz
                                                              v
                                        LowCmd.motor_cmd[i].q = q_des[i]
                                        LowCmd.motor_cmd[i].kp = policy kp[i]
                                        LowCmd.motor_cmd[i].kd = policy kd[i]
                                        LowCmd.motor_cmd[i].dq = 0
                                        LowCmd.motor_cmd[i].tau = 0
                                        i = 0..28
                                                              |
                                                              | /lowcmd
                                                              | unitree_hg/msg/LowCmd
                                                              v
                                                 Unitree low-level motor controller
                                                              |
                                                              v
                                                        G1 motors 0..28
```

There are only **two ROS 2 robot transport topics** in the policy path:

```text
SUBSCRIBE  /lowstate   unitree_hg/msg/LowState
PUBLISH    /lowcmd     unitree_hg/msg/LowCmd
```

There is one ROS 2 control topic used only to arm/disarm this deployment node:

```text
SUBSCRIBE  /ballet/enable   std_msgs/msg/Bool
```

And there is one non-ROS command transport:

```text
UDP 0.0.0.0:55001  <-  game_emulator_run_v1.py
```

---

## 2. `/lowstate`: exactly what the policy reads

The node subscribes to:

```text
Topic: /lowstate
Type:  unitree_hg/msg/LowState
```

There is **no separate IMU ROS topic** in this deploy. The required IMU signals are fields of `LowState`.

### IMU fields used for actor observations

```text
LowState.imu_state.gyroscope[0..2]
LowState.imu_state.accelerometer[0..2]
LowState.imu_state.quaternion[0..3]   # [w, x, y, z]
```

### IMU field used only for the safety gate

```text
LowState.imu_state.rpy[0]   # roll
LowState.imu_state.rpy[1]   # pitch
```

If `abs(roll)` or `abs(pitch)` exceeds `max_tilt_rad`, the node fails safe and requires an explicit re-enable.

### Joint fields used

For every G1 joint index `i = 0..28`:

```text
LowState.motor_state[i].q
LowState.motor_state[i].dq
```

Indices `0..28` are used directly. There is no deploy-side joint permutation.

---

## 3. `game_emulator_run_v1.py`: UDP command path

`game_emulator_run_v1.py` is **not a ROS 2 node**. It does not publish a ROS topic.

It sends a UDP datagram at approximately 50 Hz to:

```text
<POLICY_NODE_IP>:55001
```

The destination is configured before starting the GUI:

```bash
export BALLET_ROBOT_IP=<POLICY_NODE_IP>
export BALLET_UDP_PORT=55001
python3 gamepad/game_emulator_run_v1.py
```

If `g1_ballet_policy_node` runs on the G1 onboard computer, `<POLICY_NODE_IP>` is the reachable IP address of that onboard computer.

If the policy node runs on an external control PC connected to the G1 DDS network, `<POLICY_NODE_IP>` is the address of that external control PC instead.

### UDP wire format

One datagram is exactly:

```text
61 * float32 = 244 bytes
little-endian
```

Layout:

```text
float index    count   meaning
-----------    -----   ------------------------------------------
0..28          29      targets[29]
29..57         29      mask[29]
58             1       velocity vx
59             1       velocity vy
60             1       velocity yaw
```

Equivalent representation:

```text
[ targets(29), mask(29), velocity(vx, vy, yaw)(3) ]
```

The GUI command ranges are:

```text
vx   [-1, 1]
vy   [-1, 1]
yaw  [-1, 1]
```

For joint targets, inactive axes are transmitted with target `0`. The receiver also hard-gates targets with the received mask.

At GUI startup:

```text
all 29 masks = 0
vx = 0
vy = 0
yaw = 0
```

So the initial command is neutral unless the operator changes it.

---

## 4. Exact 186D actor observation layout

The actor input has exactly 186 floats. The order below is part of the trained ONNX ABI.

| Actor offset | Dim | Observation term | Runtime source / transform |
|---:|---:|---|---|
| `0:3` | 3 | `base_ang_vel` | `LowState.imu_state.gyroscope` |
| `3:6` | 3 | `imu_lin_acc` | clamp accelerometer to `[-30,30]`, then multiply by `0.1` |
| `6:9` | 3 | `projected_gravity` | computed from IMU quaternion `[w,x,y,z]` |
| `9:12` | 3 | `velocity_commands` | UDP `vx, vy, yaw` |
| `12:41` | 29 | `joint_pos` | `motor_state[i].q - default_q[i]` |
| `41:70` | 29 | `joint_vel` | `motor_state[i].dq` |
| `70:99` | 29 | `actions` | previous raw actor output |
| `99:128` | 29 | `axis_actual_normalized` | measured `q` normalized by training G1 joint limits and clamped to `[-1,1]` |
| `128:157` | 29 | `axis_target_normalized` | UDP target if mask active, otherwise `0` |
| `157:186` | 29 | `axis_mask` | UDP mask converted to `0/1` |
| | **186** | | |

The first observation metadata name is **`base_ang_vel`**. Its physical source is the G1 IMU gyroscope.

### Training command order vs UDP order

The UDP wire order is:

```text
[targets29, mask29, velocity3]
```

The training-side ballet command tensor is conceptually:

```text
[velocity3, targets29, mask29]
```

The C++ deploy does not feed the raw 61-float UDP array directly to the ONNX model. It parses the wire packet into named pieces, then places those pieces into the correct observation terms. Therefore the ONNX receives the same semantics and order used in training.

---

## 5. ONNX output -> physical motor target

The ONNX actor output is exactly:

```text
action[29]
```

For every joint `i = 0..28`:

```text
q_raw[i] = default_q[i] + action_scale[i] * action[i]
```

`default_q`, `action_scale`, `kp`, and `kd` are read from ONNX metadata and checked at startup.

Training does not configure an RSL-RL actor clip. Therefore the default deploy config is:

```yaml
action_clip_abs: 0.0
```

which means the actor output is not symmetrically clipped by deployment.

A separate hard mechanical guard clamps the resulting position target inside the configured physical joint range:

```text
q_des[i] = guarded(q_raw[i])
```

with `target_limit_margin` defaulting to `0.98`.

---

## 6. Joint/motor index contract

The BALLET training joint order, `game_emulator_run_v1.py` order, ONNX action order, and Unitree G1 29DoF motor index order are intentionally identical.

```text
 0  left_hip_pitch_joint
 1  left_hip_roll_joint
 2  left_hip_yaw_joint
 3  left_knee_joint
 4  left_ankle_pitch_joint
 5  left_ankle_roll_joint
 6  right_hip_pitch_joint
 7  right_hip_roll_joint
 8  right_hip_yaw_joint
 9  right_knee_joint
10  right_ankle_pitch_joint
11  right_ankle_roll_joint
12  waist_yaw_joint
13  waist_roll_joint
14  waist_pitch_joint
15  left_shoulder_pitch_joint
16  left_shoulder_roll_joint
17  left_shoulder_yaw_joint
18  left_elbow_joint
19  left_wrist_roll_joint
20  left_wrist_pitch_joint
21  left_wrist_yaw_joint
22  right_shoulder_pitch_joint
23  right_shoulder_roll_joint
24  right_shoulder_yaw_joint
25  right_elbow_joint
26  right_wrist_roll_joint
27  right_wrist_pitch_joint
28  right_wrist_yaw_joint
```

Therefore:

```text
action[i] -> q_des[i] -> LowCmd.motor_cmd[i].q
```

No `joint_ids_map` from another deployment repository should be inserted here unless the training-side joint order itself changes.

Unitree `LowCmd` has more slots than the 29 controlled joints. This runtime actively controls only `motor_cmd[0..28]`; remaining slots stay passive/disabled.

---

## 7. `/lowcmd`: exactly what is published to the robot

The node publishes:

```text
Topic: /lowcmd
Type:  unitree_hg/msg/LowCmd
Rate:  500 Hz by default
```

For active position control and every `i = 0..28`:

```text
motor_cmd[i].mode = 1
motor_cmd[i].q    = q_des[i]
motor_cmd[i].dq   = 0
motor_cmd[i].tau  = 0
motor_cmd[i].kp   = ONNX joint_stiffness[i] * gain_scale
motor_cmd[i].kd   = ONNX joint_damping[i]   * gain_scale
```

The message CRC is calculated before publish.

The policy itself is evaluated at 50 Hz. The latest `q_des[29]` is held and republished by the low-level command timer at 500 Hz:

```text
50 Hz   observation -> ONNX -> new q_des
500 Hz  latest q_des -> /lowcmd
```

This separation is intentional.

---

## 8. Arm / ramp / active state machine

Starting the ROS node does **not** immediately command the robot into the policy.

The node waits in idle until:

```text
/ballet/enable = true
```

and all checks pass.

Normal sequence:

```text
IDLE
  |
  | /ballet/enable = true
  | fresh /lowstate
  | fresh UDP v1 command
  | mode_machine == 5
  | safe roll/pitch
  | no competing /lowcmd publisher
  v
RAMPING
  |
  | current measured pose -> ONNX default pose
  | smoothstep over ramp_seconds (default 3 s)
  v
ACTIVE
  |
  | policy 50 Hz
  | lowcmd 500 Hz
  v
running
```

Disable:

```bash
ros2 topic pub --once /ballet/enable std_msgs/msg/Bool '{data: false}'
```

After the first arm, idle/fail-safe output uses damping (`kp=0`, configurable `kd`) rather than continuing the policy target.

---

## 9. Conditions that cancel control

The enable request is cleared and a new explicit enable is required if any of the following occurs:

- `/lowstate` is missing or stale;
- `game_emulator_run_v1.py` UDP is missing or stale;
- G1 `mode_machine` is not the configured 29DoF value (`5` by default);
- roll or pitch exceeds `max_tilt_rad` or is invalid;
- another ROS 2 publisher is detected on `/lowcmd`;
- an observation contains NaN/Inf;
- an actor action contains NaN/Inf;
- ONNX inference throws an exception.

The configured default timeouts are in `config/ballet_policy.yaml`.

---

## 10. Runtime graph to expect on the robot

With the policy node running:

```bash
ros2 node info /g1_ballet_onnx
```

Expected important subscriptions:

```text
/lowstate       unitree_hg/msg/LowState
/ballet/enable  std_msgs/msg/Bool
```

Expected important publisher:

```text
/lowcmd         unitree_hg/msg/LowCmd
```

Check the live robot graph:

```bash
ros2 topic info /lowstate -v
ros2 topic hz /lowstate
ros2 topic echo --once /lowstate

ros2 topic info /lowcmd -v
ros2 topic hz /lowcmd
```

Expected transport types:

```text
/lowstate  -> unitree_hg/msg/LowState
/lowcmd    -> unitree_hg/msg/LowCmd
```

If `/lowstate` is not visible, **do not arm**. Fix the Unitree ROS 2 / DDS network setup first.

---

## 11. Automated contract checks

Run from the repository root:

```bash
python3 scripts/check_contract.py
python3 scripts/check_io_contract.py
```

Before using a newly exported model:

```bash
python3 scripts/check_onnx.py policy/policy.onnx
```

`check_onnx.py` verifies, among other things:

- actor input last dimension = `186`;
- actor output last dimension = `29`;
- exact 29-joint order;
- exact observation term names/order;
- default joint pose metadata;
- action scale metadata;
- stiffness/damping metadata.

Do not bypass a failed contract check on real hardware.

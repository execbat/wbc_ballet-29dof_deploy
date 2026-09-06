# G1 Ballet ONNX Deployment — actor 186D + training gamepad

Deployment repository for running the latest **wbc_ballet G1 29DoF actor (186D)** on a physical Unitree G1 with ONNX Runtime.

This version uses the **same custom 29-axis gamepad/GUI and UDP protocol as the training repository**. The Unitree stock wireless remote is not used as the policy command source.


Link to training repository: [wbc_ballet-29dof](https://github.com/execbat/wbc_ballet-29dof)



## Runtime architecture

```text
CONTROL LAPTOP / PC                                UNITREE G1

 gamepad/game_emulator_run_v1.py
      |  50 Hz UDP
      |  [targets29, mask29, velocity3]
      |  61 x float32 = 244 bytes
      +-------------------------------------> UDP 55001
                                                       |
                                      rt/lowstate      |
                                  IMU + q/dq           |
                                         |             |
                                         +-------> ObservationBuilder
                                                   actor obs 186D
                                                        |
                                                    policy.onnx
                                                        |
                                                    action[29]
                                                        |
                                             default_q + scale*action
                                                        |
                                                     LowCmd
                                                        |
                                                   rt/lowcmd
```

The gamepad contributes exactly three observation terms:

```text
velocity_commands        3
axis_target_normalized  29
axis_mask                29
```

All other actor observations come from G1 `LowState` or controller state.

See **[docs/GAMEPAD.md](docs/GAMEPAD.md)** for the complete protocol and data-flow description.

---

# 1. Preconditions

Before attempting real hardware deployment:

- you have a Unitree G1 29DoF;
- you know how to put the robot into the Unitree debug/low-level control mode;
- the robot is safely suspended for the first tests;
- ROS 2 + Unitree `unitree_hg` messages work on the robot computer;
- you can receive G1 low state and publish low commands;
- you have the latest ballet `policy.onnx` exported from the actor186 training repository;
- your laptop/control PC and robot can reach each other over IP.

Do not begin a first low-level test with the robot standing freely.

---

# 2. Actor ABI

The latest actor is **186D**:

| #  | observation              | dim| real source |
|--- |---                       |---:|---          |
| 1  | `imu_gyro`               | 3  | `LowState.imu_state.gyroscope` |
| 2  | `imu_lin_acc`            | 3  | `LowState.imu_state.accelerometer` |
| 3  | `projected_gravity`      | 3  | reconstructed from IMU quaternion |
| 4  | `velocity_commands`      | 3  | training gamepad UDP `[58:61]` |
| 5  | `joint_pos`              | 29 | motor `q - default_q` |
| 6  | `joint_vel`              | 29 | motor `dq` |
| 7  | `actions`                | 29 | previous actor output |
| 8  | `axis_actual_normalized` | 29 | real `q` normalized to `[-1,1]` |
| 9  | `axis_target_normalized` | 29 | gamepad targets, hard-gated by mask |
| 10 | `axis_mask`              | 29 | gamepad mask |
|    | **TOTAL**                | 186|              |


---

# 3. Repository layout

```text
g1_ballet_onnx_deploy/
├── README.md
├── CMakeLists.txt
├── package.xml
├── config/
│   └── ballet_policy.yaml
├── gamepad/
│   └── game_emulator_run_v1.py
├── docs/
│   ├── ARCHITECTURE.md
│   ├── GAMEPAD.md
│   └── OBSERVATIONS_FROM_TRAINING.md
├── include/g1_ballet_onnx_deploy/
│   ├── g1_abi.hpp
│   └── motor_crc_hg.hpp
├── policy/
│   └── PUT_POLICY_HERE.txt
├── scripts/
│   ├── check_onnx.py
│   ├── inspect_gamepad_udp.py
├── reference/
│   ├── ballet_commands.py
│   ├── ballet_udp_protocol.py
│   ├── ballet_actions_cfg.py
│   ├── ballet_observations_cfg.py
│   └── g1_constants.py
└── src/
    └── g1_ballet_policy_node.cpp
```

`gamepad/game_emulator_run_v1.py` is copied from the latest training repository. The deployment copy only makes the UDP destination configurable through `BALLET_ROBOT_IP` / `BALLET_UDP_PORT` so it can send to the physical G1 instead of localhost.

---

# 4. Put the ONNX model in place

```bash
cp /path/to/policy.onnx policy/policy.onnx
python3 scripts/check_onnx.py policy/policy.onnx
```

The checker expects:

```text
input  = [1, 186]
output = [1, 29]
```

and validates metadata including canonical 29-joint order and these observation names:

```text
imu_gyro
imu_lin_acc
projected_gravity
velocity_commands
joint_pos
joint_vel
actions
axis_actual_normalized
axis_target_normalized
axis_mask
```

Do not bypass a metadata or shape mismatch.

---

# 5. Build on the robot

Example workspace:

```bash
mkdir -p ~/g1_ballet_ws/src
cp -r g1_ballet_onnx_deploy ~/g1_ballet_ws/src/
cd ~/g1_ballet_ws
```

Install/source your Unitree ROS 2 environment first. Install ONNX Runtime C/C++ and set:

```bash
export ONNXRUNTIME_ROOT=/path/to/onnxruntime
```

Build:

```bash
colcon build --packages-select g1_ballet_onnx_deploy --symlink-install
source install/setup.bash
```

---

# 6. Confirm Unitree low-state topics before motors

Check the actual topic names on your Unitree image:

```bash
ros2 topic list | grep -E 'lowstate|lowcmd'
ros2 topic hz lowstate
ros2 topic echo --once lowstate
```

Default deployment config uses:

```yaml
lowstate_topic: "lowstate"
lowcmd_topic: "lowcmd"
```

Change these only if your Unitree ROS 2 setup exposes different aliases.

Confirm that the message includes:

- 29 relevant motor states with `q` and `dq`;
- IMU quaternion;
- IMU gyroscope;
- IMU accelerometer;
- IMU RPY used for the tilt watchdog.

---

# 7. Configure the gamepad UDP receiver

Robot config:

```yaml
udp_host: "0.0.0.0"
udp_port: 55001
command_timeout_s: 0.5
```

`0.0.0.0` means listen on all robot IPv4 interfaces.

If a firewall is enabled, allow UDP 55001 only on the trusted robot-control network as appropriate for your setup.

---

# 8. Start and test the training gamepad BEFORE policy runtime

On your laptop/control PC:

```bash
cd g1_ballet_onnx_deploy
sudo apt install python3-tk   # if Tkinter is missing
python3 -m pip install numpy

export BALLET_ROBOT_IP=<G1_IP>
export BALLET_UDP_PORT=55001
python3 gamepad/game_emulator_run_v1.py
```

Keep initially:

```text
all 29 axis checkboxes OFF
Speed X = 0
Speed Y = 0
Yaw Z = 0
```

On the robot, while the policy controller is NOT running:

```bash
python3 scripts/inspect_gamepad_udp.py --host 0.0.0.0 --port 55001
```

Expected output is approximately:

```text
50.0 Hz from <laptop-ip> velocity=[0.0, 0.0, 0.0] active_axes=[] targets(active)=[]
```

Move the three velocity sliders and verify the values. Turn on exactly one axis checkbox and verify that its index appears in `active_axes`.

Then stop the inspector with Ctrl-C. It must release UDP port 55001 before the policy runtime starts.

---

# 9. Understand the exact gamepad packet

The training wire protocol is:

```text
[target_normalized(29), mask(29), velocity(3)]
```

as little-endian `float32`:

```text
indices  0..28   target_normalized
indices 29..57   mask
index   58       Speed X / vx
index   59       Speed Y / vy
index   60       Yaw Z / yaw_rate
```

Total:

```text
61 * 4 = 244 bytes
```

The GUI sends at 50 Hz.

The receiver:

1. ignores packets whose size is not exactly 244 bytes;
2. ignores NaN/Inf packets;
3. clips target values to `[-1,1]`;
4. converts mask values to `0/1` using `>=0.5`;
5. keeps the newest valid datagram;
6. timestamps every valid received command;
7. triggers fail-safe if no valid packet arrives within `command_timeout_s`.

---

# 10. How gamepad data becomes actor observations

At every 50 Hz policy tick:

```text
UDP packet
   |
   +-- values[58:61] ------------------------------> velocity_commands[3]
   |
   +-- values[0:29] -------- target * mask --------> axis_target_normalized[29]
   +-- values[29:58] ------------------------------> axis_mask[29]
```

Together with G1 state:

```text
LowState.gyroscope       -> imu_gyro
LowState.accelerometer   -> imu_lin_acc
LowState.quaternion      -> projected_gravity
LowState.motor.q         -> joint_pos + axis_actual_normalized
LowState.motor.dq        -> joint_vel
previous ONNX output     -> actions
Gamepad velocity         -> velocity_commands
Gamepad targets + mask   -> axis_target_normalized
Gamepad mask             -> axis_mask
```

The concatenation order is fixed and must remain 186D.

---

# 11. Pattern editor behavior

The same gamepad includes **Open Pattern Editor**.

A pattern is a time-varying `T x 29` normalized joint-target trajectory. When saved/played:

- a joint's mask is `1` when its pattern value differs from the baseline;
- inactive axes transmit target zero;
- the pattern is streamed at the same 50 Hz UDP rate;
- the three velocity sliders remain independent and are appended to each pattern packet.

Therefore pattern playback reaches the actor through exactly the same `axis_target_normalized`, `axis_mask`, and `velocity_commands` observation terms as live sliders.

Do not begin real-robot validation with a complex pattern. Validate neutral command first, then one active axis, then small patterns while suspended.

---

# 12. Enter Unitree debug / low-level mode

Use the procedure appropriate for your G1 software image and Unitree documentation. For the first run the robot must remain suspended and an operator must have immediate access to the physical safety controls.

The expected starting point for this repository is:

```text
G1 in debug/low-level mode
+ ROS2 lowstate visible
+ ONNX already validated
+ gamepad UDP already verified
```

---

# 13. Launch the controller DISABLED

On the robot:

```bash
cd ~/g1_ballet_ws
source install/setup.bash
export ONNXRUNTIME_ROOT=/path/to/onnxruntime

ros2 run g1_ballet_onnx_deploy g1_ballet_policy_node \
  --ros-args --params-file \
  ~/g1_ballet_ws/src/g1_ballet_onnx_deploy/config/ballet_policy.yaml
```

The node should print that it:

- loaded and verified the 186D/29DoF ONNX ABI;
- bound the gamepad receiver to `0.0.0.0:55001`.

It starts with policy **disabled**.

Now run the gamepad on the laptop. Keep it neutral.

---

# 14. Enable policy only when suspended and ready

The custom gamepad is only a ballet command source. It does **not** ARM the low-level policy.

Enable explicitly:

```bash
ros2 topic pub --once /ballet/enable std_msgs/msg/Bool '{data: true}'
```

The runtime first ramps from the current robot pose to the ONNX `default_joint_pos` for `ramp_seconds` (default 3 seconds), then enters `POLICY ACTIVE`.

Disable:

```bash
ros2 topic pub --once /ballet/enable std_msgs/msg/Bool '{data: false}'
```

After disable, the runtime publishes damping commands if it has previously been armed.

---

# 15. Recommended first command sequence

With the G1 suspended:

1. all gamepad masks OFF;
2. `vx = vy = yaw = 0`;
3. ARM policy;
4. observe several seconds of stable neutral behavior;
5. apply a small `Speed X` only;
6. return Speed X to zero;
7. apply a small yaw only;
8. return yaw to zero;
9. enable one non-dangerous joint axis with a very small target change;
10. disable that axis again;
11. only after these tests use the pattern editor.

Do not combine large body-velocity commands and large 29-axis target patterns in the first hardware test.

---

# 16. Fail-safe behavior

The node disarms and sends damping if:

- gamepad UDP is missing/stale;
- `lowstate` is stale;
- roll/pitch exceeds `max_tilt_rad`;
- an observation contains NaN/Inf;
- ONNX returns NaN/Inf;
- the internal observation dimension is wrong.

A UDP timeout therefore behaves like:

```text
Gamepad/PC/network dies
        |
        v
no valid packet for command_timeout_s
        |
        v
requested_enable = false
policy inactive
        |
        v
damping LowCmd
```

Restarting the gamepad does **not** automatically re-arm the policy. Explicitly send `/ballet/enable=true` again after inspecting the situation.

---

# 17. Joint order

Gamepad axis index, actor observation joint index, ONNX action index and Unitree G1 29DoF index are required to use the same canonical order:

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
10 right_ankle_pitch_joint
11 right_ankle_roll_joint
12 waist_yaw_joint
13 waist_roll_joint
14 waist_pitch_joint
15 left_shoulder_pitch_joint
16 left_shoulder_roll_joint
17 left_shoulder_yaw_joint
18 left_elbow_joint
19 left_wrist_roll_joint
20 left_wrist_pitch_joint
21 left_wrist_yaw_joint
22 right_shoulder_pitch_joint
23 right_shoulder_roll_joint
24 right_shoulder_yaw_joint
25 right_elbow_joint
26 right_wrist_roll_joint
27 right_wrist_pitch_joint
28 right_wrist_yaw_joint
```

The ONNX startup metadata check refuses to run if the model's joint order differs.

---

# 18. Why we do not use the Unitree stock gamepad here

The current policy was trained/tested with the custom ballet command semantics:

```text
velocity + 29 normalized target axes + 29 masks
```

The stock Unitree remote does not natively provide the 29 target/mask fields. Reusing the training gamepad preserves the command ABI exactly and reduces sim-to-real deployment differences.

The stock remote can be integrated later as another high-level command source, but it should not silently replace this protocol unless the mapping and policy training assumptions are intentionally changed.

---

# 19. Files copied from training as source of truth

For auditing, `reference/` includes the relevant training-side code:

- `ballet_udp_protocol.py` — 61-float wire ABI;
- `ballet_commands.py` — how UDP becomes the internal training command tensor;
- `ballet_observations_cfg.py` — actor term order;
- `ballet_actions_cfg.py` — action semantics;
- `g1_constants.py` — canonical joint data.

When the training repository changes any of those ABIs, update deployment before using a newly exported ONNX model.

# Ballet training gamepad on the real Unitree G1

This deployment intentionally uses the **same 29-DoF GUI/gamepad and UDP wire protocol as the training repository**. It is not the Unitree wireless remote and it does not read `LowState.wireless_remote`.

## 1. What the gamepad controls

The GUI contains:

- 29 joint rows. Each row has:
  - an enable checkbox -> `axis_mask[i]` (`0` or `1`);
  - a normalized joint target slider -> `axis_target_normalized[i]` in `[-1, 1]`.
- `Speed X` -> `velocity_commands[0]`, range `[0, 1]` in the GUI.
- `Speed Y` -> `velocity_commands[1]`, range `[-1, 1]`.
- `Yaw Z` -> `velocity_commands[2]`, range `[-1, 1]`.
- a pattern editor that can play a time-varying 29-joint normalized target trajectory.

The 29 axes use the exact canonical G1-29DoF order used by the policy, MJCF and action vector.

## 2. Wire protocol

The GUI streams UDP at 50 Hz.

Wire layout is exactly the training `wbc_ballet.teleop.protocol` layout:

```text
[target_normalized(29), mask(29), velocity(3)]
```

Data type is little-endian IEEE-754 `float32`.

```text
61 floats * 4 bytes = 244 bytes/datagram
```

Indexes:

```text
0..28   targets[29]
29..57  mask[29]
58      vx
59      vy
60      yaw_rate
```

For an inactive joint the GUI transmits target `0`, and the real runtime also hard-gates targets with the received mask before placing them into the actor observation.

## 3. Relation to the training command tensor

Do not confuse the **wire layout** with the internal MJLab command tensor.

Wire:

```text
[targets(29), mask(29), velocity(3)]
```

Training `UdpBalletCommand.compute()` receives that packet and stores:

```text
[velocity(3), targets(29), mask(29)]
```

The policy observation does not receive a single 61D block. Observation terms select three pieces:

```text
velocity_commands       <- velocity[3]
axis_target_normalized  <- target[29], hard-gated by mask
axis_mask               <- mask[29]
```

The real deployment does the same thing directly from the UDP packet.

## 4. Network topology

Recommended setup:

```text
Laptop / control PC                         Unitree G1

training gamepad GUI                       g1_ballet_policy_node
       |                                            |
       | UDP 50 Hz / 244 B                          | rt/lowstate
       | targets + mask + velocity                  | IMU + q + dq
       +---------------------> UDP :55001           |
                                                    v
                                             ObservationBuilder
                                                    |
                                                actor 186D
                                                    |
                                                policy.onnx
                                                    |
                                                 action 29D
                                                    |
                                                 rt/lowcmd
```

The laptop and G1 must be able to reach each other over IP. The G1 runtime binds `0.0.0.0:55001` by default.

## 5. Start the gamepad

On the laptop/control PC:

```bash
cd g1_ballet_onnx_deploy
python3 -m pip install numpy
```

Tkinter is normally provided by the OS. On Ubuntu/Debian:

```bash
sudo apt install python3-tk
```

Set the robot IP and start the **same GUI copied from the training repository**:

```bash
export BALLET_ROBOT_IP=<G1_IP>
export BALLET_UDP_PORT=55001
python3 gamepad/game_emulator_run_v1.py
```

Example:

```bash
export BALLET_ROBOT_IP=192.168.123.164
python3 gamepad/game_emulator_run_v1.py
```

The window footer must show something like:

```text
UDP target: 50 Hz (20 ms) to 192.168.123.164:55001
```

The original training version used `127.0.0.1:55001`. The deployment copy changes only the destination configuration: `BALLET_ROBOT_IP` and `BALLET_UDP_PORT`; packet construction, axis order, sliders, masks, pattern editor and 50 Hz scheduler remain the training implementation.

## 6. Test UDP before allowing motor commands

Do this before launching the low-level controller.

On the G1 computer:

```bash
python3 scripts/inspect_gamepad_udp.py --host 0.0.0.0 --port 55001
```

Then move `Speed X`, `Speed Y`, `Yaw Z` and activate one safe test joint in the GUI.

You should see fresh packets at approximately 50 Hz with:

```text
velocity=[...]
active_axes=[...]
targets(active)=[...]
```

Stop the inspector before starting `g1_ballet_policy_node`, because only one process should own UDP port 55001.

## 7. Runtime configuration

`config/ballet_policy.yaml`:

```yaml
udp_host: "0.0.0.0"
udp_port: 55001
command_timeout_s: 0.5
```

`command_timeout_s` is a safety watchdog. If valid UDP packets stop arriving, the policy is disarmed and the controller publishes damping commands.

The controller accepts a packet only if:

- length is exactly 244 bytes;
- all 61 values are finite;
- targets are clipped to `[-1,1]`;
- mask is binarized using `>= 0.5`.

It drains all queued datagrams each control tick and uses the newest valid command. This matches the training receiver's last-value semantics.

## 8. How the gamepad enters the 186D observation

Actor ABI:

```text
imu_gyro                  3  <- rt/lowstate IMU gyro
imu_lin_acc               3  <- rt/lowstate accelerometer
projected_gravity         3  <- rt/lowstate quaternion
velocity_commands         3  <- GAMEPAD UDP values[58:61]
joint_pos                29  <- rt/lowstate q - default_q
joint_vel                29  <- rt/lowstate dq
actions                  29  <- previous ONNX action
axis_actual_normalized   29  <- q normalized with joint limits
axis_target_normalized   29  <- GAMEPAD UDP targets[0:29], gated by mask
axis_mask                29  <- GAMEPAD UDP mask[29:58]
-----------------------------------------------------------
TOTAL                   186
```

In code this is equivalent to:

```cpp
put3(imu_gyro);
put3(imu_acc);
put3(projected_gravity);
put3(gamepad.velocity);
append(joint_pos_relative);
append(joint_vel);
append(previous_action);
append(axis_actual_normalized);
for (i = 0; i < 29; ++i)
    obs.push_back(gamepad.mask[i] ? gamepad.targets[i] : 0.0f);
append(gamepad.mask);
```

## 9. Neutral command

Neutral gamepad state is:

```text
all axis checkboxes = OFF
Speed X = 0
Speed Y = 0
Yaw Z = 0
```

The joint sliders can visually remain at their baseline values; inactive axes are transmitted as zero targets because the mask is zero.

This yields:

```text
velocity_commands       = [0,0,0]
axis_target_normalized  = 29 zeros
axis_mask               = 29 zeros
```

## 10. Safe first hardware test

1. Suspend the G1 in the official safe test fixture.
2. Enter Unitree debug/low-level mode.
3. Start ROS 2 and verify `lowstate` is fresh.
4. Start the gamepad with all masks OFF and all velocity sliders at zero.
5. Verify UDP with `inspect_gamepad_udp.py`.
6. Stop the inspector.
7. Launch `g1_ballet_policy_node`.
8. Keep policy disabled initially.
9. Start the gamepad again and confirm the runtime reports fresh command packets.
10. Enable policy using `/ballet/enable` only while the robot is suspended.
11. First test zero velocity + mask=0.
12. Then change one command at a time.

The training gamepad is a **command source**, not an emergency stop. Keep the Unitree physical E-stop / safe operator procedure available during all low-level tests.

## 11. Failure behavior

The runtime enters damping and clears its requested enable state if:

- gamepad UDP becomes stale;
- `lowstate` becomes stale;
- actor observation contains NaN/Inf;
- ONNX action contains NaN/Inf;
- roll/pitch exceeds the configured limit.

After a fail-safe, fresh UDP alone does **not** re-enable the policy. `/ballet/enable` must be explicitly asserted again.

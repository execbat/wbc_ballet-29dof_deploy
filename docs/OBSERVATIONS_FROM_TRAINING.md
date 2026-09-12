# BALLET actor observations reproduced on G1

Source snapshot: `wbc_ballet-29dof` BALLET play configuration.

The play config disables observation corruption, so robot deployment uses real
signals without training noise.  The exported ONNX contains the learned empirical
actor observation normalization.

## Exact order

```text
0:3      base_ang_vel             raw pelvis/Unitree IMU gyroscope
3:6      imu_lin_acc              clamp(accel,-30,30) * 0.1
6:9      projected_gravity        gravity in body frame
9:12     velocity_commands        v1 vx,vy,yaw
12:41    joint_pos                q - default_q
41:70    joint_vel                dq
70:99    actions                  previous actor output
99:128   axis_actual_normalized   2*(q-lower)/(upper-lower)-1, clipped [-1,1]
128:157  axis_target_normalized   target if mask active else exactly 0
157:186  axis_mask                0/1
```

Total: **186**.

### Naming trap

The first exported metadata name is `base_ang_vel`.  Earlier deploy code called it
`imu_gyro`; that would cause a valid current BALLET ONNX to fail the metadata
check.  Signal semantics did not change: its real source is the Unitree gyro.

### Previous action

`actions` is the preceding 29D actor output.  Training does not set runner
`clip_actions`, so deploy also leaves the actor output unclipped by default.
If deployment-only clipping is explicitly enabled, `actions` records the action
actually used by the deploy runtime.

### Joint target conversion

The action manager used in training is joint-position control with default offset:

```text
q_des = default_q + action_scale * action
```

`default_q`, `action_scale`, `joint_stiffness`, and `joint_damping` are read from
the ONNX metadata rather than duplicated as deployment constants.

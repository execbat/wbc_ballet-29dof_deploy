# Architecture

## Runtime loops

There are two intentionally separate cadences:

1. **Policy loop — 50 Hz**
   - drain v1 UDP and keep newest valid packet;
   - copy latest G1 LowState;
   - build the exact 186D actor observation;
   - run ONNX;
   - convert 29 actions to joint position targets;
   - store the newest `q_des`.

2. **LowCmd loop — 500 Hz**
   - publish the latest stored target with ONNX metadata `kp/kd`;
   - during ramp, publish the ramp target;
   - after disable/fault, publish damping after the controller has been armed once.

This mirrors the important timing principle in `unitrree_g1_deployment`: its FSM
publishes low-level commands at 1 kHz while its policy thread advances at the
environment `step_dt` (0.02 s for the reference policy).  For ROS2, 500 Hz is used
because Unitree's G1 high-frequency example uses a 2 ms timer.

## State flow

```text
node starts
   |
   v
IDLE (no motor output before first arm)
   |
   | /ballet/enable=true AND:
   | fresh LowState
   | fresh v1 UDP
   | safe RPY
   | mode_machine=5
   | no competing /lowcmd publisher
   v
RAMPING (3 s current q -> ONNX default_q)
   |
   v
ACTIVE (ONNX @ 50 Hz, target held on /lowcmd @ 500 Hz)
   |
   | disable or watchdog/fault
   v
IDLE + damping output after first arm
```

A fail-safe also clears the enable request, so re-entry is explicit.

## Motor index contract

Ballet uses hardware order directly.  There is no `joint_ids_map` conversion.
The ONNX `joint_names` metadata must match `include/.../g1_abi.hpp` exactly.

## CRC

`motor_crc_hg.hpp` follows Unitree's `motor_crc_hg` layout/algorithm.  The unusual
four-byte `memcpy` for the top-level reserve field is retained intentionally
because that is what the official Unitree ROS2 implementation does; changing it
would make this deploy diverge from the reference implementation.

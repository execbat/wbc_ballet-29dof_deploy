# BALLET gamepad: `game_emulator_run_v1.py`

Only **v1** is part of this deployment contract.

## Wire packet

Destination defaults to `127.0.0.1:55001` in training.  The deployment copy only
adds environment variables for the destination:

```bash
BALLET_ROBOT_IP=<robot-ip>
BALLET_UDP_PORT=55001
```

Packet at 50 Hz:

```text
float32 little-endian[61]

0..28   targets[29]     normalized [-1,1]
29..57  mask[29]        receiver converts >=0.5 to 1, otherwise 0
58      vx
59      vy
60      yaw
```

The v1 GUI slider ranges for `vx`, `vy`, `yaw` are all `[-1,1]`.

## Training-vs-wire order

Wire:

```text
[targets29, mask29, velocity3]
```

Training's `UdpBalletCommand.compute()` creates the internal command tensor:

```text
[velocity3, targets29, mask29]
```

The observation functions then expose:

```text
velocity_commands       <- command[0:3]
axis_target_normalized  <- command[3:32], gated by mask
axis_mask               <- command[32:61]
```

The C++ deploy receives the wire form and writes these three actor terms directly,
which is equivalent to the training play environment.

## Pattern editor

The pattern editor is unchanged from training v1.  It streams the same packet and
same 29-joint order.  Validate neutral commands and single joints on a supported
robot before trying a multi-joint pattern.

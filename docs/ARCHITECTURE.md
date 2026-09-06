# Runtime architecture

```text
Custom ballet training gamepad (laptop)
  game_emulator_run_v1.py
       |
       | UDP @ 50 Hz
       | wire = [targets29, mask29, velocity3]
       v
BalletUdpReceiver (G1)
       |
       +--> velocity_commands[3]
       +--> axis_target_normalized[29] (mask gated)
       +--> axis_mask[29]

Unitree rt/lowstate
       |
       +--> IMU gyro ----------> imu_gyro[3]
       +--> IMU accelerometer -> imu_lin_acc[3]
       +--> IMU quaternion ----> projected_gravity[3]
       +--> motor q -----------> joint_pos[29]
       |                    \--> axis_actual_normalized[29]
       +--> motor dq ----------> joint_vel[29]

Controller memory
       +--> previous action ---> actions[29]

                    all terms
                       |
                       v
              ObservationBuilder 186D
                       |
                       v
                    ONNX
                       |
                       v
                 action[29]
                       |
              default_q + scale*a
                       |
                       v
                  Unitree LowCmd
```

The custom training gamepad replaces the Unitree stock remote as the **policy command source**. It does not ARM the controller. ARM/DISARM remains an explicit safety channel on `/ballet/enable`.

The gamepad UDP stream is freshness-monitored. Missing valid packets while enabled force policy disarm and damping.

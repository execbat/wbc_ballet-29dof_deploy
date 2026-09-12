# Validation performed for this generated version

Completed in the artifact build environment:

- Python syntax compilation for deploy scripts and `game_emulator_run_v1.py`.
- Static deployment contract test (`scripts/check_contract.py`).
- Verified gamepad v1 packet = 61 float32 / 244 bytes and all three velocity ranges = `[-1,1]`.
- Verified copied `reference/` files are exact snapshots of the supplied BALLET training repo.
- Verified deploy 29-joint order equals the supplied training v1 gamepad order.
- Verified `g1_abi.hpp` lower/upper joint ranges against the supplied training G1 MJCF XML.
- C++17 syntax/type-structure pass with local interface stubs for ROS2, Unitree HG and ONNX Runtime.
- Verified Unitree HG CRC implementation against the official ROS2 reference before leaving its reserve-copy behavior unchanged.

Not possible in this environment:

- a real `colcon build` against the robot's exact ROS2 + `unitree_hg` installation;
- ONNX contract validation against the user's trained policy (no `.onnx`/checkpoint was present in the supplied archives);
- physical G1 execution.

Before hardware arm, run:

```bash
python3 scripts/check_contract.py
python3 scripts/check_onnx.py /absolute/path/to/policy.onnx
```

Then build with the Unitree ROS2 environment and perform the first run with the robot supported/suspended.

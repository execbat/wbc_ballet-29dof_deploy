#!/usr/bin/env python3
"""Validate the exported BALLET ONNX contract before copying it to the robot."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

try:
    import onnx
except ImportError:
    sys.exit("Missing dependency: pip install onnx")

EXPECTED_OBS = [
    "base_ang_vel",
    "imu_lin_acc",
    "projected_gravity",
    "velocity_commands",
    "joint_pos",
    "joint_vel",
    "actions",
    "axis_actual_normalized",
    "axis_target_normalized",
    "axis_mask",
]
EXPECTED_OBS_TERM_DIMS = [3, 3, 3, 3, 29, 29, 29, 29, 29, 29]
EXPECTED_OBS_DIM = sum(EXPECTED_OBS_TERM_DIMS)
EXPECTED_ACTION_DIM = 29
EXPECTED_JOINTS = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]
REQUIRED_METADATA = {
    "joint_names",
    "observation_names",
    "default_joint_pos",
    "action_scale",
    "joint_stiffness",
    "joint_damping",
}


def csv(value: str) -> list[str]:
    return [x.strip() for x in value.split(",")]


def float_csv(value: str, key: str, n: int) -> list[float]:
    parts = csv(value)
    if len(parts) != n:
        raise SystemExit(f"FAIL: {key} must contain {n} values; got {len(parts)}")
    try:
        values = [float(x) for x in parts]
    except ValueError as exc:
        raise SystemExit(f"FAIL: {key} contains a non-float value: {exc}") from exc
    if not all(math.isfinite(x) for x in values):
        raise SystemExit(f"FAIL: {key} contains NaN/Inf")
    return values


def shape_of(value_info) -> list[int | None]:
    dims: list[int | None] = []
    for dim in value_info.type.tensor_type.shape.dim:
        if dim.HasField("dim_value"):
            dims.append(int(dim.dim_value))
        else:
            dims.append(None)
    return dims


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("onnx", type=Path)
    args = parser.parse_args()

    model = onnx.load(str(args.onnx), load_external_data=False)
    md = {p.key: p.value for p in model.metadata_props}

    if len(model.graph.input) != 1 or len(model.graph.output) != 1:
        raise SystemExit(
            f"FAIL: expected exactly 1 input / 1 output; got "
            f"{len(model.graph.input)} / {len(model.graph.output)}"
        )

    input_shape = shape_of(model.graph.input[0])
    output_shape = shape_of(model.graph.output[0])
    print(f"input : {model.graph.input[0].name} {input_shape}")
    print(f"output: {model.graph.output[0].name} {output_shape}")

    if not input_shape or input_shape[-1] != EXPECTED_OBS_DIM:
        raise SystemExit(
            f"FAIL: ONNX input last dimension must be {EXPECTED_OBS_DIM}; got {input_shape}"
        )
    if not output_shape or output_shape[-1] != EXPECTED_ACTION_DIM:
        raise SystemExit(
            f"FAIL: ONNX output last dimension must be {EXPECTED_ACTION_DIM}; got {output_shape}"
        )

    missing = sorted(REQUIRED_METADATA - md.keys())
    if missing:
        raise SystemExit(f"FAIL: missing metadata: {', '.join(missing)}")

    if csv(md["joint_names"]) != EXPECTED_JOINTS:
        raise SystemExit("FAIL: joint_names mismatch; do not deploy this ONNX")
    if csv(md["observation_names"]) != EXPECTED_OBS:
        raise SystemExit(
            "FAIL: observation_names mismatch; BALLET requires base_ang_vel first and the exact 10-term ABI"
        )

    default_q = float_csv(md["default_joint_pos"], "default_joint_pos", 29)
    action_scale = float_csv(md["action_scale"], "action_scale", 29)
    kp = float_csv(md["joint_stiffness"], "joint_stiffness", 29)
    kd = float_csv(md["joint_damping"], "joint_damping", 29)

    if any(x <= 0 for x in action_scale):
        raise SystemExit("FAIL: action_scale must be positive for every joint")
    if any(x < 0 for x in kp + kd):
        raise SystemExit("FAIL: joint stiffness/damping contains a negative value")

    # Current MJLab exporter records one scale per observation term. This is an
    # extra consistency check, but older compatible exporters may omit the key.
    if "observation_terms_scale" in md:
        scales = float_csv(md["observation_terms_scale"], "observation_terms_scale", 10)
        expected = [1.0, 0.1, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        if any(abs(a - b) > 1e-6 for a, b in zip(scales, expected, strict=True)):
            raise SystemExit(
                f"FAIL: observation_terms_scale mismatch; got {scales}, expected {expected}"
            )

    if "command_names" in md and csv(md["command_names"]) != ["ballet"]:
        raise SystemExit(f"FAIL: command_names is not ['ballet']: {md['command_names']!r}")

    print("PASS: BALLET ONNX contract is compatible with this deploy runtime")
    print(f"       actor obs = {EXPECTED_OBS_DIM}, action = {EXPECTED_ACTION_DIM}")
    print(f"       default_q[0:3] = {default_q[:3]}")
    print(f"       action_scale[0:3] = {action_scale[:3]}")


if __name__ == "__main__":
    main()

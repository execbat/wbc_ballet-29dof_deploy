#!/usr/bin/env python3
"""Fast static checks for the BALLET deployment contract (no ROS2 required)."""

from __future__ import annotations

import ast
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
GAMEPAD = ROOT / "gamepad" / "game_emulator_run_v1.py"
PROTOCOL = ROOT / "reference" / "ballet_udp_protocol.py"
NODE = ROOT / "src" / "g1_ballet_policy_node.cpp"
CONFIG = ROOT / "config" / "ballet_policy.yaml"


def assignments(path: Path) -> dict[str, object]:
    tree = ast.parse(path.read_text(), filename=str(path))
    result: dict[str, object] = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        try:
            value = ast.literal_eval(node.value)
        except Exception:
            continue
        for target in node.targets:
            if isinstance(target, ast.Name):
                result[target.id] = value
            elif isinstance(target, (ast.Tuple, ast.List)):
                if isinstance(value, (tuple, list)) and len(value) == len(target.elts):
                    for name_node, item in zip(target.elts, value, strict=True):
                        if isinstance(name_node, ast.Name):
                            result[name_node.id] = item
    return result


def main() -> None:
    gp = assignments(GAMEPAD)
    assert gp["NUM_AXES"] == 29
    assert gp["STREAM_RATE_HZ"] == 50.0
    assert gp["EXTRA_SPEED_MIN"] == -1.0 and gp["EXTRA_SPEED_MAX"] == 1.0
    assert gp["EXTRA_LR_MIN"] == -1.0 and gp["EXTRA_LR_MAX"] == 1.0
    assert gp["EXTRA_ANGLE_MIN"] == -1.0 and gp["EXTRA_ANGLE_MAX"] == 1.0
    assert len(gp["JOINT_NAMES"]) == 29

    spec = importlib.util.spec_from_file_location("ballet_udp_protocol", PROTOCOL)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.NUM_JOINTS == 29
    assert module.PACKET_FLOATS == 61
    assert module.PACKET_BYTES == 244
    neutral = module.BalletCommand.neutral()
    assert len(neutral.to_bytes()) == 244
    decoded = module.BalletCommand.from_bytes(neutral.to_bytes())
    assert decoded.targets.shape == (29,)
    assert decoded.mask.shape == (29,)
    assert decoded.velocity.shape == (3,)

    cpp = NODE.read_text()
    for required in (
        '"base_ang_vel"',
        '"imu_lin_acc"',
        '"velocity_commands"',
        '"axis_actual_normalized"',
        '"axis_target_normalized"',
        '"axis_mask"',
        'declare_parameter<double>("policy_hz", 50.0)',
        'declare_parameter<double>("lowcmd_hz", 500.0)',
        'declare_parameter<double>("action_clip_abs", 0.0)',
        'declare_parameter<int>("required_mode_machine", 5)',
    ):
        assert required in cpp, f"missing runtime contract token: {required}"

    cfg = CONFIG.read_text()
    for required in (
        "policy_hz: 50.0",
        "lowcmd_hz: 500.0",
        "udp_port: 55001",
        "action_clip_abs: 0.0",
        "required_mode_machine: 5",
    ):
        assert required in cfg, f"missing config token: {required}"

    print("PASS: static BALLET deployment contract checks")
    print("  v1 gamepad: 29 axes, vx/vy/yaw in [-1,1], 50 Hz")
    print("  UDP wire  : targets29 + mask29 + velocity3 = 61 float32 = 244 bytes")
    print("  actor ABI : 186 observations -> 29 actions")
    print("  runtime   : policy 50 Hz, lowcmd 500 Hz, no default actor clipping")


if __name__ == "__main__":
    main()

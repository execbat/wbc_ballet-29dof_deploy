#!/usr/bin/env python3
"""Static end-to-end I/O contract checks for the BALLET G1 deployment.

This check is intentionally independent of ROS2/ONNX Runtime so it can be run
on any development machine before copying the package to the robot.
"""
from __future__ import annotations

import ast
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CPP = ROOT / "src" / "g1_ballet_policy_node.cpp"
GAMEPAD = ROOT / "gamepad" / "game_emulator_run_v1.py"
ABI = ROOT / "include" / "g1_ballet_onnx_deploy" / "g1_abi.hpp"
CFG = ROOT / "config" / "ballet_policy.yaml"

OFFICIAL_G1_29DOF_ORDER = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]


def python_assignment(path: Path, name: str):
    tree = ast.parse(path.read_text(), filename=str(path))
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == name:
                    return ast.literal_eval(node.value)
    raise AssertionError(f"{name} not found in {path}")


def abi_joint_names() -> list[str]:
    text = ABI.read_text()
    match = re.search(r"kJointNames\s*=\s*\{(.*?)\};", text, re.S)
    assert match, "kJointNames array missing from g1_abi.hpp"
    return re.findall(r'"([^"]+_joint)"', match.group(1))


def require(text: str, token: str, what: str) -> None:
    assert token in text, f"missing {what}: {token}"


def ordered(text: str, tokens: list[str], what: str) -> None:
    positions = [text.find(token) for token in tokens]
    assert all(p >= 0 for p in positions), f"missing token in {what}: {tokens}"
    assert positions == sorted(positions), f"wrong order in {what}: {tokens}"


def main() -> None:
    cpp = CPP.read_text()
    gp = GAMEPAD.read_text()
    cfg = CFG.read_text()

    # 1. Robot -> observation builder: official Unitree ROS2 high-frequency G1 topics/types.
    require(cpp, 'declare_parameter<std::string>("lowstate_topic", "/lowstate")', "LowState topic")
    require(cpp, 'create_subscription<unitree_hg::msg::LowState>', "LowState message type")
    require(cpp, 'declare_parameter<std::string>("lowcmd_topic", "/lowcmd")', "LowCmd topic")
    require(cpp, 'create_publisher<unitree_hg::msg::LowCmd>', "LowCmd message type")
    require(cfg, 'lowstate_topic: "/lowstate"', "configured LowState topic")
    require(cfg, 'lowcmd_topic: "/lowcmd"', "configured LowCmd topic")

    # The deployed actor only needs data present in LowState; there is no separate IMU topic.
    for source in (
        "state.imu_state.gyroscope[0]",
        "state.imu_state.accelerometer[0]",
        "state.imu_state.quaternion[0]",
        "state.motor_state[i].q",
        "state.motor_state[i].dq",
    ):
        require(cpp, source, "observation source")

    # 2. game_emulator_run_v1 -> deploy: exact 61-float wire layout.
    assert python_assignment(GAMEPAD, "NUM_AXES") == 29
    require(gp, "np.concatenate([visible_targets, mask, extra]).astype(np.float32)", "v1 UDP layout")
    require(cpp, "static constexpr size_t kPacketFloats = 61", "61-float receiver")
    ordered(
        cpp,
        [
            "targets[i] = std::clamp(values[i]",
            "mask[i] = values[kNumJoints + i]",
            "velocity[i] = values[2 * kNumJoints + i]",
        ],
        "UDP decode",
    )

    # Wire order is targets/mask/velocity, but training command tensor/actor sees velocity first.
    ordered(
        cpp,
        [
            "put3(projected_gravity(quat))",
            "put3(velocity_cmd)",
            "state.motor_state[i].q - policy_->default_q()[i]",
            "for (float x : last_action) obs[k++] = x",
            "mask[i] >= 0.5f ? targets[i] : 0.0f",
            "for (float x : mask) obs[k++] = x",
        ],
        "186D actor observation assembly",
    )

    # 3. Policy -> motors: action i becomes the position target of the same G1 motor slot i.
    require(cpp, "policy_->default_q()[i] + policy_->action_scale()[i] * action[i]", "action scaling")
    require(cpp, "cmd.motor_cmd[i].q = q_des[i]", "motor position target")
    require(cpp, "cmd.motor_cmd[i].mode = 1", "motor enable")
    require(cpp, "cmd.motor_cmd[i].kp = policy_->kp()[i] * gain_scale", "motor kp")
    require(cpp, "cmd.motor_cmd[i].kd = policy_->kd()[i] * gain_scale", "motor kd")
    require(cpp, "lowcmd_pub_->publish(cmd)", "LowCmd publication")

    gp_order = python_assignment(GAMEPAD, "JOINT_NAMES")
    abi_order = abi_joint_names()
    assert gp_order == OFFICIAL_G1_29DOF_ORDER, "gamepad joint order != G1 29DoF motor order"
    assert abi_order == OFFICIAL_G1_29DOF_ORDER, "deploy ABI joint order != G1 29DoF motor order"

    print("PASS: end-to-end BALLET I/O contract")
    print("  robot state : /lowstate  [unitree_hg/msg/LowState]")
    print("  observations: IMU + motor_state from /lowstate, v1 commands from UDP :55001")
    print("  UDP wire    : targets[29] + mask[29] + velocity[3]")
    print("  actor input : velocity is reordered into the training 186D observation ABI")
    print("  actor output: action[29] -> default_q + scale*action -> motor_cmd[0..28].q")
    print("  robot cmd   : /lowcmd    [unitree_hg/msg/LowCmd]")
    print("  joint map   : identity 0..28, matching G1 29DoF order")


if __name__ == "__main__":
    main()

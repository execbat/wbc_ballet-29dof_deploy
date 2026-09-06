#!/usr/bin/env python3
import argparse
import sys
try:
    import onnx
except ImportError:
    sys.exit("pip install onnx")
EXPECTED_OBS=["imu_gyro","imu_lin_acc","projected_gravity","velocity_commands","joint_pos","joint_vel","actions","axis_actual_normalized","axis_target_normalized","axis_mask"]
EXPECTED_JOINTS=["left_hip_pitch_joint","left_hip_roll_joint","left_hip_yaw_joint","left_knee_joint","left_ankle_pitch_joint","left_ankle_roll_joint","right_hip_pitch_joint","right_hip_roll_joint","right_hip_yaw_joint","right_knee_joint","right_ankle_pitch_joint","right_ankle_roll_joint","waist_yaw_joint","waist_roll_joint","waist_pitch_joint","left_shoulder_pitch_joint","left_shoulder_roll_joint","left_shoulder_yaw_joint","left_elbow_joint","left_wrist_roll_joint","left_wrist_pitch_joint","left_wrist_yaw_joint","right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint","right_elbow_joint","right_wrist_roll_joint","right_wrist_pitch_joint","right_wrist_yaw_joint"]
a=argparse.ArgumentParser();a.add_argument("onnx");args=a.parse_args();m=onnx.load(args.onnx,load_external_data=False);md={p.key:p.value for p in m.metadata_props}
print("inputs:",[x.name for x in m.graph.input]);print("outputs:",[x.name for x in m.graph.output]);
for k in ["joint_names","observation_names","default_joint_pos","action_scale","joint_stiffness","joint_damping"]:
    if k not in md: sys.exit(f"FAIL: missing metadata {k}")
if md["joint_names"].split(",")!=EXPECTED_JOINTS:sys.exit("FAIL: joint_names mismatch")
if md["observation_names"].split(",")!=EXPECTED_OBS:sys.exit("FAIL: observation_names mismatch")
for k in ["default_joint_pos","action_scale","joint_stiffness","joint_damping"]:
    if len(md[k].split(","))!=29:sys.exit(f"FAIL: {k} is not 29D")
print("PASS: ONNX metadata matches wbc_ballet 29DoF actor ABI")

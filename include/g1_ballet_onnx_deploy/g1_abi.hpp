#pragma once
#include <array>
#include <string_view>

namespace g1_ballet {
constexpr size_t kNumJoints = 29;
constexpr size_t kObsDim = 186;
constexpr size_t kCommandDim = 61;

inline constexpr std::array<std::string_view, kNumJoints> kJointNames = {
  "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
  "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
  "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
  "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
  "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
  "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
  "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
  "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
  "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
};

inline constexpr std::array<float, kNumJoints> kJointLower = {
 -2.5307f,-0.5236f,-2.7576f,-0.087267f,-0.87267f,-0.2618f,
 -2.5307f,-2.9671f,-2.7576f,-0.087267f,-0.87267f,-0.2618f,
 -2.618f,-0.52f,-0.52f,-3.0892f,-1.5882f,-2.618f,-1.0472f,-1.97222f,-1.61443f,-1.61443f,
 -3.0892f,-2.2515f,-2.618f,-1.0472f,-1.97222f,-1.61443f,-1.61443f
};
inline constexpr std::array<float, kNumJoints> kJointUpper = {
  2.8798f,2.9671f,2.7576f,2.8798f,0.5236f,0.2618f,
  2.8798f,0.5236f,2.7576f,2.8798f,0.5236f,0.2618f,
  2.618f,0.52f,0.52f,2.6704f,2.2515f,2.618f,2.0944f,1.97222f,1.61443f,1.61443f,
  2.6704f,1.5882f,2.618f,2.0944f,1.97222f,1.61443f,1.61443f
};
}  // namespace g1_ballet

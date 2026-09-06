#pragma once

// Derived from Unitree's unitree_ros2 example/src/include/common/motor_crc_hg.h
// and example/src/src/common/motor_crc_hg.cpp.

#include <array>
#include <cstdint>
#include <cstring>

#include "unitree_hg/msg/low_cmd.hpp"

namespace g1_ballet {

struct RawMotorCmd {
  uint8_t mode;
  float q;
  float dq;
  float tau;
  float kp;
  float kd;
  uint32_t reserve = 0;
};

struct RawLowCmd {
  uint8_t mode_pr;
  uint8_t mode_machine;
  std::array<RawMotorCmd, 35> motor_cmd;
  std::array<uint32_t, 4> reserve;
  uint32_t crc;
};

inline uint32_t crc32_core(uint32_t *ptr, uint32_t len) {
  uint32_t crc32 = 0xFFFFFFFF;
  constexpr uint32_t polynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; ++i) {
    uint32_t xbit = 1u << 31;
    const uint32_t data = ptr[i];
    for (uint32_t bits = 0; bits < 32; ++bits) {
      if (crc32 & 0x80000000u) {
        crc32 = (crc32 << 1) ^ polynomial;
      } else {
        crc32 <<= 1;
      }
      if (data & xbit) crc32 ^= polynomial;
      xbit >>= 1;
    }
  }
  return crc32;
}

inline void set_crc(unitree_hg::msg::LowCmd &msg) {
  RawLowCmd raw{};
  raw.mode_pr = msg.mode_pr;
  raw.mode_machine = msg.mode_machine;
  for (int i = 0; i < 35; ++i) {
    raw.motor_cmd[i].mode = msg.motor_cmd[i].mode;
    raw.motor_cmd[i].q = msg.motor_cmd[i].q;
    raw.motor_cmd[i].dq = msg.motor_cmd[i].dq;
    raw.motor_cmd[i].tau = msg.motor_cmd[i].tau;
    raw.motor_cmd[i].kp = msg.motor_cmd[i].kp;
    raw.motor_cmd[i].kd = msg.motor_cmd[i].kd;
    raw.motor_cmd[i].reserve = msg.motor_cmd[i].reserve;
  }
  std::memcpy(&raw.reserve[0], &msg.reserve[0], 4);
  raw.crc = crc32_core(reinterpret_cast<uint32_t *>(&raw), (sizeof(RawLowCmd) >> 2) - 1);
  msg.crc = raw.crc;
}

}  // namespace g1_ballet

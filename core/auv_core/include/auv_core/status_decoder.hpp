#pragma once
#include <array>
#include <cstdint>
#include <vector>

namespace auv_core {
struct Stm32Status {
  std::uint32_t sequence{};
  bool armed{};
  bool dual_mode{}, autonomous_mode{};
  bool telemetry_valid{};
  bool voltage_valid{};
  std::uint32_t error_flags{};
  float voltage{}, depth{}, roll{}, pitch{}, yaw{};
  std::vector<float> thruster_outputs;
};
bool decode_status(const std::vector<std::uint8_t>& payload, Stm32Status& out);
}

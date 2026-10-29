#pragma once
#include "esphome/components/zigbee_firmware_baud/flash_status.h"

namespace esphome::cc2652_flash_guard {
struct FlashSession {
  bool active{false}, completed{false}, success{false}, resume_after{false};
  uint32_t previous_sequence{0};
  bool begin(bool already_paused, uint32_t sequence) {
    if (active) return false;
    active = true; completed = success = false;
    previous_sequence = sequence; resume_after = !already_paused;
    return true;
  }
  void observe(const zigbee_firmware_baud::FlashStatus &status) {
    if (!active || status.sequence == previous_sequence || status.result < 2) return;
    completed = true; success = status.result == 2;
  }
  void end() { active = false; }
};
}  // namespace esphome::cc2652_flash_guard

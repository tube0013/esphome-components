#pragma once

#include "network_config.h"

namespace esphome::network_settings {

enum class Phase : uint32_t { PHASE_CONFIRMED = 0, PHASE_STAGED = 1, PHASE_TESTING = 2 };

struct SavedState {
  uint32_t magic{0x545A4E31};  // TZN1; changing the storage layout requires a new magic/key.
  Phase phase{Phase::PHASE_CONFIRMED};
  NetworkConfig confirmed{};
  NetworkConfig candidate{};
};
static_assert(sizeof(SavedState) == 56, "Changing the saved record layout requires a new NVS key/version");

inline bool valid_saved_state(const SavedState &state) {
  return state.magic == 0x545A4E31 && uint32_t(state.phase) <= uint32_t(Phase::PHASE_TESTING) &&
         validate_config(state.confirmed) == nullptr && validate_config(state.candidate) == nullptr;
}

// Persistence is injected so power loss, failed writes, and boot transitions can
// be tested without an ESP32. Never update the live record before flash commits.
class TrialState {
 public:
  static constexpr uint32_t TIMEOUT_MS = 120000;
  SavedState saved{};
  NetworkConfig active{};
  bool testing{false};
  uint32_t started_at{0};

  template<typename Save> bool boot(uint32_t now, Save save) {
    this->active = this->saved.confirmed;
    this->testing = false;
    if (this->saved.phase == Phase::PHASE_CONFIRMED)
      return true;
    SavedState next = this->saved;
    next.phase = this->saved.phase == Phase::PHASE_STAGED ? Phase::PHASE_TESTING : Phase::PHASE_CONFIRMED;
    if (next.phase == Phase::PHASE_CONFIRMED)
      next.candidate = {};
    if (!save(next))
      return false;
    this->saved = next;
    if (this->saved.phase == Phase::PHASE_TESTING) {
      this->active = this->saved.candidate;
      this->started_at = now;
      this->testing = true;
    }
    return true;
  }

  template<typename Save> bool stage(const NetworkConfig &config, Save save) {
    if (this->testing || this->saved.phase != Phase::PHASE_CONFIRMED || validate_config(config) != nullptr)
      return false;
    SavedState next = this->saved;
    next.candidate = config;
    next.phase = Phase::PHASE_STAGED;
    if (!save(next))
      return false;
    this->saved = next;
    return true;
  }

  bool expired(uint32_t now) const { return this->testing && uint32_t(now - this->started_at) >= TIMEOUT_MS; }
  uint32_t remaining_seconds(uint32_t now) const {
    if (!this->testing || expired(now))
      return 0;
    return (TIMEOUT_MS - uint32_t(now - this->started_at) + 999) / 1000;
  }

  template<typename Save> bool finish(bool keep, uint32_t now, Save save) {
    if (!this->testing || (keep && expired(now)))
      return false;
    SavedState next = this->saved;
    if (keep)
      next.confirmed = this->active;
    next.candidate = {};
    next.phase = Phase::PHASE_CONFIRMED;
    if (!save(next))
      return false;
    this->saved = next;
    this->testing = false;
    return true;
  }
};

}  // namespace esphome::network_settings

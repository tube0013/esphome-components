#pragma once
#include <atomic>
namespace esphome::network_settings {
class SingleFlight {
 public:
  bool begin() {
    bool expected = false;
    return this->active_.compare_exchange_strong(expected, true);
  }
  void end() { this->active_.store(false); }

 private:
  std::atomic<bool> active_{false};
};
}  // namespace esphome::network_settings

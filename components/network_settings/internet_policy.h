#pragma once
#include <atomic>

namespace esphome::network_settings {
// Fail closed until NVS has been read, before Ethernet setup and boot checks.
class InternetPolicy {
 public:
  bool allowed() const { return this->ready_.load() && !this->offline_.load(); }
  bool offline() const { return this->offline_.load(); }
  void restore(bool offline) {
    this->offline_.store(offline);
    this->ready_.store(true);
  }
  template<typename Save> bool set(bool offline, Save save) {
    if (!this->ready_.load())
      return false;
    if (this->offline_.load() == offline)
      return true;
    if (!save(offline))
      return false;
    this->offline_.store(offline);
    return true;
  }

 protected:
  std::atomic<bool> ready_{false}, offline_{true};
};
}  // namespace esphome::network_settings

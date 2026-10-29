#pragma once
#include <cstdint>
#include <functional>
#include <utility>

namespace esphome::zigbee_firmware_baud {
// Independent of log text and verbosity. Published only from native flash paths.
struct FlashStatus {
  uint32_t sequence{0};
  uint8_t result{0};  // idle, running, succeeded, failed
  int progress{-1};
  std::function<void(const FlashStatus &)> listener;
  void start() { ++sequence; result = 1; progress = -1; publish(); }
  void advance(int value) { if (value >= 0 && value <= 100 && value != progress) { progress = value; publish(); } }
  void finish(bool ok) { result = ok ? 2 : 3; if (ok) progress = 100; publish(); }
  void publish() const { if (listener) listener(*this); }
};
// Every early return reports a final failure, after the native method's cleanup.
class FlashAttempt {
 public:
  explicit FlashAttempt(FlashStatus &status) : status_(status) { status_.start(); }
  ~FlashAttempt() { status_.finish(ok_); }
  void succeed() { ok_ = true; }
  FlashAttempt(const FlashAttempt &) = delete;
  FlashAttempt &operator=(const FlashAttempt &) = delete;
 private:
  FlashStatus &status_;
  bool ok_{false};
};
}  // namespace esphome::zigbee_firmware_baud

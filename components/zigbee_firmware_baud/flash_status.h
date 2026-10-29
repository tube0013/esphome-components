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
  void start() {
    ++this->sequence;
    this->result = 1;
    this->progress = -1;
    this->publish();
  }
  void advance(int value) {
    if (value >= 0 && value <= 100 && value != this->progress) {
      this->progress = value;
      this->publish();
    }
  }
  void finish(bool ok) {
    this->result = ok ? 2 : 3;
    if (ok)
      this->progress = 100;
    this->publish();
  }
  void publish() const {
    if (this->listener)
      this->listener(*this);
  }
};
// Every early return reports a final failure, after the native method's cleanup.
class FlashAttempt {
 public:
  explicit FlashAttempt(FlashStatus &status) : status_(status) { this->status_.start(); }
  ~FlashAttempt() { this->status_.finish(this->ok_); }
  void succeed() { this->ok_ = true; }
  FlashAttempt(const FlashAttempt &) = delete;
  FlashAttempt &operator=(const FlashAttempt &) = delete;

 private:
  FlashStatus &status_;
  bool ok_{false};
};
}  // namespace esphome::zigbee_firmware_baud

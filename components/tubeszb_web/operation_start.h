#pragma once
#include <cstdint>
#include <string>
namespace esphome::tubeszb_web {
// A queued command is not proof an updater actually started. Once a real OTA
// event arrives, its backend owns completion; this deadline never cancels a flash.
class PendingStart {
 public:
  void arm(uint32_t now) { pending_ = true; since_ = now; }
  void clear() { pending_ = false; }
  bool pending() const { return pending_; }
  bool expired(uint32_t now) const { return pending_ && uint32_t(now - since_) >= 30000; }
 private:
  bool pending_{false};
  uint32_t since_{0};
};
inline bool valid_ota_url(const std::string &url) {
  const size_t prefix = url.compare(0, 7, "http://") == 0 ? 7 : url.compare(0, 8, "https://") == 0 ? 8 : 0;
  return prefix != 0 && url.size() > prefix && url[prefix] != '/';
}
}  // namespace esphome::tubeszb_web

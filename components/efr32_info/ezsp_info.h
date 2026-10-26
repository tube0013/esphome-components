#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace esphome {
namespace efr32_info {
namespace protocol {

constexpr uint16_t GET_MFG_TOKEN = 0x000B;
constexpr uint16_t CUSTOM_FRAME = 0x0047;
constexpr uint16_t GET_VALUE = 0x00AA;
constexpr uint8_t VERSION_INFO = 0x11;
constexpr uint32_t MFG_TOKEN_OVERRIDES = 1U << 2;

inline bool frame_is(const std::vector<uint8_t> &frame, uint16_t id) {
  return frame.size() >= 5 && frame[3] == (id & 0xFF) && frame[4] == (id >> 8);
}

// EZSP 14 replaced the one-byte Ember/EZSP status with four-byte sl_status_t.
inline bool success(const std::vector<uint8_t> &frame, uint8_t version, size_t &pos) {
  const size_t width = version >= 14 ? 4 : 1;
  if (pos + width > frame.size())
    return false;
  for (size_t i = 0; i < width; ++i)
    if (frame[pos + i] != 0)
      return false;
  pos += width;
  return true;
}

inline bool read_lv(const std::vector<uint8_t> &frame, size_t pos, std::vector<uint8_t> &value) {
  value.clear();
  if (pos >= frame.size() || frame.size() - pos - 1 != frame[pos])
    return false;
  value.assign(frame.begin() + pos + 1, frame.end());
  return true;
}

// Manufacturing strings are ASCII, padded with NUL or erased flash bytes.
// Do not turn absent/binary tokens into plausible board names such as FFFF... .
inline bool identity_text(const std::vector<uint8_t> &value, std::string &out) {
  out.clear();
  bool padding = false;
  for (uint8_t b : value) {
    if (b == 0 || b == 0xFF) {
      padding = true;
    } else {
      if (padding || b < 0x20 || b > 0x7E) {
        out.clear();
        return false;
      }
      out.push_back(static_cast<char>(b));
    }
  }
  if (out.find_first_not_of(' ') == std::string::npos) {
    out.clear();
    return false;
  }
  return true;
}

inline bool token(const std::vector<uint8_t> &frame, std::string &out) {
  out.clear();
  std::vector<uint8_t> value;
  // getMfgToken has only an LVBytes response, with no status prefix.
  return frame_is(frame, GET_MFG_TOKEN) && read_lv(frame, 5, value) && identity_text(value, out);
}

inline bool custom(const std::vector<uint8_t> &frame, uint8_t version, uint16_t command,
                   std::vector<uint8_t> &value) {
  value.clear();
  size_t pos = 5;
  std::vector<uint8_t> payload;
  if (!frame_is(frame, CUSTOM_FRAME) || !success(frame, version, pos) || !read_lv(frame, pos, payload) ||
      payload.size() < 3 || payload[0] != (command & 0xFF) || payload[1] != ((command >> 8) | 0x80) ||
      payload[2] != 0)
    return false;
  value.assign(payload.begin() + 3, payload.end());
  return true;
}

inline bool identity_feature(const std::vector<uint8_t> &frame, uint8_t version) {
  std::vector<uint8_t> value;
  if (!custom(frame, version, 0x0000, value) || value.size() != 4)
    return false;
  uint32_t features = uint32_t(value[0]) | (uint32_t(value[1]) << 8) |
                      (uint32_t(value[2]) << 16) | (uint32_t(value[3]) << 24);
  return (features & MFG_TOKEN_OVERRIDES) != 0;
}

struct Version {
  uint16_t build;
  uint8_t major, minor, patch, special;
};

inline bool version_info(const std::vector<uint8_t> &frame, uint8_t protocol_version, Version &version) {
  size_t pos = 5;
  std::vector<uint8_t> value;
  if (!frame_is(frame, GET_VALUE) || !success(frame, protocol_version, pos) ||
      !read_lv(frame, pos, value) || (value.size() != 6 && value.size() != 7))
    return false;
  version = {static_cast<uint16_t>(value[0] | (uint16_t(value[1]) << 8)), value[2], value[3], value[4], value[5]};
  return true;
}

}  // namespace protocol
}  // namespace efr32_info
}  // namespace esphome

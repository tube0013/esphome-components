#pragma once

#include <cstdint>
#include <cstdio>
#include <vector>

#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/time.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace efr32 {
namespace ash {

constexpr uint8_t FLAG = 0x7E;
constexpr uint8_t ESC = 0x7D;
constexpr uint8_t XON = 0x11;
constexpr uint8_t XOFF = 0x13;
constexpr uint8_t SUB = 0x18;
constexpr uint8_t CAN = 0x1A;

inline bool is_control_byte(uint8_t b) {
  return b == FLAG || b == ESC || b == XON || b == XOFF || b == SUB || b == CAN;
}

inline uint16_t crc_hqx_be(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int j = 0; j < 8; j++) {
      if ((crc & 0x8000) != 0) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

inline void randomize(uint8_t *data, size_t len) {
  uint8_t rnd = 0x42;
  for (size_t i = 0; i < len; i++) {
    data[i] ^= rnd;
    if ((rnd & 0x01) == 0) {
      rnd >>= 1;
    } else {
      rnd = static_cast<uint8_t>((rnd >> 1) ^ 0xB8);
    }
  }
}

inline void randomize(std::vector<uint8_t> &data) {
  if (!data.empty())
    randomize(data.data(), data.size());
}

inline void write_escaped(esphome::uart::UARTComponent *uart, const uint8_t *data, size_t len) {
  if (uart == nullptr)
    return;
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[i];
    if (is_control_byte(b)) {
      const uint8_t esc[2] = {ESC, static_cast<uint8_t>(b ^ 0x20)};
      uart->write_array(esc, 2);
    } else {
      uart->write_byte(b);
    }
  }
}

inline void send_flagged_frame(esphome::uart::UARTComponent *uart, const std::vector<uint8_t> &body) {
  if (uart == nullptr)
    return;
  uart->write_byte(FLAG);
  std::vector<uint8_t> frame(body);
  uint16_t crc = crc_hqx_be(frame.data(), frame.size());
  frame.push_back(static_cast<uint8_t>(crc >> 8));
  frame.push_back(static_cast<uint8_t>(crc & 0xFF));
  write_escaped(uart, frame.data(), frame.size());
  uart->write_byte(FLAG);
}

inline void send_reset(esphome::uart::UARTComponent *uart) {
  if (uart == nullptr)
    return;
  uart->write_byte(CAN);
  uart->write_byte(FLAG);
  const uint8_t rst_body[] = {0xC0, 0x38, 0xBC};
  write_escaped(uart, rst_body, sizeof(rst_body));
  uart->write_byte(FLAG);
}

inline bool recv_ash_frame_loose(esphome::uart::UARTComponent *uart, std::vector<uint8_t> &out, uint32_t timeout_ms) {
  out.clear();
  if (uart == nullptr)
    return false;
  uint32_t start = millis();
  bool escape = false;
  while (millis() - start < timeout_ms) {
    uint8_t byte;
    if (uart->available() && uart->read_byte(&byte)) {
      if (byte == FLAG) {
        if (!out.empty()) {
          if (out.size() >= 2)
            out.resize(out.size() - 2);
          return true;
        }
        continue;
      }
      if (escape) {
        out.push_back(static_cast<uint8_t>(byte ^ 0x20));
        escape = false;
      } else if (byte == ESC) {
        escape = true;
      } else {
        out.push_back(byte);
      }
    } else {
      App.feed_wdt();
      esphome::delay(2);
    }
  }
  return false;
}

enum class FrameType : uint8_t {
  NONE = 0,
  DATA,
  ACK,
  NAK,
  COMMAND,
  RESET,
  ERROR
};

inline const char *frame_type_to_string(FrameType type) {
  switch (type) {
    case FrameType::DATA:
      return "data";
    case FrameType::ACK:
      return "ack";
    case FrameType::NAK:
      return "nak";
    case FrameType::COMMAND:
      return "command";
    case FrameType::RESET:
      return "reset";
    case FrameType::ERROR:
      return "error";
    default:
      return "none";
  }
}

struct Frame {
  FrameType type{FrameType::NONE};
  uint8_t control{0};
  std::vector<uint8_t> payload;
  bool ncp_ready{false};
};

struct LinkState {
  esphome::uart::UARTComponent *uart{nullptr};
  uint8_t tx_seq{0};
  uint8_t last_rx_seq{7};
  uint8_t next_ack{0};
  uint8_t last_ack{0};
  bool owe_ack{false};

  LinkState() = default;
  explicit LinkState(esphome::uart::UARTComponent *uart_component) : uart(uart_component) {}

  void attach(esphome::uart::UARTComponent *uart_component) {
    this->uart = uart_component;
    reset_counters();
  }

  void reset_counters() {
    this->tx_seq = 0;
    this->last_rx_seq = 7;
    this->next_ack = 0;
    this->last_ack = 0;
    this->owe_ack = false;
  }

  uint8_t ack_to_send() const { return this->next_ack; }
};

inline void send_ack(LinkState &state, bool ncp_ready = false, int8_t ack_override = -1) {
  auto *uart = state.uart;
  if (uart == nullptr)
    return;
  uint8_t ack = (ack_override >= 0) ? static_cast<uint8_t>(ack_override) : state.ack_to_send();
  uint8_t ctrl = static_cast<uint8_t>(0x80 | (ncp_ready ? 0x08 : 0x00) | (ack & 0x07));
  const uint8_t body[] = {ctrl};
  send_flagged_frame(uart, std::vector<uint8_t>(body, body + 1));
  state.next_ack = ack;
  state.last_ack = ack;
  state.owe_ack = false;
}

inline void send_nak(LinkState &state, uint8_t ack_num) {
  auto *uart = state.uart;
  if (uart == nullptr)
    return;
  uint8_t ctrl = static_cast<uint8_t>(0xA0 | (ack_num & 0x07));
  const uint8_t body[] = {ctrl};
  send_flagged_frame(uart, std::vector<uint8_t>(body, body + 1));
  state.owe_ack = false;
}

inline void send_data_frame(LinkState &state, const std::vector<uint8_t> &ezsp_payload, uint8_t ack_num, bool retransmit = false) {
  auto *uart = state.uart;
  if (uart == nullptr)
    return;
  uint8_t ctrl = static_cast<uint8_t>(((state.tx_seq & 0x07) << 4) | (retransmit ? 0x08 : 0x00) | (ack_num & 0x07));
  std::vector<uint8_t> body;
  body.reserve(1 + ezsp_payload.size());
  body.push_back(ctrl);
  if (!ezsp_payload.empty()) {
    std::vector<uint8_t> rnd(ezsp_payload);
    randomize(rnd);
    body.insert(body.end(), rnd.begin(), rnd.end());
  }
  send_flagged_frame(uart, body);
  if (!retransmit) {
    state.tx_seq = static_cast<uint8_t>((state.tx_seq + 1) & 0x07);
  }
}

inline FrameType classify_control(uint8_t control) {
  if ((control & 0x80) == 0)
    return FrameType::DATA;
  if ((control & 0xE0) == 0x80)
    return FrameType::ACK;
  if ((control & 0xE0) == 0xA0)
    return FrameType::NAK;
  if (control == 0xC0 || control == 0xC1)
    return FrameType::RESET;
  if (control == 0xC2)
    return FrameType::ERROR;
  if (control >= 0xC0)
    return FrameType::COMMAND;
  return FrameType::NONE;
}

inline int &ash_guard_depth_() {
  static int depth = 0;
  return depth;
}

class AshGuardScope {
 public:
  AshGuardScope() { ash_guard_depth_()++; }
  ~AshGuardScope() { ash_guard_depth_()--; }
};

inline bool ash_guard_is_active() { return ash_guard_depth_() > 0; }

static const char *const ASH_LOG_TAG = "ash_util";

inline std::string preview_bytes(const std::vector<uint8_t> &data, size_t limit = 8) {
  std::string out;
  size_t count = std::min(limit, data.size());
  out.reserve(count * 3 + (data.size() > count ? 3 : 0));
  char buf[4];
  for (size_t i = 0; i < count; i++) {
    std::snprintf(buf, sizeof(buf), "%02X", static_cast<unsigned>(data[i]));
    out.append(buf);
    if (i + 1 < count)
      out.push_back(' ');
  }
  if (data.size() > count)
    out.append("...");
  return out;
}

inline bool recv_frame(LinkState &state, Frame &out, uint32_t timeout_ms) {
  AshGuardScope guard_scope;
  auto *uart = state.uart;
  if (uart == nullptr)
    return false;
  uint32_t start = millis();
  bool collecting = false;
  bool escape = false;
  std::vector<uint8_t> buf;
  buf.reserve(128);

  while (millis() - start < timeout_ms) {
    uint8_t byte = 0;
    if (uart->available() && uart->read_byte(&byte)) {
      start = millis();
      if (byte == CAN) {
        if (collecting && !buf.empty()) {
          ESP_LOGV(ASH_LOG_TAG, "frame cancel drop len=%zu", buf.size());
        } else {
          ESP_LOGV(ASH_LOG_TAG, "frame cancel");
        }
        buf.clear();
        collecting = false;
        escape = false;
        continue;
      }
      if (byte == FLAG) {
        if (!collecting)
          continue;
        collecting = false;
        if (buf.size() < 3) {
          ESP_LOGW(ASH_LOG_TAG, "short frame before CRC len=%zu data=%s", buf.size(),
                   preview_bytes(buf).c_str());
          buf.clear();
          escape = false;
          continue;
        }
        uint16_t fcs = static_cast<uint16_t>(buf[buf.size() - 2] << 8) | buf[buf.size() - 1];
        std::vector<uint8_t> tmp(buf.begin(), buf.end() - 2);
        if (crc_hqx_be(tmp.data(), tmp.size()) != fcs) {
          ESP_LOGW(ASH_LOG_TAG, "crc mismatch len=%zu data=%s expected=%04X got=%04X", buf.size(),
                   preview_bytes(buf).c_str(), crc_hqx_be(tmp.data(), tmp.size()), fcs);
          buf.clear();
          collecting = false;
          escape = false;
          continue;
        }
        buf.swap(tmp);
          ESP_LOGV(ASH_LOG_TAG, "frame body len=%zu data=%s", buf.size(), preview_bytes(buf).c_str());
        if (!buf.empty() && buf[0] == CAN) {
          buf.erase(buf.begin());
        }
        if (buf.empty()) {
          buf.clear();
          collecting = false;
          escape = false;
          continue;
        }
        ESP_LOGV(ASH_LOG_TAG, "frame body (post-CAN) len=%zu data=%s", buf.size(),
                 preview_bytes(buf).c_str());
        uint8_t ctrl = buf[0];
        out.control = ctrl;
        out.payload.assign(buf.begin() + 1, buf.end());
        out.type = classify_control(ctrl);
        size_t payload_len = buf.size() > 0 ? buf.size() - 1 : 0;
        ESP_LOGV(ASH_LOG_TAG, "frame ctrl=0x%02X payload_len=%zu", ctrl, payload_len);
        if (out.type == FrameType::DATA) {
          randomize(out.payload);
          uint8_t ns = static_cast<uint8_t>((ctrl >> 4) & 0x07);
          state.last_rx_seq = ns;
          state.next_ack = static_cast<uint8_t>((ns + 1) & 0x07);
          state.owe_ack = true;
          ESP_LOGV(ASH_LOG_TAG, "recv_frame ctrl=0x%02X ns=%u re=%u payload_len=%zu", ctrl, ns,
                   (ctrl & 0x08) ? 1U : 0U, out.payload.size());
        } else if (out.type == FrameType::ACK || out.type == FrameType::NAK) {
          state.last_ack = static_cast<uint8_t>(ctrl & 0x07);
          state.owe_ack = false;
          out.ncp_ready = (out.type == FrameType::ACK) && ((ctrl & 0x08) != 0);
          ESP_LOGV(ASH_LOG_TAG, "recv_frame ctrl=0x%02X type=%d", ctrl, static_cast<int>(out.type));
        }
        return true;
      }
      if (!collecting) {
        collecting = true;
        buf.clear();
        escape = false;
        ESP_LOGV(ASH_LOG_TAG, "frame start");
      }
      if (escape) {
        buf.push_back(static_cast<uint8_t>(byte ^ 0x20));
        ESP_LOGV(ASH_LOG_TAG, "frame acc (esc) size=%zu byte=%02X", buf.size(), buf.back());
        escape = false;
      } else if (byte == ESC) {
        escape = true;
        ESP_LOGV(ASH_LOG_TAG, "frame esc start");
      } else {
        buf.push_back(byte);
        ESP_LOGV(ASH_LOG_TAG, "frame acc size=%zu byte=%02X", buf.size(), buf.back());
      }
    } else {
      App.feed_wdt();
      esphome::delay(1);
    }
  }
  if (collecting && !buf.empty()) {
    ESP_LOGW(ASH_LOG_TAG, "frame timeout len=%zu data=%s", buf.size(), preview_bytes(buf).c_str());
  }
  return false;
}

inline bool recv_frame_loose(LinkState &state, Frame &out, uint32_t timeout_ms) {
  AshGuardScope guard_scope;
  std::vector<uint8_t> raw;
  if (!recv_ash_frame_loose(state.uart, raw, timeout_ms))
    return false;
  while (!raw.empty() && raw.front() == CAN) {
    raw.erase(raw.begin());
  }
  if (raw.empty())
    return false;
  ESP_LOGV(ASH_LOG_TAG, "loose frame len=%zu data=%s", raw.size(), preview_bytes(raw).c_str());
  uint8_t ctrl = raw[0];
  out.control = ctrl;
  out.payload.assign(raw.begin() + 1, raw.end());
  out.type = classify_control(ctrl);
  if (out.type == FrameType::DATA) {
    randomize(out.payload);
    state.last_rx_seq = static_cast<uint8_t>((ctrl >> 4) & 0x07);
    state.next_ack = static_cast<uint8_t>((state.last_rx_seq + 1) & 0x07);
    state.owe_ack = true;
    ESP_LOGV(ASH_LOG_TAG, "recv_frame_loose ctrl=0x%02X ns=%u re=%u payload_len=%zu", ctrl,
             static_cast<unsigned>((ctrl >> 4) & 0x07), (ctrl & 0x08) ? 1U : 0U,
             out.payload.size());
  } else if (out.type == FrameType::ACK || out.type == FrameType::NAK) {
    state.last_ack = static_cast<uint8_t>(ctrl & 0x07);
    state.owe_ack = false;
    out.ncp_ready = (out.type == FrameType::ACK) && ((ctrl & 0x08) != 0);
    ESP_LOGV(ASH_LOG_TAG, "recv_frame_loose ctrl=0x%02X type=%d", ctrl,
             static_cast<int>(out.type));
  }
  return true;
}

}  // namespace ash
}  // namespace efr32
}  // namespace esphome

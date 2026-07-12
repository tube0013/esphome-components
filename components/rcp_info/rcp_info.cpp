#include "esphome/components/rcp_info/rcp_info.h"

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#if __has_include("esphome/components/stream_server/stream_server.h")
#include "esphome/components/stream_server/stream_server.h"
#define RCP_INFO_HAS_STREAM_SERVER 1
#endif

#include <cstdio>

namespace esphome {
namespace rcp_info {

namespace {
const char *const TAG = "rcp_info";

// Bytes OpenThread's HDLC encoder escapes: flag, escape, XON, XOFF, vendor 0xF8
bool needs_escape(uint8_t b) { return b == 0x7E || b == 0x7D || b == 0x11 || b == 0x13 || b == 0xF8; }
}  // namespace

uint16_t crc16_x25(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++)
      crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : (crc >> 1);
  }
  return crc ^ 0xFFFF;
}

std::vector<uint8_t> hdlc_encode(const std::vector<uint8_t> &payload) {
  uint16_t fcs = crc16_x25(payload.data(), payload.size());
  std::vector<uint8_t> raw = payload;
  raw.push_back(static_cast<uint8_t>(fcs & 0xFF));
  raw.push_back(static_cast<uint8_t>(fcs >> 8));

  std::vector<uint8_t> out;
  out.reserve(raw.size() + 4);
  out.push_back(HDLC_FLAG);
  for (uint8_t b : raw) {
    if (needs_escape(b)) {
      out.push_back(HDLC_ESCAPE);
      out.push_back(b ^ HDLC_XOR);
    } else {
      out.push_back(b);
    }
  }
  out.push_back(HDLC_FLAG);
  return out;
}

bool HdlcDecoder::feed(uint8_t byte, std::vector<uint8_t> &out) {
  if (byte == HDLC_FLAG) {
    bool ok = false;
    if (buffer_.size() >= 3) {  // payload + 2-byte FCS
      uint16_t fcs = static_cast<uint16_t>(buffer_[buffer_.size() - 2]) |
                     (static_cast<uint16_t>(buffer_[buffer_.size() - 1]) << 8);
      if (crc16_x25(buffer_.data(), buffer_.size() - 2) == fcs) {
        out.assign(buffer_.begin(), buffer_.end() - 2);
        ok = true;
      }
    }
    buffer_.clear();
    escaped_ = false;
    return ok;
  }
  if (byte == HDLC_ESCAPE) {
    escaped_ = true;
    return false;
  }
  if (escaped_) {
    buffer_.push_back(byte ^ HDLC_XOR);
    escaped_ = false;
  } else {
    buffer_.push_back(byte);
  }
  return false;
}

void RCPInfoComponent::loop() {
  if (!want_probe_)
    return;

  want_probe_ = false;
  run_probe_();
}

void RCPInfoComponent::publish_(esphome::text_sensor::TextSensor *sensor, const std::string &value) {
  if (sensor == nullptr)
    return;
  sensor->publish_state(value);
}

void RCPInfoComponent::delay_(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) {
    App.feed_wdt();
    esphome::delay(1);
  }
}

void RCPInfoComponent::flush_uart_() {
  if (uart_ == nullptr)
    return;
  uint8_t b;
  uint32_t discarded = 0;
  for (int i = 0; i < 96; i++) {
    bool any = false;
    while (uart_->available()) {
      if (uart_->read_byte(&b))
        discarded++;
      any = true;
      App.feed_wdt();
    }
    if (!any)
      break;
    delay_(2);
  }
  if (discarded > 0)
    ESP_LOGV(TAG, "flush_uart_ discarded %u bytes", static_cast<unsigned>(discarded));
}

void RCPInfoComponent::set_baud_(uint32_t baud) {
  if (uart_->get_baud_rate() == baud)
    return;
  ESP_LOGD(TAG, "Switching UART baud to %u", static_cast<unsigned>(baud));
  uart_->set_baud_rate(baud);
  uart_->load_settings(false);
  delay_(20);
}

bool RCPInfoComponent::spinel_get_(uint8_t tid, uint8_t prop, std::vector<uint8_t> &value, uint32_t timeout_ms) {
  std::vector<uint8_t> request{static_cast<uint8_t>(SPINEL_HEADER_FLAG | tid), CMD_PROP_VALUE_GET, prop};
  std::vector<uint8_t> tx = hdlc_encode(request);
  uart_->write_array(tx.data(), tx.size());

  std::vector<uint8_t> frame;
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    bool got_byte = false;
    uint8_t b;
    while (uart_->available() && uart_->read_byte(&b)) {
      got_byte = true;
      if (!decoder_.feed(b, frame))
        continue;
      if (frame.size() < 3)
        continue;
      // Spinel v2 header: FLG must be 0b10
      if ((frame[0] & 0xC0) != SPINEL_HEADER_FLAG)
        continue;
      uint8_t rx_tid = frame[0] & 0x0F;
      uint8_t cmd = frame[1];
      uint8_t rx_prop = frame[2];
      if (rx_tid != tid || cmd != CMD_PROP_VALUE_IS)
        continue;  // unsolicited frame (e.g. TID 0 log/status notifications)
      if (rx_prop == prop) {
        value.assign(frame.begin() + 3, frame.end());
        return true;
      }
      if (rx_prop == PROP_LAST_STATUS) {
        ESP_LOGW(TAG, "RCP replied with LAST_STATUS to prop %u GET (RCP-ish response, treating as failure)",
                 static_cast<unsigned>(prop));
        return false;
      }
    }
    if (!got_byte) {
      App.feed_wdt();
      esphome::delay(1);
    }
  }
  return false;
}

bool RCPInfoComponent::probe_current_baud_() {
  flush_uart_();
  decoder_.reset();

  std::vector<uint8_t> version;
  if (!spinel_get_(1, PROP_NCP_VERSION, version, per_baud_timeout_ms_))
    return false;

  std::string version_str;
  version_str.reserve(version.size());
  for (uint8_t b : version) {
    if (b == 0x00)
      break;
    if (b >= 0x20 && b <= 0x7E)
      version_str.push_back(static_cast<char>(b));
  }
  if (version_str.empty())
    return false;
  last_version_ = version_str;

  last_ieee_.clear();
  std::vector<uint8_t> eui64;
  if (spinel_get_(2, PROP_HWADDR, eui64, per_baud_timeout_ms_) && eui64.size() >= 8) {
    char buf[3 * 8];
    for (int i = 0; i < 8; i++)
      std::snprintf(buf + i * 3, sizeof(buf) - i * 3, "%02x%s", eui64[i], i < 7 ? ":" : "");
    last_ieee_ = buf;
  } else {
    ESP_LOGW(TAG, "RCP version OK but EUI-64 query failed");
  }
  return true;
}

void RCPInfoComponent::run_probe_() {
  if (uart_ == nullptr) {
    ESP_LOGE(TAG, "UART not configured");
    publish_(status_sensor_, "uart-missing");
    return;
  }

  ESP_LOGI(TAG, "Starting OpenThread RCP probe");
  publish_(status_sensor_, "probing");

  bool paused_stream = false;
#ifdef RCP_INFO_HAS_STREAM_SERVER
  if (stream_server_ != nullptr) {
    stream_server_->pause();
    paused_stream = true;
    delay_(50);
  }
#endif

  uint32_t orig_baud = uart_->get_baud_rate();
  detected_baud_ = 0;
  bool found = false;
  for (uint32_t baud : baud_rates_) {
    ESP_LOGD(TAG, "Probing for RCP at %u baud", static_cast<unsigned>(baud));
    set_baud_(baud);
    if (probe_current_baud_()) {
      detected_baud_ = baud;
      found = true;
      break;
    }
  }

  set_baud_(orig_baud);

  if (found) {
    ESP_LOGI(TAG, "RCP detected at %u baud: %s (EUI-64 %s)", static_cast<unsigned>(detected_baud_),
             last_version_.c_str(), last_ieee_.empty() ? "unknown" : last_ieee_.c_str());
    publish_(version_sensor_, last_version_);
    if (!last_ieee_.empty())
      publish_(ieee_sensor_, last_ieee_);
    char status[24];
    std::snprintf(status, sizeof(status), "rcp@%u", static_cast<unsigned>(detected_baud_));
    publish_(status_sensor_, status);
  } else {
    ESP_LOGW(TAG, "No RCP firmware detected on any configured baud rate");
    // Leave version/IEEE sensors untouched: a Zigbee probe may own them.
    publish_(status_sensor_, "not-found");
  }

#ifdef RCP_INFO_HAS_STREAM_SERVER
  if (paused_stream)
    stream_server_->resume();
#else
  (void) paused_stream;
#endif
}

}  // namespace rcp_info
}  // namespace esphome

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/text_sensor/text_sensor.h"

class StreamServerComponent;

namespace esphome {
namespace rcp_info {

// HDLC-lite framing (RFC 1662 subset used by OpenThread Spinel)
static constexpr uint8_t HDLC_FLAG = 0x7E;
static constexpr uint8_t HDLC_ESCAPE = 0x7D;
static constexpr uint8_t HDLC_XOR = 0x20;

// Spinel constants
static constexpr uint8_t SPINEL_HEADER_FLAG = 0x80;  // FLG=0b10, IID=0; low nibble carries the TID
static constexpr uint8_t CMD_PROP_VALUE_GET = 2;
static constexpr uint8_t CMD_PROP_VALUE_IS = 6;
static constexpr uint8_t PROP_LAST_STATUS = 0;
static constexpr uint8_t PROP_NCP_VERSION = 2;
static constexpr uint8_t PROP_HWADDR = 8;

// RFC 1662 FCS-16 (CRC16/X-25): poly 0x8408 reflected, init/xorout 0xFFFF.
// Distinct from ASH's crc_hqx (poly 0x1021) in efr32_info/ash_util.h.
uint16_t crc16_x25(const uint8_t *data, size_t len);

// Wraps a Spinel payload in an HDLC-lite frame with FCS.
std::vector<uint8_t> hdlc_encode(const std::vector<uint8_t> &payload);

// Incremental HDLC-lite decoder; feed bytes, get verified payloads.
class HdlcDecoder {
 public:
  void reset() {
    buffer_.clear();
    escaped_ = false;
  }
  // Returns true when `byte` completed a CRC-valid frame; payload (without FCS) is in `out`.
  bool feed(uint8_t byte, std::vector<uint8_t> &out);

 private:
  std::vector<uint8_t> buffer_;
  bool escaped_{false};
};

class RCPInfoComponent : public Component {
 public:
  void set_uart(esphome::uart::UARTComponent *uart) { uart_ = uart; }
  void set_stream_server(::StreamServerComponent *srv) { stream_server_ = srv; }
  void set_status_sensor(esphome::text_sensor::TextSensor *sensor) { status_sensor_ = sensor; }
  void set_version_sensor(esphome::text_sensor::TextSensor *sensor) { version_sensor_ = sensor; }
  void set_ieee_sensor(esphome::text_sensor::TextSensor *sensor) { ieee_sensor_ = sensor; }
  void set_baud_rates(const std::vector<uint32_t> &bauds) { baud_rates_ = bauds; }
  void set_per_baud_timeout(uint32_t ms) { per_baud_timeout_ms_ = ms; }

  void start_probe() { want_probe_ = true; }
  uint32_t detected_baud() const { return detected_baud_; }

  void setup() override {}
  void loop() override;

 private:
  void run_probe_();
  bool probe_current_baud_();
  bool spinel_get_(uint8_t tid, uint8_t prop, std::vector<uint8_t> &value, uint32_t timeout_ms);
  void set_baud_(uint32_t baud);
  void flush_uart_();
  void delay_(uint32_t ms);
  void publish_(esphome::text_sensor::TextSensor *sensor, const std::string &value);

  esphome::uart::UARTComponent *uart_{nullptr};
  ::StreamServerComponent *stream_server_{nullptr};
  esphome::text_sensor::TextSensor *status_sensor_{nullptr};
  esphome::text_sensor::TextSensor *version_sensor_{nullptr};
  esphome::text_sensor::TextSensor *ieee_sensor_{nullptr};

  std::vector<uint32_t> baud_rates_{921600, 460800, 115200};
  uint32_t per_baud_timeout_ms_{2000};
  uint32_t detected_baud_{0};
  std::string last_version_;
  std::string last_ieee_;
  bool want_probe_{false};
  HdlcDecoder decoder_;
};

template<typename... Ts> class RCPInfoProbeAction : public esphome::Action<Ts...> {
 public:
  void set_parent(RCPInfoComponent *parent) { parent_ = parent; }
  void play(const Ts &...x) override {
    if (parent_ != nullptr)
      parent_->start_probe();
  }

 private:
  RCPInfoComponent *parent_{nullptr};
};

}  // namespace rcp_info
}  // namespace esphome

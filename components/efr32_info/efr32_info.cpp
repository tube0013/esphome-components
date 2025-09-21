#include "esphome/components/efr32_info/efr32_info.h"

#include "esphome/components/efr32_info/ash_util.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <initializer_list>
#include <vector>

namespace esphome {
namespace efr32_info {

namespace {
const char *const TAG = "efr32_info";

std::string trim_token_ascii(const std::vector<uint8_t> &bytes) {
  std::string out;
  out.reserve(bytes.size());
  for (uint8_t b : bytes) {
    if (b == 0x00 || b == 0xFF)
      break;
    if (b >= 0x20 && b <= 0x7E)
      out.push_back(static_cast<char>(b));
  }
  return out;
}

std::string find_printable_run(const std::vector<uint8_t> &bytes) {
  std::string best;
  std::string current;
  for (uint8_t b : bytes) {
    if (b >= 0x20 && b <= 0x7E) {
      current.push_back(static_cast<char>(b));
    } else {
      if (current.size() > best.size())
        best = current;
      current.clear();
    }
  }
  if (current.size() > best.size())
    best = current;
  return best;
}

std::string network_status_to_string(uint8_t status) {
  switch (status) {
    case 0x00:
      return "no_network";
    case 0x01:
      return "joining";
    case 0x02:
      return "joined";
    case 0x03:
      return "joined_no_parent";
    case 0x04:
      return "leaving";
    default: {
      char buf[16];
      std::snprintf(buf, sizeof(buf), "0x%02X", static_cast<unsigned>(status));
      return buf;
    }
  }
}

}  // namespace

static inline uint8_t ctrl_ack(uint8_t ctrl) {
  return static_cast<uint8_t>(ctrl & 0x07);
}

void EFR32InfoComponent::loop() {
  if (!want_probe_)
    return;

  want_probe_ = false;
  run_probe_();
}

void EFR32InfoComponent::set_busy_(bool on) {
  if (busy_sensor_ != nullptr && busy_state_ != on) {
    busy_sensor_->publish_state(on);
    busy_state_ = on;
  }
}

void EFR32InfoComponent::publish_text_(esphome::text_sensor::TextSensor *sensor, const std::string &value) {
  if (sensor == nullptr)
    return;
  sensor->publish_state(value);
}

void EFR32InfoComponent::update_status_text_(const std::string &value) {
  if (last_status_ == value)
    return;
  last_status_ = value;
  publish_text_(status_sensor_, last_status_);
}

void EFR32InfoComponent::delay_(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) {
    App.feed_wdt();
    esphome::delay(1);
  }
}

void EFR32InfoComponent::flush_uart_() {
  if (uart_ == nullptr)
    return;
  uint8_t b;
  uint32_t discarded = 0;
  std::vector<uint8_t> sample;
  sample.reserve(32);
  for (int i = 0; i < 96; i++) {
    bool any = false;
    while (uart_->available()) {
      if (uart_->read_byte(&b)) {
        if (sample.size() < 32)
          sample.push_back(b);
        discarded++;
      }
      any = true;
      App.feed_wdt();
    }
    if (!any)
      break;
    delay_(2);
  }
  if (discarded > 0) {
    ESP_LOGV(TAG, "flush_uart_ discarded %u bytes sample=%s", static_cast<unsigned>(discarded),
             bytes_to_hex_(sample, 64).c_str());
  }
}

bool EFR32InfoComponent::recv_any_frame_(efr32::ash::Frame &frame, uint32_t timeout_ms) {
  if (efr32::ash::recv_frame(ash_state_, frame, timeout_ms))
    return true;

  efr32::ash::Frame loose_frame;
  if (!efr32::ash::recv_frame_loose(ash_state_, loose_frame, timeout_ms))
    return false;

  frame = loose_frame;
  ESP_LOGW(TAG, "Loose frame parsed ctrl=0x%02X len=%zu", static_cast<unsigned>(frame.control),
           frame.payload.size());
  return true;
}

bool EFR32InfoComponent::wait_for_initial_data_(uint32_t timeout_ms) {
  uint32_t deadline = millis() + timeout_ms;
  bool saw_any = false;
  bool saw_data = false;
  uint32_t last_data_ms = millis();
  while (millis() < deadline) {
    efr32::ash::Frame frame;
    if (!recv_any_frame_(frame, 80)) {
      if (!efr32::ash::recv_frame_loose(ash_state_, frame, 80)) {
        if (saw_data && millis() - last_data_ms > 40)
          return true;
        delay_(5);
        continue;
      }
      ESP_LOGV(TAG, "Loose frame ctrl=0x%02X type=%d while waiting for data",
               static_cast<unsigned>(frame.control), static_cast<int>(frame.type));
    }

    if (frame.type == efr32::ash::FrameType::DATA) {
      if (raw_capture_) {
        ESP_LOGV(TAG, "RAW RX ctrl=0x%02X len=%u data=%s", static_cast<unsigned>(frame.control),
                 static_cast<unsigned>(frame.payload.size()), bytes_to_hex_(frame.payload, 80).c_str());
      }
      ESP_LOGV(TAG, "wait_initial DATA ctrl=0x%02X len=%zu", static_cast<unsigned>(frame.control),
               frame.payload.size());
      ESP_LOGV(TAG, "wait_initial DATA ctrl=0x%02X len=%zu host_rx_seq_=%u",
               static_cast<unsigned>(frame.control), frame.payload.size(),
               static_cast<unsigned>(host_rx_seq_));
      bool accepted = ack_data_frame_(frame.control, frame.payload);
      saw_any = true;
      if (accepted)
        saw_data = true;
      if (accepted && (ash_state_.next_ack & 0x07) == 4)
        return true;
      last_data_ms = millis();
      continue;
    }
    if (frame.type == efr32::ash::FrameType::RESET) {
      apply_rstack_from_payload_(frame.payload);
      saw_any = true;
      last_data_ms = millis();
      continue;
    }
  }
  ESP_LOGW(TAG, "wait_for_initial_data_ timeout saw_data=%d host_rx_seq_=%u next_ack=%u",
           saw_data ? 1 : 0, static_cast<unsigned>(host_rx_seq_),
           static_cast<unsigned>(ash_state_.next_ack & 0x07));
  return saw_data;
}

void EFR32InfoComponent::drain_frames_(uint32_t idle_timeout_ms, int target_ack) {
  uint32_t idle_start = millis();
  while (millis() - idle_start < idle_timeout_ms) {
    efr32::ash::Frame frame;
    if (!recv_any_frame_(frame, 40)) {
      App.feed_wdt();
      delay_(5);
      continue;
    }

    idle_start = millis();
    switch (frame.type) {
      case efr32::ash::FrameType::DATA: {
        bool accepted = ack_data_frame_(frame.control, frame.payload);
        ESP_LOGV(TAG, "drain DATA ctrl=0x%02X accepted=%s host_rx_seq_=%u",
                 static_cast<unsigned>(frame.control), accepted ? "true" : "false",
                 static_cast<unsigned>(host_rx_seq_));
        if (accepted && target_ack >= 0 && host_rx_seq_ == target_ack) {
          ESP_LOGI(TAG, "drain_frames_ reached target ack=%d host_rx_seq_=%u",
                   target_ack, static_cast<unsigned>(host_rx_seq_));
          return;
        }
        break;
      }
      case efr32::ash::FrameType::ACK: {
        uint8_t ack = static_cast<uint8_t>(frame.control & 0x07);
        ESP_LOGV(TAG, "drain ACK ctrl=0x%02X ack=%u", static_cast<unsigned>(frame.control),
                 static_cast<unsigned>(ack));
        break;
      }
      case efr32::ash::FrameType::NAK: {
        uint8_t ack = static_cast<uint8_t>(frame.control & 0x07);
        ESP_LOGV(TAG, "drain NAK ctrl=0x%02X ack=%u", static_cast<unsigned>(frame.control),
                 static_cast<unsigned>(ack));
        break;
      }
      case efr32::ash::FrameType::ERROR:
        ESP_LOGW(TAG, "Drain saw ASH error 0x%04X", frame.payload.size() >= 2
                                                   ? static_cast<unsigned>(frame.payload[0] << 8 | frame.payload[1])
                                                   : 0U);
        break;
      case efr32::ash::FrameType::RESET:
        apply_rstack_from_payload_(frame.payload);
        break;
      default:
        break;
    }
  }
  ESP_LOGV(TAG, "drain_frames_ exit idle_timeout=%u host_rx_seq_=%u next_ack=%u",
           static_cast<unsigned>(idle_timeout_ms), static_cast<unsigned>(host_rx_seq_),
           static_cast<unsigned>(ash_state_.next_ack & 0x07));
}

bool EFR32InfoComponent::prime_link_(uint32_t timeout_ms) {
  uint32_t deadline = millis() + timeout_ms;
  while (millis() < deadline) {
    efr32::ash::Frame frame;
    if (!recv_any_frame_(frame, 40)) {
      App.feed_wdt();
      delay_(5);
      continue;
    }

    switch (frame.type) {
      case efr32::ash::FrameType::RESET: {
        apply_rstack_from_payload_(frame.payload);
        break;
      }
      case efr32::ash::FrameType::DATA: {
        ESP_LOGV(TAG, "prime DATA ctrl=0x%02X len=%zu", static_cast<unsigned>(frame.control),
                 frame.payload.size());
        bool accepted = ack_data_frame_(frame.control, frame.payload);
        ESP_LOGV(TAG, "prime DATA ctrl=0x%02X accepted=%s host_rx_seq_=%u",
                 static_cast<unsigned>(frame.control), accepted ? "true" : "false",
                 static_cast<unsigned>(host_rx_seq_));
        if (host_rx_seq_ >= 4)
          return true;
        break;
      }
      case efr32::ash::FrameType::ACK: {
        uint8_t ack = static_cast<uint8_t>(frame.control & 0x07);
        ESP_LOGV(TAG, "prime ACK ctrl=0x%02X ack=%u", static_cast<unsigned>(frame.control),
                 static_cast<unsigned>(ack));
        break;
      }
      case efr32::ash::FrameType::NAK: {
        uint8_t ack = static_cast<uint8_t>(frame.control & 0x07);
        ESP_LOGV(TAG, "prime NAK ctrl=0x%02X ack=%u", static_cast<unsigned>(frame.control),
                 static_cast<unsigned>(ack));
        break;
      }
      case efr32::ash::FrameType::ERROR: {
        ESP_LOGW(TAG, "ASH error during prime 0x%02X",
                 frame.payload.empty() ? 0 : frame.payload[0]);
        break;
      }
      default:
        break;
    }
  }
  return host_rx_seq_ >= 4;
}

void EFR32InfoComponent::apply_rstack_from_payload_(const std::vector<uint8_t> &payload, bool loose) {
  if (payload.empty())
    return;
  uint8_t ash_ver = payload[0];
  uint8_t reset_code = payload.size() >= 2 ? payload[1] : 0xFF;
  saw_rstack_ = true;
  host_rx_seq_ = 0;
  ash_state_.next_ack = 0;
  have_last_delivered_ = false;
  last_delivered_seq_ = 0;
  bootstrap_seq_known_ = false;
  char buf[64];
  if (loose) {
    std::snprintf(buf, sizeof(buf), "ASHv%u reset=0x%02X (loose)", static_cast<unsigned>(ash_ver),
                  static_cast<unsigned>(reset_code));
  } else {
    std::snprintf(buf, sizeof(buf), "ASHv%u reset=0x%02X", static_cast<unsigned>(ash_ver),
                  static_cast<unsigned>(reset_code));
  }
  last_ezsp_version_ = buf;
  publish_text_(ezsp_version_sensor_, last_ezsp_version_);
  efr32::ash::send_ack(ash_state_, false, 0);
  ESP_LOGV(TAG, "RSTACK acked with seq=0%s", loose ? " (loose)" : "");
  if (raw_capture_) {
    ESP_LOGV(TAG, "RAW RX RSTACK ver=%u code=0x%02X (%s)", static_cast<unsigned>(ash_ver),
             static_cast<unsigned>(reset_code), loose ? "loose" : "direct");
  }
}

void EFR32InfoComponent::seed_initial_window_(uint8_t delivered_seq) {
  last_delivered_seq_ = delivered_seq;
  have_last_delivered_ = true;
  host_rx_seq_ = static_cast<uint8_t>((delivered_seq + 1) & 0x07);
  ash_state_.next_ack = host_rx_seq_;
  ash_state_.last_ack = host_rx_seq_;
  ash_state_.last_rx_seq = delivered_seq;
  bootstrap_seq_known_ = true;
  ESP_LOGI(TAG, "Seeded initial window seq=%u -> host_rx_seq_=%u", static_cast<unsigned>(delivered_seq),
           static_cast<unsigned>(host_rx_seq_));
}

bool try_loose_rstack_(esphome::uart::UARTComponent *uart, uint8_t &ash_ver, uint8_t &reset_code, uint32_t timeout_ms) {
  if (uart == nullptr)
    return false;
  uint32_t start = millis();
  std::vector<uint8_t> buf;
  buf.reserve(64);
  while (millis() - start < timeout_ms) {
    uint8_t b;
    if (uart->available() && uart->read_byte(&b)) {
      buf.push_back(b);
      if (buf.size() > 64)
        buf.erase(buf.begin(), buf.begin() + (buf.size() - 64));
      if (b == 0x7E) {
        if (buf.size() >= 5) {
          size_t end = buf.size() - 1;
          size_t limit = end >= 2 ? end - 2 : 0;
          for (size_t i = 0; i + 2 < limit; i++) {
            if (buf[i] == 0xC1) {
              ash_ver = buf[i + 1];
              reset_code = buf[i + 2];
              return true;
            }
          }
        }
      }
    } else {
      App.feed_wdt();
      esphome::delay(2);
    }
  }
  return false;
}

void EFR32InfoComponent::parse_unsolicited_ezsp_(const std::vector<uint8_t> &ezsp) {
  if (ezsp.size() < 5)
    return;

  const uint16_t frame_id = static_cast<uint16_t>(ezsp[3]) | (static_cast<uint16_t>(ezsp[4]) << 8);
  const uint8_t status = ezsp.size() > 5 ? ezsp[5] : 0xFF;

  switch (frame_id) {
    case 0x0002: {  // getVersion
      if (status != 0x00 || ezsp.size() < 11)
        break;
      uint8_t major = ezsp[6];
      uint8_t minor = ezsp[7];
      uint8_t patch = ezsp[8];
      uint8_t special = ezsp[9];
      uint32_t build = static_cast<uint32_t>(ezsp[10]);
      if (ezsp.size() >= 12)
        build |= static_cast<uint32_t>(ezsp[11]) << 8;
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%u.%u.%u.%u build %u",
                    static_cast<unsigned>(major), static_cast<unsigned>(minor),
                    static_cast<unsigned>(patch), static_cast<unsigned>(special),
                    static_cast<unsigned>(build));
      last_stack_version_ = buf;
      publish_text_(stack_version_sensor_, last_stack_version_);
      char fw_only[32];
      std::snprintf(fw_only, sizeof(fw_only), "%u.%u.%u.%u",
                    static_cast<unsigned>(major), static_cast<unsigned>(minor),
                    static_cast<unsigned>(patch), static_cast<unsigned>(special));
      last_firmware_ = fw_only;
      publish_text_(firmware_sensor_, last_firmware_);
      break;
    }
    case 0x000B: {  // getMfgToken
      if (status != 0x00 || ezsp.size() < 7)
        break;
      uint8_t length = ezsp[6];
      size_t start = 7;
      if (ezsp.size() < start + length)
        length = static_cast<uint8_t>(ezsp.size() > start ? ezsp.size() - start : 0);
      if (length == 0)
        break;
      std::vector<uint8_t> data(ezsp.begin() + start, ezsp.begin() + start + length);
      std::string text = extract_ascii_(data);
      if (text.empty())
        break;
      if (last_manufacturer_.empty()) {
        last_manufacturer_ = text;
        publish_text_(manufacturer_sensor_, last_manufacturer_);
      } else if (last_board_name_.empty() && text != last_manufacturer_) {
        last_board_name_ = text;
        publish_text_(board_name_sensor_, last_board_name_);
      }
      break;
    }
    case 0x0026: {  // getEui64
      if (status != 0x00 || ezsp.size() < 14)
        break;
      std::vector<uint8_t> eui(ezsp.begin() + 6, ezsp.begin() + 14);
      char buf[24];
      std::snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                    eui[7], eui[6], eui[5], eui[4], eui[3], eui[2], eui[1], eui[0]);
      last_ieee_ = buf;
      publish_text_(ieee_sensor_, last_ieee_);
      break;
    }
    case 0x0027: {  // getNodeId
      if (status != 0x00 || ezsp.size() < 8)
        break;
      uint16_t node = static_cast<uint16_t>(ezsp[6]) | (static_cast<uint16_t>(ezsp[7]) << 8);
      char buf[12];
      std::snprintf(buf, sizeof(buf), "0x%04X", static_cast<unsigned>(node));
      last_node_id_ = buf;
      publish_text_(node_id_sensor_, last_node_id_);
      break;
    }
    case 0x0018: {  // getNetworkParameters?
      if (status != 0x00 || ezsp.size() < 7)
        break;
      uint8_t state = ezsp[6];
      last_network_status_ = network_status_to_string(state);
      publish_text_(network_status_sensor_, last_network_status_);
      break;
    }
    case 0x0069: {  // getKeyTableEntry / security state (bitmask)
      if (status != 0x00 || ezsp.size() < 8)
        break;
      uint16_t bitmask = static_cast<uint16_t>(ezsp[6]) | (static_cast<uint16_t>(ezsp[7]) << 8);
      char buf[48];
      std::snprintf(buf, sizeof(buf), "mask=0x%04X", static_cast<unsigned>(bitmask));
      last_security_state_ = buf;
      publish_text_(security_state_sensor_, last_security_state_);
      break;
    }
    default:
      break;
  }
}

bool EFR32InfoComponent::parse_token_reply_(const std::vector<uint8_t> &resp, std::string &out) {
  if (resp.size() < 7)
    return false;
  uint8_t length = 0;
  size_t start = 0;
  if (resp[5] == 0x00 && resp.size() >= 8) {
    length = resp[6];
    start = 7;
  } else {
    length = resp[5];
    start = 6;
  }
  if (start >= resp.size())
    return false;
  if (start + length > resp.size())
    length = static_cast<uint8_t>(resp.size() - start);
  if (length == 0)
    return false;
  std::vector<uint8_t> data(resp.begin() + start, resp.begin() + start + length);
  std::string text = extract_ascii_(data);
  if (text.empty())
    text = bytes_to_hex_(data, 32);
  out = text;
  return true;
}

bool EFR32InfoComponent::parse_v8_version_reply_(const std::vector<uint8_t> &resp) {
  if (resp.size() < 9)
    return false;
  if (!(resp[1] == 0x80 && resp[2] == 0x01 && resp[3] == 0x00 && resp[4] == 0x00))
    return false;
  uint8_t status = resp[5];
  bool status_ok = (status == 0x00 || status == 0x0D);

  uint8_t stack_type = resp[6];
  uint16_t stack_version = static_cast<uint16_t>(resp[7]) | (static_cast<uint16_t>(resp[8]) << 8);
  unsigned A = (stack_version >> 12) & 0x0F;
  unsigned B = (stack_version >> 8) & 0x0F;
  unsigned C = (stack_version >> 4) & 0x0F;
  unsigned D = stack_version & 0x0F;
  char buf[80];
  std::snprintf(buf, sizeof(buf), "type=0x%02X ver=%u.%u.%u.%u (0x%04X) status=0x%02X",
                static_cast<unsigned>(stack_type), A, B, C, D, static_cast<unsigned>(stack_version),
                static_cast<unsigned>(status));
  last_stack_version_ = buf;
  publish_text_(stack_version_sensor_, last_stack_version_);
  char fw_buf[32];
  std::snprintf(fw_buf, sizeof(fw_buf), "%u.%u.%u.%u", A, B, C, D);
  last_firmware_ = fw_buf;
  publish_text_(firmware_sensor_, last_firmware_);
  if (!status_ok) {
    ESP_LOGW(TAG, "VERSION(min) returned status=0x%02X", static_cast<unsigned>(status));
  }
  return status_ok;
}

void EFR32InfoComponent::ack_re_sync_() {
  uint8_t expected = host_rx_seq_;
  if (have_last_delivered_) {
    expected = static_cast<uint8_t>((last_delivered_seq_ + 1) & 0x07);
  }
  host_rx_seq_ = expected;
  ash_state_.next_ack = expected;
  ash_state_.last_ack = expected;
  ash_state_.last_rx_seq = have_last_delivered_ ? last_delivered_seq_ : 7;
  bootstrap_seq_known_ = have_last_delivered_;
  ESP_LOGV(TAG, "ACK re-sync host_rx_seq_=%u have_last=%d",
           static_cast<unsigned>(host_rx_seq_), have_last_delivered_ ? 1 : 0);
}

bool EFR32InfoComponent::ack_data_frame_(uint8_t ctrl, const std::vector<uint8_t> &payload) {
  const bool retransmit = (ctrl & 0x08) != 0;
  const uint8_t ns = static_cast<uint8_t>((ctrl >> 4) & 0x07);
  const uint8_t frame_ack = static_cast<uint8_t>(ctrl & 0x07);
  const uint8_t expected = host_rx_seq_;
  (void) frame_ack;
  uint8_t ack_num = host_rx_seq_;
  bool accepted_new_payload = false;

  ESP_LOGV(TAG,
           "RX DATA ctrl=0x%02X ns=%u re=%u ack=%u expect=%u host_rx=%u len=%u",
           static_cast<unsigned>(ctrl), static_cast<unsigned>(ns), retransmit ? 1U : 0U,
           static_cast<unsigned>(frame_ack), static_cast<unsigned>(expected),
           static_cast<unsigned>(host_rx_seq_), static_cast<unsigned>(payload.size()));

  if (!link_bootstrapped_)
    link_bootstrapped_ = true;

  if (!payload.empty() && payload[0] == 0xC1) {
    apply_rstack_from_payload_(payload);
    host_rx_seq_ = 0;
    have_last_delivered_ = false;
    last_delivered_seq_ = 0;
    ash_state_.last_rx_seq = ns;
    ash_state_.next_ack = 0;
    ash_state_.last_ack = 0;
    efr32::ash::send_ack(ash_state_, false, 0);
    ESP_LOGV(TAG, "RSTACK via DATA frame ns=%u", static_cast<unsigned>(ns));
    return false;
  }

  uint8_t delta = static_cast<uint8_t>((ns - expected) & 0x07);
  ESP_LOGV(TAG,
           "FRAME ctrl=0x%02X ns=%u re=%u expected=%u delta=%u ack=%u host_rx=%u have_last=%d last=%u len=%zu",
           static_cast<unsigned>(ctrl), static_cast<unsigned>(ns), retransmit ? 1U : 0U,
           static_cast<unsigned>(expected), static_cast<unsigned>(delta),
           static_cast<unsigned>(frame_ack), static_cast<unsigned>(host_rx_seq_),
           have_last_delivered_ ? 1 : 0, static_cast<unsigned>(last_delivered_seq_), payload.size());

  uint8_t desired_tx_seq = frame_ack;
  if (ash_state_.tx_seq != desired_tx_seq) {
    ESP_LOGV(TAG, "Re-sync tx_seq from %u to %u based on ack", static_cast<unsigned>(ash_state_.tx_seq),
             static_cast<unsigned>(desired_tx_seq));
    ash_state_.tx_seq = desired_tx_seq;
  }

  if (retransmit) {
    if (payload.empty()) {
      ESP_LOGV(TAG, "Retransmit empty DATA ns=%u acking without advance",
               static_cast<unsigned>(ns));
    } else {
      ESP_LOGV(TAG, "Retransmit DATA seq=%u (ack=%u no advance)", static_cast<unsigned>(ns),
               static_cast<unsigned>(host_rx_seq_));
    }
    ack_num = host_rx_seq_;
  } else if (ns == expected) {
    host_rx_seq_ = static_cast<uint8_t>((host_rx_seq_ + 1) & 0x07);
    ack_num = host_rx_seq_;
    accepted_new_payload = true;
    bootstrap_seq_known_ = true;
    ESP_LOGV(TAG, "Accepting DATA seq=%u -> host_rx_seq_=%u", static_cast<unsigned>(ns),
             static_cast<unsigned>(host_rx_seq_));
  } else if (have_last_delivered_ && ns == last_delivered_seq_) {
    ESP_LOGV(TAG, "Duplicate DATA seq=%u (last=%u) acking", static_cast<unsigned>(ns),
             static_cast<unsigned>(last_delivered_seq_));
    ack_num = host_rx_seq_;
  } else {
    ESP_LOGW(TAG, "Out-of-sequence DATA frame ns=%u expected=%u delta=%u (NAK)",
             static_cast<unsigned>(ns), static_cast<unsigned>(expected),
             static_cast<unsigned>(delta));
    ash_state_.next_ack = expected;
    ash_state_.last_ack = expected;
    efr32::ash::send_nak(ash_state_, expected);
    if (raw_capture_) {
      ESP_LOGV(TAG, "RAW TX NAK ack=%u", static_cast<unsigned>(expected));
    }
    return false;
  }

  if (accepted_new_payload) {
    parse_unsolicited_ezsp_(payload);
    ash_state_.last_rx_seq = ns;
    last_delivered_seq_ = ns;
    have_last_delivered_ = true;
  }

  ash_state_.next_ack = host_rx_seq_;
  ash_state_.last_ack = ack_num;
  efr32::ash::send_ack(ash_state_, false, ack_num);

  if (raw_capture_) {
    ESP_LOGV(TAG, "RAW RX DATA ctrl=0x%02X ns=%u re=%u ack=%u len=%u data=%s",
             static_cast<unsigned>(ctrl), static_cast<unsigned>(ns), retransmit ? 1 : 0,
             static_cast<unsigned>(ack_num), static_cast<unsigned>(payload.size()),
             bytes_to_hex_(payload, 80).c_str());
    ESP_LOGV(TAG, "RAW TX ACK ack=%u", static_cast<unsigned>(ack_num));
  }

  ESP_LOGV(TAG, "ACK ctrl=0x%02X ns=%u re=%u ack=%u expected_before=%u",
           static_cast<unsigned>(ctrl), static_cast<unsigned>(ns), retransmit ? 1 : 0,
           static_cast<unsigned>(ack_num), static_cast<unsigned>(expected));

  ESP_LOGV(TAG, "FRAME handled accepted=%d host_rx_now=%u ack_sent=%u have_last=%d",
           accepted_new_payload ? 1 : 0, static_cast<unsigned>(host_rx_seq_),
           static_cast<unsigned>(ack_num), have_last_delivered_ ? 1 : 0);

  return accepted_new_payload;
}

void EFR32InfoComponent::send_rst_frame_() {
  const uint32_t start = millis();
  if (!saw_rstack_) {
    ash_state_.reset_counters();
    link_bootstrapped_ = false;
    ash_state_.next_ack = 0;
    host_rx_seq_ = 0;
    have_last_delivered_ = false;
    last_delivered_seq_ = 0;
    bootstrap_seq_known_ = false;
    efr32::ash::send_reset(uart_);
  } else {
    // Bellows sends CANCEL + RST + FLAG even when RSTACK already seen.
    auto *uart = uart_;
    if (uart != nullptr) {
      const uint8_t cancel = efr32::ash::CAN;
      const uint8_t rst_body[] = {0xC0, 0x38, 0xBC};
      uart->write_byte(cancel);
      uart->write_byte(efr32::ash::FLAG);
      efr32::ash::write_escaped(uart, rst_body, sizeof(rst_body));
      uart->write_byte(efr32::ash::FLAG);
    }
  }
  ESP_LOGD(TAG, "send_rst_frame duration=%u", static_cast<unsigned>(millis() - start));
}

void EFR32InfoComponent::send_data_frame_(const std::vector<uint8_t> &ezsp_payload, bool retransmit) {
  uint8_t ack = host_rx_seq_;
  ESP_LOGV(TAG, "TX DATA seq=%u ack=%u len=%u", static_cast<unsigned>(ash_state_.tx_seq),
           static_cast<unsigned>(ack), static_cast<unsigned>(ezsp_payload.size()));
  if (raw_capture_) {
    ESP_LOGV(TAG, "RAW TX DATA seq=%u ack=%u len=%u data=%s",
             static_cast<unsigned>(ash_state_.tx_seq), static_cast<unsigned>(ack),
             static_cast<unsigned>(ezsp_payload.size()), bytes_to_hex_(ezsp_payload, 80).c_str());
  }
  efr32::ash::send_data_frame(ash_state_, ezsp_payload, ack, retransmit);
  if (!retransmit) {
    last_tx_payload_ = ezsp_payload;
    last_tx_seq_ = static_cast<uint8_t>((ash_state_.tx_seq + 7) & 0x07);
    have_last_tx_ = true;
  }
}

void EFR32InfoComponent::send_v4_version_probe_(bool retransmit) {
  std::vector<uint8_t> payload{0x00, 0x00, 0x00, 0x04};
  ESP_LOGI(TAG, "%s legacy VERSION(min) probe", retransmit ? "Retransmitting" : "Sending");
  send_data_frame_(payload, retransmit);
}

void EFR32InfoComponent::retransmit_last_tx_(uint8_t ack_num) {
  if (!have_last_tx_) {
    ESP_LOGW(TAG, "No frame available for retransmission (ack=%u)", static_cast<unsigned>(ack_num));
    return;
  }
  uint8_t saved_seq = ash_state_.tx_seq;
  uint8_t saved_next_ack = ash_state_.next_ack;
  uint8_t saved_last_ack = ash_state_.last_ack;
  ash_state_.tx_seq = last_tx_seq_;
  ESP_LOGW(TAG, "Retransmitting seq=%u with ack=%u len=%zu", static_cast<unsigned>(last_tx_seq_),
           static_cast<unsigned>(ack_num), last_tx_payload_.size());
  if (raw_capture_) {
    ESP_LOGV(TAG, "RAW TX RETX seq=%u ack=%u len=%u data=%s", static_cast<unsigned>(last_tx_seq_),
             static_cast<unsigned>(ack_num), static_cast<unsigned>(last_tx_payload_.size()),
             bytes_to_hex_(last_tx_payload_, 80).c_str());
  }
  efr32::ash::send_data_frame(ash_state_, last_tx_payload_, ack_num, true);
  ash_state_.tx_seq = saved_seq;
  ash_state_.next_ack = saved_next_ack;
  ash_state_.last_ack = saved_last_ack;
}

bool EFR32InfoComponent::wait_for_v4_version_(uint32_t timeout_ms) {
  uint32_t deadline = millis() + timeout_ms;
  uint32_t last_progress = millis();
  uint8_t last_seen_seq = 0xFF;
  while (millis() < deadline) {
    efr32::ash::Frame frame;
    if (!recv_any_frame_(frame, 80)) {
      if (!efr32::ash::recv_frame_loose(ash_state_, frame, 120)) {
        if (millis() - last_progress > 250) {
          ESP_LOGV(TAG, "wait_v4 idle host_rx_seq_=%u next_ack=%u", static_cast<unsigned>(host_rx_seq_),
                   static_cast<unsigned>(ash_state_.next_ack & 0x07));
          last_progress = millis();
        }
        delay_(5);
        continue;
      }
      ESP_LOGV(TAG, "wait_v4 loose ctrl=0x%02X type=%d", static_cast<unsigned>(frame.control),
               static_cast<int>(frame.type));
    }

    switch (frame.type) {
      case efr32::ash::FrameType::DATA: {
        if (raw_capture_) {
          ESP_LOGV(TAG, "RAW RX V4 ctrl=0x%02X len=%u data=%s", static_cast<unsigned>(frame.control),
                   static_cast<unsigned>(frame.payload.size()), bytes_to_hex_(frame.payload, 80).c_str());
        }
        bool accepted = ack_data_frame_(frame.control, frame.payload);
        ESP_LOGV(TAG, "wait_v4 DATA ctrl=0x%02X accepted=%d host_rx_seq_=%u",
                 static_cast<unsigned>(frame.control), accepted ? 1 : 0,
                 static_cast<unsigned>(host_rx_seq_));
        uint8_t seq = static_cast<uint8_t>((frame.control >> 4) & 0x07);
        if (accepted) {
          last_progress = millis();
          last_seen_seq = seq;
        } else if (seq == last_seen_seq) {
          // duplicate burst following an accepted frame; keep waiting
          last_progress = millis();
        }
        if (!accepted)
          continue;

        const std::vector<uint8_t> &ezsp = frame.payload;
        std::string hex = bytes_to_hex_(ezsp, 32);
        ESP_LOGI(TAG, "Legacy VERSION(min) reply ezsp=%s", hex.c_str());
        last_ezsp_version_ = "v4 VERSION(min)";
        publish_text_(ezsp_version_sensor_, last_ezsp_version_);
        return true;
      }
      case efr32::ash::FrameType::ACK: {
        uint8_t ack = ctrl_ack(frame.control);
        ESP_LOGV(TAG, "wait_v4 ACK ctrl=0x%02X ack=%u", static_cast<unsigned>(frame.control),
                 static_cast<unsigned>(ack));
        break;
      }
      case efr32::ash::FrameType::NAK: {
        uint8_t ack = ctrl_ack(frame.control);
        ESP_LOGW(TAG, "wait_v4 NAK ctrl=0x%02X ack=%u -- retrying", static_cast<unsigned>(frame.control),
                 static_cast<unsigned>(ack));
        retransmit_last_tx_(ack);
        deadline = millis() + 1000;
        break;
      }
      case efr32::ash::FrameType::ERROR: {
        uint16_t code = 0;
        if (frame.payload.size() >= 2)
          code = static_cast<uint16_t>(frame.payload[0] << 8) | frame.payload[1];
        ESP_LOGE(TAG, "ASH error during v4 wait 0x%04X", static_cast<unsigned>(code));
        update_status_text_("ash-error");
        return false;
      }
      case efr32::ash::FrameType::RESET: {
        ESP_LOGW(TAG, "RESET during v4 wait; applying payload");
        apply_rstack_from_payload_(frame.payload, true);
        break;
      }
      default:
        break;
    }
  }

  ESP_LOGW(TAG, "Timeout waiting for v4 VERSION(min) host_rx_seq_=%u", static_cast<unsigned>(host_rx_seq_));
  return false;
}

bool EFR32InfoComponent::wait_for_rstack_(uint32_t timeout_ms, uint8_t &ash_version, uint8_t &reset_code) {
  uint32_t deadline = millis() + timeout_ms;
  while (millis() < deadline) {
    efr32::ash::Frame frame;
    if (!recv_any_frame_(frame, 80)) {
      if (!efr32::ash::recv_frame_loose(ash_state_, frame, 120)) {
        delay_(5);
        continue;
      }
      ESP_LOGV(TAG, "Loose frame ctrl=0x%02X type=%d while waiting for RSTACK",
               static_cast<unsigned>(frame.control), static_cast<int>(frame.type));
    }

    if (frame.type == efr32::ash::FrameType::RESET) {
      apply_rstack_from_payload_(frame.payload);
      update_status_text_("rstack");
      ESP_LOGI(TAG, "RSTACK received");
      if (raw_capture_) {
        ESP_LOGV(TAG, "RAW TX ACK ack=0 (RSTACK)");
      }
      efr32::ash::send_ack(ash_state_, false, 0);
      return true;
    }

    if (frame.type == efr32::ash::FrameType::DATA) {
      ESP_LOGW(TAG, "Data frame before RSTACK; assuming link already up");
      if (raw_capture_) {
        ESP_LOGV(TAG, "RAW RX ctrl=0x%02X len=%u data=%s", static_cast<unsigned>(frame.control),
                 static_cast<unsigned>(frame.payload.size()), bytes_to_hex_(frame.payload, 80).c_str());
      }
      if (ack_data_frame_(frame.control, frame.payload))
        return true;
      continue;
    }
    ESP_LOGV(TAG, "Ignored frame ctrl=0x%02X type=%d while waiting for RSTACK",
             static_cast<unsigned>(frame.control), static_cast<int>(frame.type));
  }
  if (try_loose_rstack_(uart_, ash_version, reset_code, 200)) {
    std::vector<uint8_t> payload;
    payload.push_back(ash_version);
    payload.push_back(reset_code);
    apply_rstack_from_payload_(payload, true);
    return true;
  }
  return false;
}

bool EFR32InfoComponent::wait_for_ezsp_(int expected_seq, uint16_t expected_frame_id, std::vector<uint8_t> &ezsp_out, uint32_t timeout_ms) {
  const bool want_specific = expected_seq >= 0;
  uint32_t deadline = millis() + timeout_ms;
  while (millis() < deadline) {
    efr32::ash::Frame frame;
    if (!recv_any_frame_(frame, 200)) {
      delay_(5);
      continue;
    }

    switch (frame.type) {
      case efr32::ash::FrameType::DATA: {
        if (raw_capture_) {
          ESP_LOGV(TAG, "RAW RX ctrl=0x%02X len=%u data=%s", static_cast<unsigned>(frame.control),
                   static_cast<unsigned>(frame.payload.size()), bytes_to_hex_(frame.payload, 80).c_str());
        }
        ESP_LOGV(TAG, "wait_ezsp DATA ctrl=0x%02X len=%zu", static_cast<unsigned>(frame.control),
                 frame.payload.size());
        if (!ack_data_frame_(frame.control, frame.payload))
          continue;
        const std::vector<uint8_t> &ezsp = frame.payload;
        if (ezsp.size() < 5) {
          continue;
        }
       uint8_t seq = ezsp[0];
       uint16_t frame_id = static_cast<uint16_t>(ezsp[3]) | (static_cast<uint16_t>(ezsp[4]) << 8);
        ESP_LOGV(TAG, "wait_ezsp got seq=%u frame_id=0x%04X len=%zu", static_cast<unsigned>(seq),
                 static_cast<unsigned>(frame_id), ezsp.size());

        if (want_specific && seq == expected_seq && frame_id == expected_frame_id) {
          ezsp_out = ezsp;
          return true;
        }

        if (want_specific) {
          ESP_LOGV(TAG, "wait_ezsp ignoring seq=%u frame_id=0x%04X want seq=%d frame_id=0x%04X",
                   static_cast<unsigned>(seq), static_cast<unsigned>(frame_id), expected_seq,
                   static_cast<unsigned>(expected_frame_id));
        }

        if (frame_id == 0x0026 && ezsp.size() >= 13) {
          std::vector<uint8_t> eui(ezsp.begin() + 5, ezsp.begin() + 13);
          char buf[24];
          std::snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                        eui[7], eui[6], eui[5], eui[4], eui[3], eui[2], eui[1], eui[0]);
          last_ieee_ = buf;
          publish_text_(ieee_sensor_, last_ieee_);
        }

        break;
      }
      case efr32::ash::FrameType::ACK: {
        uint8_t ack = static_cast<uint8_t>(frame.control & 0x07);
        ESP_LOGV(TAG, "wait ACK ctrl=0x%02X ack=%u", static_cast<unsigned>(frame.control),
                 static_cast<unsigned>(ack));
        break;
      }
      case efr32::ash::FrameType::NAK: {
        uint8_t ack = static_cast<uint8_t>(frame.control & 0x07);
        ESP_LOGW(TAG, "ASH NAK ctrl=0x%02X ack=%u", static_cast<unsigned>(frame.control),
                 static_cast<unsigned>(ack));
        retransmit_last_tx_(ack);
        break;
      }
      case efr32::ash::FrameType::ERROR: {
        uint16_t code = 0;
        if (frame.payload.size() >= 2)
          code = static_cast<uint16_t>(frame.payload[0] << 8) | frame.payload[1];
        ESP_LOGE(TAG, "ASH error frame 0x%04X", static_cast<unsigned>(code));
        update_status_text_("ash-error");
        return false;
      }
      case efr32::ash::FrameType::RESET: {
        uint8_t ash_ver = frame.payload.empty() ? 0 : frame.payload[0];
        uint8_t reset_code = frame.payload.size() >= 2 ? frame.payload[1] : 0xFF;
        char buf[48];
        std::snprintf(buf, sizeof(buf), "ASHv%u reset=0x%02X", static_cast<unsigned>(ash_ver), static_cast<unsigned>(reset_code));
        last_ezsp_version_ = buf;
        publish_text_(ezsp_version_sensor_, last_ezsp_version_);
        efr32::ash::send_ack(ash_state_, false, 0);
        break;
      }
      default:
        break;
    }
  }
  return false;
}

void EFR32InfoComponent::pump_frames_(uint32_t window_ms) {
  std::vector<uint8_t> scratch;
  wait_for_ezsp_(-1, 0, scratch, window_ms);
}

std::string EFR32InfoComponent::bytes_to_hex_(const std::vector<uint8_t> &data, size_t limit) {
  std::string out;
  size_t count = std::min(limit, data.size());
  out.reserve(count * 2 + 3);
  char tmp[3];
  for (size_t i = 0; i < count; i++) {
    std::snprintf(tmp, sizeof(tmp), "%02X", static_cast<unsigned>(data[i]));
    out.append(tmp);
  }
  if (data.size() > count)
    out.append("…");
  return out;
}

std::string EFR32InfoComponent::extract_ascii_(const std::vector<uint8_t> &data) {
  std::string trimmed = trim_token_ascii(data);
  if (!trimmed.empty())
    return trimmed;
  return find_printable_run(data);
}

void EFR32InfoComponent::run_probe_() {
  if (uart_ == nullptr) {
    ESP_LOGE(TAG, "UART not configured for efr32_info");
    update_status_text_("UART not configured");
    return;
  }

  struct GuardScope {
    EFR32InfoComponent *self;
    ~GuardScope() {
      if (self != nullptr)
        self->guard_active_ = false;
    }
  } guard_scope{this};
  guard_active_ = true;

  ash_state_.attach(uart_);
  link_bootstrapped_ = false;
  host_rx_seq_ = 0;
  have_last_delivered_ = false;
  last_delivered_seq_ = 0;
  bootstrap_seq_known_ = false;
  saw_rstack_ = false;
  ESP_LOGI(TAG, "Starting EZSP info probe");
  set_busy_(true);
  bool paused_stream = false;
  if (stream_server_ != nullptr) {
    stream_server_->pause();
    ESP_LOGI(TAG, "Stream server paused for probe");
    paused_stream = true;
    delay_(50);
  }

  update_status_text_("probing");

  uint32_t settle_deadline = millis() + 400;
  uint32_t quiet_since = millis();
  while (millis() < settle_deadline) {
    if (uart_->available()) {
      quiet_since = millis();
      flush_uart_();
    }
    if (millis() - quiet_since >= 200)
      break;
    delay_(5);
  }

  flush_uart_();

  ESP_LOGI(TAG, "ASH bootstrap attempt 1");
  send_rst_frame_();

  uint8_t ash_ver = 0;
  uint8_t reset_code = 0;
  if (!wait_for_rstack_(2000, ash_ver, reset_code)) {
    ESP_LOGE(TAG, "ASH link failed to synchronize (no RSTACK); aborting probe");
    update_status_text_("ash-sync-fail");
    if (paused_stream && stream_server_ != nullptr)
      stream_server_->resume();
    set_busy_(false);
    return;
  }

  ack_re_sync_();
  ash_state_.tx_seq = 0;
  send_v4_version_probe_();

  if (!wait_for_v4_version_(2500)) {
    ESP_LOGW(TAG, "v4 VERSION(min) not received; aborting probe");
    update_status_text_("v4-fail");
    if (paused_stream && stream_server_ != nullptr)
      stream_server_->resume();
    set_busy_(false);
    return;
  }

  auto build_ezsp = [](uint8_t seq, uint16_t frame_id, std::initializer_list<uint8_t> payload) {
    std::vector<uint8_t> frame;
    frame.reserve(5 + payload.size());
    frame.push_back(seq);
    frame.push_back(0x00);
    frame.push_back(0x01);
    frame.push_back(static_cast<uint8_t>(frame_id & 0xFF));
    frame.push_back(static_cast<uint8_t>((frame_id >> 8) & 0xFF));
    frame.insert(frame.end(), payload.begin(), payload.end());
    return frame;
  };

  auto send_and_expect = [&](uint8_t seq, uint16_t frame_id, std::initializer_list<uint8_t> payload, uint32_t timeout_ms, std::vector<uint8_t> &resp_out) -> bool {
    drain_frames_(120, -1);
    ack_re_sync_();
    drain_frames_(40, -1);
    auto payload_vec = build_ezsp(seq, frame_id, payload);
    send_data_frame_(payload_vec);
    return wait_for_ezsp_(seq, frame_id, resp_out, timeout_ms);
  };

  std::string status_summary;
  bool ok_version = false;
  bool tokens_ok = false;
  uint8_t ezsp_seq = 0;

  auto send_expect_seq = [&](uint16_t frame_id, std::initializer_list<uint8_t> payload, uint32_t timeout_ms,
                             std::vector<uint8_t> &resp_out) -> bool {
    uint8_t seq = ezsp_seq;
    ezsp_seq = static_cast<uint8_t>(ezsp_seq + 1);
    bool ok = send_and_expect(seq, frame_id, payload, timeout_ms, resp_out);
    if (!ok) {
      ESP_LOGW(TAG, "EZSP command 0x%04X seq=%u timed out", static_cast<unsigned>(frame_id),
               static_cast<unsigned>(seq));
    }
    return ok;
  };

  {
    std::vector<uint8_t> resp;
    if (send_expect_seq(0x0000, {0x0D}, 2500, resp)) {
      if (parse_v8_version_reply_(resp))
        ok_version = true;
    }
  }

  {
    std::vector<uint8_t> resp;
    if (send_expect_seq(0x0001, {}, 900, resp)) {
      std::string text;
      if (resp.size() >= 6) {
        uint8_t status = resp[5];
        char buf[32];
        std::snprintf(buf, sizeof(buf), "status=0x%02X", static_cast<unsigned>(status));
        text = buf;
        ESP_LOGI(TAG, "Library status=0x%02X", static_cast<unsigned>(status));
      } else {
        text = "raw=" + bytes_to_hex_(resp, 24);
        ESP_LOGW(TAG, "Library status short response len=%zu", resp.size());
      }
      last_library_status_ = text;
      publish_text_(library_status_sensor_, last_library_status_);
    }
  }

  {
    std::vector<uint8_t> resp;
    if (send_expect_seq(0x0013, {}, 900, resp) && resp.size() >= 7 && resp[5] == 0x00) {
      uint8_t state = resp[6];
      last_network_status_ = network_status_to_string(state);
      publish_text_(network_status_sensor_, last_network_status_);
      ESP_LOGI(TAG, "Network status=0x%02X (%s)", static_cast<unsigned>(state),
               last_network_status_.c_str());
    }
  }

  {
    std::vector<uint8_t> resp;
    if (send_expect_seq(0x0028, {}, 900, resp)) {
      if (resp.size() >= 26 && resp[5] == 0x00) {
        size_t pos = 7;  // start of EmberNetworkParameters
        std::vector<uint8_t> ext_pan(resp.begin() + pos, resp.begin() + pos + 8);
        pos += 8;
        uint16_t pan_id = static_cast<uint16_t>(resp[pos]) | (static_cast<uint16_t>(resp[pos + 1]) << 8);
        pos += 2;
        int8_t tx_power = static_cast<int8_t>(resp[pos++]);
        uint8_t radio_channel = resp[pos++];
        uint8_t join_method = resp[pos++];
        uint16_t manager_id = static_cast<uint16_t>(resp[pos]) | (static_cast<uint16_t>(resp[pos + 1]) << 8);
        pos += 2;
        uint8_t update_id = resp[pos++];
        uint32_t channel_mask = static_cast<uint32_t>(resp[pos]) |
                                (static_cast<uint32_t>(resp[pos + 1]) << 8) |
                                (static_cast<uint32_t>(resp[pos + 2]) << 16) |
                                (static_cast<uint32_t>(resp[pos + 3]) << 24);

        char chan_buf[16];
        std::snprintf(chan_buf, sizeof(chan_buf), "%u", static_cast<unsigned>(radio_channel));
        last_channel_ = chan_buf;
        publish_text_(channel_sensor_, last_channel_);
        ESP_LOGI(TAG, "Channel %u (txpwr=%d dBm update_id=%u mask=0x%08X)",
                 static_cast<unsigned>(radio_channel), static_cast<int>(tx_power),
                 static_cast<unsigned>(update_id), static_cast<unsigned>(channel_mask));

        char pan_buf[32];
        std::snprintf(pan_buf, sizeof(pan_buf), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X/0x%04X",
                      ext_pan[7], ext_pan[6], ext_pan[5], ext_pan[4], ext_pan[3], ext_pan[2], ext_pan[1], ext_pan[0],
                      static_cast<unsigned>(pan_id));
        ESP_LOGI(TAG, "PAN %s manager=0x%04X join_method=0x%02X", pan_buf,
                 static_cast<unsigned>(manager_id), static_cast<unsigned>(join_method));

        last_network_status_ = "joined";
        publish_text_(network_status_sensor_, last_network_status_);
      } else {
        ESP_LOGW(TAG, "Network parameters unexpected resp=%s", bytes_to_hex_(resp, 48).c_str());
      }
    }
  }

  {
    std::vector<uint8_t> resp;
    send_expect_seq(0x0147, {0x00, 0x03, 0x00, 0x00, 0x00}, 900, resp);
  }

  auto handle_token = [&](uint8_t token_id, std::string &target, esphome::text_sensor::TextSensor *sensor) -> bool {
    std::vector<uint8_t> resp;
    if (!send_expect_seq(0x000B, {token_id}, 1000, resp)) {
      ESP_LOGW(TAG, "Token 0x%02X request timed out", static_cast<unsigned>(token_id));
      return false;
    }
    if (!parse_token_reply_(resp, target)) {
      ESP_LOGW(TAG, "Token 0x%02X parse failed resp=%s", static_cast<unsigned>(token_id),
               bytes_to_hex_(resp, 48).c_str());
      return false;
    }
    publish_text_(sensor, target);
    ESP_LOGI(TAG, "Token 0x%02X value '%s'", static_cast<unsigned>(token_id), target.c_str());
    return true;
  };

  if (handle_token(0x01, last_manufacturer_, manufacturer_sensor_))
    tokens_ok = true;
  if (handle_token(0x02, last_board_name_, board_name_sensor_))
    tokens_ok = true;

  {
    std::vector<uint8_t> resp;
    if (send_expect_seq(0x0026, {}, 800, resp)) {
      size_t start = 0;
      if (resp.size() >= 14 && resp[5] == 0x00) {
        start = 6;
      } else if (resp.size() >= 13) {
        start = resp.size() - 8;
      } else {
        ESP_LOGW(TAG, "IEEE reply too short len=%zu", resp.size());
        start = 0;
      }
      if (start + 8 <= resp.size()) {
        std::vector<uint8_t> eui(resp.begin() + start, resp.begin() + start + 8);
      char buf[24];
      std::snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                    eui[7], eui[6], eui[5], eui[4], eui[3], eui[2], eui[1], eui[0]);
      last_ieee_ = buf;
      publish_text_(ieee_sensor_, last_ieee_);
      ESP_LOGI(TAG, "IEEE address %s", last_ieee_.c_str());
      } else {
        ESP_LOGW(TAG, "IEEE parse failed len=%zu start=%zu", resp.size(), start);
      }
    }
  }

  {
    std::vector<uint8_t> resp;
    if (send_expect_seq(0x0002, {0x40}, 800, resp) && resp.size() >= 11 && resp[5] == 0x00) {
      uint8_t major = resp[6];
      uint8_t minor = resp[7];
      uint8_t patch = resp[8];
      uint8_t special = resp[9];
      uint32_t build = static_cast<uint32_t>(resp[10]);
      if (resp.size() >= 12)
        build |= static_cast<uint32_t>(resp[11]) << 8;
      char buf[64];
    std::snprintf(buf, sizeof(buf), "%u.%u.%u.%u build %u",
                  static_cast<unsigned>(major), static_cast<unsigned>(minor),
                  static_cast<unsigned>(patch), static_cast<unsigned>(special),
                  static_cast<unsigned>(build));
    last_stack_version_ = buf;
    publish_text_(stack_version_sensor_, last_stack_version_);
    char fw_only[32];
    std::snprintf(fw_only, sizeof(fw_only), "%u.%u.%u.%u",
                  static_cast<unsigned>(major), static_cast<unsigned>(minor),
                  static_cast<unsigned>(patch), static_cast<unsigned>(special));
    last_firmware_ = fw_only;
    publish_text_(firmware_sensor_, last_firmware_);
      ok_version = true;
    }
  }

  {
    std::vector<uint8_t> resp;
    if (send_expect_seq(0x0027, {}, 800, resp)) {
      uint16_t node = 0xFFFF;
      bool parsed = false;
      if (resp.size() >= 8 && resp[5] == 0x00) {
        node = static_cast<uint16_t>(resp[6]) | (static_cast<uint16_t>(resp[7]) << 8);
        parsed = true;
      } else if (resp.size() >= 7) {
        node = static_cast<uint16_t>(resp[resp.size() - 2]) |
               (static_cast<uint16_t>(resp[resp.size() - 1]) << 8);
        parsed = true;
      }
      if (parsed) {
        char buf[12];
        std::snprintf(buf, sizeof(buf), "0x%04X", static_cast<unsigned>(node));
        last_node_id_ = buf;
        publish_text_(node_id_sensor_, last_node_id_);
        ESP_LOGI(TAG, "Node ID %s", last_node_id_.c_str());
      } else {
        ESP_LOGW(TAG, "Node ID parse failed len=%zu", resp.size());
      }
    }
  }

  {
    std::vector<uint8_t> resp;
    if (send_expect_seq(0x0018, {}, 800, resp) && resp.size() >= 7 && resp[5] == 0x00) {
      uint8_t state = resp[6];
      last_network_status_ = network_status_to_string(state);
      publish_text_(network_status_sensor_, last_network_status_);
    }
  }

  {
    std::vector<uint8_t> resp;
    if (send_expect_seq(0x0069, {}, 900, resp)) {
      if (resp.size() >= 8 && resp[5] == 0x00) {
        uint16_t bitmask = static_cast<uint16_t>(resp[6]) | (static_cast<uint16_t>(resp[7]) << 8);
        char buf[48];
        std::snprintf(buf, sizeof(buf), "mask=0x%04X", static_cast<unsigned>(bitmask));
        last_security_state_ = buf;
        publish_text_(security_state_sensor_, last_security_state_);
        ESP_LOGI(TAG, "Security bitmask=0x%04X", static_cast<unsigned>(bitmask));
      } else {
        std::string hex = bytes_to_hex_(resp, 32);
        last_security_state_ = "raw=" + hex;
        publish_text_(security_state_sensor_, last_security_state_);
        ESP_LOGW(TAG, "Security state unexpected resp=%s", hex.c_str());
      }
    }
  }

  if (tokens_ok) {
    status_summary = "ok";
  } else if (ok_version) {
    status_summary = "version-only";
  } else if (saw_rstack_) {
    status_summary = "partial";
  } else {
    status_summary = "no-response";
  }
  update_status_text_(status_summary);

  if (paused_stream && stream_server_ != nullptr) {
    stream_server_->resume();
    ESP_LOGI(TAG, "Stream server resumed after probe");
  }
  set_busy_(false);
}

}  // namespace efr32_info
}  // namespace esphome

#pragma once

#include <string>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/efr32_info/ash_util.h"
#include "esphome/components/stream_server/stream_server.h"

namespace esphome {
namespace efr32_info {

class EFR32InfoComponent : public Component {
 public:
  void set_uart(esphome::uart::UARTComponent *uart) {
    uart_ = uart;
    ash_state_.attach(uart);
    ash_state_.next_ack = 0;
    ash_state_.last_rx_seq = 7;
    host_rx_seq_ = 0;
  }
  void set_stream_server(::StreamServerComponent *srv) { stream_server_ = srv; }
  void set_busy_sensor(esphome::binary_sensor::BinarySensor *sensor) { busy_sensor_ = sensor; }
  void set_status_sensor(esphome::text_sensor::TextSensor *sensor) { status_sensor_ = sensor; }
  void set_ezsp_version_sensor(esphome::text_sensor::TextSensor *sensor) { ezsp_version_sensor_ = sensor; }
  void set_library_status_sensor(esphome::text_sensor::TextSensor *sensor) { library_status_sensor_ = sensor; }
  void set_stack_version_sensor(esphome::text_sensor::TextSensor *sensor) { stack_version_sensor_ = sensor; }
  void set_manufacturer_sensor(esphome::text_sensor::TextSensor *sensor) { manufacturer_sensor_ = sensor; }
  void set_board_name_sensor(esphome::text_sensor::TextSensor *sensor) { board_name_sensor_ = sensor; }
  void set_firmware_sensor(esphome::text_sensor::TextSensor *sensor) { firmware_sensor_ = sensor; }
  void set_ieee_sensor(esphome::text_sensor::TextSensor *sensor) { ieee_sensor_ = sensor; }
  void set_channel_sensor(esphome::text_sensor::TextSensor *sensor) { channel_sensor_ = sensor; }
  void set_node_id_sensor(esphome::text_sensor::TextSensor *sensor) { node_id_sensor_ = sensor; }
  void set_network_status_sensor(esphome::text_sensor::TextSensor *sensor) { network_status_sensor_ = sensor; }
  void set_security_state_sensor(esphome::text_sensor::TextSensor *sensor) { security_state_sensor_ = sensor; }
  void set_raw_capture(bool raw) { raw_capture_ = raw; }

  void start_probe() { want_probe_ = true; }

  void setup() override {}
  void loop() override;

 private:
  void run_probe_();
  void set_busy_(bool on);
  void publish_text_(esphome::text_sensor::TextSensor *sensor, const std::string &value);
  void update_status_text_(const std::string &value);
  void delay_(uint32_t ms);
  void flush_uart_();
  bool wait_for_rstack_(uint32_t timeout_ms, uint8_t &ash_version, uint8_t &reset_code);
  void send_rst_frame_();
  void send_data_frame_(const std::vector<uint8_t> &ezsp_payload, bool retransmit = false);
  void send_v4_version_probe_(bool retransmit = false);
  void retransmit_last_tx_(uint8_t ack_num);
  bool wait_for_v4_version_(uint32_t timeout_ms);
  bool wait_for_ezsp_(int expected_seq, uint16_t expected_frame_id, std::vector<uint8_t> &ezsp_out, uint32_t timeout_ms);
  void pump_frames_(uint32_t window_ms);
  bool recv_any_frame_(efr32::ash::Frame &frame, uint32_t timeout_ms);
  bool ack_data_frame_(uint8_t ctrl, const std::vector<uint8_t> &payload);
  void drain_frames_(uint32_t idle_timeout_ms, int target_ack = -1);
  bool prime_link_(uint32_t timeout_ms);
  bool wait_for_initial_data_(uint32_t timeout_ms);
  void apply_rstack_from_payload_(const std::vector<uint8_t> &payload, bool loose = false);
  void seed_initial_window_(uint8_t delivered_seq);
  void parse_unsolicited_ezsp_(const std::vector<uint8_t> &ezsp);
  void ack_re_sync_();
  bool parse_token_reply_(const std::vector<uint8_t> &resp, std::string &out);
  bool parse_v8_version_reply_(const std::vector<uint8_t> &resp);

  static std::string bytes_to_hex_(const std::vector<uint8_t> &data, size_t limit = 32);
  static std::string extract_ascii_(const std::vector<uint8_t> &data);

  esphome::uart::UARTComponent *uart_{nullptr};
  ::StreamServerComponent *stream_server_{nullptr};
  esphome::binary_sensor::BinarySensor *busy_sensor_{nullptr};

  esphome::text_sensor::TextSensor *status_sensor_{nullptr};
  esphome::text_sensor::TextSensor *ezsp_version_sensor_{nullptr};
  esphome::text_sensor::TextSensor *library_status_sensor_{nullptr};
  esphome::text_sensor::TextSensor *stack_version_sensor_{nullptr};
  esphome::text_sensor::TextSensor *manufacturer_sensor_{nullptr};
  esphome::text_sensor::TextSensor *board_name_sensor_{nullptr};
  esphome::text_sensor::TextSensor *firmware_sensor_{nullptr};
  esphome::text_sensor::TextSensor *ieee_sensor_{nullptr};
  esphome::text_sensor::TextSensor *channel_sensor_{nullptr};
  esphome::text_sensor::TextSensor *node_id_sensor_{nullptr};
  esphome::text_sensor::TextSensor *network_status_sensor_{nullptr};
  esphome::text_sensor::TextSensor *security_state_sensor_{nullptr};

  std::string last_status_;
  std::string last_ezsp_version_;
  std::string last_stack_version_;
  std::string last_library_status_;
  std::string last_manufacturer_;
  std::string last_board_name_;
  std::string last_firmware_;
  std::string last_ieee_;
  std::string last_channel_;
  std::string last_node_id_;
  std::string last_network_status_;
  std::string last_security_state_;

  bool want_probe_{false};
  bool busy_state_{false};
  efr32::ash::LinkState ash_state_{};
  bool raw_capture_{false};
  bool link_bootstrapped_{false};
  uint8_t host_rx_seq_{0};
  uint8_t last_delivered_seq_{0};
  bool have_last_delivered_{false};
  bool bootstrap_seq_known_{false};
  bool saw_rstack_{false};
  bool guard_active_{false};
  std::vector<uint8_t> last_tx_payload_{};
  uint8_t last_tx_seq_{0};
  bool have_last_tx_{false};
  uint8_t ezsp_protocol_version_{0};
};

class EFR32InfoProbeAction : public esphome::Action<> {
 public:
  void set_parent(EFR32InfoComponent *parent) { parent_ = parent; }
  void play() override {
    if (parent_ != nullptr)
      parent_->start_probe();
  }

 private:
  EFR32InfoComponent *parent_{nullptr};
};

}  // namespace efr32_info
}  // namespace esphome

#include "tubeszb_web.h"

#ifdef USE_TUBESZB_CC2652
#include "cc2652_log.h"
#include "esphome/components/logger/logger.h"
#include <algorithm>
#include <cstring>

namespace esphome::tubeszb_web {

const std::string &TubesZBWeb::radio_source_key_() const { return this->radio_source_key_cache_; }

void TubesZBWeb::refresh_radio_source_key_() {
  // Inputs change only on sensor/channel/URL callbacks, never during idle sampling.
  this->radio_source_key_cache_ = std::string(this->radio_.zigbee_beta ? "beta|" : "stable|") + this->radio_.board;
}

void TubesZBWeb::setup_radio_() {
  this->radio_flasher_->set_flash_listener([this](const zigbee_firmware_baud::FlashStatus &status) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->cc_started_ = false;
    this->radio_.operation = "flash";
    this->radio_.result = status.result;
    this->radio_.progress = status.progress;
    this->radio_flash_active_ = status.result == 1;
    this->radio_.busy = status.result == 1;
    this->radio_command_at_ = millis();
    this->radio_.status = status.result == 1   ? "Zigbee flash in progress"
                          : status.result == 2 ? "Zigbee firmware flash complete"
                                               : "Zigbee firmware flash failed; open device logs";
  });
  auto bind = [this](text_sensor::TextSensor *sensor, std::string RadioState::*field) {
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->radio_.*field = sensor->state;
    }
    sensor->add_on_state_callback([this, field](const std::string &value) {
      std::lock_guard<std::mutex> lock(this->mutex_);
      const bool board_changed = field == &RadioState::board && this->radio_.board != value;
      if (field == &RadioState::latest && this->radio_.operation == "check" && this->cc_started_)
        this->cc_fresh_latest_ = !value.empty();
      this->radio_.*field = value;
      if (board_changed) {
        this->radio_.checked = false;
        this->radio_checked_key_.clear();
        this->refresh_radio_source_key_();
      }
    });
  };
  bind(this->firmware_sensor_, &RadioState::firmware);
  bind(this->latest_sensor_, &RadioState::latest);
  bind(this->board_sensor_, &RadioState::board);
  bind(this->ieee_sensor_, &RadioState::ieee);
  this->radio_.manufacturer = "Texas Instruments";
  this->radio_.status = "Ready";
  this->zigbee_beta_->add_on_state_callback([this](bool value) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (this->radio_.zigbee_beta != value) {
      this->radio_.zigbee_beta = value;
      this->radio_.checked = false;
      this->radio_checked_key_.clear();
      this->refresh_radio_source_key_();
    }
  });
  this->gateway_beta_->add_on_state_callback([this](bool value) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->radio_.gateway_beta = value;
    this->verified_source_.clear();
    this->snapshot_.update_verified = false;
  });
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->radio_.gateway_beta = this->gateway_beta_->state;
    this->radio_.zigbee_beta = this->zigbee_beta_ != nullptr && this->zigbee_beta_->state;
    this->radio_baud_rate_ = this->uart_->get_baud_rate();
    this->radio_.selected_baud = std::to_string(this->radio_baud_rate_);
    this->refresh_radio_source_key_();
  }
  if (logger::global_logger != nullptr) {
    logger::global_logger->add_log_callback(
        this, [](void *self, uint8_t level, const char *tag, const char *message, size_t length) {
          static_cast<TubesZBWeb *>(self)->radio_log_(level, tag, message, length);
        });
  }
}

void TubesZBWeb::radio_log_(uint8_t level, const char *tag, const char *message, size_t length) {
  if (strcmp(tag, "cc2652_flasher") != 0)
    return;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (this->radio_flash_active_)
      return;  // FlashStatus already owns these events; avoid even copying the log.
  }
  const std::string line(message, std::min(length, size_t(512)));
  const auto event = decode_cc2652_log(line);
  using E = CC2652LogEvent::Kind;
  std::lock_guard<std::mutex> lock(this->mutex_);
  if (this->radio_flasher_->get_flash_status().result == 1 || event.kind == E::KIND_FLASH_START ||
      event.kind == E::KIND_FLASH_OK || event.kind == E::KIND_PROGRESS)
    return;  // Native listener owns flash results.
  if (event.kind == E::KIND_CHECK_START || event.kind == E::KIND_FLASH_START || event.kind == E::KIND_PROBE_START ||
      event.kind == E::KIND_BOOTLOADER_START) {
    this->radio_.operation = event.kind == E::KIND_CHECK_START   ? "check"
                             : event.kind == E::KIND_FLASH_START ? "flash"
                             : event.kind == E::KIND_PROBE_START ? "probe"
                                                                 : "bootloader";
    this->radio_.result = 1;
    this->radio_.busy = true;
    this->radio_.progress = -1;
    this->radio_command_at_ = millis();
    this->radio_.status = "CC2652 operation in progress";
    this->cc_started_ = event.kind != E::KIND_BOOTLOADER_START;
    this->cc_failed_ = this->cc_fresh_latest_ = this->cc_flash_ok_ = this->cc_probe_ok_ = false;
    if (this->radio_flasher_->get_flash_status().result == 1 || event.kind == E::KIND_FLASH_START ||
        event.kind == E::KIND_FLASH_OK || event.kind == E::KIND_PROGRESS)
      return;  // Native listener owns flash results.
    if (event.kind == E::KIND_CHECK_START) {
      this->radio_.checked = false;
      this->radio_checked_key_.clear();
      this->radio_requested_key_ = this->radio_source_key_();
    }
  }
  if (event.kind == E::KIND_PROGRESS)
    this->radio_.progress = event.progress;
  if (event.kind == E::KIND_FLASH_OK)
    this->cc_flash_ok_ = true;
  if (event.kind == E::KIND_PROBE_OK)
    this->cc_probe_ok_ = true;
  if (event.kind == E::KIND_PROBE_FAILED || (level <= ESPHOME_LOG_LEVEL_ERROR && this->radio_.busy))
    this->cc_failed_ = true;
  if (event.kind == E::KIND_BOOTLOADER_READY) {
    this->radio_.busy = false;
    this->radio_.result = 2;
    this->radio_.status = "Bootloader ready; reset the radio to resume normal operation";
  }
}

void TubesZBWeb::queue_cc2652_automatic_(Command command) {
  if (this->settings_->is_trial_active())
    return;
  std::lock_guard<std::mutex> lock(this->mutex_);
  if (this->command_ != Command::COMMAND_NONE || this->action_pending_ || this->gateway_ota_state_ == 1 ||
      this->checking_ || this->radio_.busy || this->radio_.result == 1)
    return;
  this->command_ = command;
  this->action_pending_ = true;
}
void TubesZBWeb::request_cc2652_check() {
  if (!this->settings_->internet_allowed())
    return;
  this->queue_cc2652_automatic_(Command::COMMAND_RADIO_CHECK);
}
void TubesZBWeb::request_cc2652_probe() { this->queue_cc2652_automatic_(Command::COMMAND_RADIO_PROBE); }

void TubesZBWeb::run_radio_command_(Command command, const std::string &value) {
  if ((command == Command::COMMAND_RADIO_CHECK || command == Command::COMMAND_RADIO_FLASH) &&
      !this->settings_->internet_allowed())
    return;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->radio_.result = 1;
    this->radio_.busy = true;
    this->radio_.progress = -1;
    this->radio_command_at_ = millis();
    this->cc_started_ = this->cc_failed_ = this->cc_fresh_latest_ = this->cc_flash_ok_ = this->cc_probe_ok_ = false;
    this->radio_.operation = command == Command::COMMAND_RADIO_FLASH        ? "flash"
                             : command == Command::COMMAND_RADIO_CHECK      ? "check"
                             : command == Command::COMMAND_RADIO_PROBE      ? "probe"
                             : command == Command::COMMAND_RADIO_BOOTLOADER ? "bootloader"
                                                                            : "settings";
    this->radio_.status = "CC2652 action queued";
    if (command == Command::COMMAND_RADIO_CHECK) {
      this->radio_.checked = false;
      this->radio_checked_key_.clear();
    }
  }
  // Both frontends invoke the same native button and shared pause/flash/resume script.
  switch (command) {
    case Command::COMMAND_RADIO_CHECK:
      this->check_button_->press();
      break;
    case Command::COMMAND_RADIO_FLASH:
      this->flash_button_->press();
      break;
    case Command::COMMAND_RADIO_PROBE:
      this->probe_button_->press();
      break;
    case Command::COMMAND_RADIO_RESET:
      this->reset_button_->press();
      break;
    case Command::COMMAND_RADIO_BOOTLOADER:
      this->bootloader_button_->press();
      break;
    case Command::COMMAND_SERIAL_PAUSE:
      this->pause_button_->press();
      break;
    case Command::COMMAND_SERIAL_RESUME:
      this->resume_button_->press();
      break;
    case Command::COMMAND_GATEWAY_CHANNEL:
      this->gateway_beta_->control(value == "beta");
      break;
    case Command::COMMAND_ZIGBEE_CHANNEL:
      this->zigbee_beta_->control(value == "beta");
      break;
    default:
      break;
  }
}

void TubesZBWeb::sample_radio_() {
  const uint32_t now = App.get_loop_component_start_time();
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->radio_.connected = this->stream_->is_connected();
  this->radio_.paused = this->stream_->is_paused();
  if (this->radio_baud_rate_ != this->uart_->get_baud_rate()) {
    this->radio_baud_rate_ = this->uart_->get_baud_rate();
    this->radio_.selected_baud = std::to_string(this->radio_baud_rate_);
  }
  if (this->cc_started_) {
    // These native flasher methods run synchronously on the main loop. Reaching
    // this sample means they returned. Completion still requires their real result.
    this->cc_started_ = false;
    this->radio_.busy = false;
    bool ok = this->radio_.operation == "flash"   ? this->cc_flash_ok_
              : this->radio_.operation == "probe" ? this->cc_probe_ok_
                                                  : !this->cc_failed_ && this->cc_fresh_latest_;
    this->radio_.result = ok ? 2 : 3;
    this->radio_.status = ok ? "CC2652 operation complete" : "CC2652 operation failed; open device logs";
    if (ok && this->radio_.operation == "flash")
      this->radio_.progress = 100;
    if (this->radio_.operation == "check" && ok && this->radio_requested_key_ == this->radio_source_key_())
      this->radio_checked_key_ = this->radio_requested_key_;
  } else if (this->radio_.result == 1 && this->radio_.operation == "settings" &&
             int32_t(now - this->radio_command_at_) > 1000) {
    this->radio_.busy = false;
    this->radio_.result = 2;
    this->radio_.status = "CC2652 settings applied";
  } else if (this->radio_.result == 1 && int32_t(now - this->radio_command_at_) > 35000) {
    this->radio_.busy = false;
    this->radio_.result = 3;
    this->radio_.status = "CC2652 action did not complete; open device logs";
  }
  // Keep maintenance controls locked through the shared script's readback and
  // bridge restoration, including failures and legacy ESPHome button presses.
  this->radio_.busy =
      this->radio_.result == 1 || this->cc_flash_script_->is_running() || this->cc_boot_probe_script_->is_running();
  this->radio_.paused = this->stream_->is_paused();
  this->radio_.checked = !this->radio_checked_key_.empty() && this->radio_checked_key_ == this->radio_source_key_();
}

void TubesZBWeb::radio_json_(JsonObject root) {
  auto data = root["zigbee"].to<JsonObject>();
  data["busy"] = this->radio_.busy;
  data["connected"] = this->radio_.connected;
  data["paused"] = this->radio_.paused;
  data["firmware"] = this->radio_.firmware;
  data["latest"] = this->radio_.latest;
  data["board_name"] = this->radio_.board;
  data["manufacturer"] = this->radio_.manufacturer;
  data["ieee"] = this->radio_.ieee;
  data["info_status"] = this->radio_.status;
  data["custom_url"] = "";
  data["baud"] = this->radio_.selected_baud;
  data["baud_options"].to<JsonArray>();
  data["gateway_beta"] = this->radio_.gateway_beta;
  data["zigbee_beta"] = this->radio_.zigbee_beta;
  data["checked"] = this->radio_.checked;
  data["operation"] = this->radio_.operation;
  data["result"] = this->radio_.result;
  data["progress"] = this->radio_.progress;
  data["port"] = 6638;
}
}  // namespace esphome::tubeszb_web
#endif

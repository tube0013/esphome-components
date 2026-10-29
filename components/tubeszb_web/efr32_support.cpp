#include "tubeszb_web.h"

#ifdef USE_TUBESZB_EFR32
#include "esphome/components/logger/logger.h"
#include <algorithm>
#include <cstdlib>
#include <cstring>

namespace esphome::tubeszb_web {

const std::string &TubesZBWeb::radio_source_key_() const { return this->radio_source_key_cache_; }

void TubesZBWeb::refresh_radio_source_key_() {
  // Inputs change only on sensor/channel/URL callbacks, never during idle sampling.
  this->radio_source_key_cache_ =
      std::string(this->radio_.zigbee_beta ? "beta|" : "stable|") + this->radio_.url + "|" + this->radio_.board;
}

void TubesZBWeb::setup_radio_() {
  this->radio_flasher_->set_flash_listener([this](const zigbee_firmware_baud::FlashStatus &status) {
    std::lock_guard<std::mutex> lock(this->mutex_);
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
  this->busy_->add_on_state_callback([this](bool busy) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->radio_.busy = busy;
  });
  // Sensor callbacks keep the HTTP snapshot current even while the existing
  // EFR32 flasher/probe owns the main loop for a long operation.
  auto bind = [this](text_sensor::TextSensor *sensor, std::string RadioState::*field) {
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->radio_.*field = sensor->state;
    }
    sensor->add_on_state_callback([this, field](const std::string &value) {
      std::lock_guard<std::mutex> lock(this->mutex_);
      const bool board_changed = field == &RadioState::board && this->radio_.board != value;
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
  bind(this->manufacturer_sensor_, &RadioState::manufacturer);
  bind(this->ieee_sensor_, &RadioState::ieee);
  bind(this->status_sensor_, &RadioState::status);
  if (this->zigbee_beta_ != nullptr)
    this->zigbee_beta_->add_on_state_callback([this](bool value) {
      std::lock_guard<std::mutex> lock(this->mutex_);
      if (this->radio_.zigbee_beta != value) {
        this->radio_.zigbee_beta = value;
        this->radio_.checked = false;
        this->radio_checked_key_.clear();
        this->refresh_radio_source_key_();
      }
    });
  this->custom_url_->add_on_state_callback([this](const std::string &value) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (this->radio_.url != value) {
      this->radio_.url = value;
      this->radio_.checked = false;
      this->radio_checked_key_.clear();
      this->refresh_radio_source_key_();
    }
  });
  this->baud_->add_on_state_callback([this](const std::string &value, size_t) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->radio_.selected_baud = value;
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
    this->radio_.url = this->custom_url_->state;
    this->radio_.selected_baud = this->baud_->current_option();
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
  if (strcmp(tag, "efr32_flasher") != 0)
    return;
  const std::string line(message, std::min(length, size_t(512)));
  std::lock_guard<std::mutex> lock(this->mutex_);
  if (this->radio_.operation != "flash" && level <= ESPHOME_LOG_LEVEL_ERROR && this->radio_.result == 1)
    this->radio_.result = 3;
}

void TubesZBWeb::run_radio_command_(Command command, const std::string &value) {
  if ((command == Command::COMMAND_RADIO_CHECK || command == Command::COMMAND_RADIO_FLASH) &&
      !this->settings_->internet_allowed())
    return;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->radio_.result = 1;
    this->radio_.progress = -1;
    this->radio_command_at_ = millis();
    this->radio_.operation = command == Command::COMMAND_RADIO_FLASH        ? "flash"
                             : command == Command::COMMAND_RADIO_CHECK      ? "check"
                             : command == Command::COMMAND_RADIO_PROBE      ? "probe"
                             : command == Command::COMMAND_RADIO_BOOTLOADER ? "bootloader"
                                                                            : "settings";
    if (command == Command::COMMAND_RADIO_CHECK) {
      this->radio_.checked = false;
      this->radio_checked_key_.clear();
      this->radio_requested_key_ = this->radio_source_key_();
    }
  }
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
    case Command::COMMAND_SET_BAUD:
      this->baud_->make_call().set_option(value).perform();
      break;
    case Command::COMMAND_SET_URL:
      this->radio_checked_key_.clear();
      this->custom_url_->make_call().set_value(value).perform();
      break;
    case Command::COMMAND_GATEWAY_CHANNEL:
      this->verified_source_.clear();
      this->gateway_beta_->control(value == "beta");
      ESP_LOGI("tubeszb_web", "Gateway firmware channel: %s. Check for updates before installing.", value.c_str());
      break;
    case Command::COMMAND_ZIGBEE_CHANNEL:
      this->radio_checked_key_.clear();
      this->zigbee_beta_->control(value == "beta");
      ESP_LOGI("tubeszb_web", "Zigbee firmware channel: %s. Check Zigbee firmware before flashing.", value.c_str());
      break;
    default:
      break;
  }
}

void TubesZBWeb::sample_radio_() {
  const uint32_t now = App.get_loop_component_start_time();
  std::lock_guard<std::mutex> lock(this->mutex_);
  const bool manual_bootloader = this->bootloader_script_->is_running();
  this->radio_.busy = this->busy_->state || manual_bootloader;
  // Also reflect presses from the original ESPHome interface using this script.
  if (manual_bootloader && this->radio_.operation != "bootloader") {
    this->radio_.operation = "bootloader";
    this->radio_.result = 1;
    this->radio_.progress = -1;
    this->radio_command_at_ = now;
  }
  this->radio_.connected = this->stream_->is_connected();
  this->radio_.paused = this->stream_->is_paused();
  if (this->radio_.result == 1 && !this->radio_.busy && this->radio_.operation != "flash" &&
      int32_t(now - this->radio_command_at_) > 1000) {
    this->radio_.result = 2;
    if (this->radio_.operation == "check" && this->radio_requested_key_ == this->radio_source_key_())
      this->radio_checked_key_ = this->radio_requested_key_;
  }
  if (this->radio_.result == 1 && !this->radio_.busy && this->radio_.operation == "flash" &&
      int32_t(now - this->radio_command_at_) >= 35000) {
    this->radio_.result = 3;
    this->radio_.status = "Zigbee flash did not start; open device logs and try again";
  }
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
  data["custom_url"] = this->radio_.url;
  data["baud"] = this->radio_.selected_baud;
  auto options = data["baud_options"].to<JsonArray>();
  for (const char *option : this->baud_->traits.get_options())
    options.add(option);
  data["gateway_beta"] = this->radio_.gateway_beta;
  data["zigbee_beta"] = this->radio_.zigbee_beta;
  data["supports_zigbee_beta"] = this->zigbee_beta_ != nullptr;
  data["checked"] = this->radio_.checked;
  data["operation"] = this->radio_.operation;
  data["result"] = this->radio_.result;
  data["progress"] = this->radio_.progress;
  data["port"] = 6638;
}

}  // namespace esphome::tubeszb_web
#endif

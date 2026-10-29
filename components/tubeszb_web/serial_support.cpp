#include "tubeszb_web.h"
#ifdef USE_TUBESZB_SERIAL_RADIO
namespace esphome::tubeszb_web {
const std::string &TubesZBWeb::radio_source_key_() const { return this->radio_source_key_cache_; }
void TubesZBWeb::refresh_radio_source_key_() {}
void TubesZBWeb::setup_radio_() {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->radio_.board = this->radio_model_;
  this->radio_.manufacturer = "Silicon Labs";
  this->radio_.status = "Zigbee serial bridge ready";
  this->radio_baud_rate_ = this->uart_->get_baud_rate();
  this->radio_.selected_baud =
      this->baud_ != nullptr ? this->baud_->current_option() : std::to_string(this->radio_baud_rate_);
  if (this->baud_ != nullptr)
    this->baud_->add_on_state_callback([this](size_t) {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->radio_.selected_baud = this->baud_->current_option();
    });
}
void TubesZBWeb::run_radio_command_(Command command, const std::string &value) {
  switch (command) {
    case Command::COMMAND_RADIO_RESET:
      this->reset_button_->press();
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
    default:
      return;
  }
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->radio_.operation = "settings";
  this->radio_.result = 2;
}
void TubesZBWeb::sample_radio_() {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->radio_.connected = this->stream_->is_connected();
  if (this->radio_.paused != this->stream_->is_paused()) {
    this->radio_.paused = this->stream_->is_paused();
    this->radio_.status = this->radio_.paused ? "Zigbee serial bridge paused" : "Zigbee serial bridge ready";
  }
  if (this->baud_ == nullptr && this->radio_baud_rate_ != this->uart_->get_baud_rate()) {
    this->radio_baud_rate_ = this->uart_->get_baud_rate();
    this->radio_.selected_baud = std::to_string(this->radio_baud_rate_);
  }
}
void TubesZBWeb::radio_json_(JsonObject root) {
  auto data = root["zigbee"].to<JsonObject>();
  data["connected"] = this->radio_.connected;
  data["paused"] = this->radio_.paused;
  data["busy"] = false;
  data["result"] = this->radio_.result;
  data["operation"] = this->radio_.operation;
  data["progress"] = -1;
  data["board_name"] = this->radio_.board;
  data["manufacturer"] = this->radio_.manufacturer;
  data["info_status"] = this->radio_.status;
  data["baud"] = this->radio_.selected_baud;
  data["firmware"] = "";
  data["ieee"] = "";
  data["latest"] = "";
  data["custom_url"] = "";
  data["checked"] = false;
  data["gateway_beta"] = false;
  data["zigbee_beta"] = false;
  auto options = data["baud_options"].to<JsonArray>();
  if (this->baud_ != nullptr)
    for (const auto &option : this->baud_->traits.get_options())
      options.add(option);
  data["port"] = 6638;
}
}  // namespace esphome::tubeszb_web
#endif

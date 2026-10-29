#include "tubeszb_web.h"
#include "esphome/components/network_settings/request_helpers.h"
#include "gbl_url.h"

#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/version.h"
#include "esphome/components/api/api_server.h"
#include "esphome/components/json/json_util.h"
#ifdef USE_ZWAVE_PROXY
#include "esphome/components/zwave_proxy/zwave_proxy.h"
#endif
#include <esp_heap_caps.h>
#include <algorithm>
#include <cmath>

namespace esphome::tubeszb_web {

ESPHOME_LOG_TAG(TAG, "tubeszb_web");

void TubesZBWeb::setup() {
  this->nonce_ = network_settings::web::make_nonce();
  char mac[MAC_ADDRESS_PRETTY_BUFFER_SIZE];
  this->mac_ = this->ethernet_->get_eth_mac_address_pretty_into_buffer(mac);
  // The standard middleware applies any configured ESPHome HTTP authentication.
  this->base_->add_handler(this);
  ota::get_global_ota_callback()->add_global_state_listener(this);
  this->latest_version_ = this->update_->update_info.latest_version;
  this->update_->add_on_state_callback([this]() {
    if (this->checking_ && !this->update_->status_has_error() && this->check_source_ == this->update_->get_source_url())
      this->verified_source_ = this->check_source_;
    this->checking_ = false;
    this->check_timeout_ = false;
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->snapshot_.checking = false;
    this->latest_version_ = this->update_->update_info.latest_version;
#ifdef USE_TUBESZB_ZIGBEE
    this->snapshot_.update_verified =
        !this->verified_source_.empty() && this->verified_source_ == this->update_->get_source_url();
#endif
  });
#ifdef USE_TUBESZB_ZIGBEE
  this->setup_radio_();
#endif
  this->settings_->set_mutation_guard([this]() {
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->gateway_ota_state_ != 1 && !this->action_pending_ &&
           this->snapshot_.update_state != update::UPDATE_STATE_INSTALLING
#ifdef USE_TUBESZB_ZIGBEE
           && !this->radio_.busy && this->radio_.result != 1
#endif
        ;
  });
}

void TubesZBWeb::on_ota_global_state(ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *) {
  // OTA callbacks keep HTTP progress current while the download blocks the main loop.
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->gateway_start_.clear();
  switch (state) {
    case ota::OTA_STARTED:
      this->gateway_ota_state_ = 1;
      this->gateway_ota_progress_ = 0.0f;
      this->gateway_ota_error_ = 0;
      break;
    case ota::OTA_IN_PROGRESS:
      this->gateway_ota_state_ = 1;
      if (std::isfinite(progress))
        this->gateway_ota_progress_ = std::clamp(progress, 0.0f, 100.0f);
      break;
    case ota::OTA_COMPLETED:
      this->gateway_ota_state_ = 2;
      this->gateway_ota_progress_ = 100.0f;
      this->gateway_ota_error_ = 0;
      break;
    case ota::OTA_ABORT:
    case ota::OTA_ERROR:
      this->gateway_ota_state_ = 3;
      this->gateway_ota_error_ = error;
      break;
  }
}

void TubesZBWeb::check_gateway_update() {
  if (!this->settings_->internet_allowed())
    return;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (this->restart_guard_.requested() || this->checking_ || this->command_ != Command::COMMAND_NONE ||
        this->gateway_ota_state_ == 1 || this->update_->state == update::UPDATE_STATE_INSTALLING ||
        !this->ethernet_->is_connected())
      return;
#ifdef USE_TUBESZB_ZIGBEE
    if (this->radio_.busy || this->radio_.result == 1)
      return;
#endif
    if (this->gateway_ota_state_ == 3) {
      this->gateway_ota_state_ = 0;
      this->gateway_ota_progress_ = -1.0f;
      this->gateway_ota_error_ = 0;
    }
    this->checking_ = true;
    this->check_timeout_ = false;
    this->check_started_ = millis();
    this->check_source_ = this->update_->get_source_url();
    this->snapshot_.checking = true;
  }
  this->update_->status_clear_error();
  this->update_->check();
}

void TubesZBWeb::loop() {
  const uint32_t now = App.get_loop_component_start_time();
  Command command;
  std::string value;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    command = this->command_;
    this->command_ = Command::COMMAND_NONE;
    value.swap(this->command_value_);
  }
  if (command == Command::COMMAND_RESTART) {
    this->restart_at_ = now + 1500;
    this->restart_pending_ = true;
  } else if (command == Command::COMMAND_CHECK) {
    check_gateway_update();
  } else if (command == Command::COMMAND_INSTALL) {
    // Revalidate on the main loop; the HTTP snapshot may be stale.
    if (this->settings_->internet_allowed() && this->update_->state == update::UPDATE_STATE_AVAILABLE &&
        valid_ota_url(this->update_->update_info.firmware_url)) {
      {
        std::lock_guard<std::mutex> lock(this->mutex_);
        this->gateway_start_.arm(now);
      }
      this->update_->perform(false);
    } else {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->gateway_ota_state_ = 3;
      this->gateway_ota_progress_ = -1.0f;
      this->gateway_ota_error_ = http_request::OTA_BAD_URL;
    }
  }
#ifdef USE_TUBESZB_ZIGBEE
  else if (command != Command::COMMAND_NONE)
    this->run_radio_command_(command, value);
#endif
  bool missing_ota_start = false;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    missing_ota_start = this->gateway_start_.expired(now);
    if (missing_ota_start) {
      this->gateway_start_.clear();
      this->gateway_ota_state_ = 3;
      this->gateway_ota_progress_ = -1.0f;
      this->gateway_ota_error_ = http_request::OTA_BAD_URL;
    }
  }
  if (missing_ota_start) {
    // perform() may publish INSTALLING although flash() never emits OTA_STARTED.
    // Report the failed start to the update entity as well, releasing both guards.
    this->update_->on_ota_state(ota::OTA_ERROR, 0.0f, http_request::OTA_BAD_URL);
    ESP_LOGE(TAG, "Gateway firmware installation did not start; controls unlocked");
  }
  if (this->checking_ && (this->update_->status_has_error() || int32_t(now - this->check_started_) >= 30000)) {
    this->check_timeout_ = !this->update_->status_has_error();
    this->checking_ = false;
  }
  bool restart_now = false;
  if (this->restart_pending_ && int32_t(now - this->restart_at_) >= 0) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    restart_now = this->restart_guard_.ready(this->gateway_ota_state_ == 1, this->radio_flash_active_);
  }
  if (restart_now)
    App.safe_reboot();
  if (command == Command::COMMAND_NONE && uint32_t(now - this->sampled_at_) < 1000)
    return;
  this->sampled_at_ = now;
  Snapshot next;
  next.ethernet = this->ethernet_->is_connected();
  next.api = api::global_api_server != nullptr && api::global_api_server->is_connected();
#ifdef USE_ZWAVE_PROXY
  auto *proxy = zwave_proxy::global_zwave_proxy;
  next.zwave_connected = proxy != nullptr && proxy->get_api_connection() != nullptr;
  next.proxy = next.zwave_connected;
  next.zwave_baud = (this->zwave_uart_ != nullptr ? this->zwave_uart_ : this->uart_)->get_baud_rate();
  next.home_id = proxy != nullptr ? proxy->get_home_id() : 0;
#endif
#ifdef USE_TUBESZB_ZIGBEE
  this->sample_radio_();
  next.proxy = this->stream_->is_connected();
  next.update_verified = !this->verified_source_.empty() && this->verified_source_ == this->update_->get_source_url();
#endif
  next.uptime = static_cast<uint32_t>(millis_64() / 1000);
  next.free_heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  next.baud = this->uart_->get_baud_rate();
  next.update_state = this->update_->state;
  next.update_error = this->update_->status_has_error() || this->check_timeout_;
  next.checking = this->checking_;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->snapshot_ = std::move(next);
    this->action_pending_ =
        this->restart_pending_ || this->gateway_start_.pending() || this->command_ != Command::COMMAND_NONE;
  }
}

void TubesZBWeb::dump_config() { ESP_LOGCONFIG(TAG, "TubesZB frontend: /; original ESPHome frontend: /esphome"); }

bool TubesZBWeb::canHandle(AsyncWebServerRequest *request) const {
  char buffer[AsyncWebServerRequest::URL_BUF_SIZE];
  auto url = request->url_to(buffer);
  return url == "/" || url == "/esphome" || url == "/esphome/" || url == "/tubeszb/status" ||
         url == "/tubeszb/restart" || url == "/tubeszb/check-update" || url == "/tubeszb/install-update"
#ifdef USE_TUBESZB_ZIGBEE
         || url == "/tubeszb/zigbee-action"
#endif
      ;
}

void TubesZBWeb::send_json_(AsyncWebServerRequest *request, int code, const std::string &body) {
  network_settings::web::send_json(request, code, body);
}

void TubesZBWeb::error_(AsyncWebServerRequest *request, int code, const char *message) {
  ESP_LOGW(TAG, "Frontend request rejected (%d): %s", code, message);
  network_settings::web::send_error(request, code, message);
}

bool TubesZBWeb::authorized_(AsyncWebServerRequest *request) {
  if (!network_settings::web::authorized(request, "X-TubesZB-UI", this->nonce_)) {
    this->error_(request, 401, "Reload this device's page before making changes.");
    return false;
  }
  return true;
}

void TubesZBWeb::status_(AsyncWebServerRequest *request) {
  std::string payload;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    auto body = json::build_json([this](JsonObject root) {
      root["nonce"] = this->nonce_;
      root["name"] = App.get_name().c_str();
      root["model"] = this->model_;
      root["board"] = this->board_;
#ifdef USE_TUBESZB_ZIGBEE
#ifdef USE_TUBESZB_CC2652
      root["radio_type"] = "cc2652";
#elif defined(USE_TUBESZB_EFR32)
      root["radio_type"] = "efr32";
#else
      root["radio_type"] = "serial";
#endif
      this->radio_json_(root);
#else
      root["radio_type"] = "zwave";
#endif
#ifdef USE_TUBESZB_DUAL_RADIO
      root["dual_radio"] = true;
#else
      root["dual_radio"] = false;
#endif
#ifdef USE_ZWAVE_PROXY
      auto zwave = root["zwave"].to<JsonObject>();
      zwave["connected"] = this->snapshot_.zwave_connected;
      zwave["home_id"] = this->snapshot_.home_id;
      zwave["baud_rate"] = this->snapshot_.zwave_baud;
#endif
      root["project_version"] = ESPHOME_PROJECT_VERSION;
      root["esphome_version"] = ESPHOME_VERSION;
      root["mac"] = this->mac_;
      root["ethernet_connected"] = this->snapshot_.ethernet;
      root["api_connected"] = this->snapshot_.api;
      root["proxy_connected"] = this->snapshot_.proxy;
      root["home_id"] = this->snapshot_.home_id;
      root["baud_rate"] = this->snapshot_.baud;
      root["uptime"] = this->snapshot_.uptime;
      root["free_heap"] = this->snapshot_.free_heap;
      root["gateway_ota_state"] = this->gateway_ota_state_;
      root["gateway_ota_progress"] = this->gateway_ota_progress_;
      root["gateway_ota_error"] = this->gateway_ota_error_;
      root["update_state"] = unsigned(this->snapshot_.update_state);
      root["latest_version"] = this->latest_version_;
      root["update_error"] = this->snapshot_.update_error;
      root["update_verified"] = this->snapshot_.update_verified;
      root["checking"] = this->snapshot_.checking || this->command_ == Command::COMMAND_CHECK;
      root["action_pending"] = this->action_pending_;
      root["restart_allowed"] = this->restart_guard_.allowed(this->gateway_ota_state_ == 1, this->radio_flash_active_);
    });
    payload.assign(body.c_str(), body.size());
  }
  this->send_json_(request, 200, payload);
}

void TubesZBWeb::handleRequest(AsyncWebServerRequest *request) {
  char buffer[AsyncWebServerRequest::URL_BUF_SIZE];
  auto url = request->url_to(buffer);
  if (request->method() == HTTP_GET && url == "/") {
    auto *response = request->beginResponse(200, "text/html; charset=utf-8", FRONTEND_GZ, FRONTEND_GZ_SIZE);
    response->addHeader("Content-Encoding", "gzip");
    response->addHeader("Cache-Control", "no-store");
    response->addHeader("X-Frame-Options", "DENY");
    request->send(response);
    return;
  }
  if (request->method() == HTTP_GET && (url == "/esphome" || url == "/esphome/")) {
    this->server_->handle_index_request(request);
    return;
  }
  if (request->method() == HTTP_GET && url == "/tubeszb/status") {
    this->status_(request);
    return;
  }
  if (request->method() != HTTP_POST) {
    this->error_(request, 400, "Use GET for pages and status; POST for actions.");
    return;
  }
  if (!this->authorized_(request))
    return;
  const bool network_trial = this->settings_->is_trial_active();
  const bool internet_allowed = this->settings_->internet_allowed();
  std::unique_lock<std::mutex> lock(this->mutex_);
  auto reject = [&](int code, const char *message) {
    lock.unlock();
    this->error_(request, code, message);
  };
  // Recovery must remain available when a queued updater never starts. Native
  // OTA/flash events, rather than optimistic UI flags, protect real writes.
  if (url == "/tubeszb/restart") {
    if (!this->restart_guard_.request(this->gateway_ota_state_ == 1, this->radio_flash_active_)) {
      reject(409, "Wait for the active firmware write or requested restart to finish.");
      return;
    }
    this->command_ = Command::COMMAND_RESTART;
    this->command_value_.clear();
    this->action_pending_ = true;
    lock.unlock();
    this->send_json_(request, 200, "{\"ok\":true}");
    return;
  }
  if (this->restart_guard_.requested() || this->action_pending_ || this->gateway_ota_state_ == 1 ||
      this->snapshot_.update_state == update::UPDATE_STATE_INSTALLING || this->snapshot_.checking) {
    reject(409, "Another device action is in progress.");
    return;
  }
#ifdef USE_TUBESZB_ZIGBEE
  if (this->radio_.busy || this->radio_.result == 1) {
    reject(409, "Wait for the active Zigbee operation to finish.");
    return;
  }
#endif
  if (network_trial) {
    reject(409, "Keep or revert the network trial before updating firmware.");
    return;
  }
  const auto radio_action = request->arg("action");
  if (!internet_allowed &&
      (url == "/tubeszb/check-update" || url == "/tubeszb/install-update" ||
       (url == "/tubeszb/zigbee-action" && (radio_action == "check" || radio_action == "flash")))) {
    reject(409, "Use without internet is enabled. Disable it in Network to check or download firmware.");
    return;
  }
  if (url == "/tubeszb/check-update") {
    if (!this->snapshot_.ethernet) {
      reject(409, "Ethernet is not connected.");
      return;
    }
    this->command_ = Command::COMMAND_CHECK;
  } else if (url == "/tubeszb/install-update") {
    if (this->snapshot_.update_state != update::UPDATE_STATE_AVAILABLE || this->snapshot_.update_error ||
        !this->snapshot_.update_verified) {
      reject(409, "Check for a valid firmware update before installing.");
      return;
    }
    this->gateway_ota_state_ = 0;
    this->gateway_ota_progress_ = -1.0f;
    this->gateway_ota_error_ = 0;
    this->command_ = Command::COMMAND_INSTALL;
#ifdef USE_TUBESZB_ZIGBEE
  } else if (url == "/tubeszb/zigbee-action") {
    const auto action = request->arg("action");
    const auto value = request->arg("value");
    if (action == "check") {
      if (this->check_button_ == nullptr) {
        reject(400, "Radio firmware checks are unavailable for this model.");
        return;
      }
      if (!this->radio_.url.empty()) {
        reject(400, "Clear the custom URL to check the selected release channel.");
        return;
      }
      this->command_ = Command::COMMAND_RADIO_CHECK;
    } else if (action == "probe") {
      if (this->probe_button_ == nullptr) {
        reject(400, "Radio identity probing is unavailable for this model.");
        return;
      }
      this->command_ = Command::COMMAND_RADIO_PROBE;
    } else if (action == "reset")
      this->command_ = Command::COMMAND_RADIO_RESET;
    else if (action == "bootloader") {
      if (this->bootloader_button_ == nullptr) {
        reject(400, "Manual bootloader mode is unavailable for this radio.");
        return;
      }
      this->command_ = Command::COMMAND_RADIO_BOOTLOADER;
    } else if (action == "pause")
      this->command_ = Command::COMMAND_SERIAL_PAUSE;
    else if (action == "resume")
      this->command_ = Command::COMMAND_SERIAL_RESUME;
    else if (action == "flash") {
      if (this->flash_button_ == nullptr) {
        reject(400, "Radio firmware flashing is unavailable for this model.");
        return;
      }
      if (!this->radio_.url.empty() && !is_gbl_url(this->radio_.url)) {
        reject(400, "Custom radio firmware must be a direct HTTP(S) .gbl file URL.");
        return;
      }
      if (this->radio_.url.empty() && !this->radio_.checked) {
        reject(409, "Check Zigbee firmware on the selected channel before flashing.");
        return;
      }
      this->command_ = Command::COMMAND_RADIO_FLASH;
    } else if (action == "baud") {
#if defined(USE_TUBESZB_EFR32) || defined(USE_TUBESZB_SERIAL_RADIO)
      if (this->baud_ == nullptr || !this->baud_->has_option(value)) {
        reject(400, "Unsupported baud rate.");
        return;
      }
      this->command_ = Command::COMMAND_SET_BAUD;
#else
      reject(400, "Changing the CC2652 baud rate is not supported here.");
      return;
#endif
    } else if (action == "url") {
#ifdef USE_TUBESZB_EFR32
      if (this->custom_url_ == nullptr) {
        reject(400, "Custom firmware URLs are not supported for this radio.");
        return;
      }
      if (!value.empty() && !is_gbl_url(value)) {
        reject(400, "Enter a direct HTTP(S) .gbl file URL, or leave it empty.");
        return;
      }
      this->command_ = Command::COMMAND_SET_URL;
#else
      reject(400, "Custom firmware URLs are not supported for this radio.");
      return;
#endif
    } else if (action == "gateway-channel" || action == "zigbee-channel") {
      if ((action == "gateway-channel" ? this->gateway_beta_ : this->zigbee_beta_) == nullptr) {
        reject(400, "Firmware channels are unavailable for this model.");
        return;
      }
      if (value != "stable" && value != "beta") {
        reject(400, "Choose Stable or Beta.");
        return;
      }
      this->command_ = action == "gateway-channel" ? Command::COMMAND_GATEWAY_CHANNEL : Command::COMMAND_ZIGBEE_CHANNEL;
    } else {
      reject(400, "Unknown Zigbee action.");
      return;
    }
    this->command_value_ = value;
#endif
  } else {
    reject(404, "Unknown device action.");
    return;
  }
  this->action_pending_ = true;
  lock.unlock();
  // The ESPHome IDF server maps unsupported codes (including 202) to HTTP 500.
  // Acknowledge that the command was queued; status reports its completion.
  this->send_json_(request, 200, "{\"ok\":true}");
}

}  // namespace esphome::tubeszb_web

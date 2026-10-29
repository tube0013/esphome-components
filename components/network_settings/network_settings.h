#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/ethernet/ethernet_component.h"
#include "esphome/components/web_server_base/web_server_base.h"
#include "trial_state.h"
#include "guarded_http.h"
#include "single_flight.h"
#include "esphome/components/http_request/update/http_request_update.h"
#include <mutex>
#include <functional>

namespace esphome::network_settings {

extern const char NETWORK_PAGE[];

class NetworkSettings : public Component, public AsyncWebHandler {
 public:
  NetworkSettings(http_request::HttpRequestComponent *request, http_request::HttpRequestUpdate *update,
                  ethernet::EthernetComponent *ethernet, web_server_base::WebServerBase *base)
      : ethernet_(ethernet), base_(base), http_request_(request), update_(update) {}
  bool internet_allowed() const { return this->internet_policy_.allowed(); }
  bool use_without_internet() const { return this->internet_policy_.offline(); }
  bool begin_connectivity_check() { return this->connectivity_check_.begin(); }
  void end_connectivity_check() { this->connectivity_check_.end(); }
  bool set_use_without_internet(bool value);
  template<typename F> void add_on_internet_enabled_callback(F &&callback) {
    this->internet_enabled_callbacks_.add(std::forward<F>(callback));
  }
  float get_setup_priority() const override { return setup_priority::ETHERNET + 1.0f; }
  void setup() override;
  void loop() override;
  void dump_config() override;
  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;
  bool is_trial_active() {
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->state_.testing || this->state_.saved.phase == Phase::PHASE_STAGED;
  }
  template<typename F> void set_mutation_guard(F &&guard) { this->mutation_guard_ = std::forward<F>(guard); }

 protected:
  bool save_(const SavedState &state);
  bool set_use_without_internet_(bool value);
  void send_json_(AsyncWebServerRequest *request, int code, const std::string &body);
  void send_error_(AsyncWebServerRequest *request, int code, const char *message);
  void status_(AsyncWebServerRequest *request);
  bool authorize_(AsyncWebServerRequest *request);
  bool at_trial_address_(AsyncWebServerRequest *request);
  void apply_(AsyncWebServerRequest *request);
  void queue_restart_();

  ethernet::EthernetComponent *ethernet_{nullptr};
  web_server_base::WebServerBase *base_{nullptr};
  SingleFlight connectivity_check_;
  InternetPolicy internet_policy_;
  LazyCallbackManager<void()> internet_enabled_callbacks_;
  bool internet_enabled_pending_{false};
  GuardedHttp guarded_http_;
  http_request::HttpRequestComponent *http_request_{nullptr};
  http_request::HttpRequestUpdate *update_{nullptr};
  TrialState state_{};
  std::mutex mutex_;
  std::string nonce_;
  bool storage_ready_{true};
  bool restart_pending_{false};
  uint32_t restart_at_{0};
  std::function<bool()> mutation_guard_;
};

}  // namespace esphome::network_settings

#pragma once
#include "baud_rate.h"
#include "esphome/core/preferences.h"
#include "esphome/core/log.h"
#include "esphome/components/select/select.h"
#include <string>
#include <cstdlib>

namespace esphome::zigbee_firmware_baud {
class RuntimeBaud {
 public:
  void set_preference_hash(uint32_t hash) { this->preference_hash_ = hash; }
  void set_selector(select::Select *selector) { this->selector_ = selector; }
  void setup(uint32_t initial) {
    this->preference_ = global_preferences->make_preference<uint32_t>(this->preference_hash_);
    this->current_ = initial;
    if (this->selector_ != nullptr && !this->selector_->current_option().empty()) {
      const uint32_t selected = std::strtoul(this->selector_->current_option().c_str(), nullptr, 10);
      if (valid_baud(selected))
        this->current_ = selected;
    }
    uint32_t saved = 0;
    if (this->preference_.load(&saved) && valid_baud(saved))
      this->current_ = saved;
    this->ready_ = true;
    // The numeric saved baud survives selector option reordering across builds.
    if (this->selector_ != nullptr && supports(this->current_) &&
        this->selector_->current_option() != std::to_string(this->current_)) {
      this->applying_manifest_ = true;
      this->selector_->make_call().set_option(std::to_string(this->current_)).perform();
      this->applying_manifest_ = false;
      if (!global_preferences->sync())
        ESP_LOGE("zigbee_firmware_baud", "Could not synchronize saved baud selector");
    }
  }
  uint32_t current() const { return this->current_; }
  bool applying_manifest() const { return this->applying_manifest_; }
  bool supports(uint32_t baud) const {
    return valid_baud(baud) && (this->selector_ == nullptr || this->selector_->has_option(std::to_string(baud)));
  }
  void manual(uint32_t baud) {
    if (!this->ready_ || !valid_baud(baud) || baud == this->current_)
      return;
    this->current_ = baud;
    if (!this->persist_())
      ESP_LOGE("zigbee_firmware_baud", "Could not save application baud %u", unsigned(baud));
  }
  // Called only after a verified transfer and after retuning the actual UART.
  // Bound template selectors must skip their UART set_action while this flag is
  // true; their state and saved option still update via the normal select call.
  bool commit(uint32_t baud) {
    if (!this->ready_ || !supports(baud))
      return false;
    this->current_ = baud;
    this->applying_manifest_ = true;
    if (this->selector_ != nullptr)
      this->selector_->make_call().set_option(std::to_string(baud)).perform();
    this->applying_manifest_ = false;
    return this->persist_();
  }

 private:
  bool persist_() {
    for (unsigned attempt = 0; attempt < 3; ++attempt) {
      if (this->preference_.save(&this->current_) && global_preferences->sync())
        return true;
    }
    return false;
  }
  ESPPreferenceObject preference_;
  select::Select *selector_{nullptr};
  uint32_t preference_hash_{0}, current_{0};
  bool ready_{false}, applying_manifest_{false};
};
}  // namespace esphome::zigbee_firmware_baud

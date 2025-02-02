#pragma once

#include "esphome/core/action.h"
#include "cc2652_flasher.h"

namespace esphome {
namespace cc2652_flasher {

/** 
 * CC2652FlasherAction
 *
 * This custom action calls the flash_firmware() method of your CC2652FlasherComponent.
 */
class CC2652FlasherAction : public Action<> {
 public:
  explicit CC2652FlasherAction(CC2652FlasherComponent *component) { component_ = component; }
  void play(ActionCall back) override {
    if (component_ != nullptr) {
      component_->flash_firmware();
    }
    // Signal that the action is complete.
    callback();
  }
 protected:
  CC2652FlasherComponent *component_;
};

}  // namespace cc2652_flasher
}  // namespace esphome

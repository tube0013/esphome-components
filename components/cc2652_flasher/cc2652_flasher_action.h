#pragma once

#include "esphome/core/action.h"
#include "cc2652_flasher.h"

namespace esphome {
namespace cc2652_flasher {

/** 
 * CC2652FlasherAction
 *
 * A custom action that calls the flash_firmware() method of a CC2652FlasherComponent.
 */
class CC2652FlasherAction : public Action<> {
 public:
  explicit CC2652FlasherAction(CC2652FlasherComponent *component) { component_ = component; }
  
  void play(ActionCall callback) override {
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

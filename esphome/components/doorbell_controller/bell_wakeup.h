#pragma once

namespace esphome {
namespace doorbell_controller {

class BellWakeup {
 public:
  enum Level { DISABLED, LOW, HIGH };

  bool accept_press() {
    if (this->cooling_down_)
      return false;
    this->cooling_down_ = true;
    return true;
  }

  void finish_cooldown() { this->cooling_down_ = false; }

  Level wake_level(bool asserted) const {
    if (this->cooling_down_)
      return DISABLED;
    // Wake on release if the button is still held after the cooldown.
    return asserted ? LOW : HIGH;
  }

 protected:
  bool cooling_down_{false};
};

}  // namespace doorbell_controller
}  // namespace esphome

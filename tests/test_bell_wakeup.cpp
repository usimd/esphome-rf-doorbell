#include <cassert>

#include "esphome/components/doorbell_controller/bell_wakeup.h"

using esphome::doorbell_controller::BellWakeup;

int main() {
  BellWakeup bell;
  assert(bell.wake_level(false) == BellWakeup::HIGH);

  assert(bell.accept_press());
  assert(!bell.accept_press());
  assert(bell.wake_level(true) == BellWakeup::DISABLED);
  assert(bell.wake_level(false) == BellWakeup::DISABLED);

  bell.finish_cooldown();
  // A held button must wake on release, not continuously on its asserted level.
  assert(bell.wake_level(true) == BellWakeup::LOW);
  assert(bell.wake_level(false) == BellWakeup::HIGH);
  assert(bell.accept_press());
  bell.finish_cooldown();
  assert(bell.wake_level(false) == BellWakeup::HIGH);
}

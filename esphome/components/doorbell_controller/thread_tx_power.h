#pragma once

#include "esphome/core/defines.h"

#ifdef USE_OPENTHREAD
#include "esphome/components/openthread/openthread.h"
#include "esphome/core/log.h"
#include <openthread/platform/radio.h>

namespace esphome {
namespace doorbell_controller {

// Stores the power for the stack start-up path and, once the stack runs, applies it live.
inline void apply_thread_tx_power(int8_t dbm) {
  using openthread::InstanceLock;
  auto *ot = openthread::global_openthread_component;
  if (ot == nullptr)
    return;
  ot->set_output_power(dbm);
  if (!ot->is_lock_initialized())
    return;
  auto lock = InstanceLock::try_acquire(100);
  if (!lock) {
    ESP_LOGW("thread_tx_power", "OpenThread lock busy; %d dBm not applied", dbm);
    return;
  }
  if (otPlatRadioSetTransmitPower(lock.get_instance(), dbm) != OT_ERROR_NONE) {
    ESP_LOGW("thread_tx_power", "Failed to set %d dBm", dbm);
    return;
  }
  ESP_LOGI("thread_tx_power", "Thread TX power %d dBm", dbm);
}

}  // namespace doorbell_controller
}  // namespace esphome

#endif  // USE_OPENTHREAD

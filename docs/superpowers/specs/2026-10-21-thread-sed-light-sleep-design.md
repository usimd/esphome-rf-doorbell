# Thread SED + Light Sleep Firmware — Design

Date: 2026-10-21
Status: Approved (implementation gated on ESPHome 2026.11 beta)

## Goal

Replace the WiFi deep-sleep firmware with an always-connected OpenThread Sleepy End Device
(SED) using ESP-IDF automatic light sleep. HA→device commands arrive within the poll
period (≤5 s) and the async "desired state" reconcile machinery disappears.

## Upstream dependencies

| PR | Status | Needed |
|----|--------|--------|
| esphome#11766 `openthread.pollperiod` action | Merged (2026.7) | Yes |
| esphome#12325 `power_management` / `esp32_power_management` | Merged to dev 2026-10-12, expected 2026.11 | Yes (gate) |
| esphome#16332, #16334 OpenThread shutdown/rewrite | Merged | Nice-to-have |
| esphome#16387 child timeout scaling | Open | No (poll ≤ 60 s) |
| esphome#16407, #14526 | Open | No |

Build target: first ESPHome 2026.11 beta, then 2026.11 stable.

## Architecture

- `esphome/doorbell.yaml` becomes the Thread firmware. WiFi config is removed (kept in git
  history). Draft `esphome/doorbell-thread.yaml` and `esphome/restart_into_ota_window.py`
  are deleted.
- Removed: `deep_sleep`, wake-cause handling, `extend_wake_time`, `quick_sync_and_sleep`,
  `cold_boot_ota_window`, `handle_interrupt_wakeup`, `reconcile_bell_mute`,
  `sync_bell_mute_desired_*`, wake-count globals, HA `input_boolean.doorbell_bell_mute_desired`
  and `input_number.doorbell_sleep_duration_desired` bindings.
- Added:
  - `openthread: { device_type: MTD, tlv: !secret thread_tlv, poll_period: 5s }`
  - `power_management: [{ platform: esp32_power_management, enable_light_sleep: true, power_down_peripherals: false }]`
  - Template number "Thread Poll Period" (1–60 s, step 1, default 5, restored) whose
    `set_action` calls `openthread.pollperiod`.
  - "Bell Mute" exposed directly to HA (`RESTORE_DEFAULT_OFF`).

### `doorbell_controller` changes

- Remove `deep_sleep_id` option, `set_deep_sleep()`, `SleepDurationNumber`, and
  `get/set_sleep_duration()`.
- Configure GPIO4 (bell, active high) and GPIO2 (battery alert, active low) as light-sleep
  wake sources (`gpio_wakeup_enable` + `esp_sleep_enable_gpio_wakeup`).
- Ensure GPIO5 (bell mute SSR) and GPIO14 (door opener) keep their level during light sleep;
  replace the deep-sleep hold logic accordingly.
- Supply voltage / charge current update interval: 60 s.

### `max17260`

- `update_interval: 20min` (no code change expected).

## Behaviour

- **Bell press:** GPIO4 level wakes the CPU → interrupt-driven `binary_sensor` → publish
  `Bell Signal`, fire `esphome.doorbell_ring` and notify immediately. 30 s cooldown guard is
  kept. During cooldown the GPIO4 wake source is disabled so a stuck button cannot keep the
  chip awake; re-enabled after cooldown.
- **HA commands:** delivered within the current poll period; no reconciliation.
- **OTA:** `ota.on_begin` → `openthread.pollperiod: 0s`; `on_error` → restore the number's
  value. First flash from WiFi firmware must be via USB-UART.
- **Thread down:** bell mute keeps working locally; events while disconnected are dropped;
  state resyncs on reconnect. No WiFi fallback.
- **Reboot:** bell mute state restored from flash.

## Bring-up tuning (measured, not assumed)

1. `power_down_peripherals: false` first. Then try `true`; keep only if I2C (MAX17260,
   ISL23315), ADC and SPI (SX1262) keep working after sleep.
2. Tune main loop interval if the 16 ms default prevents meaningful sleep.

## Testing

1. Compile against the 2026.11 beta (project venv).
2. USB-UART flash; verify join to the Sonoff Dongle v2 border router, `Thread Role` = child.
3. Bell press → HA event latency; Bell Mute toggle latency at 5 s poll; poll-period number
   changes take effect.
4. OTA over Thread.
5. Current measurement with `power_down_peripherals` off vs on.
6. 48 h soak: battery SoC trend and absence of spurious bell events.

## Out of scope

LoRa RX, HA notification-dismiss issues, hardware changes.

# ESPHome Firmware

`doorbell.yaml` is the single supported configuration: ESP32-C6 OpenThread MTD/SED
with automatic light sleep. The earlier WiFi firmware is preserved in git history.
There is no deep sleep or wake/reconcile cycle.

## Setup and build

From the repository root:

```bash
.venv/bin/python -m pip install -r esphome/requirements.txt
.venv/bin/esphome compile esphome/doorbell.yaml
```

The toolchain is pinned to ESPHome **2026.11.0b3**. The local, ignored
`esphome/secrets.yaml` must provide:

- `thread_tlv`: the full operational dataset from the existing HA Thread network
- `doorbell_api_encryption_key`: retain the existing device's native API key
- `doorbell_ota_password`: retain the existing OTA password

The public test dataset in `.github/workflows/secrets-ci.yaml` is only for CI.
Never use it on your real network. A placeholder `thread_tlv` fails validation.

Use USB-UART for the initial migration, with physical recovery available:

```bash
.venv/bin/esphome run esphome/doorbell.yaml --device /dev/ttyUSB0
```

Select the actual serial port. Ensure Home Assistant and the build machine can
reach the Thread network over IPv6 through the border router. After migration,
check HA's ESPHome integration for the device's new address if it does not reconnect.

## Runtime controls

- **Bell Mute** directly controls GPIO5 and restores its state on reboot.
- **Thread Poll Period** is restored, defaults to 5 seconds and accepts 1-60 seconds.
  Try 2-3 seconds if incoming commands feel slow. Polling is not a hard end-to-end
  latency guarantee: network retries and TCP can add delay.
- Supply voltage and charge current update every minute; battery percentage every
  20 minutes. Battery alerts also request a fuel-gauge update and clear alert flags.
- Outbound bell events transmit immediately when HA is connected. Offline events
  are logged and not queued for later notification.
- A 30-second cooldown suppresses repeat bell events. Bell GPIO wake is disabled
  during cooldown; a still-held button then wakes on release before being rearmed.
- Battery-alert wake also switches to the release level while asserted, preventing
  an uncleared alert from continuously waking the CPU.

GPIO wake and normal GPIO interrupts share their trigger type in ESP-IDF. The two
GPIO binary sensors therefore use polling, not edge ISRs: a level wakes the CPU
and the resumed main loop samples it. Keep the default main-loop interval during
bring-up; increasing it could miss short bell pulses.

The obsolete HA `input_boolean.doorbell_bell_mute_desired` and
`input_number.doorbell_sleep_duration_desired` helpers are no longer used.
Update HA automations to toggle the device's **Bell Mute** switch directly.
The sync-status, mute proxy, sleep-duration and wake-counter entities are removed;
delete obsolete entries from HA's entity registry if they remain after migration.

## OTA

Native ESPHome OTA switches to receive-on (`openthread.set_poll_period: 0s`) when
transfer begins, restores the selected poll period on error/abort, and restores
it on boot after a successful update. Changing the number during OTA records the
requested value without taking the receiver out of OTA mode.

## Hardware validation still required

Verify Thread attachment, HA command delivery at 5s and 2s polling, short bell
pulses, a held bell input past 30s, mute retention, the opener's 5s shutoff, battery
alerts, and OTA success/failure recovery. Then measure current and run a 48-hour
soak. No measured current or battery-life claim is made for this board yet.

`power_down_peripherals: false` is intentional for the first test. Only enable it
after verifying I2C (MAX17260/ISL23315), ADC and SPI (SX1262) across light sleep.
GPIO5 and GPIO14 keep their output state through upstream ESPHome's tickless-idle
GPIO setup; they no longer use custom deep-sleep holds.

LoRa remains configured with RX disabled, as before. No RF wake path is added.
The hardware's existing GPIO12/13 USB-Serial-JTAG and GPIO4/5 strapping warnings
are expected.

## Host regression test

```bash
g++ -std=c++17 -Wall -Wextra -Werror -I. tests/test_bell_wakeup.cpp -o /tmp/bell-wakeup-test
/tmp/bell-wakeup-test
```

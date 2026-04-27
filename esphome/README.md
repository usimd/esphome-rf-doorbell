# ESPHome Firmware

This directory contains the active Rev3 firmware for the base doorbell board.

## Active Files

- `doorbell.yaml`: main configuration used for builds
- `components/doorbell_controller/`: charger and power-management helper component
- `components/max17260/`: MAX17260 fuel gauge component
- `secrets.yaml`: local machine-specific secrets, ignored by git

## Rev3 Firmware Model

The supported firmware path is built around:

- ESP32-C6
- LTC4079 charger with ISL23315 digipot control
- MAX17260 fuel gauge
- ESPHome `sx126x` integration for the Wio-SX1262 module
- deep sleep wakeups from bell input, SX1262 DIO1, battery alert, and timer

There is no supported legacy firmware configuration in this directory anymore.

## Build

```bash
esphome compile doorbell.yaml
```

## OTA Recovery Helper

When the installed node is only reachable during short wake windows, use the repo-local helper to press the ESPHome `Restart` button over the native API and force the device into its existing `cold_boot_ota_window`.

```bash
../.venv/bin/python restart_into_ota_window.py --restart
```

Without `--restart`, the helper only prints the live device metadata and available button entities.
Add `--wait-timeout 180` if you want it to keep polling through the current deep-sleep cycle until the device wakes up.
The helper prefers `esphome-rf-doorbell.local` by default because the active firmware does not currently configure a fixed `manual_ip`.

## Runtime Notes

- `doorbell_controller` updates the real `deep_sleep` component sleep duration at runtime.
- `max17260` restores learned parameters from preferences and now treats POR-read failures as setup failures instead of healthy state.
- The firmware intentionally uses local external components from `components/`.

## Known Bring-Up Warnings

ESPHome currently warns that some selected GPIOs are USB-Serial-JTAG or strapping pins on ESP32-C6. These warnings are expected with the present hardware design and do not indicate a configuration parse failure.

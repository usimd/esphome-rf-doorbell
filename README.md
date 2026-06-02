# ESPHome RF Doorbell

![License: CERN-OHL-W-2.0](https://img.shields.io/badge/license_HW-CERN--OHL--W--2.0-blue)
[![CI](https://github.com/usimd/esphome-rf-doorbell/actions/workflows/ci.yaml/badge.svg)](https://github.com/usimd/esphome-rf-doorbell/actions/workflows/ci.yaml)
[![Upload to AISLER](https://img.shields.io/badge/Upload_to_-AISLER-ff8000)](https://aisler.net/p/new?url=https://raw.githubusercontent.com/usimd/esphome-rf-doorbell/refs/heads/main/base/esphome-rf-doorbell.kicad_pcb&ref=github)

Battery-powered ESPHome doorbell hardware and firmware centered on the current Rev3 base board.

## Current Scope

- `base/` is the active hardware target.
- `esphome/doorbell.yaml` is the active firmware target.
- Rev3 uses ESP32-C6, LTC4079, ISL23315, MAX17260, and ESPHome's built-in `sx126x` support.
- `remote/` is intentionally kept as unfinished WIP hardware. It is not a supported production target.

## Rev3 Hardware Summary

| Ref | Part | Purpose |
|-----|------|---------|
| U1 | LTC4079 | Li-ion charger |
| U3 | TPS63900 | Low-power buck-boost regulator |
| U4 | MAX17260 | Fuel gauge |
| U5 | LTV-354T | Doorbell input optocoupler |
| U6 | G3VM-31DR | Door opener relay |
| U7 | Wio-SX1262 | SX1262 radio module |
| U8 | G3VM-63ER | Bell mute relay |
| U9 | ESP32-C6-MINI-1 | Main MCU |
| U10 | ISL23315 | Digipot for charger control |

The base board already includes an SX126x receive path and that remains part of the intended design, even though the remote side is still unfinished.

## Repository Layout

```text
base/                    Active Rev3 KiCad project
datasheets/              Datasheets and hardware notes used during development
esphome/                 Active ESPHome firmware
esphome/components/      Custom ESPHome components
lib/                     Project KiCad footprints and symbols
remote/                  Unfinished remote hardware kept as WIP
```

## Firmware

The active firmware is in `esphome/doorbell.yaml`.

Implemented custom components:

- `doorbell_controller`: charger control, supply sensing, runtime sleep duration control
- `max17260`: fuel gauge driver with POR handling and learned-parameter persistence

Radio support uses ESPHome's built-in `sx126x` component rather than a custom RF stack.

## Build

From `esphome/`:

```bash
esphome compile doorbell.yaml
```

The configuration expects `esphome/secrets.yaml` to exist locally.

## Status

- Legacy firmware and custom components are removed from the supported path.
- The active Rev3 ESPHome config compiles successfully.
- CI builds the Rev3 firmware and the base KiCad project.
- `remote/` remains a hardware placeholder for future work and still contains legacy symbols/footprints inside that WIP design.

## Notes

- The ESP32-C6 pin usage in this design currently triggers ESPHome warnings for USB-Serial-JTAG pins and strapping pins. Those warnings reflect the current hardware choices and should be kept in mind during bring-up.
- `esphome/charging_test.yaml` and `esphome/charging_test_simple.yaml` currently exist as untracked placeholders and are not part of the supported firmware path.

## AISLER discount

In case you want to order this (or any other) project at AISLER, here's a 10€ discount code: `MakeInEurope-ZDCTN`



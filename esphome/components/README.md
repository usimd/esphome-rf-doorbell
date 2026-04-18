# ESPHome Custom Components

This directory contains the custom components used by the active Rev3 firmware.

## Components

### `doorbell_controller`

Controls the LTC4079 support circuitry around the ISL23315 digipot and exposes:

- VINDPM threshold control
- charging enable/disable
- charge-disable override
- supply voltage measurement
- charge current estimation from `V_PROG`
- runtime deep sleep duration control

This component explicitly re-enables the ESP-IDF `esp_adc` component because ESPHome excludes it by default for faster builds.

### `max17260`

Fuel gauge driver for the MAX17260 with support for:

- battery percentage
- voltage, current, temperature
- remaining/full capacity
- cycle count
- time-to-empty and time-to-full
- device name and serial number
- POR detection and reinitialization
- persistence of learned parameters in ESPHome preferences

## Loading

`doorbell.yaml` loads both components from the local `components/` directory via `external_components`.

## Support Boundary

Only `doorbell_controller` and `max17260` are part of the supported Rev3 path here. Legacy custom components were removed from this repository path.

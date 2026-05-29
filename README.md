# ESP32 Propane Scale

Compact ESP32-based propane tank scale that measures tank weight, subtracts the tank tare, and reports the propane weight and fill percentage.

This version is serial monitor based, and serves as starting point for wifi and web ble versions.

## Versions

1. Serial monitor
2. Web BLE
3. Wifi

## Quick highlights

- Weighs tanks using four load cells + HX711 amplifier
- Supports automatic and manual calibration workflows
- Saves calibration/tare values to EEPROM
- Built with `arduino-cli` and VS Code tasks for rapid iteration

## Quick Start

1. Install Arduino CLI
2. Install Espressif toolchain
3. Install ESP32 board support and the HX711 library
4. Open this workspace in VS Code. Use the provided tasks to build/flash (see 'Tasks' section)
5. Edit `PropaneScale/config.h` to set the pins and default calibration/tare values before flashing
6. Connect to a serial console at `115200` to interact with the command interface

## Building and Uploading

Building & Uploading can be done with VS Code compound task calls:

- `ESP32 Thing Quick Compile & Quick Flash`
- `ESP32 Thing Clean Rebuild & Quick Flash`
- Refer to building and upload section for details

### Building

Building is being done via arduino-cli.

- Using arduino-cli for convenience
- Have experience with it
- Already installed and configured
- Precludes having to re-learn CMake & Ninja
- Use the included VS Code tasks: `ESP32 Thing Quick Compile`, `ESP32 Thing Clean Rebuild`
- The FQBN (full qualified board name) is hard coded because the project is esp32 specific
- Example arduino-cli build command (used by the tasks):

  ```bash
  arduino-cli compile -v --fqbn esp32:esp32:esp32thing PropaneScale
  ```

### Upload

Uploading is done via esptool.exe

- Planning on eventually using esp tools entirely for build and upload
- Since esp32 is only chip used, it is hard coded, as is the upload speed, write flash location, and binary file
- the *.ino.merged.bin file must be used for an esp32
- Use the include VS Code tasks:
  - `ESP32 Thing Quick Flash`
    - This task uses `esp32-thing-quick-flash.ps1` powershell script, which in turn calls `list_ports.py`
    - These scripts return the com port the esp32 thing is attached to
    - Example cli:

    ```bash
    esptool.exe --chip esp32 --port COM3 --baud 921600 write-flash -z 0x0 D:/MyArduino/Projects/ESP32_Scale/PropaneScale/build/esp32.esp32.esp32thing/PropaneScale.ino.merged.bin
    ```

  - `ESPTool ESP32 COMPort Merged Bin Upload`
    - This task needs com port import
    - The com port input uses `list_ports.py` directly
    - Example cli:

    ```bash
    esptool.exe --chip esp32 --port COM4 --baud 921600 write-flash -z 0x0 D:/MyArduino/Projects/ESP32_Scale/PropaneScale/build/esp32.esp32.esp32thing/PropaneScale.ino.merged.bin
    ```

## Configuration

- Edit `PropaneScale/config.h` to change:
  - `CLK_PIN`, `DOUT_PIN` (HX711 pins)
    - While these can be almost any of the available pins, using 16 & 17 so the JTAG pins (12, 13, 14, 15) remain open
  - `BAUD` (serial baud rate)
    - Needs to be 115200, as that is the hard coded default boot speed of the ESP32 Thing
    - using any other speed results in ascii garbage hen the REST button is hit
  - `DEF_CALIBRATION_FACTOR`
    - Computed by running SparkFun_HX711_Calibration.ino in examples directory and noting the value
  - `DEF_KNOWN_WEIGHT`
    - The sample weight used to compute the default calibration factor
  - `DEF_TANK_TARE`
    - midpoint of average 20 lb tank tares
  - `DEF_MAX_PROPANE`
    - the legal maximum of propane for 20 lb tank: 80% x 20 = 16

## Hardware Summary

- Microcontroller: SparkFun ESP32 Thing (DEV-13907)
- Amplifier: SparkFun HX711 (BOB-13879)
- Load cells: SparkFun 50 kg (SEN-10245) — 4x in a full-bridge arrangement

### HX711 pins (default)

| HX711 | ESP32 |
| --- | --- |
| CLK | GPIO 17 |
| DOUT | GPIO 16 |

## Serial Command Interface

- Commands:
  - `a` — Automatic calibration
  - `c` — Print current runtime values
  - `d` — Reset EEPROM value to default configuration
  - `e` — Print EEPROM values
  - `h` — Print menu
  - `m` — Manual calibration (use `+` / `-`, save with `s`)
  - `k` - Set known weight for calibrations (enter value, `s` to save)
  - `l` — Single reading of propane weight and level
  - `p` — Set tank tare (enter value, `s` to save)
  - `q` — Cancellation for all input workflows
  - `r` — Re-zero scale, `q` to force-confirm
  - `t` - Set tank tare (enter value, `s` to save)

## Output example

```text
Scale load: 39.5 lbs, Calculated propane: 15.7 lbs, Level: 65.4%
```

## Project Layout

- [PropaneScale](PropaneScale): main sketch
- `config.h` (pin and calibration constants)
- Documentation under the `Documentation/` folder
- Examples under the `Examples/` folder

## Development Notes

- This workspace includes tasks for build/flash and a Doxygen setup in `Documentation/Doxygen/`.
- See `Documentation/Copilot/` for notes and design plans.

Contributing

- Fixes, improvements, and pull requests welcome. Please open issues describing bugs or requested features.

License

- This repository does not include a license file. Add one if you intend to publish the code.

---
Updated README: streamlined quick-start, build tasks, and configuration notes.

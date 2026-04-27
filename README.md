# ESP32 Propane Scale

An ESP32-based scale for measuring propane tank fill level. Weighs the tank, subtracts the known empty-tank tare, and reports the estimated propane weight and fill percentage over USB serial.

## Hardware

| Component | Part | Count |
| --------- | ---- | ----- |
| Microcontroller | [SparkFun ESP32 Thing (DEV-13907)](https://www.sparkfun.com/sparkfun-esp32-thing.html) | 1 |
| Load cell amplifier | [SparkFun HX711 (BOB-13879)](https://www.sparkfun.com/sparkfun-load-cell-amplifier-hx711.html) | 1 |
| Load cell | [Sparkfun 50 kg (SEN-10245)](https://www.sparkfun.com/load-sensor-50kg-generic.html) | 4 |

### Wiring

| HX711 Pin | ESP32 Pin |
| --------- | ------- |
| CLK | GPIO 4 |
| DOUT | GPIO 16 |
| VDD | 5V |
| VCC | 3V3 |
| GND | GND |

## Software Dependencies

- [SparkFun HX711 Arduino Library](https://github.com/sparkfun/SparkFun_HX711_Arduino_Library)
- Arduino ESP32 board support (`esp32:esp32`)

Install the HX711 library via Arduino CLI:

```bash
arduino-cli lib install "SparkFun HX711"
```

## Building

```bash
arduino-cli compile -v --fqbn esp32:esp32:esp32thing PropaneScale/
```

Upload:

```bash
arduino-cli upload -v -p <COMPort> --fqbn esp32:esp32:esp32thing PropaneScale/
```

VS Code tasks for compile, clean rebuild, upload, and serial monitor are provided in `.vscode/tasks.json`.

## Configuration

Edit `PropaneScale/config.h` to change defaults before flashing:

| Constant | Default | Description |
| -------- | ------- | ----------- |
| `CLK_PIN` | 4 | HX711 clock pin |
| `DOUT_PIN` | 16 | HX711 data pin |
| `BAUD` | 115200 | Serial baud rate |
| `DEF_CALIBRATION_FACTOR` | -10422.95 | Initial HX711 scale factor |
| `CAL_KNOWN_WEIGHT_LBS` | 36.8 | Reference weight for automatic calibration (lbs) |
| `DEF_TANK_TARE` | 23.5 | Empty tank tare weight (lbs) — 30 lb tank default |
| `MAX_PROPANE_LBS` | 24.0 | Maximum legal propane fill (lbs) — 80% of 30 lb tank capacity |
| `PLATEN_TARE` | 0.33125 | Platform platen tare weight (lbs) |

Commented-out presets for 20 lb and 50 lb reference weights are also provided.

## EEPROM Persistence

Three values are stored in ESP32 EEPROM across power cycles:

| Value | Magic | Byte addresses |
| ----- | ----- | -------------- |
| Calibration factor | `CAL1` | 0–7 |
| Tank tare | `TARE` | 8–15 |
| Max propane weight | `MAXP` | 16–23 |

Each record uses a 4-byte magic marker followed by a 4-byte float. If the magic is absent, the compiled-in default is written to EEPROM on first boot.

## Serial Command Interface

Connect at **115200 baud**. On startup the device prompts you to confirm the scale is empty before taring.

| Command | Description |
| ------- | ----------- |
| `a` | Automatic calibration — tares empty scale, detects placed weight, computes factor |
| `e` | Display all saved EEPROM values with validity status |
| `l` | Display one propane reading |
| `m` | Manual calibration — adjust factor with `+`/`-` keys, save with `q` |
| `p` | Set propane tank tare weight (enter value + Enter, then `s` to save) |
| `w` | Set maximum legal propane weight (enter value + Enter to save) |
| `z` | Re-zero scale (prompts to remove all weight first) |

### Propane Reading Output Format

```text
Scale load: 39.5 lbs, Calculated propane: 15.7 lbs, Level: 65.4%
```

- **Scale load** — raw weight reported by HX711 after calibration
- **Calculated propane** — scale load minus tank tare and platen tare
- **Level** — propane weight as a percentage of max legal fill

## Calibration Workflow

### Automatic (recommended)

1. Send `a`
2. Confirm scale is empty (`y`)
3. Place the known reference weight defined by `CAL_KNOWN_WEIGHT_LBS`
4. Factor is computed and saved automatically

### Manual

1. Send `m`
2. Confirm scale is empty (`y`)
3. Place a known weight
4. Adjust with `+` / `-` until the reading matches the known weight (step halves on direction reversal)
5. Send `q` to accept and save

## Tank Tare Reference Values

| Tank size | Typical empty tare |
| --------- | ------------------ |
| 20 lb | 16–18 lbs |
| 30 lb | 23–26 lbs |

Set the tare for the specific tank using the `p` command, then save to EEPROM.

## Project Structure

```text
PropaneScale/
  PropaneScale.ino   Main sketch
  config.h           Pin assignments, calibration constants, EEPROM layout
Documentation/       Fritzing schematics and component data sheets
Examples/            Reference sketches (HX711 examples, IoT hello-world)
3D/STL/              3D-printable enclosure parts
```

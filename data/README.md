# LittleFS data upload — PropaneScale

Place the `data/` folder next to `PropaneScale.ino` (already present). These steps show how to compile the sketch and upload the LittleFS image using `arduino-cli`, `mklittlefs`, and `esptool`.

1) Compile the sketch

Run from the sketch folder (the folder that contains `PropaneScale.ino` and this `data/` directory):

```bash
arduino-cli compile --fqbn esp32:esp32:esp32thing PropaneScale
```

2) Create a LittleFS image

- Install `mklittlefs` / `mklittlefs.exe` (available from LittleFS builds or your distribution).
- Create an image from the `data/` folder. Example:

```bash
mklittlefs -c data -p 256 -s 0x200000 -o littlefs.bin
```

Adjust `-s` (total size) to match the LittleFS partition size in your board's partition scheme.

3) Upload the LittleFS image with `esptool`

Find the correct LittleFS partition offset for your board/partition scheme (DO NOT guess). A common offset example is `0x110000` but verify in your board package/partition table.

Windows (example using Arduino's bundled esptool):

```powershell
& "$env:LOCALAPPDATA\Arduino15\packages\esp32\tools\esptool_py\5.1.0\esptool.exe" --chip esp32 --port COM3 --baud 921600 write_flash -z 0x110000 littlefs.bin
```

Linux/macOS example:

```bash
esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600 write_flash -z 0x110000 littlefs.bin
```

4) Verify on the device

- Open the serial monitor at the sketch baud (e.g. `115200`) and look for a LittleFS mount message:

```bash
arduino-cli monitor -p COM3 -b esp32:esp32:esp32thing
```

- The firmware prints `LittleFS mounted` on successful mount (and will serve `/index.html`, `/styles.css`, `/app.js`).

Alternative methods

- Arduino IDE: install the **ESP32 LittleFS Data Upload** (ESP32FS) tool and use Tools → ESP32 LittleFS Data Upload.
- PlatformIO: use `pio run --target uploadfs -e <environment>`.

Notes

- Always run these commands from the sketch root where `PropaneScale.ino` and `data/` live.
- Double-check partition offsets and sizes before writing with `esptool` to avoid overwriting firmware.

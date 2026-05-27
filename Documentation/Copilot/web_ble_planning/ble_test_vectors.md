# BLE Test Vectors — PropaneScale

This file provides example hex test vectors for each characteristic defined in `firmware/ble_service_spec.md`. Use these with nRF Connect (or similar BLE tool) and as validation inputs for the Web Bluetooth client.

## How to use
- Open nRF Connect (or any BLE explorer).
- Connect to a PropaneScale device advertising service UUID `0000FEED-0000-1000-8000-00805F9B34FB`.
- For notify characteristics (Weight, Raw ADC, Battery), use the "Enable notifications" action and observe incoming packets; compare against these vectors.
- For write characteristics (Tare, Calibrate, Sampling Rate), use the "Write" feature and paste the hex bytes below.

---

## 1) Weight (Notify / Read) — `0000BE01-0000-1000-8000-00805F9B34FB`
Format (9 bytes):
- `uint8 version` (1)
- `int32 weight_mg` (LE)
- `uint16 seq` (LE)
- `int16 reserved`

Example: weight = 1,000 g → weight_mg = 1,000,000 (0x0F4240)
- Payload (hex): `01 40 42 0F 00 01 00 00 00`
  - Breakdown: `01`=version, `40 42 0F 00`=1,000,000 LE, `01 00`=seq=1, `00 00`=reserved

## 2) Tare Command (Write Without Response) — `0000BE02-...`
Format (1 byte):
- `uint8 cmd` (0x01 = TARE_NOW)

Example write (hex): `01`

## 3) Calibrate Command (Write) — `0000BE03-...`
Format (6 bytes):
- `uint8 cmd` (0x01 = CAL_START_SINGLE_WEIGHT)
- `uint32 mass_g` (LE)
- `uint8 options`

Example: calibrate with 500 g
- Payload (hex): `01 F4 01 00 00 00`
  - `01`=cmd, `F4 01 00 00`=500 LE, `00`=options

## 4) Raw ADC (Notify / Read) — `0000BE04-...`
Format (9 bytes):
- `uint8 version` (1)
- `int32 adc_counts` (LE)
- `uint16 seq` (LE)
- `int16 reserved`

Example: adc_counts = 123,456 (0x01E240)
- Payload (hex): `01 40 E2 01 00 01 00 00 00`
  - `01`=version, `40 E2 01 00`=123456 LE, `01 00`=seq=1

## 5) Sampling Rate (Read/Write) — `0000BE05-...`
Format (2 bytes):
- `uint16 interval_ms` (LE)

Example: 100 ms
- Payload (hex): `64 00`

## 6) Battery (Read / Notify) — `0000BE06-...`
Format (3 bytes):
- `uint8 version` (1)
- `uint16 battery_mv` (LE)

Example: 3700 mV
- Payload (hex): `01 74 0E`
  - `0x0E74` = 3700 → LE bytes `74 0E`

## 7) Device Info / Version (Read) — `0000BE07-...`
- UTF-8 text, up to 32 bytes: `manufacturer|model|fw_version|build`
- Example string: `Acme|PropaneScale|v1.2.0|2026-05-19`

---

## Quick Web Bluetooth parse example (weight notification)
Use this as a reference to validate notification parsing in the browser client:

```javascript
function parseWeightNotification(value) {
  // value is a DataView or ArrayBuffer from characteristicvaluechanged
  const dv = new DataView(value);
  const version = dv.getUint8(0);
  const weight_mg = dv.getInt32(1, true); // little-endian
  const seq = dv.getUint16(5, true);
  return { version, weight_mg, seq };
}
```

---

If you want, I can also generate a machine-readable JSON file of these vectors (for automated tests) and add a small Node.js script to replay write vectors via `noble` on a test rig.
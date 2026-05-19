# BLE Service Specification — PropaneScale (Firmware)

This document defines exact UUIDs, characteristic properties, and payload formats for the BLE GATT service the `PropaneScale` ESP32 firmware will expose. This is a protocol specification only — no implementation is included.

Service
- Name: PropaneScale Service (custom)
- UUID (128-bit): 0000FEED-0000-1000-8000-00805F9B34FB

Characteristic summary (all UUIDs are 128-bit within same vendor base):
- Weight (Notify, Read) — UUID: 0000BE01-0000-1000-8000-00805F9B34FB
- Tare Command (Write Without Response) — UUID: 0000BE02-0000-1000-8000-00805F9B34FB
- Calibrate Command (Write) — UUID: 0000BE03-0000-1000-8000-00805F9B34FB
- Raw ADC (Notify, Read) — UUID: 0000BE04-0000-1000-8000-00805F9B34FB
- Sampling Rate (Read/Write) — UUID: 0000BE05-0000-1000-8000-00805F9B34FB
- Battery (Read, Notify) — UUID: 0000BE06-0000-1000-8000-00805F9B34FB
- Device Info / Version (Read) — UUID: 0000BE07-0000-1000-8000-00805F9B34FB

General encoding rules
- Endianness: Little-endian for multi-byte numeric fields.
- Numeric types: use standard sizes (uint8, int16, int32, uint16, uint32, float32 as IEEE-754 where indicated).
- Versioning: each notification payload starts with `uint8 version` (currently `1`).
- Sequence counter: optional `uint16 seq` included in streaming payloads to detect loss.
- All characteristic lengths are fixed where practical to simplify parsing.

Characteristic details

1) Weight — `0000BE01-...`
- Properties: Notify, Read
- Purpose: current processed weight in grams (scaled integer) with sign support.
- Payload (9 bytes):
  - byte 0: `uint8 version` (1)
  - byte 1-4: `int32 weight_mg` — weight in milligrams (signed, little-endian)
  - byte 5-6: `uint16 seq` — packet sequence number (wraps)
  - byte 7-8: `int16 reserved` — reserved (set 0)
- Example: weight = 1234.5 g → weight_mg = 1234500 = 0x12 CA 35 00 (LE) so payload:
  - [0x01][0x64][0x1A][0x12][0x00][0x00][0x01][0x00][0x00]
  (example seq=1)

2) Tare Command — `0000BE02-...`
- Properties: Write Without Response (fast), firmware acts immediately.
- Purpose: request firmware to perform tare (zero) operation.
- Payload (1 byte):
  - byte 0: `uint8 cmd` where 0x01 = TARE_NOW
- Example: to tare, client writes `[0x01]`.
- Security: firmware must validate source and rate-limit successive TARE requests (e.g., minimum 1s between tarries).

3) Calibrate Command — `0000BE03-...`
- Properties: Write (request-response via Device Info or event)
- Purpose: initiate calibration sequence. Two modes supported.
- Payload (6 bytes):
  - byte 0: `uint8 cmd` (0x01 = CAL_START_SINGLE_WEIGHT)
  - byte 1-4: `uint32 mass_g` (mass in grams for calibration reference; little-endian)
  - byte 5: `uint8 options` (bitmask reserved)
- Example: calibrate with a 500 g weight: `[0x01][0xF4][0x01][0x00][0x00][0x00]` (500 = 0x01F4)
- Response: firmware writes a one-time Device Info/Version update or a separate calibration status event (can reuse Weight or Device Info characteristic to signal success/failure).

4) Raw ADC — `0000BE04-...`
- Properties: Notify, Read
- Purpose: transmit raw ADC counts (signed 32-bit) for debugging/advanced smoothing on client side.
- Payload (9 bytes):
  - byte 0: `uint8 version` (1)
  - byte 1-4: `int32 adc_counts` (LE)
  - byte 5-6: `uint16 seq`
  - byte 7-8: `int16 reserved`
- Note: ADC notifications can be sent at a higher rate but consider BLE throughput limits.

5) Sampling Rate — `0000BE05-...`
- Properties: Read, Write
- Purpose: allow client to read or set the sampling interval used by firmware for notifications.
- Payload (2 bytes):
  - `uint16 interval_ms` (little-endian) — notification interval in milliseconds. Range: 20..2000 (firmware enforces limits).
- Example: set 100 ms interval: `[0x64 0x00]`.

6) Battery — `0000BE06-...`
- Properties: Read, Notify
- Purpose: report battery level.
- Payload (3 bytes):
  - byte 0: `uint8 version` (1)
  - byte 1-2: `uint16 battery_mv` (millivolts, LE)
- Firmware should send periodic notifications on significant changes or on client request.

7) Device Info / Version — `0000BE07-...`
- Properties: Read
- Purpose: static device information and firmware version.
- Payload (variable up to 32 bytes, UTF-8 text): `manufacturer|model|fw_version|build` (pipe-separated)
- Example: `Acme|PropaneScale|v1.2.0|2026-05-19`

Operations & behaviors
- Discovery: client filters by Service UUID `0000FEED-0000-1000-8000-00805F9B34FB` during device request.
- Notifications: Weight and Raw ADC should include `seq` to permit loss detection. Clients should apply simple smoothing/filtering for UI.
- Rate limiting: Tare write requests must be rate-limited by firmware (recommended 1 second cooldown).
- Atomicity: firmware must apply commands atomically; if calibrate is in-progress, reject new calibrate requests with a status code via Device Info or a writeable status char (not defined here).

Security recommendations (firmware-facing)
- Optionally implement a simple application-layer token flow:
  - `auth` token stored in NVS; client must write a `0xA5` + token sequence to a secure characteristic before control char writes accepted. (Left as optional — implement only if needed.)
- Validate and bound incoming values (sampling interval range, calibration mass limits).

Versioning & extensibility
- Always increment `version` byte on payload format changes. Clients must check `version` and refuse or attempt fallback if unsupported.
- Reserve characteristic UUIDs `0000BE08..0000BE0F` for future use.

Testing notes
- Provide an example test vector file for each characteristic (hex sequences) included in `Documentation/Copilot/examples` (to be created separately).
- Validate end-to-end with Web Bluetooth client and a BLE explorer app (nRF Connect) before integrating UI control flows.

Change log
- 2026-05-19: Initial spec created by Copilot (proposal).


---

File location suggestions:
- Add this spec to `firmware/ble_service_spec.md` (this file).
- Add simple example packets to `Documentation/Copilot/examples/ble_test_vectors.md` next.

If you'd like, I can now scaffold `Documentation/Copilot/examples/ble_test_vectors.md` and a minimal `web-bluetooth-client/` example. Which should I do next?
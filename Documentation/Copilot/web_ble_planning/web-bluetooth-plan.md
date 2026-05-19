# Web Bluetooth Interface — Proposal for ESP32 Scale

Reference: https://randomnerdtutorials.com/esp32-web-bluetooth/

## Purpose
Propose a phased plan for adding a Web Bluetooth client to interact with the ESP32-based `PropaneScale` project. The goal is a minimal, secure, cross-platform web UI that can connect to the device, show live weight, perform tare/calibration, and receive telemetry (battery, sampling rate, raw ADC) with low latency.

## Constraints & Notes
- Web Bluetooth requires HTTPS (or localhost). Web Bluetooth support is strongest in Chromium-based browsers on desktop and Android. iOS Safari has limited/no support.
- The ESP32 will act as a BLE peripheral exposing a custom GATT service.
- Follow guidance from the Random Nerd Tutorials article above for connection and characteristic examples.

## High-Level Milestones
- Draft API (GATT) and message formats
- Design web UI/UX and connection flows
- Implement firmware GATT service on ESP32
- Implement Web Bluetooth client (connect, read, notify, write)
- Test across browsers/devices, add reconnection logic
- Document usage and publish examples

## Proposed GATT Service & Characteristics
Custom Service UUID: `0000FEED-0000-1000-8000-00805F9B34FB` (example — choose/re-roll)

Characteristics (suggested):
- Weight (Notify, Read) — int32 or float (scaled grams) — UUID: (eg) `0000BE01-0000-1000-8000-00805F9B34FB`
- Tare Command (Write) — write-only command (0x01 = tare) — UUID: `0000BE02-...`
- Calibrate Command (Write) — write weight or command sequence — UUID: `0000BE03-...`
- Raw ADC / Loadcell (Notify, Read) — int32 — UUID: `0000BE04-...`
- Sampling Rate (Read/Write) — uint16 ms or hz — UUID: `0000BE05-...`
- Battery (Read, Notify) — uint8/uint16 percent or mV — UUID: `0000BE06-...`

Design notes:
- Use Notify for live streaming (weight, raw ADC, battery). Use Read for on-demand values.
- Keep characteristic payloads small and fixed-length for predictable parsing.
- Use little-endian numeric encodings and include a simple version byte in payloads to allow future changes.

## Web App Architecture
- Single-page web app using Web Bluetooth API (vanilla JS or lightweight framework like Preact).
- Components: Device selector & connect button, Live weight display, Controls (Tare, Calibrate, Sampling Rate), Telemetry panel (battery, raw ADC), Logs & error display.
- On connect: discover service by UUID, subscribe to notify characteristics, start a local UI update loop throttled to UI frame rate (e.g., 50–100 ms updates).
- Reconnection: remember device id (if supported), attempt reconnect on disconnect with exponential backoff.

## UX Flow
1. User opens app on HTTPS origin or localhost.
2. Click `Connect` → browser device chooser (filtered by service UUID).
3. After connect: show device name, current weight, live updates.
4. Controls: `Tare` (writes to Tare characteristic), `Calibrate` (guided dialog writing mass and running calibration), `Set Sampling Rate` (write to characteristic).
5. Status indicators: connection state, last packet timestamp, battery.
6. Error handling UI for permission denied, unsupported browser, and BLE disconnects.

## Security & Permissions
- Use HTTPS for deployed web app. For local testing, `localhost` is acceptable.
- Limit device discovery to the specific custom service UUID to reduce accidental pairing.
- Consider simple application-level auth: on first connect generate a short pairing token stored in device NVS; require token write before control commands are honored.
- Recommend implementing firmware-side rate limiting and command validation to guard against accidental/malicious repeated writes.

## Data Format & Protocol Suggestions
- All numeric values little-endian. Add `version` byte at start of notifications.
- Weight payload example (5 bytes): [version=1][int32 weight_mg]
- Tare/Command payloads: single byte command or small struct [cmd][arg...].
- Include timestamp or sequence counter occasionally for loss detection.

## Testing Matrix
- Browsers: Chrome/Edge (desktop), Chrome on Android; test fallback behavior on Firefox and Safari (report unsupported features).
- Devices: Windows 10/11 laptops with BLE, Android phones, MacBooks with BLE.
- Test scenarios: connect/disconnect, rapid updates, write commands during streaming, OTA of calibration, power/battery low notifications.

## Performance & Latency
- Aim for notify intervals <= 100 ms for a responsive UI.
- If raw ADC is noisy, perform smoothing on the client or send averaged values from firmware.
- Measure end-to-end latency (weight sample → BLE notify → UI render) and tune characteristic update intervals accordingly.

## Deliverables
- `web-bluetooth-client/` example folder with a minimal SPA (not implemented in this proposal)
- `firmware/ble_service_spec.md` documenting UUIDs and payloads
- Integration test checklist and example test scripts
- User guide: how to host (HTTPS), how to connect, browser compatibility notes

## Risks & Mitigations
- iOS lack of Web Bluetooth support: provide native fallback (e.g., small native Android app) or document limitations.
- BLE limits: throughput constrained — prefer compact payloads and server-side aggregation.
- Security: BLE pairing is limited — implement app-layer simple token exchange.

## Estimated Effort (Rough)
- Design & spec: 1–2 days
- Firmware implementation & unit tests: 2–4 days
- Web client MVP: 2–3 days
- Cross-device testing & polish: 1–2 days

## References
- Random Nerd Tutorials: ESP32 + Web Bluetooth — https://randomnerdtutorials.com/esp32-web-bluetooth/
- Web Bluetooth API docs: https://developer.mozilla.org/en-US/docs/Web/API/Web_Bluetooth_API

---

If you want, I can now create the `firmware/ble_service_spec.md` stub and an example characteristic definition file, or scaffold a minimal client sample in `Documentation/Copilot/examples` — say the word and I’ll proceed.

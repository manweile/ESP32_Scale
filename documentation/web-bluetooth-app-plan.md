# Web Bluetooth App Plan for ESP32 Propane Scale

## Summary
This document outlines a plan for a Web Bluetooth application to interact with the ESP32-based Propane Scale (ESP32_Scale). The app will allow users to connect to the scale over Bluetooth Low Energy (BLE) using the browser's Web Bluetooth API to read weight, view tank level, perform taring and calibration workflows, and update runtime parameters (known weight, max propane, tare). The goal is to provide a responsive, secure, and easy-to-use web UI that mirrors the serial command flows and workflows currently available over USB.

## Goals
- Provide a browser-based UI to monitor weight and tank level in near-real-time.
- Expose the same workflows available via serial commands: tare, known weight update, manual/automatic calibration, set max propane weight, startup tare, and level read.
- Allow updating and persisting runtime configuration values on the ESP32.
- Keep interactions simple and clearly indicate device state, errors, and required user actions.

## Constraints & Considerations
- Web Bluetooth is only available in Chromium-based browsers and requires HTTPS (or localhost for development).
- ESP32 must expose a BLE GATT server with well-defined services/characteristics to support app operations.
- Keep BLE characteristic design minimal and robust against partial writes, concurrent writes, and reconnections.
- Consider power usage and minimize continuous polling when the UI is open.

## High-level Architecture
- Browser (Web App) using Web Bluetooth API (JavaScript / TypeScript + lightweight UI framework or vanilla JS).
- ESP32 firmware exposes a BLE GATT service (e.g., 128-bit service UUID) with characteristics for telemetry, commands, and configuration.
- Optional: A small backend for hosting the web app (static hosting on GitHub Pages / Netlify) — not required for BLE connectivity, but helpful for distribution.

## BLE GATT Design (recommended)
Service UUID: 0000ee00-0000-1000-8000-00805f9b34fb (example)

Characteristics:
- Telemetry (Notify): 0000ee01-0000-1000-8000-00805f9b34fb
  - Properties: Notify
  - Payload: JSON or compact binary frame with fields: {timestamp, weight_lbs (float), level_pct (float), status_flags}
  - Frequency: on change or at a configurable interval while client subscribed

- Command (Write / WriteWithoutResponse): 0000ee02-0000-1000-8000-00805f9b34fb
  - Properties: Write, WriteWithoutResponse
  - Payload: ASCII/UTF-8 string commands or compact binary opcode + payload
  - Example commands: "TARE", "REZERO", "CAL_AUTO", "CAL_MAN:<factor>", "SET_KNOWN:<lbs>", "SET_MAX:<lbs>"

- Config Read/Write (Read, Write): 0000ee03-0000-1000-8000-00805f9b34fb
  - Properties: Read, Write
  - Payload format: JSON with persisted runtime values: {calibrationFactor, knownWeight, maxPropane, tankTare}
  - Allows the web app to load/save runtime persisted values atomically

- Control/Response (Notify / Read): 0000ee04-0000-1000-8000-00805f9b34fb
  - Properties: Notify, Read
  - Payload: responses, status messages, diagnostic strings

Security & Robustness
- No authentication at BLE level (ESP32 classic BLE); rely on physical proximity.
- Implement simple command acknowledgements and result reporting so the UI can show progress and errors.
- Ensure commands are idempotent where possible; the firmware should validate inputs and ignore invalid values.
- Timeouts and retries in the web app for operations that expect a response.

## Web App Technical Stack
- JavaScript or TypeScript (TypeScript recommended for correctness)
- Bundler: Vite or rollup (optionally plain static JS for minimal app)
- UI: Vanilla with small component approach, or lightweight framework (Svelte, React, or Preact)
- Build & deploy: GitHub Pages / Netlify / Vercel

## UX / UI Flows
1. Connect flow
   - User clicks "Connect" → browser shows BLE chooser filtered to device name/service
   - App connects, subscribes to Telemetry and Control notifications, reads Config
   - Show live weight and level tiles

2. Live monitoring
   - Show current weight (lbs), weight history (small sparkline), and level percent
   - Toggle auto-refresh interval and subscribe/unsubscribe to telemetry

3. Tare / Re-zero
   - Button "Tare" or "Re-zero" sends command. UI shows progress and acknowledgement from device.
   - For startup tare workflow, the device can push state messages via Control/Response characteristic and the UI can present prompts or show status.

4. Calibration
   - Manual: user inputs calibration factor (or known weight and measured raw) and sends CAL_MAN command
   - Automatic: send CAL_AUTO, and display progress messages from device

5. Configuration
   - Edit known weight, max propane, tank tare in a simple form and save to device (writes to Config characteristic)
   - Read current config and show last updated timestamp

6. Diagnostics & Logs
   - Display recent control messages / errors
   - Provide a way to download a short CSV or copy readings for debugging

## Data Formats
- Telemetry JSON example:
  {"ts":1685612345000, "weight":12.34, "level":23.5, "status":0}
- Config JSON example:
  {"calibrationFactor":12345.67, "knownWeight":5.0, "maxPropane":20.0, "tankTare":12.0}
- Command strings: simple uppercase text or short opcodes for readability

## Implementation Phases
Phase 1 — Prototype (1-2 days)
- Create minimal web app that connects to ESP32 BLE and subscribes to telemetry
- Display live weight and level
- Implement Tare command

Phase 2 — Workflows (2-3 days)
- Implement UI for known weight, max propane, tank tare updates
- Implement manual and automatic calibration flows and show progress
- Implement persistent config read/write

Phase 3 — Polish & Deployment (1-2 days)
- Improve UI, add small charts, logs
- Add tests and retries
- Deploy to GitHub Pages

## Testing Plan
- Test on Chromium-based browsers (Chrome, Edge) with HTTPS or localhost
- Test connection/paired behavior, reconnection handling
- Test partial writes and malformed payloads handling
- Validate calibration and tare workflows against serial interactions

## Example Web Bluetooth Code Snippets (concept)
- Connect and get characteristics:
  navigator.bluetooth.requestDevice({filters:[{services:['0000ee00-0000-1000-8000-00805f9b34fb']}], optionalServices:[]})

- Subscribe to telemetry:
  telemetryChar.startNotifications();
  telemetryChar.addEventListener('characteristicvaluechanged', handleTelemetry);

- Send command:
  const encoder = new TextEncoder();
  await commandChar.writeValue(encoder.encode('TARE'));

## Deployment
- Host as a static site (GitHub Pages / Netlify). Note: to use Web Bluetooth over HTTPS the site must be served via HTTPS.
- Provide a short README and instructions for pairing and browser support.

## Tasks / Checklist
- [ ] Define BLE UUIDs in firmware and update ESP32 firmware to expose GATT service
- [ ] Implement telemetry and control characteristics on the ESP32
- [ ] Create web app skeleton and connect flow
- [ ] Implement UI components for monitoring, calibration, and config
- [ ] Add error handling, timeouts, and retry logic
- [ ] Write README and user instructions
- [ ] Deploy static site

## Next Steps (for me)
1. Decide on characteristic payload formats (JSON vs binary) and finalize UUIDs.
2. Add BLE GATT service to ESP32 firmware (brief patch to repository in /src or main sketch to include BLE server code).
3. Create a documentation page and a minimal Web Bluetooth prototype in a new `web` folder.

---

This file was added to the repository to document the plan for a Web Bluetooth application for the ESP32 Propane Scale.

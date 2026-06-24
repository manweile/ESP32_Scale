# Asus Tab A — Web Bluetooth Test Checklist

Purpose: quick, repeatable test steps to validate Web Bluetooth behavior on an Asus Tab A (Android) using Chrome and Firefox.

Prerequisites
- ESP32 running a test firmware exposing service UUID `0000FEED-0000-1000-8000-00805F9B34FB` (or update to match your firmware).
- `nRF Connect` (Android) installed for BLE inspection.
- Test web app hosted over HTTPS (GitHub Pages or local HTTPS). For quick local tests, use a device-hosted `localhost` or a desktop-HTTPS server accessible by the tablet.

Checklist
1. Browser capability
   - Open Chrome on the tablet, navigate to `about:blank`, open DevTools (via remote debugging) or use a page with a console.
   - Run in console:
     ```javascript
     console.log('navigator.bluetooth' in window, navigator.bluetooth !== undefined);
     ```
   - Expect: `true true` in Chrome. If false, try Firefox; if Firefox is false, prefer Chrome.

2. BLE advertisement check (nRF Connect)
   - Open `nRF Connect`, scan, and confirm the ESP32 advertises the expected Service UUID.
   - Inspect advertised characteristics and descriptors for presence of expected UUIDs.

3. Connect via browser (Chrome recommended)
   - Open the hosted Web BLE test page (HTTPS) on the tablet.
   - Click `Connect`, select the device in the chooser (filtered by service UUID if implemented).
   - Verify UI shows `Connected` and the device name.

4. Notifications & streaming
   - Enable notifications for Weight and/or Raw ADC; observe updates.
   - Target update interval: ~100 ms. Verify UI updates at expected rate; measure jitter and latency qualitatively.

5. Control commands
   - Press `Tare` → ensure device acknowledges (weight resets to near zero) and no error is thrown.
   - Send `Calibrate` with a known mass and verify a calibration success path (firmware response or device info change).

6. Disconnect and reconnect
   - Test clean disconnect, and automatic reconnect behavior if implemented.
   - Verify sequence numbers (if present) continue or reset meaningfully.

7. Cross-browser check (optional)
   - Repeat steps 3–6 in Firefox on the tablet and note any differences (API availability, chooser behavior, UUID discovery quirks). Record required flags or version numbers if Firefox needed special config.

8. Record results
   - Note browser name & version, Android version, firmware build, and any flags set.
   - Save logs/screenshots and file them under `Documentation/Copilot/web_ble_planning/logs/`.

Notes
- If the tablet cannot reach an HTTPS host you control, use GitHub Pages for easy HTTPS hosting of the test page.
- If Firefox requires `experimental-web-platform-features` or similar flags, document steps to enable these flags for future reproductions.

Reference
- See `Documentation/Copilot/examples/ble_test_vectors.md` for test payloads and expected hex sequences.

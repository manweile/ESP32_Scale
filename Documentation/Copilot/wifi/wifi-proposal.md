# WiFi Refactor Proposal for PropaneScale

Date: 2026-05-27

Target branch: `wifi` (already created/pushed by you)

Purpose
- Propose a minimal, low-risk refactor that extracts WiFi, HTTP server, and mDNS responsibilities out of example and sketch files into a dedicated `wifi` module. The goal is separation of concerns, easier maintenance, and a foundation for future improvements (non-blocking handlers, runtime credential storage, captive portal, AsyncWebServer).

Context
- The current example sketch [Examples/mine/copilot_scale.ino] inlines WiFi, WebServer, and HTTP route handlers that directly access global scale state and perform blocking sampling/calibration. `PropaneScale` contains persistent storage helpers (`src/eeprom_store.*`) and a back-up BLE implementation at `backup/web_ble/*`.

Goals
1. Isolate networking code from scale logic.
2. Avoid blocking long-running scale operations inside request handlers.
3. Provide a stable API for telemetry and control endpoints that core code can implement or enqueue work against.
4. Leave room for opt-in improvements (AsyncWebServer, captive portal, NVS-based credential storage).

User access constraints
- Primary client: Android tablet (prefer Firefox; Chrome available).
- Must not assume cell service or home WiFi availability; device should support device-hosted WiFi (ESP32 AP) as a primary access mode.
- It's acceptable that the mobile device connects to the ESP32-generated WiFi network for configuration and everyday UX.

Recommended approach (minimal-first)
1. Create a `wifi` module inside the `PropaneScale/src/` directory:
   - `PropaneScale/src/wifi.h`
   - `PropaneScale/src/wifi.cpp`
2. API surface (C-style for minimal friction):
   - `void wifi_init();`            // called from `setup()`
   - `void wifi_tick();`            // called from `loop()`
   - `void wifi_register_callbacks(const WifiCallbacks* cb);` // register accessors/actions
   - `IPAddress wifi_get_ip();`     // query current IP
   - Optional: `wifi_start_ap_mode()` / `wifi_stop_ap_mode()`
3. Move `setupWiFi()`, `setupWebServer()`, route registrations, and MDNS setup into `wifi.cpp`. Keep handlers thin and only call callbacks or enqueue operations.
4. Add a small `WifiCallbacks` struct with function pointers that core code implements for telemetry and control, e.g.:
   - `String (*get_telemetry_json)();`
   - `void (*enqueue_tare)();`
   - `void (*enqueue_calibrate)(float knownWeight);`
   - `void (*save_calibration)();`
5. Update example sketches (`Examples/mine/copilot_scale.ino`) to call `wifi_register_callbacks()` and use `wifi_init()`/`wifi_tick()` instead of inlined server code.

Notes for tablet UX and offline operation
- Design the web UI to work locally over the ESP32 AP (for example at `http://192.168.4.1`) and avoid reliance on external CDNs or web services so Firefox on Android can load the full UI while connected to the device-only hotspot.
- Web Bluetooth is inconsistently supported in Android browsers (Firefox does not support Web Bluetooth; Chrome has better support but requires secure contexts). Therefore design the UI so core functionality (telemetry, tare, calibrate) works via plain HTTP REST endpoints and does not require Web Bluetooth. If Web Bluetooth features are desired later, document that Chrome may be required and that HTTPS or origin allowances could be necessary.
- Provide a simple captive-portal or AP landing page for initial credential entry (if migrating to STA mode is implemented). For the minimal-first iteration, expose an AP mode with static HTTP endpoints and a clear IP to visit from the tablet.
- Test the UI on Firefox for Android first; use Chrome only for advanced features that require Web Bluetooth.

Files to change (first PR)
- `Examples/mine/copilot_scale.ino` — remove/replace inlined WiFi and server code with `wifi` calls.
- `PropaneScale/src/wifi.h` & `PropaneScale/src/wifi.cpp` — new module.
- `PropaneScale/config.h` — optionally add WiFi defaults or reference a new `wifi_config.h`.
- `PropaneScale/src/*` — add thin accessor functions if handlers currently touch globals (e.g., `scale_get_weight_json()`, `scale_enqueue_tare()`)

Follow-up improvements (future PRs)
- Replace `WebServer` with `AsyncWebServer` for non-blocking serving.
- Add runtime credential management (NVS/Preferences or LittleFS), with an AP/captive portal flow for setup.
- Integrate BLE implementation from `backup/web_ble/` into `PropaneScale/src/web_ble.*` if desired.

Risks and mitigations
- Blocking handlers: move long ops to enqueue model and return immediate responses. Add status endpoint to poll progress.
- Tight coupling to globals: introduce thin accessors and small work-queue APIs before moving handler logic.
- Build order / Arduino sketch rules: keep new `.cpp`/`.h` under `PropaneScale/src/` so Arduino CLI compiles them. Verify with existing build tasks.
- Dependency changes: document any new libraries (e.g., `ESPAsyncWebServer`) and keep that as a follow-up.

Verification (acceptance criteria)
- Sketches compile on branch `wifi` with new files present.
- Web endpoints return telemetry and accept control requests that enqueue actions.
- Running a long operation (calibration sampling) does not block the server from responding to simple queries.
- Device can boot in STA with compile-time credentials (initially unchanged) and serve REST endpoints.
- Verify that the UI loads correctly when the tablet is connected directly to the ESP32 AP (no upstream internet).
- Confirm basic flows (view telemetry, tare, start calibration) work in Firefox on Android while connected to the ESP32 AP. Use Chrome only to verify any optional Web Bluetooth flows later.

Next steps I can take (if you want me to implement minimally):
- Add `PropaneScale/src/wifi.h` + `PropaneScale/src/wifi.cpp` stubs and wire `Examples/mine/copilot_scale.ino` to call `wifi_init()`/`wifi_tick()` with callbacks. No behavioral changes, just extraction and compilation fixes. (Will not change runtime behavior beyond where code is located.)

---
References
- Example with inlined WiFi & handlers: `Examples/mine/copilot_scale.ino`
- EEPROM helpers: `PropaneScale/src/eeprom_store.*`
- Backup BLE: `backup/web_ble/*`

Signed-off-by: Copilot (draft proposal)

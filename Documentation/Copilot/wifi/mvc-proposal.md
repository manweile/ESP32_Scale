## Plan: MVC refactor for WiFi subsystem

TL;DR - Propose an incremental Model–View–Controller refactor of the PropaneScale WiFi subsystem so responsibilities are explicit, testable, and easier to evolve. Keep the cooperative, loop-driven runtime model; avoid large rewrites of non-blocking workflows. Start with low-risk view and adapter changes, then encapsulate model state and finally tighten controller interfaces.

**Steps**
1. Extract View assets: move HTML/JS in [PropaneScale/src/web_root.h](PropaneScale/src/web_root.h) into a static resource (e.g., data/www/index.html) and make `handleRoot()` serve that file. *(low-risk)*
2. Formalize WiFi→App adapter: define a `WifiAdapter`/`WifiCallbacks` interface header that expresses the callbacks used by the HTTP handlers and implement it in [PropaneScale/src/wifi_bridge.cpp](PropaneScale/src/wifi_bridge.cpp). Replace direct `extern` use in the bridge with accessor calls where practical. *depends on step 1 (compatibility)*
3. Encapsulate persisted config: wrap `eeprom_store.*` functions behind a small `ConfigStore` API (get/set/save) and change callers to use it. Keep underlying EEPROM calls intact initially. *parallel with step 2*
4. Introduce Model API for scale and calibration: create a `ScaleModel` abstraction exposing `getWeight()`, `tare()`, `setCalibration()`, `getCalibration()` backed by `scale_io.*` and `commands.*`. Migrate direct global accesses (e.g., `calibrationFactor`) to this API. *depends on step 3; medium-risk*
5. Replace String-heavy telemetry: change `getTelemetry()` and HTTP responders to use a small fixed buffer or streaming JSON writer to avoid heap fragmentation. *medium-risk*
6. Tighten Controller boundaries: refactor `workflows/*` to accept explicit model interfaces instead of reading globals. Do this in small parts: pick one workflow (e.g., level_workflow) and migrate it first. Add `tick()`/`start()`/`cancel()` methods to the controller interfaces.
7. Integration and testing: compile after each step, run device smoke tests (AP start, Basic endpoints, level start/stop), and manual check of EEPROM persistence.
8. Cleanup: remove remaining `extern` globals, update headers, and add documentation.

**Relevant files**
- [PropaneScale/src/wifi.cpp](PropaneScale/src/wifi.cpp) — route registration, HTTP handlers (View responsibilities to keep minimal)
- [PropaneScale/src/web_root.h](PropaneScale/src/web_root.h) — static UI assets (move to data/www/)
- [PropaneScale/src/wifi_bridge.cpp](PropaneScale/src/wifi_bridge.cpp) — current adapter between HTTP and app logic (convert to adapter implementation)
- [PropaneScale/src/eeprom_store.cpp](PropaneScale/src/eeprom_store.cpp) — persisted config (wrap behind ConfigStore)
- [PropaneScale/src/scale_io.cpp](PropaneScale/src/scale_io.cpp) — low-level HX711 interactions (Model implementation)
- [PropaneScale/src/workflows/level_workflow.cpp](PropaneScale/src/workflows/level_workflow.cpp) — Controller candidate (refactor to accept Model interfaces)
- [PropaneScale/src/parsing_utils.cpp](PropaneScale/src/parsing_utils.cpp) — utility functions (no change, can be reused)

**Verification**
1. Unit/local tests: validate parsing utilities and ConfigStore getters/setters on host (where possible) or via a small test harness.
2. Compile on device after each step using the existing tasks (`ESP32 Thing Quick Compile`) and run smoke checks.
3. Manual runtime checks: AP mode starts, `GET /api/telemetry` returns expected fields, level workflow start/stop via UI works, and EEPROM retains saved calibration after soft reset.
4. Memory and blocking checks: verify heap usage after telemetry changes, and ensure `tick*` functions keep running without regressions.

**Decisions & Assumptions**
- Keep C++/Arduino runtime and loop-driven cooperative model; do not introduce RTOS tasks.
- Preserve existing public HTTP endpoints for backward compatibility during incremental migration.
- Avoid large-scale rewrite of `workflows/*` until Model and Adapter boundaries are stable.
- Prefer non-allocating telemetry responses to reduce heap fragmentation.

**Further Considerations**
1. Option: Replace `String` with `StaticJsonDocument` (ArduinoJson) or streaming JSON to reduce allocations. Evaluate binary size vs benefits.
2. Migration plan: perform changes in small PR-sized commits with device smoke tests. Start with `web_root` extraction and `wifi_bridge` adapter.
3. Testing: add a minimal test harness that can run on host for parsing and config store logic to reduce device iteration time.

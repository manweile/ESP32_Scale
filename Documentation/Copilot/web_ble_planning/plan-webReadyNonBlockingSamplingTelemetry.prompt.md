**Plan: Web-Ready Non-blocking Sampling & Telemetry**

TL;DR - Replace blocking HX711 sampling with a non-blocking start/poll/get sampler, adapt workflows to poll from `loop()`/ticks, and introduce a telemetry wrapper that forwards structured one-line messages to both Serial (for debugging) and a web telemetry buffer/WebSocket. This preserves the serial monitor while enabling a web interface.

**Steps**
1. Implement non-blocking sampler in `src/scale_io.h` / `src/scale_io.cpp`: add `startSampleBatch(n)`, `pollSample()`, `isSampleDone()`, `getSampleResult()`; use `is_ready()` and read one sample per `pollSample()` call. (*depends on step 2*)
2. Refactor workflows to use the new sampler: change callers in `src/workflows/*` (calibration_workflow.cpp, level_workflow.cpp, startup_tare_workflow.cpp) to `startSampleBatch()` and poll from their `tick*()` functions instead of calling `readAveragedUnits()` directly.
3. Add `telemetry_write()` helper and small ring buffer in `src/scale_io.cpp` (or new `telemetry.cpp`) that enqueues one-line JSON/CSV lines and conditionally forwards human-readable messages to Serial when a debug flag is enabled; use `Serial.availableForWrite()` checks before immediate writes.
4. Add lightweight web telemetry endpoints: a JSON HTTP endpoint for last N samples and an `AsyncWebSocket` push path (reuse code from the example).
5. Throttle and coalesce telemetry emissions (e.g., publish rate ~1 Hz) and expose runtime config.
6. Validation: measure loop latency, verify Serial output order and content, verify web telemetry matches Serial, and verify workflows produce consistent results.

**Relevant files**
- [PropaneScale/PropaneScale.ino](PropaneScale/PropaneScale.ino) — adapt main loop / tick calls.
- [PropaneScale/src/scale_io.h](src/scale_io.h) — declare new sampler API.
- [PropaneScale/src/scale_io.cpp](src/scale_io.cpp) — refactor blocking `readAveragedUnits()`.
- [PropaneScale/src/workflows/calibration_workflow.cpp](src/workflows/calibration_workflow.cpp) — update to poll-based sampling.
- [PropaneScale/src/workflows/level_workflow.cpp](src/workflows/level_workflow.cpp) — update to poll-based sampling.
- [PropaneScale/src/workflows/startup_tare_workflow.cpp](src/workflows/startup_tare_workflow.cpp) — update to poll-based sampling.
- [PropaneScale/src/commands.cpp](src/commands.cpp) and [PropaneScale/src/app_startup.cpp](src/app_startup.cpp) — route prints through telemetry helper.
- [Examples/mine/copilot_scale.ino](Examples/mine/copilot_scale.ino) — webserver/WebSocket reference.

**Verification**
- Measure `loop()` timing to confirm blocking removed and responsiveness improved.
- Confirm Serial monitor still shows startup, prompts, and important events in order.
- Confirm web endpoint / WebSocket receive structured telemetry matching Serial.
- Confirm calibration/level results remain within acceptable tolerances.

**Decisions & Assumptions**
- Keep Serial monitor enabled by default; telemetry will forward human-readable messages conditionally.
- Use one-line JSON (recommended) or CSV for telemetry lines.
- Do not change baud or strip Serial in compile.

**Further Considerations**
1. JSON vs CSV? I recommend JSON for extensibility.
2. Use `AsyncWebSocket` to avoid blocking.
3. I can implement the non-blocking sampler and update `src/workflows/level_workflow.cpp` as a proof-of-concept — shall I proceed with that patch?

**Mobile & Offline Considerations**

- Local-first connectivity: assume the phone may have limited or no cellular coverage. Design the web interface to work entirely over a local Wi-Fi connection between the phone and the ESP32 (station or AP mode) with no cloud dependency.
- Dual connectivity modes: support both STA (ESP32 joins existing Wi-Fi) and AP mode (ESP32 hosts) with a simple captive-portal or QR-code setup flow to simplify pairing on a phone.
- Minimal, low-bandwidth UI: serve a very small PWA-style frontend (tiny HTML/CSS/vanilla JS) to minimize download size and loading time on mobile data or when caching is required.
- Offline-capable shell: make the frontend cache the UI shell and static assets (Service Worker) so the app loads from local cache even if cellular is unavailable; keep cached assets small to fit ESP32 storage limits.
- Resilient telemetry transport: use WebSocket with automatic reconnect and a simple HTTP polling fallback; keep payloads compact (one-line JSON, delta updates) and allow the client to request coarser update intervals when bandwidth is constrained.
- Coalesced & delta updates: publish only changed fields or compact deltas rather than full verbose messages; this reduces airtime and keeps mobile responsiveness higher.
- Explicit user controls: provide UI toggles for publish frequency, verbose vs compact output, and an option to mirror every telemetry line to Serial or to only show critical events on Serial.
- UX feedback: show clear connection status, last-updated timestamp, and allow the user to switch to AP/setup mode from the UI (or via a hardware button) when no network is available.
- Security & privacy: keep authentication lightweight and local (short-lived tokens or device-paired keys) to avoid relying on external identity providers when offline.
- Testing matrix: validate the full experience on iOS Safari and Android Chrome in airplane mode and in local Wi-Fi-only scenarios (AP mode and STA with no upstream internet).

These mobile-specific constraints reinforce earlier choices: prefer one-line JSON telemetry (item 12), non-blocking sampling (item 1), and a small `telemetry_write()` that can both mirror to Serial and feed the web buffer with minimal bandwidth.

Update 2: Priority (High → Low)

High
- Item 1 — Split blocking measurement from async acquisition: Recommended / High priority. Implement a non-blocking HX711 sampler (`startSampleBatch`, `pollSample`, `isSampleDone`, `getSampleResult`) and update workflows to poll once-per-loop tick.

High‑Medium
- Item 12 — Consolidate per-loop status into a single structured line: Deferred / Recommended for web phase. Standardize on one-line JSON for both Serial and WebSocket.

Medium
- Item 2 — Replace switch-to-string with lookup tables: Worth doing when editing diagnostic prints.
- Item 4 — Adaptive thresholds (hysteresis + debounce): Useful after sampler refactor.
- Item 5 — Fast/slow sampling profiles: Define after non-blocking sampler exists.

Medium‑Low
- Item 8 — Move high-frequency math to integer forms: Consider after profiling.

Low
- Item 6 — Cache rarely changing computed values: Optional micro-optimizations.

Dropped / Not applicable
- Item 7 — Minimize EEPROM commits: Dropped (user-driven writes).
- Item 9 — Baud change: Not applicable (ROM/boot constraints).
- Item 10 — Tick output rate-limiting: Dropped (no net benefit for current flow).

Next immediate action: implement Item 1 (non-blocking sampler) and add `telemetry_write()` to emit one-line JSON to Serial and the web buffer.

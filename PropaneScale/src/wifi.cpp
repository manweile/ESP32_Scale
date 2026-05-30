/**
 * @file wifi.cpp
 * @author Gerald Manweiler
 *
 * @brief Implements the WiFi module for the PropaneScale project.
 *
 * @details This module sets up an ESP32-hosted WiFi access point and HTTP server to provide a web interface for monitoring telemetry and triggering actions like tare and calibration. It defines the HTTP handlers for the API endpoints and bridges them to the core workflow functions via a struct of function pointers.
 *
 * @version 0.1
 * @date 2026-05-27
 * @copyright Copyright (c) 2026 Gerald Manweiler
 */

// Standard library headers
#include <Arduino.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFi.h>

// Local library headers
#include "config.h"
#include "web_root.h"
#include "wifi.h"
#include "src/eeprom_store.h"
#include "src/scale_io.h"
#include "src/workflows/level_workflow.h"
#include "src/workflows/startup_tare_workflow.h"
#include "src/workflows/workflows_contexts.h"

// Global Static Constants & Variables
static bool IsApMode = false;                               /**< WiFi running in AP mode to avoid STA reconnect attempts */
static WebServer server(WEB_SERVER_PORT);                   /**< WebServer running on configured port to handle incoming HTTP requests */
static const WifiCallbacks* CALLBACKS = nullptr;            /**< Registered WifiCallbacks for bridging HTTP handlers to core workflows */

// Externally declared UI variables
extern String LastStartupReport;                            /**< Last human-readable report produced by the most recent startup tare attempt. */
extern String LastStartupPrompt;                            /**< Last human-readable prompt produced by the most recent startup tare attempt. */

// Definitions for HTTP handlers

void handleAppStatus()
{
  // Return structured JSON with EEPROM and calibration fields requested by UI
  extern bool eepromReady;
  extern float calibrationFactor;
  extern float knownWeight;
  extern float maxPropane;
  extern float tankTare;

  // Attempt to read persisted runtime tare offset from EEPROM
  float savedRuntimeOffset = 0.0f;
  bool hasRuntimeOffset = loadFromEeprom(savedRuntimeOffset,
                                        HX711_OFFSET_EEPROM_MAGIC_ADDR,
                                        HX711_OFFSET_EEPROM_MAGIC,
                                        HX711_OFFSET_EEPROM_VALUE_ADDR);

  String payload = "{";
  payload += "\"EEPROM ready\":" + String(eepromReady ? "true" : "false") + ",";
  payload += "\"calibrationFactor\":" + String(calibrationFactor, 6) + ",";
  payload += "\"knownWeight\":" + String(knownWeight, 3) + ",";
  payload += "\"maxPropane\":" + String(maxPropane, 3) + ",";
  payload += "\"tankTare\":" + String(tankTare, 3) + ",";

  payload += "\"runtime tare offset\":";
  if (hasRuntimeOffset) {
    // Provide runtime offset as a number (counts)
    // Print without decimal fraction for readability
    payload += String(static_cast<long>(savedRuntimeOffset));
  } else {
    payload += "null";
  }

  payload += "}";

  server.send(200, "application/json", payload);
}

void handleCalibrate()
{
  float weight = 0.0f;

  if (server.hasArg("weight")) {
    weight = server.arg("weight").toFloat();

    if (CALLBACKS && CALLBACKS->enqueue_calibrate) {
      CALLBACKS->enqueue_calibrate(weight);
    }

    server.send(200, "text/plain", "ok");
  } else {
    server.send(400, "text/plain", "missing weight param");
  }
}

void handleLevelAck()
{
  // Clear server-side stored prompt/report so browser won't see stale values
  LastLevelReport = String("");
  LastLevelPrompt = String("");
  server.send(200, "application/json", "{\"success\":true}");
}

void handleLevelCancel()
{
  // Cancel any in-progress level workflow
  if (levelCtx.state != LevelState::IDLE) {
    cancelThresholdDetect();
    levelCtx.avgPending = false;
    levelCtx.thresholdPending = false;
    levelCtx.probePending = false;
    levelCtx.state = LevelState::IDLE;
    LastLevelPrompt = String("Level read cancelled.");
    LastLevelReport = String("Level read cancelled.");
  }

  server.send(200, "application/json", "{\"success\":true,\"active\":false,\"message\":\"Level read cancelled\"}");
}

void handleLevelStart()
{
  // Start the level read workflow; liquidLevel() will guard against concurrent runs.
  liquidLevel();
  server.send(200, "text/plain", "ok");
}

void handleLevelStatus()
{
  // Return a small JSON object with the workflow state and last report.
  const char* stateName = "IDLE";

  switch (levelCtx.state) {
  case LevelState::IDLE:
    stateName = "IDLE";
    break;

  case LevelState::WAIT_LOAD:
    stateName = "WAIT_LOAD";
    break;

  case LevelState::SETTLING:
    stateName = "SETTLING";
    break;

  case LevelState::READING:
    stateName = "READING";
    break;
  }

  String payload = "{";
  payload += "\"state\":\"" + String(stateName) + "\",";
  payload += "\"avgPending\":" + String(levelCtx.avgPending ? "true" : "false") + ",";
  payload += "\"report\":";

  if (LastLevelReport.length() == 0) {
    payload += "null";
  } else {
    payload += "\"" + LastLevelReport + "\"";
  }

  payload += ",";
  payload += "\"prompt\":";

  if (LastLevelPrompt.length() == 0) {
    payload += "null";
  } else {
    payload += "\"" + LastLevelPrompt + "\"";
  }

  payload += "}";

  server.send(200, "application/json", payload);
}

void handleRoot()
{
  server.send(200, "text/html", ROOT_PAGE);
}

void handleSave()
{
  if (CALLBACKS && CALLBACKS->save_calibration) {
    CALLBACKS->save_calibration();
  }

  server.send(200, "text/plain", "ok");
}

void handleStartupAck()
{
  // Acknowledge startup report so UI doesn't show stale results on next poll
  LastStartupReport = String("");
  LastStartupPrompt = String("");
  server.send(200, "application/json", "{\"success\":true}\n");
}

void handleStartupCancel()
{
  // Cancel any in-progress startup tare workflow
  if (tareCtx.state != TareState::IDLE) {
    tareCtx.baselinePending = false;
    tareCtx.probePending = false;
    tareCtx.state = TareState::SKIP;
    LastStartupReport = String("Startup tare cancelled.");
  }

  server.send(200, "application/json", "{\"success\":true}\n");
}

void handleStartupStatus()
{
  // Provide the current startup tare state, last prompt and last report for the browser UI
  const char* stateName = "IDLE";

  switch (tareCtx.state) {
  case TareState::IDLE:
    stateName = "IDLE";
    break;
  case TareState::WAIT_STABLE:
    stateName = "WAIT_STABLE";
    break;
  case TareState::TARE:
    stateName = "TARE";
    break;
  case TareState::SKIP:
    stateName = "SKIP";
    break;
  }

  String payload = "{";
  payload += "\"state\":\"" + String(stateName) + "\",";
  payload += "\"prompt\":";

  if (LastStartupPrompt.length() == 0) {
    payload += "null";
  } else {
    payload += "\"" + LastStartupPrompt + "\"";
  }

  payload += ",\"report\":";

  if (LastStartupReport.length() == 0) {
    payload += "null";
  } else {
    payload += "\"" + LastStartupReport + "\"";
  }

  payload += "}";

  server.send(200, "application/json", payload);
}

void handleStartupSkip()
{
  if (CALLBACKS && CALLBACKS->skip_startup_tare) {
    CALLBACKS->skip_startup_tare();
  }

  server.send(200, "application/json", "{\"success\":true}\n");
}

void handleStartupForce()
{
  if (CALLBACKS && CALLBACKS->force_startup_tare) {
    CALLBACKS->force_startup_tare();
  }

  server.send(200, "application/json", "{\"success\":true}\n");
}

void handleTare()
{
  if (CALLBACKS && CALLBACKS->enqueue_tare) {
    CALLBACKS->enqueue_tare();
  }

  server.send(200, "text/plain", "ok");
}

void handleTelemetry()
{
  if (CALLBACKS && CALLBACKS->get_telemetry_json) {
    String payload = CALLBACKS->get_telemetry_json();
    server.send(200, "application/json", payload);
  } else {
    server.send(204, "text/plain", "");
  }
}

bool initWifi()
{
  // Initialize serial here so only WiFi module performs console diagnostics
  Serial.begin(BAUD);
  Serial.println(F("\nInitializing WiFi..."));

  // ESP32 is prone to weird issues if the SSID is invalid (including empty) and it's easy to misconfigure at compile time
  if (WIFI_SSID[0] == '\0') {
    Serial.println(F("Warning: WIFI_SSID is empty — expected compile-time configuration"));
  }

  Serial.print(F("Attempting STA connect to: "));
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(MDNS_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // ESP32 is touchy, so need automatic reconnect and no modem sleeping
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);

 // Wait for connection with timeout, then fall back to AP mode if it fails
 // This allows the device to be used even if the configured WiFi credentials are wrong or the network is down,
 // and avoids getting stuck in a long STA reconnect loop.
  unsigned long start = millis();
  const unsigned long timeout = WIFI_TIMEOUT_MS;
  while (millis() - start < timeout) {
    if (WiFi.status() == WL_CONNECTED) break;
    delay(200);
  }

  bool started = false;

  if (WiFi.status() == WL_CONNECTED) {
    IsApMode = false;
    Serial.print(F("STA connected, IP: "));
    Serial.println(WiFi.localIP());

    if (MDNS.begin(MDNS_HOSTNAME)) {
      Serial.print(F("mDNS responder started: "));
      Serial.println(MDNS_HOSTNAME);
    }

    started = true;
  } else {
    Serial.println(F("STA connect failed, attempting AP mode"));
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);

    // Attempt to start AP (secure if password provided)
    bool apOk = false;
    if (AP_PASSWORD[0] == '\0') {
      apOk = WiFi.softAP(AP_SSID);
    } else {
      apOk = WiFi.softAP(AP_SSID, AP_PASSWORD);
    }

    if (apOk) {
      IsApMode = true;
      Serial.print(F("AP started, IP: "));
      Serial.println(WiFi.softAPIP());
      started = true;
    } else {
      Serial.println(F("Failed to start AP"));
      started = false;
    }
  }

  if (!started) {
    LastStartupReport = String("WiFi initialization failed (STA and AP both failed).");
    return false;
  }

  // Register routes and start server only if network is up
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/telemetry", HTTP_GET, handleTelemetry);
  server.on("/api/tare", HTTP_POST, handleTare);
  server.on("/api/calibrate", HTTP_POST, handleCalibrate);
  server.on("/api/save", HTTP_POST, handleSave);
  server.on("/api/level", HTTP_POST, handleLevelStart);
  server.on("/api/level/status", HTTP_GET, handleLevelStatus);
  server.on("/api/level/cancel", HTTP_POST, handleLevelCancel);
  server.on("/api/level/ack", HTTP_POST, handleLevelAck);
  server.on("/api/startup/status", HTTP_GET, handleStartupStatus);
  server.on("/api/startup/cancel", HTTP_POST, handleStartupCancel);
  server.on("/api/startup/ack", HTTP_POST, handleStartupAck);
  server.on("/api/startup/skip", HTTP_POST, handleStartupSkip);
  server.on("/api/startup/force", HTTP_POST, handleStartupForce);
  server.on("/api/app/status", HTTP_GET, handleAppStatus);

  server.begin();
  Serial.println(F("HTTP server started"));

  return true;
}

void registerCallbacks(const WifiCallbacks* cb)
{
  CALLBACKS = cb;
}

void tickWifi()
{
  server.handleClient();

  // If configured for STA mode and not running as AP, attempt a throttled reconnect when disconnected
  if (!IsApMode) {
    if (WiFi.status() != WL_CONNECTED) {
      static unsigned long lastReconnect = 0;
      unsigned long now = millis();

      if (now - lastReconnect > 15000) {
        Serial.println(F("WiFi disconnected — attempting reconnect"));
        WiFi.reconnect();
        lastReconnect = now;
      }
    }
  }

  // When running as AP, periodically print diagnostics (station count, IP, free heap)
  if (IsApMode) {
    static unsigned long lastApDiag = 0;
    unsigned long now = millis();

    if (now - lastApDiag > 60000) {
      int stations = WiFi.softAPgetStationNum();
      IPAddress ip = WiFi.softAPIP();
      Serial.print(F("AP status IP: "));
      Serial.println(ip);
      Serial.print(F("AP stations connected: "));
      Serial.println(stations);
      Serial.print(F("Free heap: "));
      Serial.println(ESP.getFreeHeap());
      lastApDiag = now;
    }
  }
}

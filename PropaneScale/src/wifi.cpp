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
#include "workflows/workflows_contexts.h"
#include "src/workflows/level_workflow.h"
#include "src/scale_io.h"

// Global Static Constants & Variables
static const WifiCallbacks* g_callbacks = nullptr;          /**< Registered WifiCallbacks for bridging HTTP handlers to core workflows */
static WebServer server(WEB_SERVER_PORT);                   /**< WebServer running on configured port to handle incoming HTTP requests */
static bool g_isApMode = false;                             /**< WiFi running in AP mode to avoid STA reconnect attempts */

// Definitions for HTTP handlers

void handleCalibrate()
{
  float weight = 0.0f;

  if (server.hasArg("weight")) {
    weight = server.arg("weight").toFloat();

    if (g_callbacks && g_callbacks->enqueue_calibrate) {
      g_callbacks->enqueue_calibrate(weight);
    }

    server.send(200, "text/plain", "ok");
  } else {
    server.send(400, "text/plain", "missing weight param");
  }
}

void handleLevelAck()
{
  // Clear server-side stored prompt/report so browser won't see stale values
  lastLevelReport = String("");
  lastLevelPrompt = String("");
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
    lastLevelPrompt = String("Level read cancelled.");
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

  if (lastLevelReport.length() == 0) {
    payload += "null";
  } else {
    payload += "\"" + lastLevelReport + "\"";
  }

  payload += ",";
  payload += "\"prompt\":";

  if (lastLevelPrompt.length() == 0) {
    payload += "null";
  } else {
    payload += "\"" + lastLevelPrompt + "\"";
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
  if (g_callbacks && g_callbacks->save_calibration) {
    g_callbacks->save_calibration();
  }

  server.send(200, "text/plain", "ok");
}


void handleTare()
{
  if (g_callbacks && g_callbacks->enqueue_tare) {
    g_callbacks->enqueue_tare();
  }

  server.send(200, "text/plain", "ok");
}

void handleTelemetry()
{
  if (g_callbacks && g_callbacks->get_telemetry_json) {
    String payload = g_callbacks->get_telemetry_json();
    server.send(200, "application/json", payload);
  } else {
    server.send(204, "text/plain", "");
  }
}


void initWifi()
{
  
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

  if (WiFi.status() == WL_CONNECTED) {
    g_isApMode = false;
    Serial.print(F("STA connected, IP: "));
    Serial.println(WiFi.localIP());

    // we want mDNS in both AP and STA modes because it's a nice-to-have for users in either mode, 
    //and it allows the same hostname to be used for both modes which is simpler and more intuitive
    if (MDNS.begin(MDNS_HOSTNAME)) {
      Serial.print(F("mDNS responder started: "));
      Serial.println(MDNS_HOSTNAME);
    }
  } else {
    Serial.println(F("STA connect failed, falling back to AP"));
    WiFi.mode(WIFI_AP);
    
    // Disable modem sleep to improve AP stability
    WiFi.setSleep(false);
    g_isApMode = true;

    // AP mode requires a different SSID 
    // AP_PASSWORD must be at least 8 chars for WPA2; if it's empty or too short, start an open AP instead.
    if (AP_PASSWORD[0] == '\0') {
      if (WiFi.softAP(AP_SSID)) {
        Serial.print(F("AP started, IP: "));
        Serial.println(WiFi.softAPIP());
      } else {
        Serial.println(F("Failed to start AP"));
      }
    } else {
      if (WiFi.softAP(AP_SSID, AP_PASSWORD)) {
        Serial.print(F("AP started (secure), IP: "));
        Serial.println(WiFi.softAPIP());
      } else {
        Serial.println(F("Failed to start AP"));
      }
    }
  }

  // Register routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/telemetry", HTTP_GET, handleTelemetry);
  server.on("/api/tare", HTTP_POST, handleTare);
  server.on("/api/calibrate", HTTP_POST, handleCalibrate);
  server.on("/api/save", HTTP_POST, handleSave);
  server.on("/api/level", HTTP_POST, handleLevelStart);
  server.on("/api/level/status", HTTP_GET, handleLevelStatus);
  server.on("/api/level/cancel", HTTP_POST, handleLevelCancel);
  server.on("/api/level/ack", HTTP_POST, handleLevelAck);

  server.begin();
  Serial.println(F("HTTP server started"));
}

void registerCallbacks(const WifiCallbacks* cb)
{
  g_callbacks = cb;
}

void tickWifi()
{
  server.handleClient();

  // If configured for STA mode and not running as AP, attempt a throttled reconnect when disconnected
  if (!g_isApMode) {
    if (WiFi.status() != WL_CONNECTED) {
      static unsigned long lastReconnect = 0;
      unsigned long now = millis();

      if (now - lastReconnect > 5000) {
        Serial.println(F("WiFi disconnected — attempting reconnect"));
        WiFi.reconnect();
        lastReconnect = now;
      }
    }
  }

  // When running as AP, periodically print diagnostics (station count, IP, free heap)
  if (g_isApMode) {
    static unsigned long lastApDiag = 0;
    unsigned long now = millis();

    if (now - lastApDiag > 5000) {
      int stations = WiFi.softAPgetStationNum();
      IPAddress ip = WiFi.softAPIP();
      Serial.print(F("AP status — IP: "));
      Serial.println(ip);
      Serial.print(F("AP stations connected: "));
      Serial.println(stations);
      Serial.print(F("Free heap: "));
      Serial.println(ESP.getFreeHeap());
      lastApDiag = now;
    }
  }
}
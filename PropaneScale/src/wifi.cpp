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
#include <WebServer.h>
#include <WiFi.h>

// Local library headers
#include "web_root.h"
#include "wifi.h"
#include "workflows/workflows_contexts.h"
#include "src/workflows/level_workflow.h"
#include "src/scale_io.h"

// forward declaration for cancel handler
void handleLevelCancel();

// Static Variables
static const WifiCallbacks* g_callbacks = nullptr;          /**< Static pointer to the registered WifiCallbacks struct instance for bridging HTTP handlers to core workflows */
static WebServer server(80);                                /**< Global instance of the WebServer running on port 80 to handle incoming HTTP requests */
static const char* DEFAULT_AP_SSID = "PropaneScale";        /**< Default SSID for the ESP32-hosted WiFi access point */

// Definitions for HTTP handlers

void initWifi()
{
  // Start device-hosted AP so tablet can connect directly
  Serial.print("\nStarting AP: ");
  Serial.println(DEFAULT_AP_SSID);
  WiFi.mode(WIFI_AP);

  if (WiFi.softAP(DEFAULT_AP_SSID)) {
    Serial.print("AP started, IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("Failed to start AP");
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

  server.begin();
  Serial.println("HTTP server started");
}

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
    case LevelState::IDLE: stateName = "IDLE"; break;
    case LevelState::WAIT_LOAD: stateName = "WAIT_LOAD"; break;
    case LevelState::SETTLING: stateName = "SETTLING"; break;
    case LevelState::READING: stateName = "READING"; break;
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

void registerCallbacks(const WifiCallbacks* cb)
{
  g_callbacks = cb;
}

void tickWifi()
{
  server.handleClient();
}
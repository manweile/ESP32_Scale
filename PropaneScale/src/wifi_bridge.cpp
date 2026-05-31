/**
 * @file wifi_bridge.cpp
 * @author Gerald Manweiler
 *
 * @brief Concrete WifiCallbacks implementation that bridges HTTP handlers to core workflows.
 *
 * @details This file defines the functions that will be called by the WiFi module's HTTP handlers,
 * and implements the logic to bridge those calls to the core workflow functions defined in the main application.
 *
 * @version 0.1
 * @date 2024-06-01
 * @copyright Copyright (c) 2024 Gerald Manweiler
 */

// Standard library headers
#include <Arduino.h>

// Third party library headers
#include "HX711.h"

// Local library headers
#include "config.h"
#include "src/eeprom_store.h"
#include "src/wifi.h"
#include "src/wifi_bridge.h"
#include "src/workflows/calibration_workflow.h"
#include "src/workflows/startup_tare_workflow.h"

// External Global State Variables and Functions
extern float calibrationFactor;
extern float knownWeight;
extern float maxPropane;
extern HX711 scale;
extern float tankTare;

// Global Callbacks
extern "C" const WifiCallbacks g_wifi_callbacks = {
  enqueueCalibrate,
  enqueueTare,
  getTelemetry,
  saveCalibration,
  skipStartupTare,
  forceStartupTare
};

// Definitions for WiFi callback implementations

void enqueueCalibrate(float known)
{
  knownWeight = known;
  automaticCalibration();
}

void enqueueTare()
{
  // reZero() is non-blocking and will progress via tickCalibration()
  reZero();
}

String getTelemetry()
{
  // Provide telemetry about calibration and saved settings. Omit a live weight
  // sample here to avoid any heap/latency issues; include runtime tare offset instead.
  String s = "{";
  s += "\"calibrationFactor\":" + String(calibrationFactor, 2);
  s += ",\"knownWeight\":" + String(knownWeight, 2);
  s += ",\"maxPropane\":" + String(maxPropane, 2);
  s += ",\"tankTare\":" + String(tankTare, 2);

  // Include the HX711 runtime tare offset (raw counts) for diagnostics
  // Read from EEPROM to avoid blocking caused by calling into the HX711 driver from an HTTP handler
  float savedRuntimeOffset = 0.0f;
  bool hasRuntimeOffset = loadFromEeprom(savedRuntimeOffset,
                                         HX711_OFFSET_EEPROM_MAGIC_ADDR,
                                         HX711_OFFSET_EEPROM_MAGIC,
                                         HX711_OFFSET_EEPROM_VALUE_ADDR);
  s += ",\"Runtime tare offset\":";

  if (hasRuntimeOffset) {
    long runtimeOffset = static_cast<long>(savedRuntimeOffset);
    s += String(runtimeOffset);
  } else {
    s += "null";
  }

  s += "}";

  return s;
}

void saveCalibration()
{
  // Save using the same constants used elsewhere in the codebase
  saveToEeprom(calibrationFactor, CAL_EEPROM_MAGIC, CAL_EEPROM_MAGIC_ADDR, CAL_EEPROM_VALUE_ADDR);
}

void skipStartupTare()
{
  // Bridge the HTTP handler into the startup tare workflow
  webSkipStartupTare();
}

void forceStartupTare()
{
  webForceStartupTare();
}
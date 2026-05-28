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

/**
 * @brief Global instance of WifiCallbacks used by the core to register callback functions.
 * 
 * @details This struct instance contains function pointers to the implementations defined above, 
 * which bridge the WiFi HTTP handlers to the core workflow functions.
 * By defining this as a global instance, 
 * it can be easily registered with the WiFi module during setup without tight coupling between the modules.
 */
extern "C" const WifiCallbacks g_wifi_callbacks = {
  enqueueCalibrate,
  enqueueTare,
  getTelemetry,
  saveCalibration
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
  float weight = NAN;

  if (scale.is_ready()) {
    // Non-blocking single-sample read; may return stale/quick reading but is safe here
    weight = scale.get_units();
  }

  String s = "{";
  s += "\"weight\":";

  if (!isfinite(weight)) {
    s += "null";
  } else {
    s += String(weight, 2);
  }

  s += ",\"calibrationFactor\":" + String(calibrationFactor, 2);
  s += ",\"knownWeight\":" + String(knownWeight, 2);
  s += ",\"maxPropane\":" + String(maxPropane, 2);
  s += ",\"tankTare\":" + String(tankTare, 2);
  s += "}";

  return s;
}

void saveCalibration()
{
  // Save using the same constants used elsewhere in the codebase
  saveToEeprom(calibrationFactor, CAL_EEPROM_MAGIC, CAL_EEPROM_MAGIC_ADDR, CAL_EEPROM_VALUE_ADDR);
}
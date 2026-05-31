/**
 * @file wifi_bridge.h
 * @author Gerald Manweiler
 *
 * @brief Declaration for the Wifi bridge callbacks instance.
 *
 * @details This header declares the global instance of `WifiCallbacks` that is defined in `wifi_bridge.cpp`.
 * This instance is used to bridge the WiFi HTTP handlers to the core workflow functions in the
 * main application, allowing the web interface to trigger actions like tare and calibration, and to provide telemetry data, without tight coupling between the WiFi module and the core application logic.
 * By providing an extern declaration here, other translation units can reference the `g_wifi_callbacks` instance defined in `wifi_bridge.cpp` without needing to include the full implementation details of the WiFi module.
 *
 * @version 0.1
 * @date 2024-06-01
 * @copyright Copyright (c) 2024 Gerald Manweiler
 */

#pragma once

// Standard library headers
#include <Arduino.h>

// Local library headers
#include "workflows/workflows_contexts.h"

// Forward declaration of global callbacks
extern "C" const WifiCallbacks g_wifi_callbacks;

// Declaration for WiFi callback implementations

/**
 * @brief Get the Telemetry Json Impl object
 *
 * @details Returns a JSON string containing the current weight reading (or null if not ready), calibration factor, known weight, max propane, tank tare, and uptime in milliseconds.
 * Uses a non-blocking single-sample read from the HX711 to avoid delays.
 *
 * @return String JSON string containing the telemetry data.
 * JSON Format: {
 *   "weight": <float|null>,
 *   "calibrationFactor": <float>,
 *   "knownWeight": <float>,
 *   "maxPropane": <float>,
 *   "tankTare": <float>,
 *   "uptime": <unsigned long>
 * }
 *
 * @throws {none} This function does not throw exceptions.
 */
String getTelemetry();
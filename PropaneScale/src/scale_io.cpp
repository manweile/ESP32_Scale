/**
 * @file scale_io.cpp
 * @author Gerald Manweiler
 * 
 * @brief Definition of input/output functions for user workflows and HX711 interactions.
 * 
 * @details Implements helper functions for user initiated workflows and HX711 interactions.
 * 
 * @version 0.1
 * @date 2026-05-07
 * 
 * @copyright Copyright (c) 2026 Gerald Manweiler
 */

/**
 * @section Standard library headers
 */

#include <Arduino.h>                                        // Arduino core library for Serial communication and basic types
#include <math.h>                                           // Math library for fabsf() and other mathematical functions

/**
 * @subsection Third party library headers
 */

#include "HX711.h"                                          // HX711 library for interfacing with the load cell amplifier to read weight data

/** 
 * @section Local library headers
 */

#include "config.h"                                         // Configuration constants for the ESP32-based propane level scale
#include "eeprom_store.h"                                   // EEPROM storage functions
#include "scale_io.h"                                       // Input/output functions for user workflows and HX711 interactions

/**
 * @section External Global State Variables
 */

extern HX711 scale;                                         // HX711 instance for interacting with the load cell amplifier

/**
 * @section Private Helper Functions
 */

/**
 * @brief Probes the HX711 with multiple reads to determine if it is producing a responsive signal.
 * 
 * @details Secondary check to detect if HX711 is powered but not properly connected.
 * Intentionally private implementation detail, only used as part of the scale ready workflow.
 * 
 * @return true if the HX711 is producing a responsive signal with variability across multiple reads; false otherwise.
 * 
 * @throws none This function does not throw exceptions.
 */
static bool hasResponsiveHx711Signal() {
  const int probeReads = 6;
  bool haveSample = false;
  long minRaw = 0;
  long maxRaw = 0;

  for (int i = 0; i < probeReads; ++i) {
    if (!scale.wait_ready_timeout(HX711_READY_TIMEOUT_MS)) {
      return false;
    }

    long raw = scale.read();
    if (!haveSample) {
      minRaw = raw;
      maxRaw = raw;
      haveSample = true;
      continue;
    }

    if (raw < minRaw) minRaw = raw;
    if (raw > maxRaw) maxRaw = raw;
  }

  if (!haveSample) {
    return false;
  }

  // A completely flat raw stream is typically a stuck/disconnected signal path.
  return maxRaw != minRaw;
}

/**
 * @section Definitions for internally used helper functions
 */

float computeLoadDetectThreshold(float minimumThresholdLbs) {
  float noise = fabsf(readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES));
  float threshold = noise * 20.0f;
  return (threshold >= minimumThresholdLbs) ? threshold : minimumThresholdLbs;
}

bool ensureScaleReady(const char* operation) {
  if (scale.wait_ready_timeout(HX711_READY_TIMEOUT_MS)) {
    if (hasResponsiveHx711Signal()) {
      return true;
    }
  }

  printScaleNotReadyDiagnostic(operation);
  return false;
}

void flushSerialInput() {
  while (Serial.available()) {
    Serial.read();
  }
}

void printScaleNotReadyDiagnostic(const char* operation) {
  Serial.print("HX711 not ready");
  if (operation != nullptr && operation[0] != '\0') {
    Serial.print(" during ");
    Serial.print(operation);
  }
  Serial.println('.');
  Serial.println("Check HX711 wiring, power, and data pins (DOUT/CLK).");
  Serial.println();
}

float readAveragedUnits(int readings, int samplesPerReading) {
  float avgWeight = 0.0f;               // Computed average weight in pounds to return at the end of the function.
  int   collected  = 0;                 // Number of samples actually read (may be less than requested if HX711 not ready)
  float totalUnits = 0.0f;              // Accumulator summing weight readings across all iterations for averaging
  
  // Use bounded wait to avoid infinite blocking while preserving the original
  // per-reading averaging semantics used across workflows.
  for (int readingIndex = 0; readingIndex < readings; ++readingIndex) {
    if (!scale.wait_ready_timeout(HX711_READY_TIMEOUT_MS)) {
      continue;
    }

    totalUnits += scale.get_units(samplesPerReading);
    collected++;
  }

  if (collected == 0) {
    return NAN;
  }

  avgWeight = totalUnits / collected;
  return avgWeight;
}

void saveRuntimeTareOffset() {
  float offsetToSave = static_cast<float>(scale.get_offset());
  if (!saveToEeprom(offsetToSave,
                    HX711_OFFSET_EEPROM_MAGIC,
                    HX711_OFFSET_EEPROM_MAGIC_ADDR,
                    HX711_OFFSET_EEPROM_VALUE_ADDR)) {
    Serial.println("Warning: failed to save runtime tare offset to EEPROM.");
  }
}
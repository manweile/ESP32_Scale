/**
 * @file scale_io.cpp
 * @author Gerald Manweiler
 * 
 * @brief Input/output functions for user workflows and HX711 interactions.
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
#include <Arduino.h>
#include <math.h>

/**
 * @subsection Third party library headers
 */
#include "HX711.h"                                          // HX711 library for interfacing with the load cell amplifier to read weight data

/** 
 * @section Local library headers
 */
#include "config.h"
#include "scale_io.h"

extern HX711 scale;                                         // HX711 instance for interacting with the load cell amplifier

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

float computeLoadDetectThreshold(float minimumThresholdLbs) {
  float noise = fabsf(readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES));
  float threshold = noise * 20.0f;
  return (threshold >= minimumThresholdLbs) ? threshold : minimumThresholdLbs;
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
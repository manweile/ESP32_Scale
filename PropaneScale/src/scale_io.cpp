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

// Standard library headers
#include <Arduino.h>                                        // Arduino core library for Serial communication and basic types
#include <math.h>                                           // Math library for fabsf() and other mathematical functions
#include <string.h>                                         // String helpers for non-blocking serial queue management

// Third party library headers
#include "HX711.h"                                          // HX711 library for interfacing with the load cell amplifier to read weight data

// Local library headers
#include "config.h"                                         // Configuration constants for the ESP32-based propane level scale
#include "eeprom_store.h"                                   // EEPROM storage functions
#include "scale_io.h"                                       // Input/output functions for user workflows and HX711 interactions
#include "workflows/workflows_contexts.h"                   // Workflow context types for managing state across non-blocking workflow steps

// External Global State Variables
extern HX711 scale;                                         /**< HX711 instance for interacting with the load cell amplifier */

// Global State Variables
String LastDiagnostic = "";                                 /**< Last HX711 diagnostic message suitable for web UI display. */

// Global Averaging Context Variables
AvgContext avgCtx;                                          /**< Averaging context instance to hold state for non-blocking average computations */
AvgContext thresholdCtx;                                    /**< Shared threshold averaging context for both level and calibration */
ProbeContext probeCtx;                                      /**< Probe context instance to hold state for non-blocking HX711 responsiveness checks */

// Public Definitions for input/output functions

bool averageUnits(int readings, int samplesPerReading, float& outAvg)
{
  if (!avgCtx.active || avgCtx.requestedReadings != readings || avgCtx.samplesPerReading != samplesPerReading) {
    avgCtx.requestedReadings = readings;
    avgCtx.samplesPerReading = samplesPerReading;
    avgCtx.index = 0;
    avgCtx.collected = 0;
    avgCtx.total = 0.0f;
    avgCtx.active = true;
  }

  // If scale isn't ready right now, caller should call again later
  if (!scale.is_ready()) {
    return false;
  }

  // Take a single averaged reading (samplesPerReading) when ready
  float units = scale.get_units(samplesPerReading);
  avgCtx.total += units;
  avgCtx.collected++;
  avgCtx.index++;

  // When we've got the requested number of readings, finish and return result
  if (avgCtx.index >= avgCtx.requestedReadings) {
    if (avgCtx.collected == 0) {
      outAvg = NAN;
    } else {
      outAvg = avgCtx.total / static_cast<float>(avgCtx.collected);
    }

    avgCtx.active = false;
    return true;
  }

  // Not finished yet
  return false;
}

void cancelThresholdDetect()
{
  thresholdCtx.active = false;
  thresholdCtx.index = 0;
  thresholdCtx.collected = 0;
  thresholdCtx.total = 0.0f;
}

bool pollProbe(bool &outResponsive, unsigned long timeoutMs, int targetSamples)
{
  // save cycles by returning early when probe isn't active, caller should call again later when it is
  if (!probeCtx.active) return false;

  // determine configured timeout and sample target from probe context if set, else fall back to provided defaults
  unsigned long cfgTimeout = probeCtx.timeoutMs ? probeCtx.timeoutMs : timeoutMs;
  int cfgTargetSamples = probeCtx.targetSamples ? probeCtx.targetSamples : targetSamples;

  // if we exceed the timeout, end the probe and report unresponsive
  if ((millis() - probeCtx.startMs) > cfgTimeout) {
    probeCtx.active = false;
    outResponsive = false;
    return true;
  }

  // if the scale isn't ready right now, caller should call again later
  if (!scale.is_ready()) return false;

  long raw = scale.read();
  probeCtx.samplesTaken++;
  probeCtx.minRaw = min(probeCtx.minRaw, raw);
  probeCtx.maxRaw = max(probeCtx.maxRaw, raw);

  // once we've taken the target number of samples, we can conclude responsiveness based on signal variability and end the probe
  if (probeCtx.samplesTaken >= cfgTargetSamples) {
    probeCtx.active = false;
    outResponsive = (probeCtx.maxRaw != probeCtx.minRaw);
    return true;
  }

  return false;
}

bool pollThresholdDetect(float& outThreshold)
{
  // save cycles by returning early when threshold detect isn't active, caller should call again later when it is
  if (!thresholdCtx.active) {
    return false;
  }

  // if the scale isn't ready right now, caller should call again later
  if (!scale.is_ready()) {
    return false;
  }

  float units = scale.get_units(thresholdCtx.samplesPerReading);
  thresholdCtx.total += units;
  thresholdCtx.collected++;
  thresholdCtx.index++;

  // once we've taken the requested number of readings, we can compute the threshold and end the detection
  if (thresholdCtx.index >= thresholdCtx.requestedReadings) {
    float avg;

    // if we didn't collect any readings, we can't compute an average, so set to NAN to trigger fallback to minimum threshold floor
    if (thresholdCtx.collected == 0) {
      avg = NAN;
    } else {
      avg = thresholdCtx.total / static_cast<float>(thresholdCtx.collected);
    }

    thresholdCtx.active = false;

    // if the average is not a finite number,
    // we likely had an issue with the scale reading and should fall back to the minimum threshold floor
    // otherwise, compute the threshold based on the average noise level
    if (!isfinite(avg)) {
      outThreshold = thresholdCtx.minimumThreshold;
    } else {
      float noise = fabsf(avg);
      outThreshold = fmaxf(noise * 20.0f, thresholdCtx.minimumThreshold);
    }

    return true;
  }

  return false;
}

void printDiagnostic(const char* operation)
{
  char buf[128];

  if (operation != nullptr && operation[0] != '\0') {
    snprintf(buf, sizeof(buf), "HX711 not ready during %s.\nCheck HX711 wiring, power, and data pins (DOUT/CLK).\n\n", operation);
  } else {
    snprintf(buf, sizeof(buf), "HX711 not ready.\nCheck HX711 wiring, power, and data pins (DOUT/CLK).\n\n");
  }

  queueSerialOutput(buf);
}

void saveRuntimeTareOffset()
{
  float offsetToSave = static_cast<float>(scale.get_offset());

  bool success = saveToEeprom(offsetToSave, HX711_OFFSET_EEPROM_MAGIC, HX711_OFFSET_EEPROM_MAGIC_ADDR, HX711_OFFSET_EEPROM_VALUE_ADDR);

  if (!success) {
    LastDiagnostic = String("Warning: failed to save runtime tare offset to EEPROM.");
  } else {
    LastDiagnostic = String("Saved runtime tare offset to EEPROM.");
  }

  // Trim any trailing newline characters for safe JSON embedding in web responses
  uint8_t lenDiag = LastDiagnostic.length() - 1;
  char lastChar = LastDiagnostic.charAt(lenDiag);

  while (lenDiag > 0 && (lastChar == '\n' || lastChar == '\r')) {
    LastDiagnostic.remove(lenDiag);
    lenDiag--;
    lastChar = LastDiagnostic.charAt(lenDiag);
  }
}

void startProbe(unsigned long timeoutMs, int targetSamples)
{
  probeCtx.active = true;
  probeCtx.maxRaw = LONG_MIN;
  probeCtx.minRaw = LONG_MAX;
  probeCtx.samplesTaken = 0;
  probeCtx.startMs = millis();
  probeCtx.timeoutMs = timeoutMs;
  probeCtx.targetSamples = targetSamples;
}

void startThresholdDetect(float minimumThresholdLbs)
{
  thresholdCtx.requestedReadings = UNLOAD_CHECK_COUNT;
  thresholdCtx.samplesPerReading = LIVE_SAMPLES;
  thresholdCtx.index = 0;
  thresholdCtx.collected = 0;
  thresholdCtx.total = 0.0f;
  thresholdCtx.minimumThreshold = minimumThresholdLbs;
  thresholdCtx.active = true;
}

void webDiagnostic(const char* operation)
{
  char buf[128];

  if (operation != nullptr && operation[0] != '\0') {
    snprintf(buf, sizeof(buf), "HX711 not ready during %s. Check HX711 wiring, power, and data pins (DOUT/CLK).", operation);
  } else {
    snprintf(buf, sizeof(buf), "HX711 not ready. Check HX711 wiring, power, and data pins (DOUT/CLK).");
  }

  // Trim any trailing newline characters for safe JSON embedding in web responses
  LastDiagnostic = String(buf);
  uint8_t lenDiag = LastDiagnostic.length() - 1;
  char lastChar = LastDiagnostic.charAt(lenDiag);

  while (lenDiag > 0 && (lastChar == '\n' || lastChar == '\r')) {
    LastDiagnostic.remove(lenDiag);
    lenDiag--;
    lastChar = LastDiagnostic.charAt(lenDiag);
  }
}
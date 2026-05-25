/**
 * @file startup_tare_workflow.cpp
 * @author Gerald Manweiler
 *
 * @brief Definitions for the startup tare workflow.
 *
 * @details Implements the functions for starting and advancing the startup tare workflow.
 *
 * @version 0.1
 * @date 2026-05-09
 *
 * @copyright Copyright (c) 2026 Gerald Manweiler
 */

// Standard library headers
#include <Arduino.h>
#include <math.h>
#include <stdio.h>

// Third party library headers
#include "HX711.h"                                       // HX711 library for scale type declaration

// Local library headers
#include "config.h"                                         // Configuration constants for the ESP32-based propane level scale
#include "src/runtime_report.h"                             // For printStartupSummary
#include "src/scale_io.h"                                   // Input/output functions for user workflows and HX711 interactions
#include "src/workflows/workflows_contexts.h"               // Context definitions for non-blocking workflows

// External Global State Variables and Functions

extern float calibrationFactor;                             // Calibration factor for converting raw HX711 readings to weight in pounds
extern float knownWeight;                                   // Known weight for calibration
extern float maxPropane;                                    // Maximum legal propane weight in pounds
extern float tankTare;                                      // Tare weight of the empty propane tank in pounds
extern HX711 scale;                                         // HX711 instance owned by PropaneScale.ino
extern void helpMenu();                                     // Function to display the help menu

// Definitions for startup tare workflow functions

void beginStartupTare()
{
  const float startupThreshold = computeThreshold(tankTare, maxPropane);

  // start a non-blocking probe to verify HX711 responsiveness before starting
  // the startup tare workflow. tickTare() will poll the probe and continue.
  startProbe(POLL_TIMEOUT_MS, LIVE_SAMPLES);
  tareCtx.probePending = true;
  // probeOp removed; tickTare will use a fixed diagnostic message on failure
  return;

  scale.set_scale(calibrationFactor);

  // request non-blocking baseline acquisition; tickTare() will complete it.
  tareCtx.baselinePending = true;
  tareCtx.baselineReadings = UNLOAD_CHECK_COUNT;
  tareCtx.baselineSamples = LIVE_SAMPLES;

  // prepare WAIT_STABLE state; baseline will be set by tickTare() when ready
  tareCtx.stableChecks = 0;
  tareCtx.stateStartMs = millis();
  tareCtx.state = TareState::WAIT_STABLE;

  printStartupSummary();

  const unsigned long autoTimeout = CONFIRM_TIMEOUT_MS / 1000UL;
  char startupPrompt[512];
  const int startupPromptLen = snprintf(startupPrompt,
                                        sizeof(startupPrompt),
                                        "Startup tare: waiting for empty scale...\n"
                                        "Auto-detect is active.\n"
                                        "Auto-detect timeout: %lu seconds.\n\n"
                                        "Not-empty threshold: >= %.2f lbs (tank tare + max propane + margin).\n"
                                        "Stability tolerance: +/- %.2f lbs once below not-empty threshold.\n"
                                        "Timeout expiry with empty + stable readings auto-confirms taring workflow.\n"
                                        "Send 'q' to skip startup tare.\n\n",
                                        autoTimeout, startupThreshold, SETUP_EMPTY_WEIGHT);

  if (startupPromptLen > 0 && startupPromptLen < static_cast<int>(sizeof(startupPrompt))) {
    queueSerialOutput(startupPrompt);
  }
}

bool handleStartupTareInput(char incoming)
{
  // workflow - waiting for stable empty condition
  if (tareCtx.state == TareState::WAIT_STABLE) {

    if (incoming == 'q' || incoming == 'Q') {
      Serial.println("Startup tare skipped by user.");
      tareCtx.baselinePending = false;
      tareCtx.state = TareState::SKIP;
    } else
      if (incoming != '\r' && incoming != '\n') {
        Serial.print("Invalid startup tare key: '");
        Serial.print(incoming);
        Serial.println("'. Send 'q' to skip startup tare.");
      }

    // made it here, so input was handled, return true to indicate that
    return true;
  }

  return false;
}

void tickTare()
{
  // Threshold is default value for a not-empty propane tank
  const float startupNotEmptyThreshold = computeThreshold(tankTare, maxPropane);

  // handle pending HX711 probe responsiveness check for workflow start
  // returns false, means still pending and we should try again on the next tick,
  // if true, means probe completed and we can check the result
  if (tareCtx.probePending) {
    bool responsive = false;

    if (!pollProbe(responsive, READY_TIMEOUT_MS, LIVE_SAMPLES)) {
      return; // still probing; try again next tick
    }

    // timeout or unresponsive, warn user to prevent long waits and provide diagnostic info
    tareCtx.probePending = false;

    if (!responsive) {
      printDiagnostic("startup tare cancelled");
      tareCtx.state = TareState::SKIP;
      return;
    }

    // workflow state will be advanced when threshold detect completes or times out
    scale.set_scale(calibrationFactor);
    tareCtx.baselinePending = true;
    tareCtx.baselineReadings = UNLOAD_CHECK_COUNT;
    tareCtx.baselineSamples = LIVE_SAMPLES;
    tareCtx.stableChecks = 0;
    tareCtx.stateStartMs = millis();
    tareCtx.state = TareState::WAIT_STABLE;

    printStartupSummary();

    const unsigned long autoTimeout = CONFIRM_TIMEOUT_MS / 1000UL;
    char startupPrompt[512];
    const int startupPromptLen = snprintf(startupPrompt,
                                          sizeof(startupPrompt),
                                          "Startup tare: waiting for empty scale...\n"
                                          "Auto-detect is active.\n"
                                          "Auto-detect timeout: %lu seconds.\n\n"
                                          "Not-empty threshold: >= %.2f lbs (tank tare + max propane + margin).\n"
                                          "Stability tolerance: +/- %.2f lbs once below not-empty threshold.\n"
                                          "Timeout expiry with empty + stable readings auto-confirms taring workflow.\n"
                                          "Send 'q' to skip startup tare.\n\n",
                                          autoTimeout, startupNotEmptyThreshold, SETUP_EMPTY_WEIGHT);

    if (startupPromptLen > 0 && startupPromptLen < static_cast<int>(sizeof(startupPrompt))) {
      queueSerialOutput(startupPrompt);
    }
  }

  // fast idle detect to save cycles when we are not in a tare workflow
  if (tareCtx.state == TareState::IDLE) return;

  if (tareCtx.state == TareState::TARE) {
    Serial.println("Stable scale detected, proceeding with tare.");
    scale.tare();
    saveRuntimeTareOffset();
    Serial.println("Scale is tared and ready.");
    tareCtx.state = TareState::IDLE;
    helpMenu();
    return;
  }

  if (tareCtx.state == TareState::SKIP) {
    Serial.println("Continuing without startup tare.");
    Serial.println("Remove propane weight and send 'r' to re-zero when ready.");
    tareCtx.state = TareState::IDLE;
    helpMenu();
    return;
  }

  // if we have gotten here, we are in WAIT_STABLE,
  // delegate serial handling to the dedicated input handler
  if (Serial.available()) {
    char c = Serial.read();

    if (handleStartupTareInput(c)) {
      return;
    }
  }

  // will only see this on application initialization
  if (tareCtx.baselinePending) {
    // scale not ready, try again next tick
    float base;

    if (!averageUnits(tareCtx.baselineReadings, tareCtx.baselineSamples, base)) {
      return;
    }

    tareCtx.baselinePending = false;

    // bad scale reading, warn user to check hardware and skip tare workflow
    if (!isfinite(base)) {
      printDiagnostic("startup tare");
      tareCtx.state = TareState::SKIP;
      return;
    }

    // good scale reading, continue into regular WAIT_STABLE processing
    tareCtx.baseline = base;
  }

  // still in WAIT_STABLE, need to quick check scale is ready to avoid long blocking
  if ((millis() - tareCtx.stateStartMs) >= CONFIRM_TIMEOUT_MS) {

    // Use non-blocking averaged read driven by loop() ticks
    float m;

    if (!averageUnits(1, AVG_SAMPLES, m)) {
      // averaging in progress; try again on next loop tick
      return;
    }

    // bad scale reading, warn user to check hardware
    if (!isfinite(m)) {
      printDiagnostic("startup tare");
      tareCtx.state = TareState::SKIP;
      return;
    }

    char diag[128];
    snprintf(diag, sizeof(diag), "Startup tare timeout check: reading=%.2f lbs, baseline=%.2f lbs\n", m, tareCtx.baseline);
    Serial.print(diag);

    // a negative reading is typically a false non-empty
    if (m < -1.0f) {
      Serial.println("Negative weight detected at startup, auto re-zeroing (tare).");
      tareCtx.state = TareState::TARE;
      return;
    }

    // a true non-empty, but we require near-zero and sustained stability
    if (fabsf(m) >= startupNotEmptyThreshold) {
      Serial.println("Startup tare timeout: scale not empty, skipping tare.");
      tareCtx.state = TareState::SKIP;
      return;
    }

    bool nearZero = fabsf(m) <= MINIMUM_LOAD_WEIGHT;
    bool stableFromBaseline = fabsf(m - tareCtx.baseline) <= SETUP_EMPTY_WEIGHT;
    bool stableLongEnough = tareCtx.stableChecks >= UNLOAD_CHECK_COUNT;

    if (nearZero && stableFromBaseline && stableLongEnough) {
      Serial.println("Startup tare auto-confirmed empty at timeout.");
      tareCtx.state = TareState::TARE;
    } else {
      Serial.println("Startup tare timeout: scale not-empty or unstable, skipping tare.");
      tareCtx.state = TareState::SKIP;
    }

    return;
  }

  // probably at stable point where we can update state

  // averaging in progress continue on next loop tick
  float m;

  if (!averageUnits(1, AVG_SAMPLES, m)) {
    return;
  }

  // bad scale check to avoid long blocking if HX711 is not responding
  if (!isfinite(m)) {
    printDiagnostic("startup tare");
    tareCtx.state = TareState::SKIP;
    return;
  }

  // scale is not empty, we want to reset stability checks
  // requires new window of stable readings below the not-empty threshold before auto-confirming
  if (fabsf(m) >= startupNotEmptyThreshold) {
    tareCtx.stableChecks = 0;
    return;
  }

  // stable relative to baseline, can increment stable check count for auto-confirm tare at timeout
  if (fabsf(m - tareCtx.baseline) <= SETUP_EMPTY_WEIGHT) {
    tareCtx.stableChecks++;
  } else {
    tareCtx.stableChecks = 0;
  }
}
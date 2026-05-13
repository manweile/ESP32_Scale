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

/**
 * @section Standard library headers
 */

#include <Arduino.h>
#include <math.h>
#include <stdio.h>

/**
 * @section Third party library headers
 */

#include "HX711.h"                                       // HX711 library for scale type declaration

/**
 * @section Local library headers
 */

#include "config.h"                                         // Configuration constants for the ESP32-based propane level scale
#include "src/runtime_report.h"                             // For printStartupSummary
#include "src/scale_io.h"                                   // Input/output functions for user workflows and HX711 interactions
#include "src/workflows/workflows_contexts.h"               // Context definitions for non-blocking workflows

/**
 * @section External Global State Variables and Functions
 */

extern float calibrationFactor;                             // Calibration factor for converting raw HX711 readings to weight in pounds
extern float knownWeight;                                   // Known weight for calibration
extern float maxPropane;                                    // Maximum legal propane weight in pounds
extern float tankTare;                                      // Tare weight of the empty propane tank in pounds
extern HX711 scale;                                         // HX711 instance owned by PropaneScale.ino
extern void helpMenu();                                     // Function to display the help menu

/**
 * @section Definitions for startup tare workflow functions
 */

void beginTare() {
  const float startupNotEmptyThreshold = computeStartupNotEmptyThreshold(tankTare, maxPropane);

  if (!ensureScaleReady("startup tare")) {
    tareCtx.state = TareState::SKIP;
    return;
  }

  scale.set_scale(calibrationFactor);

  // Establish baseline before tare; stability is checked relative to this reading.
  tareCtx.baseline     = readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES);

  // if the HX711 is not producing a valid signal, skip the tare workflow instead of blocking on instability.
  if (!isfinite(tareCtx.baseline)) {
    printScaleNotReadyDiagnostic("startup tare");
    tareCtx.state = TareState::SKIP;
    return;
  }

  tareCtx.stableChecks = 0;
  tareCtx.stateStartMs = millis();
  tareCtx.state        = TareState::WAIT_STABLE;

  printStartupSummary();

  char startupPrompt[512];
  int startupPromptLen = snprintf(startupPrompt,
                                  sizeof(startupPrompt),
                                  "Startup tare: waiting for empty scale...\n"
                                  "Auto-detect is active.\n"
                                  "Auto-detect timeout: %lu seconds.\n\n"
                                  "Not-empty threshold: >= %.2f lbs (tank tare + max propane + margin).\n"
                                  "Stability tolerance: +/- %.2f lbs once below not-empty threshold.\n"
                                  "Timeout expiry with empty + stable readings auto-confirms taring workflow.\n"
                                  "Send 'q' to skip startup tare.\n\n",
                                  static_cast<unsigned long>(EMPTY_CONFIRM_TIMEOUT_MS / 1000UL),
                                  startupNotEmptyThreshold,
                                  SETUP_EMPTY_WEIGHT);

  if (startupPromptLen > 0) {
    Serial.print(startupPrompt);
  }
}
/**
 * @file level_workflow.cpp
 * @author Gerald Manweiler
 * 
 * @brief Implements the liquid level read workflow.
 * 
 * @details Defines the functions for starting and advancing the liquid level read workflow.
 * 
 * @version 0.1
 * @date 2026-05-08
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
#include "src/scale_io.h"                                   // Input/output functions for user workflows and HX711 interactions
#include "src/workflows/workflows_contexts.h"               // Context definitions for non-blocking workflows

/**
 * @section External Global State Variables and Functions
 */

extern float calibrationFactor;                             // Calibration factor for converting raw HX711 readings to weight in pounds
extern float maxPropane;                                    // Maximum legal propane weight in pounds
extern float tankTare;                                      // Tare weight of the empty propane tank in pounds
extern HX711 scale;                                         // HX711 instance owned by PropaneScale.ino
extern bool ensureScaleReady(const char* operation);        // Checks if the HX711 is ready and prints diagnostics if not


void liquidLevel() {
  if (levelCtx.state != LevelState::IDLE) {
    Serial.println("Level read already in progress. Send 'q' to cancel first.");
    return;
  }

  if (!ensureScaleReady("level read")) {
    return;
  }

  scale.set_scale(calibrationFactor);

  levelCtx.loadDetectThreshold = computeLoadDetectThreshold(MINIMUM_LOAD_WEIGHT);
  levelCtx.stateStartMs        = millis();
  levelCtx.state               = LevelState::WAIT_LOAD;

  char levelPrompt[128];
  unsigned long loadDetectSeconds = CONFIRM_TIMEOUT_MS / 1000UL;
  snprintf(levelPrompt, sizeof(levelPrompt),
           "\nPlace propane tank on scale.\n"
           "Waiting for tank placement...\n"
           "Load placement timeout: %lu seconds.\n"
           "Send 'q' to cancel.\n",
           loadDetectSeconds);
  Serial.print(levelPrompt);
}

void tickLevelRead() {
  if (levelCtx.state == LevelState::IDLE) {
    return;
  }

  if (levelCtx.state == LevelState::WAIT_LOAD) {
    if ((millis() - levelCtx.stateStartMs) >= CONFIRM_TIMEOUT_MS) {
      Serial.println("Tank placement timed out; cancelled.");
      levelCtx.state = LevelState::IDLE;
      return;
    }

    float measuredUnits = readAveragedUnits(1, POLL_SAMPLES);
    if (!isfinite(measuredUnits)) {
      printScaleNotReadyDiagnostic("tank placement detection");
      Serial.println("Level read cancelled.");
      levelCtx.state = LevelState::IDLE;
      return;
    }

    if (fabsf(measuredUnits) >= levelCtx.loadDetectThreshold) {
      char buf[80];
      unsigned long settleSeconds = CAL_SETTLE_DELAY_MS / 1000UL;
      snprintf(buf, sizeof(buf), "Tank detected. Settling for %lu seconds before final read...\n",
               settleSeconds);
      Serial.print(buf);
      levelCtx.stateStartMs = millis();
      levelCtx.state = LevelState::SETTLING;
    }
    return;
  }

  if (levelCtx.state == LevelState::SETTLING) {
    if ((millis() - levelCtx.stateStartMs) < CAL_SETTLE_DELAY_MS) {
      return;
    }

    levelCtx.state = LevelState::READING;
    return;
  }

  if (levelCtx.state == LevelState::READING) {
    Serial.println("Reading tank weight...");
    float rawWeight = readAveragedUnits(CAL_SAMPLES, LIVE_SAMPLES);

    if (!isfinite(rawWeight)) {
      printScaleNotReadyDiagnostic("final tank reading");
      Serial.println("Level read cancelled.");
      levelCtx.state = LevelState::IDLE;
      return;
    }

    float propaneWeight = rawWeight - tankTare - PLATEN_TARE;
    if (propaneWeight < 0.0f) {
      propaneWeight = 0.0f;
    }

    float propaneLevel = (maxPropane > 0.0f) ? (propaneWeight / maxPropane) * 100.0f : 0.0f;

    char buf[96];
    snprintf(buf, sizeof(buf),
             "Scale load: %.1f lbs, Calculated propane: %.1f lbs, Propane level: %.1f%%\n",
             rawWeight, propaneWeight, propaneLevel);
    Serial.print(buf);

    levelCtx.state = LevelState::IDLE;
  }
}
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

// Standard library headers
#include <Arduino.h>
#include <math.h>
#include <stdio.h>

// Third party library headers
#include "HX711.h"                                       // HX711 library for scale type declaration

// Local library headers
#include "config.h"                                         // Configuration constants for the ESP32-based propane level scale
#include "src/scale_io.h"                                   // Input/output functions for user workflows and HX711 interactions
#include "src/workflows/workflows_contexts.h"               // Context definitions for non-blocking workflows

// External Global State Variables and Functions
extern float calibrationFactor;                             // Calibration factor for converting raw HX711 readings to weight in pounds
extern float maxPropane;                                    // Maximum legal propane weight in pounds
extern float tankTare;                                      // Tare weight of the empty propane tank in pounds
extern HX711 scale;                                         // HX711 instance owned by PropaneScale.ino
extern bool ensureScaleReady(const char* operation);        // Checks if the HX711 is ready and prints diagnostics if not

// Definitions for level read workflow functions

bool handleLevelReadInput(char incoming) {
  // workflow - waiting for load placement or settling
  if (levelCtx.state == LevelState::WAIT_LOAD || levelCtx.state == LevelState::SETTLING) {
    
    if (incoming == 'q' || incoming == 'Q') {
      Serial.println("Level read cancelled.");
      levelCtx.avgPending = false;
      cancelThresholdDetect();
      levelCtx.thresholdPending = false;
      levelCtx.state = LevelState::IDLE;
    } else if (incoming != '\r' && incoming != '\n') {
      Serial.print("Invalid level read key: '");
      Serial.print(incoming);
      Serial.println("'. Send 'q' to cancel.");
    }
    // made it here, so whatever the input, got handled by this workflow
    return true;
  }
  return false;
}

void liquidLevel() {
  if (levelCtx.state != LevelState::IDLE) {
    Serial.println("Level read already in progress. Send 'q' to cancel first.");
    return;
  }

  if (!ensureScaleReady("level read")) {
    return;
  }

  // we always ensure the scale is ready before starting the workflow
  scale.set_scale(calibrationFactor);

  startThresholdDetect(MINIMUM_LOAD_WEIGHT);
  levelCtx.thresholdPending = true;
  levelCtx.thresholdStartMs = millis();
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
  queueSerialOutput(levelPrompt);
}

void tickLevelRead() {
  if (levelCtx.state == LevelState::IDLE) {
    return;
  }

  // workflow - waiting for load placement
  if (levelCtx.state == LevelState::WAIT_LOAD) {
    // always check for timeout first to avoid long waits if tank is never placed;
    // if it's just a slow read, we'll check again on the next tick
    if ((millis() - levelCtx.stateStartMs) >= CONFIRM_TIMEOUT_MS) {
      Serial.println("Tank placement timed out; cancelled.");
      levelCtx.avgPending = false;
      cancelThresholdDetect();
      levelCtx.thresholdPending = false;
      levelCtx.state = LevelState::IDLE;
      return;
    }

    // If threshold computation is still pending, poll it first.
    if (levelCtx.thresholdPending) {
      float thr = 0.0f;
      if (!pollThresholdDetect(thr)) {
        // threshold still being computed; try again next tick
        return;
      }
      levelCtx.loadDetectThreshold = thr;
      levelCtx.thresholdPending = false;
    }

    // if averaging in progress try again on next tick
    float measuredUnits;
    if (!averageUnits(1, POLL_SAMPLES, measuredUnits)) {
      return;
    }

    // bad scale check to avoid long blocking if HX711 is not responding
    if (!isfinite(measuredUnits)) {
      printScaleNotReadyDiagnostic("tank placement detection");
      Serial.println("Level read cancelled.");
      levelCtx.avgPending = false;
      cancelThresholdDetect();
      levelCtx.thresholdPending = false;
      levelCtx.state = LevelState::IDLE;
      return;
    }

    // clean signal check for load placement
    if (fabsf(measuredUnits) >= levelCtx.loadDetectThreshold) {
      char buf[80];
      unsigned long settleSeconds = CAL_SETTLE_DELAY_MS / 1000UL;
      snprintf(buf, sizeof(buf), "Tank detected. Settling for %lu seconds before final read...\n", settleSeconds);
      Serial.print(buf);
      levelCtx.avgPending = false;
      levelCtx.stateStartMs = millis();
      levelCtx.state = LevelState::SETTLING;
    }
    return;
  }

  // workflow - settling after load placement
  if (levelCtx.state == LevelState::SETTLING) {

    // if settling delay hasn't elapsed, just return and check again on the next tick
    if ((millis() - levelCtx.stateStartMs) < CAL_SETTLE_DELAY_MS) {
      return;
    }

    levelCtx.avgPending = false;
    levelCtx.state = LevelState::READING;
    return;
  }

  // workflow - taking final reading after settling
  if (levelCtx.state == LevelState::READING) {
    
    // user needs feedback, print the prompt only once when starting the averaging
    if (!levelCtx.avgPending) {
      levelCtx.avgPending = true;
      Serial.println("Reading tank weight...");
    }

    // averaging still in progress continue next tick
    float rawWeight;
    if (!averageUnits(CAL_SAMPLES, LIVE_SAMPLES, rawWeight)) {
      return;
    }

    levelCtx.avgPending = false;

    // bad scale check to avoid long blocking if HX711 is not responding
    if (!isfinite(rawWeight)) {
      printScaleNotReadyDiagnostic("final tank reading");
      Serial.println("Level read cancelled.");
      levelCtx.state = LevelState::IDLE;
      return;
    }

    // negative weight against laws of physics but we'll settle for zero if it happens due to noise or scale issues
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
    levelCtx.avgPending = false;
  }
}
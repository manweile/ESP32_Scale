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
#include "HX711.h"                                          // HX711 library for scale type declaration

// Local library headers
#include "config.h"                                         // Configuration constants for the ESP32-based propane level scale
#include "src/scale_io.h"                                   // Input/output functions for user workflows and HX711 interactions
#include "src/workflows/workflows_contexts.h"               // Context definitions for non-blocking workflows

// External Global State Variables and Functions
extern float calibrationFactor;                             // Calibration factor for converting raw HX711 readings to weight in pounds
extern float maxPropane;                                    // Maximum legal propane weight in pounds
extern float tankTare;                                      // Tare weight of the empty propane tank in pounds
extern HX711 scale;                                         // HX711 instance owned by PropaneScale.ino

// Global State Variables
String LastLevelReport = "";                                /**< Last human-readable report produced by the most recent level read attempt. */
String LastLevelPrompt = "";                                /**< Last short prompt/message associated with an active or recent level read. */

// Definitions for level read workflow functions

void liquidLevel()
{
  // guard against starting workflow when one is already active,
  // also cannot allow concurrency with calibration workflows,
  // due to shared contexts and potential for HX711 conflicts,
  // also UX confusion with multiple concurrent workflows
  if (levelCtx.state != LevelState::IDLE) {
    String message = "Level read already in progress. Click cancel to abort.";
    LastLevelReport = message;
    LastLevelPrompt = message;
    return;
  }

  // poll HX711 responsiveness before starting workflow,
  // to avoid long waits if it's not responding,
  // and to fail fast if it's not working at all
  startProbe(POLL_TIMEOUT_MS, LIVE_SAMPLES);
  levelCtx.probePending = true;
  return;
}

void tickLevelRead()
{
  // handle pending HX711 probe responsiveness check for workflow start
  // returns false, means still pending and we should try again on the next tick,
  // if true, means probe completed and we can check the result
  if (levelCtx.probePending) {

    // keep probe active until it has a result or hits its timeout
    bool responsive = false;

    if (!pollProbe(responsive, READY_TIMEOUT_MS, LIVE_SAMPLES)) {
      return;
    }

    // timeout or unresponsive, warn user to prevent long waits and provide diagnostic info
    levelCtx.probePending = false;

    if (!responsive) {
      webDiagnostic("level read");
      LastLevelReport = String("Level read cancelled.");

      // Reset level workflow context to initial state
      cancelThresholdDetect();
      levelCtx.avgPending = false;
      levelCtx.thresholdPending = false;
      levelCtx.probePending = false;
      levelCtx.loadDetectThreshold = 0.0f;
      levelCtx.stateStartMs = 0;
      levelCtx.thresholdStartMs = 0;
      levelCtx.state = LevelState::IDLE;
      return;
    }

    // workflow state will be advanced when threshold detect completes or times out
    scale.set_scale(calibrationFactor);
    startThresholdDetect(MINIMUM_LOAD_WEIGHT);
    levelCtx.thresholdPending = true;
    levelCtx.thresholdStartMs = millis();
    levelCtx.stateStartMs = millis();
    levelCtx.state = LevelState::WAIT_LOAD;

    unsigned long loadDetectSeconds = CONFIRM_TIMEOUT_MS / 1000UL;

    String levelPrompt = String("\nPlace propane tank on scale.\n")
                         + "Waiting for tank placement...\n"
                         + "Load placement timeout: "
                         + String(loadDetectSeconds)
                         + " seconds.\n"
                         + "Click cancel to abort.\n";

    LastLevelPrompt = levelPrompt;
  }

  // workflow - waiting for load placement
  if (levelCtx.state == LevelState::WAIT_LOAD) {

    // always check for timeout first to avoid long waits if tank is never placed;
    // if it's just a slow read, we'll check again on the next tick
    if ((millis() - levelCtx.stateStartMs) >= CONFIRM_TIMEOUT_MS) {
      // Record a user-visible report so web UI clients see the timeout
      LastLevelReport = String("Tank placement timed out; cancelled.");
      LastLevelPrompt = String("Tank placement timed out.");
      // Reset level workflow context to initial state
      levelCtx.avgPending = false;
      cancelThresholdDetect();
      levelCtx.thresholdPending = false;
      levelCtx.probePending = false;
      levelCtx.loadDetectThreshold = 0.0f;
      levelCtx.stateStartMs = 0;
      levelCtx.thresholdStartMs = 0;
      levelCtx.state = LevelState::IDLE;
      return;
    }

    // If threshold computation is still pending, must try again next tick
    if (levelCtx.thresholdPending) {
      float thr = 0.0f;

      if (!pollThresholdDetect(thr)) {
        return;
      }

      levelCtx.loadDetectThreshold = thr;
      levelCtx.thresholdPending = false;
    }

    // if averaging in progress must try again on next tick
    float measuredUnits;

    if (!averageUnits(1, AVG_SAMPLES, measuredUnits)) {
      return;
    }

    // bad scale check to avoid long blocking if HX711 is not responding
    if (!isfinite(measuredUnits)) {
      webDiagnostic("tank placement detection");
      LastLevelReport = String("Level read cancelled.");

      // Reset level workflow context to initial state
      levelCtx.avgPending = false;
      cancelThresholdDetect();
      levelCtx.thresholdPending = false;
      levelCtx.probePending = false;
      levelCtx.loadDetectThreshold = 0.0f;
      levelCtx.stateStartMs = 0;
      levelCtx.thresholdStartMs = 0;
      levelCtx.state = LevelState::IDLE;
      return;
    }

    // clean signal check for load placement
    if (fabsf(measuredUnits) >= levelCtx.loadDetectThreshold) {
      levelCtx.avgPending = false;
      levelCtx.stateStartMs = millis();
      levelCtx.state = LevelState::SETTLING;

      unsigned long settleSeconds = CAL_SETTLE_DELAY_MS / 1000UL;
      String buf = String("Tank detected. Settling for ") + String(settleSeconds) + " seconds before final read...\n";
      LastLevelPrompt = buf;
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
      LastLevelPrompt = String("Reading tank weight...");
    }

    // averaging still in progress continue next tick
    float rawWeight;

    if (!averageUnits(CAL_SAMPLES, LIVE_SAMPLES, rawWeight)) {
      return;
    }

    levelCtx.avgPending = false;

    // bad scale check to avoid long blocking if HX711 is not responding
    if (!isfinite(rawWeight)) {
      webDiagnostic("final tank reading");
      LastLevelReport = String("Level read cancelled.");

      // Reset level workflow context to initial state
      levelCtx.avgPending = false;
      cancelThresholdDetect();
      levelCtx.thresholdPending = false;
      levelCtx.probePending = false;
      levelCtx.loadDetectThreshold = 0.0f;
      levelCtx.stateStartMs = 0;
      levelCtx.thresholdStartMs = 0;
      levelCtx.state = LevelState::IDLE;
      return;
    }

    // negative weight against laws of physics,
    // but we'll settle for zero if it happens due to noise or scale issues
    float propaneWeight = rawWeight - tankTare - PLATEN_TARE;

    if (propaneWeight < 0.0f) {
      propaneWeight = 0.0f;
    }

    float propaneLevel = (maxPropane > 0.0f) ? (propaneWeight / maxPropane) * 100.0f : 0.0f;

    char buf[96];
    snprintf(buf, sizeof(buf),
             "Scale load: %.1f lbs, Calculated propane: %.1f lbs, Propane level: %.1f%%",
             rawWeight, propaneWeight, propaneLevel);

    // Save human-readable report for web UI
    LastLevelReport = String(buf);

    levelCtx.state = LevelState::IDLE;
    levelCtx.avgPending = false;
  }
}
/**
 * @file calibration_workflow.cpp
 * @author Gerald Manweiler
 *
 * @brief Implements the manual calibration workflow.
 *
 * @details Defines the manual calibration workflow function and its associated tick function for advancing the workflow state machine.
 *
 * @version 0.1
 * @date 2024-06-01
 * @copyright Copyright (c) 2024 Gerald Manweiler
 */

// Standard library headers
#include <Arduino.h>
#include <math.h>
#include <stdio.h>

// Third party library headers
#include "HX711.h"                                          // HX711 library for interfacing with the load cell amplifier to read weight data

// Local library headers
#include "config.h"                                         // Configuration constants for the ESP32-based propane level scale
#include "src/eeprom_store.h"                               // EEPROM storage functions
#include "src/scale_io.h"                                   // Input/output functions for user workflows and HX711 interactions
#include "src/workflows/workflows_contexts.h"               // Context definitions for non-blocking workflows

// External Global State Variables and Functions
extern float calibrationFactor;                             /**< Calibration factor for converting raw HX711 readings to weight in pounds */
extern float knownWeight;                                   /**< Known weight for calibration */
extern HX711 scale;                                         /**< HX711 instance owned by PropaneScale.ino */
extern const char CALIBRATION_SAVE_FAILURE_MSG[];           /**< Message to display when saving calibration factor to EEPROM fails */
extern const char CALIBRATION_SAVE_SUCCESS_MSG[];           /**< Message to display when saving calibration factor to EEPROM succeeds */

// Declaration & Definition of Private Helper Functions

/**
 * @brief Queues the current manual calibration reading snapshot when needed.
 *
 * @details Applies the current calibration factor, reads the current weight,
 * and queues a formatted snapshot on first entry or whenever the displayed
 * reading/factor/step changes. Returns false when the scale is not ready yet.
 *
 * @return {bool} True when a valid reading was acquired; false when the HX711 is not ready.
 *
 * @throws {none} This function does not throw exceptions.
 */
static bool queueManualAdjustmentSnapshot()
{
  // manual calibration always needs to apply current factor to reflect latest adjustment
  scale.set_scale(calibrationFactor);

  // skip until the first valid reading is available after more iteration(s)
  if (!scale.is_ready()) {
    return false;
  }

  float readingWeight = scale.get_units();

  // Quantize to integers for change detection — float comparison is unreliable across loop ticks
  int readingTenth      = static_cast<int>(lroundf(readingWeight         * 10.0f));
  int factorHundredth   = static_cast<int>(lroundf(calibrationFactor     * 100.0f));
  int stepTenThousandth = static_cast<int>(lroundf(calCtx.adjustmentStep * 10000.0f));

  bool tenthChanged = (readingTenth != calCtx.lastReadingTenth);
  bool factorChanged = (factorHundredth != calCtx.lastFactorHundredth);
  bool stepChanged = (stepTenThousandth != calCtx.lastStepTenThousandth);

  if (!calCtx.hasManualDisplay || tenthChanged || factorChanged || stepChanged) {
    char buf[80];
    snprintf(buf, sizeof(buf), "Reading: %.1f lbs  factor: %.2f  step: %.4f\n", readingWeight, calibrationFactor, calCtx.adjustmentStep);
    queueSerialOutput(buf);

    calCtx.hasManualDisplay = true;
    calCtx.lastReadingTenth = readingTenth;
    calCtx.lastFactorHundredth = factorHundredth;
    calCtx.lastStepTenThousandth = stepTenThousandth;
  }

  return true;
}

/**
 * @brief Advances the calibration context from WAIT_EMPTY to the next appropriate state.
 *
 * @details Used for AUTO, MANUAL, REZERO modes,
 * declared and implemented as a static function to allow reuse of the WAIT_EMPTY -> WAIT_LOAD transition logic.
 *
 * @throws {none} This function does not throw exceptions.
 */
static void transitionFromWaitEmpty()
{
  // workflow - user has removed weight from scale, or user has force-confirmed empty reading in REZERO mode
  if (calCtx.mode == CalMode::REZERO) {
    scale.set_scale();
    scale.tare();
    saveRuntimeTareOffset();
    scale.set_scale(calibrationFactor);
    Serial.println("Scale re-zero complete.");
    calCtx.state = CalState::IDLE;
    calCtx.mode  = CalMode::NONE;
    return;
  }

  Serial.println("Empty scale confirmed. Taring now...");
  scale.set_scale();
  scale.tare();
  saveRuntimeTareOffset();
  calCtx.loadDetectChecks = 0;

  // start asynchronous threshold computation for load detection
  startThresholdDetect(MINIMUM_LOAD_WEIGHT);
  calCtx.thresholdPending = true;
  calCtx.thresholdStartMs = millis();

  // workflow - auto calibration waiting on user to place known weight on scale after empty confirmation
  if (calCtx.mode == CalMode::AUTO) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Place the known weight on the scale: %.2f lbs\n", knownWeight);
    queueSerialOutput(buf);
  }

  // workflow - manual calibration waiting on user to place known weight on scale after empty confirmation,
  // but user will provide the known weight value instead of using the default
  if (calCtx.mode == CalMode::MANUAL) {
    Serial.println("Place known weight on scale");
  }

  // Defensive guard: only AUTO/MANUAL should reach this point.
  if (calCtx.mode != CalMode::AUTO && calCtx.mode != CalMode::MANUAL) {
    Serial.println("Invalid calibration mode; cancelling workflow.");
    calCtx.state = CalState::IDLE;
    calCtx.mode = CalMode::NONE;
    return;
  }

  char buf[96];
  unsigned long loadDetectSeconds = CONFIRM_TIMEOUT_MS / 1000UL;
  snprintf(buf, sizeof(buf), "Waiting for weight placement on scale...\nLoad placement timeout: %lu seconds.\n", loadDetectSeconds);
  queueSerialOutput(buf);

  calCtx.stateStartMs = millis();
  calCtx.state = CalState::WAIT_LOAD;
}

// Definitions for calibration workflow functions

void automaticCalibration()
{
  /**
   * Automatic Calibration Workflow State Progression:
   * 1. IDLE -> WAIT_EMPTY:
   *  System prompts user to empty the scale, waits for confirmation of empty scale.
   * 2. WAIT_EMPTY -> WAIT_LOAD:
   *  Empty scale is confirmed, system tares the scale, prompts user to place a known weight on the scale, waits for weight placement.
   * 3. WAIT_LOAD -> SETTLING:
   *  System waits for the weight to mechanically settle to ensure stable readings.
   * 4. SETTLING -> ADJUSTING:
   *  System takes measurements, adjusts calibration factor based on known weight and measured weight until displayed weight matches known weight.
   * 5. ADJUSTING -> IDLE:
   *  System saves the new calibration factor to EEPROM and returns to idle state.
   *
   * Note:
   * Any point during WAIT_EMPTY or WAIT_LOAD, user can cancel workflow by sending 'q', which will revert any changes and return to IDLE state.
   * In REZERO mode, user can also force-confirm an empty reading by sending 'z' to transition from WAIT_EMPTY to WAIT_LOAD
   * Useful for scales that have a false non-empty reading due to noise or offsets.
   */

  // cannot allow concurrency with the other manual & rezero workflows
  // since they share the same state variable and user inputs
  // also cannot allow concurrency with the level read workflow,
  // due to shared contexts and potential for HX711 conflicts,
  // also UX confusion with multiple concurrent workflows
  if (calCtx.state != CalState::IDLE) {
    Serial.println("Automatic calibration already in progress. Send 'q' to cancel first.");
    return;
  }

  calCtx.mode = CalMode::AUTO;
  calCtx.originalCalibrationFactor = calibrationFactor;
  calCtx.measuredUnits = 0.0f;

  // poll HX711 ready before starting manual calibration workflow,
  // to avoid long waits if it's not responding,
  // and to fail fast if it's not working at all
  startProbe(POLL_TIMEOUT_MS, LIVE_SAMPLES);
  calCtx.probePending = true;
  return;
}

void handleCalibrationInput(char serialchar)
{
  // workflow - in WAIT_EMPTY or WAIT_LOAD, only valid inputs are 'q' and 'z'
  // we dont care about the 3rd 'wait' state - SETTLING - because it is time-based and not user-input based
  if (calCtx.state == CalState::WAIT_EMPTY || calCtx.state == CalState::WAIT_LOAD) {

    // handle the force-confirm first because it is the most likely user input in WAIT_EMPTY,
    // in case the user is trying to work around a scale that is giving a false non-empty reading due to noise or an offset
    if (calCtx.mode == CalMode::REZERO && calCtx.state == CalState::WAIT_EMPTY && (serialchar == 'z' || serialchar == 'Z')) {
      Serial.println("Runtime re-zero force-confirmed by user.");
      Serial.println();
      transitionFromWaitEmpty();
      return;
    }

    // a user typo is always possible
    if (serialchar != 'q' && serialchar != 'Q') {
      if (calCtx.mode == CalMode::AUTO) {
        char buf[64];
        snprintf(buf, sizeof(buf), "Invalid automatic calibration key: '%c'. Use 'q' to cancel.\n", serialchar);
        Serial.print(buf);
      } else
        if (calCtx.mode == CalMode::REZERO) {
          char buf[96];
          snprintf(buf, sizeof(buf), "Invalid re-zero key: '%c'. Use 'q' to cancel or 'z' to force re-zero.\n", serialchar);
          Serial.print(buf);
        }

      return;
    }

    // AUTO & MANUAL wouldn't have had chance to change the calibration factor yet
    // REZERO doesn't change it at all

    if (calCtx.mode == CalMode::AUTO || calCtx.mode == CalMode::MANUAL) {
      Serial.println("Calibration cancelled. Changes were not saved.");
      calibrationFactor = calCtx.originalCalibrationFactor;

      if (calCtx.thresholdPending) {
        cancelThresholdDetect();
        calCtx.thresholdPending = false;
      }

      scale.set_scale(calibrationFactor);
    }

    if (calCtx.mode == CalMode::REZERO) {
      Serial.println("Runtime re-zero cancelled.");
    }

    calCtx.state = CalState::IDLE;
    calCtx.mode  = CalMode::NONE;
    return;
  }

  // workflow - in manual adjustment state
  // possible input +, -, s, q (adjust calibration factor up/down, save, cancel)
  if (calCtx.state == CalState::ADJUSTING) {

    // last direction indicates which 'direction' we are adjusting in
    // same direction: keep the same step
    // switched direction: reduce the step to allow finer adjustments
    // we always halve the step on direction switch to allow for finer control,
    // but we also enforce a minimum step size to prevent it from getting too small and making adjustments impossible

    if (serialchar == '+') {
      if (calCtx.lastDirection == -1) {
        calCtx.adjustmentStep = max(calCtx.adjustmentStep * 0.5f, calCtx.minStep);
      }

      calibrationFactor  += calCtx.adjustmentStep;
      calCtx.lastDirection = 1;
    } else
      if (serialchar == '-') {
        if (calCtx.lastDirection == 1) {
          calCtx.adjustmentStep = max(calCtx.adjustmentStep * 0.5f, calCtx.minStep);
        }

        calibrationFactor  -= calCtx.adjustmentStep;
        calCtx.lastDirection = -1;

      } else
        if (serialchar == 's' || serialchar == 'S') {
          char buf[80];
          snprintf(buf, sizeof(buf), "Manual calibration complete, computed calibration factor: %.2f\n", calibrationFactor);
          Serial.print(buf);

          if (!saveToEeprom(calibrationFactor, CAL_EEPROM_MAGIC, CAL_EEPROM_MAGIC_ADDR, CAL_EEPROM_VALUE_ADDR)) {
            Serial.println("Failed to save calibration to EEPROM.");
          } else {
            Serial.println("Calibration saved to EEPROM.");
          }

          calCtx.state = CalState::IDLE;
          calCtx.mode  = CalMode::NONE;

        } else
          if (serialchar == 'q' || serialchar == 'Q') {
            calibrationFactor = calCtx.originalCalibrationFactor;
            scale.set_scale(calibrationFactor);
            Serial.println("Manual calibration cancelled. Changes were not saved.");
            calCtx.state = CalState::IDLE;
            calCtx.mode  = CalMode::NONE;

          } else {
            char buf[96];
            snprintf(buf, sizeof(buf), "Invalid manual calibration key: '%c'. Use '+', '-', 's' (save), or 'q' (cancel).\n", serialchar);
            Serial.print(buf);
          }
  }
}

void manualCalibration()
{
  // cannot allow concurrency with the other auto and rezero workflows
  // since they share the same state variable and user inputs
  // also cannot allow concurrency with the level read workflow,
  // due to shared contexts and potential for HX711 conflicts,
  // also UX confusion with multiple concurrent workflows
  if (calCtx.state != CalState::IDLE) {
    Serial.println("Manual calibration already in progress. Send 'q' to cancel first.");
    return;
  }

  float step = fabsf(calibrationFactor) * 0.01f;
  float minStep = fabsf(calibrationFactor) * 0.0001f;

  if (step == 0.0f) step    = 10.0f;

  if (minStep == 0.0f) minStep = 0.001f;

  calCtx.adjustmentStep = step;
  calCtx.hasManualDisplay = false;
  calCtx.lastDirection = 0;
  calCtx.measuredUnits = 0.0f;
  calCtx.minStep = minStep;
  calCtx.mode = CalMode::MANUAL;
  calCtx.originalCalibrationFactor = calibrationFactor;

  // poll HX711 ready before starting manual calibration workflow,
  // since it relies on live readings to guide the user adjustments
  startProbe(POLL_TIMEOUT_MS, LIVE_SAMPLES);
  calCtx.probePending = true;

  return;
}

void reZero()
{
  // cannot allow concurrency with the other auto and manual workflows
  // since they share the same state variable and user inputs
  // also cannot allow concurrency with the level read workflow,
  // due to shared contexts and potential for HX711 conflicts,
  // also UX confusion with multiple concurrent workflows
  if (calCtx.state != CalState::IDLE) {
    Serial.println("Runtime re-zero already in progress. Send 'q' to cancel first.");
    return;
  }

  calCtx.mode = CalMode::REZERO;
  calCtx.measuredUnits = 0.0f;

  // poll HX711 ready before starting re-zero workflow,
  // since it relies on live readings to confirm the empty condition
  startProbe(POLL_TIMEOUT_MS, LIVE_SAMPLES);
  calCtx.probePending = true;
  return;
}

void tickCalibration()
{
  // handle pending HX711 probe responsiveness first because if the HX711 isn't responsive,
  // no point in doing any of the rest of the workflow logic which relies on it,
  // and we can fail fast with a clear diagnostic message to the user about what went wrong
  if (calCtx.probePending) {

    // keep probe active until it has a result or hits its timeout
    bool responsive = false;

    if (!pollProbe(responsive, READY_TIMEOUT_MS, LIVE_SAMPLES)) {
      return;
    }

    // timeout or unresponsive, warn user to prevent long waits and provide diagnostic info
    calCtx.probePending = false;

    if (!responsive) {
      const char* op = "calibration";

      if (calCtx.mode == CalMode::AUTO) {
        op = "automatic calibration";
      } else if (calCtx.mode == CalMode::MANUAL) {
        op = "manual calibration";
      } else if (calCtx.mode == CalMode::REZERO) {
        op = "re-zero";
      }

      printDiagnostic(op);
      calCtx.mode = CalMode::NONE;
      calCtx.state = CalState::IDLE;
      return;
    }

    // both AUTO and MANUAL workflows rely on live readings to guide the user,
    // need to ensure readings in live prompts are accurate
    scale.set_scale(calibrationFactor);

    // finish initialization depending on requested mode
    {
      const char* header = "Calibration";
      const char* extraLine = "";
      unsigned long userConfirmSeconds = CONFIRM_TIMEOUT_MS / 1000UL;

      if (calCtx.mode == CalMode::AUTO) {
        header = "Automatic calibration mode";
      } else if (calCtx.mode == CalMode::MANUAL) {
        header = "Manual calibration mode";
      } else if (calCtx.mode == CalMode::REZERO) {
        header = "Runtime re-zero requested.";
        extraLine = "If reading is offset-biased, send 'z' to force re-zero after verifying empty scale.\n";
      }

      char calPrompt[288];
      snprintf(calPrompt, sizeof(calPrompt),
           "\n%s\n"
           "\nRemove all weight from scale.\n"
           "Auto-detect is active.\n"
           "Send 'q' to cancel.\n"
           "%s"
           "Confirmation timeout: %lu seconds.\n",
           header,
           extraLine,
           userConfirmSeconds);

      queueSerialOutput(calPrompt);

      calCtx.state = CalState::WAIT_EMPTY;
      calCtx.stateStartMs = millis();
      calCtx.measuredUnits = 0.0f;
      return;
    }
  }

  // no active workflow — skip all scale reads and checks
  if (calCtx.state == CalState::IDLE) {
    return;
  }

  // workflow - waiting on user to remove all weight from platen
  if (calCtx.state == CalState::WAIT_EMPTY) {
    // Serial input handling is centralized in `handleCalibrationInput()` and
    // forwarded from the main loop when calibration workflows are active.
    // Do not poll Serial here to avoid duplicating input handling logic.

    // Only check for empty at timeout, not on every tick
    if ((millis() - calCtx.stateStartMs) >= CONFIRM_TIMEOUT_MS) {

      float tmpAvg = 0.0f;

      if (!averageUnits(1, AVG_SAMPLES, tmpAvg)) {
        calCtx.avgPhase = AvgPhase::EMPTY_CONFIRM;
        return;
      }

      calCtx.avgPhase = AvgPhase::NONE;
      calCtx.measuredUnits = tmpAvg;

      bool emptyDetected = fabsf(calCtx.measuredUnits) <= MINIMUM_LOAD_WEIGHT;

      if (emptyDetected) {
        Serial.println("Empty scale auto-confirmed at timeout (stable scale).");
        Serial.println();
        transitionFromWaitEmpty();
      } else {
        Serial.println("Confirmation timed out: scale not empty; cancelled.");

        if (calCtx.mode == CalMode::REZERO) {
          Serial.println("If scale is confirmed empty, send 'z' during re-zero to seed runtime offset.");
        }

        Serial.println();
        calCtx.state = CalState::IDLE;
        calCtx.mode  = CalMode::NONE;
      }

      return;
    }

    // Do not auto-detect early; just wait for timeout or user input
    return;
  }

  // workflow - waiting on user to place known weight on scale after empty confirmation
  if (calCtx.state == CalState::WAIT_LOAD) {

    // If threshold computation is pending, have to poll it first
    if (calCtx.thresholdPending) {
      float thr = 0.0f;

      if (!pollThresholdDetect(thr)) {
        return;
      }

      // enforce sanity bounds on the computed threshold to guard against edge cases:
      // noise is very low (which could cause false positives)
      // noise is very high (which could cause false negatives and user confusion).

      calCtx.loadDetectThreshold = thr;

      if (!isfinite(calCtx.loadDetectThreshold) || calCtx.loadDetectThreshold < MINIMUM_LOAD_WEIGHT) {
        calCtx.loadDetectThreshold = MINIMUM_LOAD_WEIGHT;
      }

      if (calCtx.loadDetectThreshold > (MINIMUM_LOAD_WEIGHT * 8.0f)) {
        calCtx.loadDetectThreshold = MINIMUM_LOAD_WEIGHT * 8.0f;
      }

      calCtx.thresholdPending = false;
      calCtx.stateStartMs = millis();
    }

    unsigned long elapsedMs = millis() - calCtx.stateStartMs;

    if (elapsedMs < CONFIRM_TIMEOUT_MS) {
      return;
    }

    float tmpAvg = 0.0f;

    if (!averageUnits(1, AVG_SAMPLES, tmpAvg)) {
      calCtx.avgPhase = AvgPhase::LOAD_DETECT; // in-progress
      return;
    }

    calCtx.avgPhase = AvgPhase::NONE;
    calCtx.measuredUnits = tmpAvg;

    if (fabsf(calCtx.measuredUnits) < calCtx.loadDetectThreshold) {
      Serial.println("Weight placement timed out; calibration cancelled.");
      calCtx.state = CalState::IDLE;
      calCtx.mode  = CalMode::NONE;
      return;
    }

    char buf[80];
    unsigned long settleSeconds = CAL_SETTLE_DELAY_MS / 1000UL;
    snprintf(buf, sizeof(buf), "Weight detected. Settling for %lu seconds before measuring...\n", settleSeconds);
    Serial.print(buf);
    calCtx.stateStartMs = millis();
    calCtx.state        = CalState::SETTLING;
    return;
  }

  // workflow - waiting for placed weight to stop moving before taking final measurement
  // can be for auto or manual calibration depending on users choice of calibration mode
  if (calCtx.state == CalState::SETTLING) {

    if ((millis() - calCtx.stateStartMs) < CAL_SETTLE_DELAY_MS) {
      return;
    }

    // workflow - auto calibration takes a measurement and computes new factor and saves to eeprom
    if (calCtx.mode == CalMode::AUTO) {

      // Print a measuring message both when starting the final measurement
      // and on subsequent ticks while the non-blocking operation is in-progress.
      if (calCtx.avgPhase == AvgPhase::NONE) {
        Serial.println("Measuring stable reading...");
        float tmp = 0.0f;

        if (!averageUnits(CAL_SAMPLES, LIVE_SAMPLES, tmp)) {
          calCtx.avgPhase = AvgPhase::FINAL_MEAS;
          return;
        }

        calCtx.avgPhase = AvgPhase::NONE;
        calCtx.measuredUnits = tmp;
      }

      if (calCtx.avgPhase == AvgPhase::FINAL_MEAS) {
        float tmp = 0.0f;

        if (!averageUnits(CAL_SAMPLES, LIVE_SAMPLES, tmp)) {
          return; // still measuring
        }

        calCtx.measuredUnits = tmp;
        calCtx.avgPhase = AvgPhase::NONE;
      }

      if (knownWeight == 0.0f || calCtx.measuredUnits == 0.0f) {
        Serial.println("Automatic calibration failed: invalid known weight or reading.");
        calCtx.state = CalState::IDLE;
        calCtx.mode  = CalMode::NONE;
        return;
      }

      // units/lb: maps raw ADC counts to pounds
      calibrationFactor = calCtx.measuredUnits / knownWeight;
      scale.set_scale(calibrationFactor);

      // re-read to confirm factor produces correct output
      float verifiedUnits = 0.0f;

      if (!averageUnits(CAL_SAMPLES, LIVE_SAMPLES, verifiedUnits)) {
        calCtx.avgPhase = AvgPhase::VERIFICATION; // verification in-progress
        return;
      }

      char buf[224];
      bool saveSucceeded = saveToEeprom(calibrationFactor, CAL_EEPROM_MAGIC, CAL_EEPROM_MAGIC_ADDR, CAL_EEPROM_VALUE_ADDR);
      snprintf(buf, sizeof(buf),
               "Initial calibration factor estimate: %.2f\n"
               "Verified reading: %.2f lbs\n"
               "Automatic calibration complete, computed calibration factor: %.2f\n"
               "%s\n",
               calibrationFactor, verifiedUnits, calibrationFactor,
               saveSucceeded ? "Calibration saved to EEPROM." : "Failed to save calibration to EEPROM.");
      queueSerialOutput(buf);

      calCtx.state = CalState::IDLE;
      calCtx.mode  = CalMode::NONE;
    }

    // workflow - manual calibration transitions to adjustment state with user input
    if (calCtx.mode == CalMode::MANUAL) {
      queueSerialOutput("Adjust calibration until the reading matches the known weight.\n"
                        "Send '+' to increase calibration factor\n"
                        "Send '-' to decrease calibration factor\n"
                        "(step halves on direction reversal)\n"
                        "Send 's' to save and finish manual calibration.\n"
                        "Send 'q' to cancel manual calibration without saving.\n");

      // force first print on next ADJUSTING tick
      calCtx.hasManualDisplay = false;
      calCtx.state            = CalState::ADJUSTING;
      queueManualAdjustmentSnapshot();
    }

    // auto is done or we are in manual and need to await user adjustment input
    return;
  }

  // workflow - still in manual adjustment state, waiting for user to adjust factor and save or cancel
  if (calCtx.state == CalState::ADJUSTING) {
    queueManualAdjustmentSnapshot();
  }
}

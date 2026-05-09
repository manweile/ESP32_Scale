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

/**
 * @section Standard library headers
 */

#include <Arduino.h>
#include <math.h>
#include <stdio.h>

/**
 * @subsection Third party library headers
 */

#include "HX711.h"                                          // HX711 library for interfacing with the load cell amplifier to read weight data

/**
 * @section Local library headers
 */

#include "config.h"                                         // Configuration constants for the ESP32-based propane level scale
#include "src/eeprom_store.h"                               // EEPROM storage functions
#include "src/scale_io.h"                                   // Input/output functions for user workflows and HX711 interactions
#include "src/workflows/workflows_contexts.h"               // Context definitions for non-blocking workflows

/**
 * @section External Global State Variables and Functions
 */

extern float calibrationFactor;                             // Calibration factor for converting raw HX711 readings to weight in pounds
extern float knownWeight;                                   // Known weight for calibration
extern HX711 scale;                                         // HX711 instance owned by PropaneScale.ino
extern const char CALIBRATION_SAVE_FAILURE_MSG[];           // Message to display when saving calibration factor to EEPROM fails
extern const char CALIBRATION_SAVE_SUCCESS_MSG[];           // Message to display when saving calibration factor to EEPROM succeeds

/**
 * @section Private Helper Functions
 */

/**
 * @brief Advances the calibration context from WAIT_EMPTY to the next appropriate state.
 *
 * @details For REZERO mode, tares the scale and returns to IDLE.
 * For AUTO and MANUAL modes, tares, measures the noise threshold, and transitions to WAIT_LOAD.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
static void transitionFromWaitEmpty() {
  // workflow - waiting on user to remove all weight from platen
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
  calCtx.loadDetectThreshold = computeLoadDetectThreshold(MINIMUM_LOAD_THRESHOLD);
  if (!isfinite(calCtx.loadDetectThreshold) || calCtx.loadDetectThreshold < MINIMUM_LOAD_THRESHOLD) {
    calCtx.loadDetectThreshold = MINIMUM_LOAD_THRESHOLD;
  }
  if (calCtx.loadDetectThreshold > (MINIMUM_LOAD_THRESHOLD * 8.0f)) {
    calCtx.loadDetectThreshold = MINIMUM_LOAD_THRESHOLD * 8.0f;
  }

  // workflow - auto calibration waiting on user to place known weight on scale after empty confirmation
  if (calCtx.mode == CalMode::AUTO) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Place the known weight on the scale: %.2f lbs\n", knownWeight);
    Serial.print(buf);
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
    calCtx.mode  = CalMode::NONE;
    return;
  }
  
  char buf[96];
  unsigned long loadDetectSeconds = EMPTY_CONFIRM_TIMEOUT_MS / 1000UL;
  snprintf(buf, sizeof(buf),
           "Waiting for weight placement on scale...\nLoad placement timeout: %lu seconds.\n",
           loadDetectSeconds);
  Serial.print(buf);

  calCtx.stateStartMs = millis();
  calCtx.state        = CalState::WAIT_LOAD;
}

/**
 * @section Definitions for calibration workflow functions
 */

void automaticCalibration() {
  if (calCtx.state != CalState::IDLE) {
    Serial.println("Automatic calibration already in progress. Send 'q' to cancel first.");
    return;
  }

  if (!ensureScaleReady("automatic calibration")) {
    return;
  }

  Serial.println("\nAutomatic calibration mode");
  scale.set_scale(calibrationFactor);

  char calPrompt[192];
  unsigned long userConfirmSeconds = USER_CONFIRM_TIMEOUT_MS / 1000UL;
  snprintf(calPrompt, sizeof(calPrompt),
           "\nRemove all weight from scale.\n"
           "Auto-detect is active.\n"
           "Empty threshold: +/- %.2f lbs.\n"
           "Send 'q' to cancel.\n"
           "Confirmation timeout: %lu seconds.\n",
           MINIMUM_LOAD_WEIGHT,
           userConfirmSeconds);
  Serial.print(calPrompt);

  calCtx.mode              = CalMode::AUTO;
  calCtx.state             = CalState::WAIT_EMPTY;
  calCtx.stateStartMs      = millis();
  calCtx.stableEmptyChecks = 0;
  calCtx.measuredUnits     = 0.0f;
}

void handleCalibrationInput(char serialchar) {
  // 
  if (calCtx.state == CalState::WAIT_EMPTY || calCtx.state == CalState::WAIT_LOAD || calCtx.state == CalState::SETTLING) {
    // From WAIT_EMPTY and WAIT_LOAD states, 'q' cancels the workflow, 
    // but only AUTO mode needs to reset the calibration factor since MANUAL mode didn't change it yet, 
    // and REZERO didn't change it either since it applies a runtime offset without modifying the calibration factor. 
    // Additionally, from WAIT_EMPTY state, 'z' forces confirmation of an empty condition for the re-zero workflow.
    if (calCtx.mode == CalMode::REZERO && calCtx.state == CalState::WAIT_EMPTY && (serialchar == 'z' || serialchar == 'Z')) {
      Serial.println("Runtime re-zero force-confirmed by user.");
      Serial.println();
      transitionFromWaitEmpty();
      return;
    }

    if (serialchar != 'q' && serialchar != 'Q') {
      if (calCtx.mode == CalMode::AUTO) {
        char buf[64];
        snprintf(buf, sizeof(buf), "Invalid automatic calibration key: '%c'. Use 'q' to cancel.\n", serialchar);
        Serial.print(buf);
      } else if (calCtx.mode == CalMode::REZERO) {
        char buf[96];
        snprintf(buf, sizeof(buf), "Invalid re-zero key: '%c'. Use 'q' to cancel or 'z' to force re-zero.\n", serialchar);
        Serial.print(buf);
      }
      return;
    }

    // workflow - cancel from waiting states goes back to IDLE,
    // but only AUTO mode needs to reset the calibration factor since MANUAL mode didn't change it yet, 
    // and REZERO didn't change it either since it applies a runtime offset without modifying the calibration factor
    if (calCtx.mode == CalMode::AUTO) {
      Serial.println("Automatic calibration cancelled.");
      calibrationFactor = DEF_CALIBRATION_FACTOR;

      if(!saveToEeprom(calibrationFactor, CAL_EEPROM_MAGIC, CAL_EEPROM_MAGIC_ADDR, CAL_EEPROM_VALUE_ADDR)) {
        Serial.println(CALIBRATION_SAVE_FAILURE_MSG);
      } else {
        Serial.println(CALIBRATION_SAVE_SUCCESS_MSG);
      }
    } 
    
    if (calCtx.mode == CalMode::MANUAL) {
      calibrationFactor = calCtx.originalCalibrationFactor;
      scale.set_scale(calibrationFactor);
      Serial.println("Manual calibration cancelled. Changes were not saved.");
    } 
    
    if (calCtx.mode == CalMode::REZERO) {
      Serial.println("Runtime re-zero cancelled.");
    }

    calCtx.state = CalState::IDLE;
    calCtx.mode  = CalMode::NONE;
    return;
  }

  // workflow - in manual adjustment state,
  // increasing & decreasing calibration factor, 
  // saving calibration factor or cancelling workflow altogether,
  // handling an invalid key
  if (calCtx.state == CalState::ADJUSTING) {
    if (serialchar == '+') {

      if (calCtx.lastDirection == -1) {
        calCtx.adjustmentStep = max(calCtx.adjustmentStep * 0.5f, calCtx.minStep);
      }
      calibrationFactor  += calCtx.adjustmentStep;
      calCtx.lastDirection = 1;

    } else if (serialchar == '-') {

      if (calCtx.lastDirection == 1) {
        calCtx.adjustmentStep = max(calCtx.adjustmentStep * 0.5f, calCtx.minStep);
      }

      calibrationFactor  -= calCtx.adjustmentStep;
      calCtx.lastDirection = -1;

    } else if (serialchar == 's' || serialchar == 'S') {

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

    } else if (serialchar == 'q' || serialchar == 'Q') {

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

void manualCalibration() {
  if (calCtx.state != CalState::IDLE) {
    Serial.println("Manual calibration already in progress. Send 'q' to cancel first.");
    return;
  }

  if (!ensureScaleReady("manual calibration")) {
    return;
  }

  float step    = fabsf(calibrationFactor) * 0.01f;
  float minStep = fabsf(calibrationFactor) * 0.0001f;
  if (step    == 0.0f) step    = 10.0f;
  if (minStep == 0.0f) minStep = 0.001f;

  Serial.println("\nManual calibration mode");
  scale.set_scale(calibrationFactor);

  char calPrompt[192];
  unsigned long userConfirmSeconds = USER_CONFIRM_TIMEOUT_MS / 1000UL;
  snprintf(calPrompt, sizeof(calPrompt),
           "\nRemove all weight from scale.\n"
           "Auto-detect is active.\n"
           "Empty threshold: +/- %.2f lbs.\n"
           "Send 'q' to cancel.\n"
           "Confirmation timeout: %lu seconds.\n",
           MINIMUM_LOAD_WEIGHT,
           userConfirmSeconds);
  Serial.print(calPrompt);

  calCtx.adjustmentStep            = step;
  calCtx.hasManualDisplay          = false;
  calCtx.lastDirection             = 0;
  calCtx.measuredUnits             = 0.0f;
  calCtx.minStep                   = minStep;
  calCtx.mode                      = CalMode::MANUAL;
  calCtx.originalCalibrationFactor = calibrationFactor;
  calCtx.stableEmptyChecks         = 0;
  calCtx.state                     = CalState::WAIT_EMPTY;
  calCtx.stateStartMs              = millis();
}

void reZero() {
  if (calCtx.state != CalState::IDLE) {
    Serial.println("Runtime re-zero already in progress. Send 'q' to cancel first.");
    return;
  }

  if (!ensureScaleReady("re-zero")) {
    return;
  }

  Serial.println("\nRuntime re-zero requested.");
  scale.set_scale(calibrationFactor);

  char calPrompt[256];
  unsigned long userConfirmSeconds = USER_CONFIRM_TIMEOUT_MS / 1000UL;
  snprintf(calPrompt, sizeof(calPrompt),
           "\nRemove all weight from scale.\n"
           "Auto-detect is active.\n"
           "Empty threshold: +/- %.2f lbs.\n"
           "Send 'q' to cancel.\n"
           "If reading is offset-biased, send 'z' to force re-zero after verifying empty scale.\n"
           "Confirmation timeout: %lu seconds.\n",
           MINIMUM_LOAD_WEIGHT,
           userConfirmSeconds);
  Serial.print(calPrompt);

  calCtx.mode              = CalMode::REZERO;
  calCtx.state             = CalState::WAIT_EMPTY;
  calCtx.stateStartMs      = millis();
  calCtx.stableEmptyChecks = 0;
  calCtx.measuredUnits     = 0.0f;
}

void tickCalibration() {
  // no active workflow — skip all scale reads and checks
  if (calCtx.state == CalState::IDLE) {
    return; 
  }

  // workflow - waiting on user to remove all weight from platen
  if (calCtx.state == CalState::WAIT_EMPTY) {
    
    // user never confirmed empty by removing weight or pressing 'q
    if ((millis() - calCtx.stateStartMs) >= USER_CONFIRM_TIMEOUT_MS) {
      
      // take one final reading to decide auto-confirm vs. abort
      calCtx.measuredUnits = readAveragedUnits(1, POLL_SAMPLES);
      
      if (!isfinite(calCtx.measuredUnits)) {
        printScaleNotReadyDiagnostic("empty-scale confirmation");
        Serial.println("Calibration cancelled.");
        calCtx.state = CalState::IDLE;
        calCtx.mode  = CalMode::NONE;
        return;
      }

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

    // Poll each tick to detect empty early (before timeout expires)
    calCtx.measuredUnits = readAveragedUnits(1, POLL_SAMPLES);

    if (!isfinite(calCtx.measuredUnits)) {
      printScaleNotReadyDiagnostic("empty-scale confirmation");
      Serial.println("Calibration cancelled.");
      calCtx.state = CalState::IDLE;
      calCtx.mode  = CalMode::NONE;
      return;
    }

    bool emptyDetected = fabsf(calCtx.measuredUnits) <= MINIMUM_LOAD_WEIGHT;

    if (emptyDetected) {

      // require consecutive readings to suppress transient spikes
      calCtx.stableEmptyChecks++; 
      
      if (calCtx.stableEmptyChecks >= UNLOAD_CHECK_COUNT) {
        Serial.println("Empty scale auto-detected, continuing with calibration.");
        Serial.println();
        transitionFromWaitEmpty();
      }
    } else {
      // non-empty reading breaks streak; must restart count
      calCtx.stableEmptyChecks = 0; 
    }
    return;
  }

  // workflow - waiting on user to place known weight on scale after empty confirmation
  if (calCtx.state == CalState::WAIT_LOAD) {
    unsigned long elapsedMs = millis() - calCtx.stateStartMs;

    // If the computed threshold is too strict, progressively relax it through
    // the wait window so a valid placed load can still trigger.
    float midWindowThreshold = MINIMUM_LOAD_THRESHOLD * 0.5f;
    float lateWindowThreshold = MINIMUM_LOAD_THRESHOLD * 0.25f;

    if (elapsedMs >= ((EMPTY_CONFIRM_TIMEOUT_MS * 3UL) / 4UL) &&
        calCtx.loadDetectThreshold > lateWindowThreshold) {
      calCtx.loadDetectThreshold = lateWindowThreshold;
    } else if (elapsedMs >= (EMPTY_CONFIRM_TIMEOUT_MS / 2UL) &&
               calCtx.loadDetectThreshold > midWindowThreshold) {
      calCtx.loadDetectThreshold = midWindowThreshold;
    }

    // Poll each tick to detect load placement as soon as possible
    calCtx.measuredUnits = readAveragedUnits(1, POLL_SAMPLES);

    if (!isfinite(calCtx.measuredUnits)) {
      printScaleNotReadyDiagnostic("weight placement detection");
      Serial.println("Calibration cancelled.");
      calCtx.state = CalState::IDLE;
      calCtx.mode  = CalMode::NONE;
      return;
    }

    // below noise-derived threshold — no real load yet
    if (fabsf(calCtx.measuredUnits) < calCtx.loadDetectThreshold) {
      if (elapsedMs >= EMPTY_CONFIRM_TIMEOUT_MS) {
        Serial.println("Weight placement timed out; calibration cancelled.");
        calCtx.state = CalState::IDLE;
        calCtx.mode  = CalMode::NONE;
      }
      return; 
    }

    char buf[80];
    unsigned long settleSeconds = CAL_SETTLE_DELAY_MS / 1000UL;
    snprintf(buf, sizeof(buf), "Weight detected. Settling for %lu seconds before measuring...\n",
             settleSeconds);
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

    if (calCtx.mode == CalMode::AUTO) {
      Serial.println("Measuring stable reading...");

      // take final measurement to compute calibration factor
      calCtx.measuredUnits = readAveragedUnits(CAL_SAMPLES, LIVE_SAMPLES);

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
      float verifiedUnits = readAveragedUnits(CAL_SAMPLES, LIVE_SAMPLES);

      char buf[160];
      snprintf(buf, sizeof(buf),
               "Initial calibration factor estimate: %.2f\n"
               "Verified reading: %.2f lbs\n"
               "Automatic calibration complete, computed calibration factor: %.2f\n",
               calibrationFactor, verifiedUnits, calibrationFactor);
      Serial.print(buf);

      if (!saveToEeprom(calibrationFactor, CAL_EEPROM_MAGIC, CAL_EEPROM_MAGIC_ADDR, CAL_EEPROM_VALUE_ADDR)) {
        Serial.println("Failed to save calibration to EEPROM.");
      } else {
        Serial.println("Calibration saved to EEPROM.");
      }

      calCtx.state = CalState::IDLE;
      calCtx.mode  = CalMode::NONE;
    }

    if (calCtx.mode == CalMode::MANUAL) {
      // serial input handled by handleCalibrationInput()
      Serial.print("Adjust calibration until the reading matches the known weight.\n"
                   "Send '+' to increase calibration factor\n"
                   "Send '-' to decrease calibration factor\n"
                   "(step halves on direction reversal)\n"
                   "Send 's' to save and finish manual calibration.\n"
                   "Send 'q' to cancel manual calibration without saving.\n");

      // force first print on next ADJUSTING tick
      calCtx.hasManualDisplay = false;
      calCtx.state            = CalState::ADJUSTING;
    }

    return;
  }

  // workflow - interactive manual calibration adjustment
  if (calCtx.state == CalState::ADJUSTING) {

    // apply current factor before reading so display reflects latest adjustment
    scale.set_scale(calibrationFactor);

    // skip the rest of the logic until the first valid reading is available after more iteration(s)
    // guards against HX711 becoming uninitialized due to power down and/or disconnection
    if (!scale.is_ready()) {
      return;
    }

    float readingWeight = scale.get_units();

    // Quantize to integers for change detection — float comparison is unreliable across loop ticks
    int readingTenth      = static_cast<int>(lroundf(readingWeight         * 10.0f));
    int factorHundredth   = static_cast<int>(lroundf(calibrationFactor     * 100.0f));
    int stepTenThousandth = static_cast<int>(lroundf(calCtx.adjustmentStep * 10000.0f));

    // always print once on first entry
    if (!calCtx.hasManualDisplay || readingTenth != calCtx.lastReadingTenth || factorHundredth != calCtx.lastFactorHundredth || stepTenThousandth != calCtx.lastStepTenThousandth) {
      char buf[80];
      snprintf(buf, sizeof(buf), "Reading: %.1f lbs  factor: %.2f  step: %.4f\n",
               readingWeight, calibrationFactor, calCtx.adjustmentStep);
      Serial.print(buf);

      // suppress repeat prints until a value actually changes
      calCtx.hasManualDisplay      = true; 
      calCtx.lastReadingTenth      = readingTenth;
      calCtx.lastFactorHundredth   = factorHundredth;
      calCtx.lastStepTenThousandth = stepTenThousandth;
    }
  }
}

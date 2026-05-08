/**
 * @file input_propane_weight.cpp
 * @author Gerald Manweiler (gerald@domain.com)
 * 
 * @brief Implementation of the user input workflow to update the maximum legal propane weight.
 * 
 * @details Defines the function to initiate the propane weight update workflow, which prompts the user for input, validates it, and saves it to EEPROM if confirmed. Also includes the function to handle serial input for
 * this workflow, which processes user input character by character in a non-blocking manner.
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
 * @section Local library headers
 */

#include "config.h"                                         // Configuration constants for the ESP32-based propane level scale
#include "src/eeprom_store.h"                               // EEPROM storage functions
#include "src/parsing_utils.h"                              // Utility functions for validating and parsing input values 
#include "src/scale_io.h"                                   // Input/output functions for user workflows and HX711 interactions
#include "src/workflows/input_context.h"                    // Input context definitions for non-blocking user input workflows

/**
 * @section External Global State Variables and Functions
 */

extern float maxPropane;                                    // Maximum legal propane weight in pounds, defined in PropaneScale.ino
extern void resetInputContext();                            // Resets the input context to its default state, defined in PropaneScale.ino

void handlePropaneWeightInput(char incoming) {
  // Ignore carriage return characters to prevent interference with parsing logic
  if (incoming == '\r') {
    return;
  }

  // workflow - waiting for save confirmation after valid numeric entry
  // ignore newlines, only accept cancelling & saving
  if (inputCtx.state == InputState::WAIT_SAVE_CONFIRM) {
    if (incoming == '\n') {
      return;
    }

    if (incoming == 'q' || incoming == 'Q') {
      Serial.println("Max propane weight update cancelled.");
      resetInputContext();
      return;
    }

    if (incoming == 's' || incoming == 'S') {
      
      float deltaMaxPropane = fabsf(inputCtx.parsedValue - maxPropane);
      
      if (deltaMaxPropane >= maxPropane * CHANGE_WARN_PCT) {
        char warnBuf[192];
        snprintf(warnBuf, sizeof(warnBuf),
                 "Warning: Max propane weight changed by %.2f lbs. Verify tank tare ('t') and known calibration weight ('k') are still accurate, and recalibrate ('a' or 'm') if needed.\n",
                 deltaMaxPropane);
        Serial.print(warnBuf);
      }

      maxPropane = inputCtx.parsedValue;
      bool eepromSuccess = saveToEeprom(maxPropane, MAX_PROPANE_EEPROM_MAGIC, MAX_PROPANE_EEPROM_MAGIC_ADDR, MAX_PROPANE_EEPROM_VALUE_ADDR);
      
      if (!eepromSuccess) {
        Serial.println("Failed to save max propane weight to EEPROM.");
      } else {
        char buf[48];
        snprintf(buf, sizeof(buf), "Max propane weight updated: %.2f lbs\n", maxPropane);
        Serial.print(buf);
      }
      resetInputContext();
      return;
    }

    Serial.println("Invalid response. Send 's' to save, or 'q' to cancel.");
    return;
  }

  // workflow - collecting numeric value text 
  // waiting cancel, newline, or save command, 
  // ignore other characters except for buffering valid numeric input
  if (inputCtx.state == InputState::ENTER_VALUE) {

    // got cancel as first character, treat it as a quick cancel command
    if ((incoming == 'q' || incoming == 'Q') && inputCtx.index == 0) {
      Serial.println("Max propane weight update cancelled.");
      resetInputContext();
      return;
    }

    // got the newline, attempt to parse the entered text as a float value, validate it, and if valid prompt user to save or cancel
    if (incoming == '\n') {
      inputCtx.buffer[inputCtx.index] = '\0';

      // if the index is 1 and the only character is 'q', treat it as a cancel command to allow quick cancellation
      if (inputCtx.index == 1 && (inputCtx.buffer[0] == 'q' || inputCtx.buffer[0] == 'Q')) {
        Serial.println("Max propane weight update cancelled.");
        resetInputContext();
        return;
      }

      // defensive initialization ensures parsedValue is set to a known state before parsing attempts
      // parsing function modifies by reference on parsing success, but need failsafe on parsing failure
      inputCtx.parsedValue = 0.0f;

      // local scope because only needed for this "cancel" parsing attempt,
      // waste of ram/cpu cycles & more logic required to instantiate/initialize & use in broader scope
      bool parseSuccess = parseNonNegativeFloat(inputCtx.buffer, inputCtx.parsedValue);
      bool validValue = isValidBoundedFloat(inputCtx.parsedValue, MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT);

      if (parseSuccess && validValue) {
        char buf[96];
        snprintf(buf, sizeof(buf), "New max propane weight entered: %.2f lbs\nSend 's' to save this value to EEPROM, or 'q' to cancel.\n", inputCtx.parsedValue);
        Serial.print(buf);
        inputCtx.state = InputState::WAIT_SAVE_CONFIRM;
        inputCtx.index = 0;
        inputCtx.buffer[0] = '\0';
        return;
      }

      char buf[96];
      snprintf(buf, sizeof(buf), "Invalid max propane weight. Enter a number from %.2f to %.2f lbs, or 'q' to cancel.\n", MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT);
      Serial.print(buf);
      inputCtx.index = 0;
      inputCtx.buffer[0] = '\0';
      return;
    }

    // user trying to save
    if (incoming == 's' || incoming == 'S') {
      if (inputCtx.index == 0) {
        Serial.println("Enter a max propane weight first, then send 's' to save.");
        return;
      }

      // Ensure buffer is null-terminated before parsing since function expects a C-style string
      inputCtx.buffer[inputCtx.index] = '\0';

      // local scope because waste of ram/cpu cycles to re-use variable from "cancel" parsing attempt
      // since this is a separate user action with separate validation requirements
      bool parseSuccess = parseNonNegativeFloat(inputCtx.buffer, inputCtx.parsedValue);
      bool validValue = isValidBoundedFloat(inputCtx.parsedValue, MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT);

      if (!parseSuccess || !validValue) {
        char buf[96];
        snprintf(buf, sizeof(buf), "Invalid max propane weight. Enter a number from %.2f to %.2f lbs, or 'q' to cancel.\n", MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT);
        Serial.print(buf);
        inputCtx.index = 0;
        inputCtx.buffer[0] = '\0';
        return;
      }

      float deltaMaxPropane = fabsf(inputCtx.parsedValue - maxPropane);

      if (deltaMaxPropane >= maxPropane * CHANGE_WARN_PCT) {
        char warnBuf[192];
        snprintf(warnBuf, sizeof(warnBuf),
                 "Warning: Max propane weight changed by %.2f lbs. Verify tank tare ('t') and known calibration weight ('k') are still accurate, and recalibrate ('a' or 'm') if needed.\n",
                 deltaMaxPropane);
        Serial.print(warnBuf);
      }

      maxPropane = inputCtx.parsedValue;
      bool eepromSuccess = saveToEeprom(maxPropane, MAX_PROPANE_EEPROM_MAGIC, MAX_PROPANE_EEPROM_MAGIC_ADDR, MAX_PROPANE_EEPROM_VALUE_ADDR);

      if (!eepromSuccess) {
        Serial.println("Failed to save max propane weight to EEPROM.");
      } else {
        char buf[48];
        snprintf(buf, sizeof(buf), "Max propane weight updated: %.2f lbs\n", maxPropane);
        Serial.print(buf);
      }
      resetInputContext();
      return;
    }

    bool isValidChar = inputCtx.index < static_cast<int>(sizeof(inputCtx.buffer) - 1);

    if (isValidChar) {
      inputCtx.buffer[inputCtx.index++] = incoming;
    } else {
      Serial.println("Input too long. Enter a shorter number, or 'q' to cancel.");
      inputCtx.index = 0;
      inputCtx.buffer[0] = '\0';
    }
    return;
  }

  Serial.println("Max propane weight input state invalid; cancelling workflow.");
  resetInputContext();
}

void propaneWeightUpdate() {
  if (inputCtx.mode != InputMode::NONE) {
    Serial.println("Propane weight input workflow already in progress. Send 'q' to cancel first.");
    return;
  }

  flushSerialInput();

  inputCtx.mode = InputMode::PROPANE_WEIGHT;
  inputCtx.state = InputState::ENTER_VALUE;
  inputCtx.index = 0;
  inputCtx.parsedValue = 0.0f;
  inputCtx.buffer[0] = '\0';
  unsigned long userConfirmSeconds = USER_CONFIRM_TIMEOUT_MS / 1000UL;
  char prompt[192];
  snprintf(prompt, sizeof(prompt),
           "\nCurrent max propane weight: %.2f lbs\n"
           "Enter new max propane weight in lbs (%.2f to %.2f), then press Enter.\n"
           "After entry, send 's' to save or 'q' to cancel.\n",
           maxPropane, MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT);
  Serial.print(prompt);
}

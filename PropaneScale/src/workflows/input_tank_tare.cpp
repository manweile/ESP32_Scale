/**
 * @file input_known_weight.cpp
 * @author Gerald Manweiler
 * 
 * @brief Header file for handling user input workflow to update the tank tare value.
 * 
 * @details Processes one serial character per loop() iteration,
 * managing the stepwise collection of a new tank tare value and user confirmation to save or cancel.
 * 
 * @version 0.1
 * @date 2026-05-07
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
#include "src/eeprom_store.h"                               // EEPROM storage functions for loading and saving calibration and tare values
#include "src/parsing_utils.h"                              // Utility functions for validating and parsing input values
#include "src/scale_io.h"                                   // Input/output functions for user workflows and HX711 interactions
#include "src/workflows/input_context.h"                    // Input context definitions for non-blocking user input workflows

/**
 * @section External Global State Variables and Functions
 */

extern float tankTare;                                      // Tare weight of the empty propane tank in pounds
extern void resetInputContext();                            // Resets the input context to its initial state

void handleTankTareInput(char incoming) {
  // ignore carriage return characters to prevent interference with parsing logic
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
      Serial.println("Tank tare update cancelled.");
      resetInputContext();
      return;
    }

    if (incoming == 's' || incoming == 'S') {

      float deltaTankTare = fabsf(inputCtx.parsedValue - tankTare);

      if (deltaTankTare >= tankTare * CHANGE_WARN_PCT) {
        char warnBuf[192];
        snprintf(warnBuf, sizeof(warnBuf),
                 "Warning: Tank tare changed by %.2f lbs.\nUpdate max legal propane ('p') and known calibration weight ('k'), then recalibrate ('a' or 'm') to maintain accuracy.\n",
                 deltaTankTare);
        Serial.print(warnBuf);
      }

      tankTare = inputCtx.parsedValue;
      bool eepromSuccess = saveToEeprom(tankTare, TARE_EEPROM_MAGIC, TARE_EEPROM_MAGIC_ADDR, TARE_EEPROM_VALUE_ADDR);
      
      if (!eepromSuccess) {
        Serial.println("Failed to save tank tare to EEPROM.");
      } else {
        char buf[56];
        snprintf(buf, sizeof(buf), "Tank tare saved successfully: %.2f lbs\n", tankTare);
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
      Serial.println("Tank tare update cancelled.");
      resetInputContext();
      return;
    }

    // got the newline, attempt to parse the entered text as a float value, validate it, and if valid prompt user to save or cancel
    if (incoming == '\n') {
      inputCtx.buffer[inputCtx.index] = '\0';

      // if the index is 1 and the only character is 'q', treat it as a cancel command to allow quick cancellation
      if (inputCtx.index == 1 && (inputCtx.buffer[0] == 'q' || inputCtx.buffer[0] == 'Q')) {
        Serial.println("Tank tare update cancelled.");
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
        snprintf(buf, sizeof(buf), "New tank tare entered: %.2f lbs\nSend 's' to save this value to EEPROM, or 'q' to cancel.\n", inputCtx.parsedValue);
        Serial.print(buf);
        inputCtx.state = InputState::WAIT_SAVE_CONFIRM;
        inputCtx.index = 0;
        inputCtx.buffer[0] = '\0';
        return;
      }

      char buf[96];
      snprintf(buf, sizeof(buf), "Invalid tank tare. Enter a number from %.2f to %.2f lbs, or 'q' to cancel.\n", MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT);
      Serial.print(buf);
      inputCtx.index = 0;
      inputCtx.buffer[0] = '\0';
      return;
    }

    // user trying to save
    if (incoming == 's' || incoming == 'S') {
      if (inputCtx.index == 0) {
        Serial.println("Enter a tare value first, then send 's' to save.");
        return;
      }

      // ensure buffer is null-terminated before parsing since function expects a C-style string
      inputCtx.buffer[inputCtx.index] = '\0';

      // local scope because waste of ram/cpu cycles to re-use variable from "cancel" parsing attempt
      // since this is a separate user action with separate validation requirements
      bool parseSuccess = parseNonNegativeFloat(inputCtx.buffer, inputCtx.parsedValue);
      bool validValue = isValidBoundedFloat(inputCtx.parsedValue, MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT);

      if (!parseSuccess || !validValue) {
        char buf[96];
        snprintf(buf, sizeof(buf), "Invalid tank tare. Enter a number from %.2f to %.2f lbs, or 'q' to cancel.\n", MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT);
        Serial.print(buf);
        inputCtx.index = 0;
        inputCtx.buffer[0] = '\0';
        return;
      }

      float deltaTankTare = fabsf(inputCtx.parsedValue - tankTare);

      if (deltaTankTare >= tankTare * CHANGE_WARN_PCT) {
        char warnBuf[192];
        snprintf(warnBuf, sizeof(warnBuf),
                 "Warning: Tank tare changed by %.2f lbs.\nUpdate max legal propane ('p') and known calibration weight ('k'), then recalibrate ('a' or 'm') to maintain accuracy.\n",
                 deltaTankTare);
        Serial.print(warnBuf);
      }

      tankTare = inputCtx.parsedValue;
      bool eepromSuccess = saveToEeprom(tankTare, TARE_EEPROM_MAGIC, TARE_EEPROM_MAGIC_ADDR, TARE_EEPROM_VALUE_ADDR);

      if (!eepromSuccess) {
        Serial.println("Failed to save tank tare to EEPROM.");
      } else {
        char buf[56];
        snprintf(buf, sizeof(buf), "Tank tare saved successfully: %.2f lbs\n", tankTare);
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

  Serial.println("Tank tare input state invalid; cancelling workflow.");
  resetInputContext();
}

void tankTareUpdate() {
  if (inputCtx.mode != InputMode::NONE) {
    Serial.println("Tank tare input workflow already in progress. Send 'q' to cancel first.");
    return;
  }

  flushSerialInput();

  inputCtx.mode = InputMode::TANK_TARE;
  inputCtx.state = InputState::ENTER_VALUE;
  inputCtx.index = 0;
  inputCtx.parsedValue = 0.0f;
  inputCtx.buffer[0] = '\0';

  char prompt[192];
  snprintf(prompt, sizeof(prompt),
           "\nCurrent tank tare: %.2f lbs\n"
           "Enter new tank tare in lbs (%.2f to %.2f), then press Enter.\n"
           "After entry, send 's' to save or 'q' to cancel.\n",
           tankTare, MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT);
  Serial.print(prompt);
}
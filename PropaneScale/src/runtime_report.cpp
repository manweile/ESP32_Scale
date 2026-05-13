/**
 * @file runtime_report.cpp
 * @author Gerald Manweiler
 * 
 * @brief Implementation of runtime reporting functions for the ESP32-based propane level scale application.
 * 
 * @details Implements functions for reporting current EEPROM values, resetting EEPROM to defaults, and displaying the help menu.
 * 
 * @version 0.1
 * @date 2026-05-12
 * @copyright Copyright (c) 2026 Gerald Manweiler
 */

/**
 * @section Standard library headers
 */

#include <Arduino.h>                                         // Arduino core library for serial communication and basic functions
#include <stdio.h>                                            // Standard I/O library for string formatting functions

/** 
 * @section Local library headers
 */

#include "config.h"                                         // Configuration constants for the ESP32-based propane level scale
#include "src/eeprom_store.h"                               // EEPROM storage functions
#include "src/runtime_report.h"                             // Declarations for runtime reporting functions
#include "src/workflows/input_context.h"                    // Input context definitions for non-blocking user input workflows
#include "src/workflows/workflows_contexts.h"               // Context definitions for non-blocking workflows

/**
 * @section External Global State Variables and Functions
 */

extern float calibrationFactor;
extern float knownWeight;
extern float maxPropane;
extern float tankTare;

extern bool eepromReady;

/**
 * @section Definitions for runtime reporting functions
 */

void currentRuntimeValues() {
  Serial.println();
  Serial.println(APP_TITLE);
  Serial.println();
  
  Serial.println("Current Runtime Values");

  Serial.print("EEPROM ready: ");
  Serial.println(eepromReady ? "yes" : "no");

  Serial.print("Calibration factor: ");
  Serial.println(calibrationFactor, 2);

  Serial.print("Known calibration weight: ");
  Serial.print(knownWeight, 2);
  Serial.println(" lbs");

  Serial.print("Max propane weight: ");
  Serial.print(maxPropane, 2);
  Serial.println(" lbs");

  Serial.print("Tank tare: ");
  Serial.print(tankTare, 2);
  Serial.println(" lbs");

  const char* calModeName = "?";
  switch (calCtx.mode) {
    case CalMode::NONE:   calModeName = "NONE";   break;
    case CalMode::AUTO:   calModeName = "AUTO";   break;
    case CalMode::MANUAL: calModeName = "MANUAL"; break;
    case CalMode::REZERO: calModeName = "REZERO"; break;
  }
  Serial.print("Calibration mode: ");
  Serial.print(static_cast<int>(calCtx.mode));
  Serial.print(" (");
  Serial.print(calModeName);
  Serial.println(")");

  const char* calStateName = "?";
  switch (calCtx.state) {
    case CalState::IDLE:       calStateName = "IDLE";       break;
    case CalState::WAIT_EMPTY: calStateName = "WAIT_EMPTY"; break;
    case CalState::WAIT_LOAD:  calStateName = "WAIT_LOAD";  break;
    case CalState::SETTLING:   calStateName = "SETTLING";   break;
    case CalState::ADJUSTING:  calStateName = "ADJUSTING";  break;
  }
  Serial.print("Calibration state: ");
  Serial.print(static_cast<int>(calCtx.state));
  Serial.print(" (");
  Serial.print(calStateName);
  Serial.println(")");

  const char* levelStateName = "?";
  switch (levelCtx.state) {
    case LevelState::IDLE:      levelStateName = "IDLE";      break;
    case LevelState::WAIT_LOAD: levelStateName = "WAIT_LOAD"; break;
    case LevelState::SETTLING:  levelStateName = "SETTLING";  break;
    case LevelState::READING:   levelStateName = "READING";   break;
  }
  Serial.print("Level state: ");
  Serial.print(static_cast<int>(levelCtx.state));
  Serial.print(" (");
  Serial.print(levelStateName);
  Serial.println(")");

  const char* tareStateName = "?";
  switch (tareCtx.state) {
    case TareState::IDLE:        tareStateName = "IDLE";        break;
    case TareState::WAIT_STABLE: tareStateName = "WAIT_STABLE"; break;
    case TareState::TARE:        tareStateName = "TARE";        break;
    case TareState::SKIP:        tareStateName = "SKIP";        break;
  }
  Serial.print("Tare state: ");
  Serial.print(static_cast<int>(tareCtx.state));
  Serial.print(" (");
  Serial.print(tareStateName);
  Serial.println(")");

  const char* inputModeName = "?";
  switch (inputCtx.mode) {
    case InputMode::NONE:           inputModeName = "NONE";           break;
    case InputMode::TANK_TARE:      inputModeName = "TANK_TARE";      break;
    case InputMode::PROPANE_WEIGHT: inputModeName = "PROPANE_WEIGHT"; break;
    case InputMode::KNOWN_WEIGHT:   inputModeName = "KNOWN_WEIGHT";   break;
  }
  Serial.print("Input mode: ");
  Serial.print(static_cast<int>(inputCtx.mode));
  Serial.print(" (");
  Serial.print(inputModeName);
  Serial.println(")");

  const char* inputStateName = "?";
  switch (inputCtx.state) {
    case InputState::IDLE:              inputStateName = "IDLE";              break;
    case InputState::ENTER_VALUE:       inputStateName = "ENTER_VALUE";       break;
    case InputState::WAIT_SAVE_CONFIRM: inputStateName = "WAIT_SAVE_CONFIRM"; break;
  }
  Serial.print("Input state: ");
  Serial.print(static_cast<int>(inputCtx.state));
  Serial.print(" (");
  Serial.print(inputStateName);
  Serial.println(")");
}

void eepromValues() {
  if (!eepromReady) {
    Serial.println("EEPROM is not initialized; no saved values can be read.");
    return;
  }

  Serial.println();
  Serial.println("EEPROM Saved Values");

  printEepromField("Calibration factor",
                   CAL_EEPROM_MAGIC_ADDR, CAL_EEPROM_MAGIC, CAL_EEPROM_VALUE_ADDR,
                   CAL_FACTOR_ABS_MIN, CAL_FACTOR_ABS_MAX, true, nullptr);

  printEepromField("Known calibration weight",
                   KNOWN_WEIGHT_EEPROM_MAGIC_ADDR, KNOWN_WEIGHT_EEPROM_MAGIC, KNOWN_WEIGHT_EEPROM_VALUE_ADDR,
                   MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT, false, " lbs");

  printEepromField("Max propane weight",
                   MAX_PROPANE_EEPROM_MAGIC_ADDR, MAX_PROPANE_EEPROM_MAGIC, MAX_PROPANE_EEPROM_VALUE_ADDR,
                   MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT, false, " lbs");

  printEepromField("Tank tare",
                   TARE_EEPROM_MAGIC_ADDR, TARE_EEPROM_MAGIC, TARE_EEPROM_VALUE_ADDR,
                   MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT, false, " lbs");

  float savedRuntimeOffset = 0.0f;
  if (loadFromEeprom(savedRuntimeOffset,
                     HX711_OFFSET_EEPROM_MAGIC_ADDR,
                     HX711_OFFSET_EEPROM_MAGIC,
                     HX711_OFFSET_EEPROM_VALUE_ADDR)) {
    Serial.print("Runtime tare offset (HX711 counts): ");
    Serial.println(savedRuntimeOffset, 0);
  } else {
    Serial.println("Runtime tare offset (HX711 counts): <invalid or not set>");
  }
}

void helpMenu() {
  char helpText[768];
  int helpTextLen = snprintf(helpText,
                             sizeof(helpText),
                             "\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n",
                             CMD_AUTO_CAL_MSG,
                             CMD_CURRENT_VALUES_MSG,
                             CMD_DEFAULT_EEPROM_MSG,
                             CMD_EEPROM_MSG,
                             CMD_HELP_MSG,
                             CMD_KNOWN_WEIGHT_MSG,
                             CMD_LEVEL_MSG,
                             CMD_MANUAL_CAL_MSG,
                             CMD_PROPANE_WEIGHT_MSG,
                             CMD_REZERO_MSG,
                             CMD_TANK_TARE_MSG);

  if (helpTextLen > 0) {
    Serial.print(helpText);
  }
}

void printStartupSummary() {
  char startupSummary[384];
  int startupSummaryLen = snprintf(startupSummary,
                                   sizeof(startupSummary),
                                   "\n%s\n\n"
                                   "Loaded calibration factor from EEPROM: %.2f\n"
                                   "Loaded known calibration weight from EEPROM: %.2f lbs\n"
                                   "Loaded max propane weight from EEPROM: %.2f lbs\n"
                                   "Loaded tank tare from EEPROM: %.2f lbs\n\n",
                                   APP_TITLE,
                                   calibrationFactor,
                                   knownWeight,
                                   maxPropane,
                                   tankTare);

  if (startupSummaryLen > 0) {
    Serial.print(startupSummary);
  }
}
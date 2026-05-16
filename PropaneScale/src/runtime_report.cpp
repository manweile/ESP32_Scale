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

#include <Arduino.h>                                        // Arduino core library for serial communication and basic functions
#include <stdarg.h>                                         // Variadic formatting helpers for queued runtime reports
#include <stdio.h>                                          // Standard I/O library for string formatting functions

/** 
 * @section Local library headers
 */

#include "config.h"                                         // Configuration constants for the ESP32-based propane level scale
#include "src/eeprom_store.h"                               // EEPROM storage functions
#include "src/runtime_report.h"                             // Declarations for runtime reporting functions
#include "src/scale_io.h"                                   // Non-blocking serial output helpers
#include "src/workflows/input_context.h"                    // Input context definitions for non-blocking user input workflows
#include "src/workflows/workflows_contexts.h"               // Context definitions for non-blocking workflows

/**
 * @section External Global State Variables
 */

extern float calibrationFactor;
extern bool eepromReady;
extern float knownWeight;
extern float maxPropane;
extern float tankTare;

/**
 * @section Private Helper Functions
 */

/**
 * @brief Appends a formatted string to a buffer, updating the used length.
 * 
 * @details Helper function for building runtime reports with multiple pieces of information. 
 * Uses vsnprintf to safely append formatted data to the buffer, ensuring no overflow occurs. 
 * Updates the used length of the buffer accordingly.
 * 
 * @param buffer {char*} The buffer to append the formatted string to.
 * @param bufferSize {size_t} The total size of the buffer.
 * @param usedLength {int&} The current length of data in the buffer, updated after appending.
 * @param format {const char*} The format string, similar to printf.
 * @param ... Additional arguments for the format string.
 * @return true if the string was successfully appended, false otherwise.
 * 
 * @throws {none} This function does not throw exceptions. It returns false if the buffer is full or if formatting fails, and true if the string was successfully appended.
 */
static bool appendReportf(char* buffer, size_t bufferSize, int& usedLength, const char* format, ...) {
  if (usedLength < 0 || static_cast<size_t>(usedLength) >= bufferSize) {
    return false;
  }

  // va_list handles the variable arguments for formatting the string safely into the buffer
  va_list args;
  va_start(args, format);
  int written = vsnprintf(buffer + usedLength, bufferSize - static_cast<size_t>(usedLength), format, args);
  va_end(args);

  if (written < 0) {
    return false;
  }

  if (static_cast<size_t>(written) >= (bufferSize - static_cast<size_t>(usedLength))) {
    usedLength = static_cast<int>(bufferSize - 1);
    buffer[usedLength] = '\0';
    return false;
  }

  usedLength += written;
  return true;
}

/**
 * @brief Returns the name of the calibration mode.
 * 
 * @details Helper function for translating CalMode enum values to human-readable strings for runtime reports.
 * 
 * @param mode {CalMode} The calibration mode.
 * @return const char* The name of the calibration mode.
 * 
 * @throws {none} This function does not throw exceptions. It returns a string representation of the calibration mode, or "?" if the mode is unrecognized.
 */
static const char* calModeNameFor(CalMode mode) {
  switch (mode) {
    case CalMode::NONE:   return "NONE";
    case CalMode::AUTO:   return "AUTO";
    case CalMode::MANUAL: return "MANUAL";
    case CalMode::REZERO: return "REZERO";
  }

  // Return "?" for unrecognized values to avoid printing raw integers in reports, 
  // which is less user-friendly and may indicate a bug if it happens.
  return "?";
}

/** 
 * @brief Returns the name of the calibration state.
 * 
 * @details Helper function for translating CalState enum values to human-readable strings for runtime reports.
 * 
 * @param state {CalState} The calibration state.
 * @return const char* The name of the calibration state.
 * 
 * @throws {none} This function does not throw exceptions. It returns a string representation of the calibration state, or "?" if the state is unrecognized.
 */
static const char* calStateNameFor(CalState state) {
  switch (state) {
    case CalState::IDLE:       return "IDLE";
    case CalState::WAIT_EMPTY: return "WAIT_EMPTY";
    case CalState::WAIT_LOAD:  return "WAIT_LOAD";
    case CalState::SETTLING:   return "SETTLING";
    case CalState::ADJUSTING:  return "ADJUSTING";
  }

  return "?";
}

/**
 * @brief Returns the name of the input mode.
 * 
 * @details Helper function for translating InputMode enum values to human-readable strings for runtime reports.
 * 
 * @param mode {InputMode} The input mode.
 * @return const char* The name of the input mode.
 * 
 * @throws {none} This function does not throw exceptions. It returns a string representation of the input mode, or "?" if the mode is unrecognized.
 */
static const char* inputModeNameFor(InputMode mode) {
  switch (mode) {
    case InputMode::NONE:           return "NONE";
    case InputMode::TANK_TARE:      return "TANK_TARE";
    case InputMode::PROPANE_WEIGHT: return "PROPANE_WEIGHT";
    case InputMode::KNOWN_WEIGHT:   return "KNOWN_WEIGHT";
  }

  return "?";
}

/**
 * @brief Returns the name of the input state.
 * 
 * @details Helper function for translating InputState enum values to human-readable strings for runtime reports.
 * 
 * @param state {InputState} The input state.
 * @return const char* The name of the input state.
 * 
 * @throws {none} This function does not throw exceptions. It returns a string representation of the input state, or "?" if the state is unrecognized.
 */
static const char* inputStateNameFor(InputState state) {
  switch (state) {
    case InputState::IDLE:              return "IDLE";
    case InputState::ENTER_VALUE:       return "ENTER_VALUE";
    case InputState::WAIT_SAVE_CONFIRM: return "WAIT_SAVE_CONFIRM";
  }

  return "?";
}

/**
 * @brief Returns the name of the level state.
 * 
 * @details Helper function for translating LevelState enum values to human-readable strings for runtime reports.
 * 
 * @param state {LevelState} The level state.
 * @return const char* The name of the level state.
 * 
 * @throws {none} This function does not throw exceptions. It returns a string representation of the level state, or "?" if the state is unrecognized.
 */
static const char* levelStateNameFor(LevelState state) {
  switch (state) {
    case LevelState::IDLE:      return "IDLE";
    case LevelState::WAIT_LOAD: return "WAIT_LOAD";
    case LevelState::SETTLING:  return "SETTLING";
    case LevelState::READING:   return "READING";
  }

  return "?";
}

/**
 * @brief Returns the name of the tare state.
 * 
 * @details Helper function for translating TareState enum values to human-readable strings for runtime reports.
 * 
 * @param state {TareState} The tare state.
 * @return const char* The name of the tare state.
 * 
 * @throws {none} This function does not throw exceptions. It returns a string representation of the tare state, or "?" if the state is unrecognized.
 */
static const char* tareStateNameFor(TareState state) {
  switch (state) {
    case TareState::IDLE:        return "IDLE";
    case TareState::WAIT_STABLE: return "WAIT_STABLE";
    case TareState::TARE:        return "TARE";
    case TareState::SKIP:        return "SKIP";
  }

  return "?";
}

/**
 * @section Definitions for runtime reporting functions
 */

void currentRuntimeValues() {
  char report[768];
  int usedLength = 0;

  appendReportf(report, sizeof(report), usedLength,
                "\n%s\n\n"
                "Current Runtime Values\n"
                "EEPROM ready: %s\n"
                "Calibration factor: %.2f\n"
                "Known calibration weight: %.2f lbs\n"
                "Max propane weight: %.2f lbs\n"
                "Tank tare: %.2f lbs\n"
                "Calibration mode: %d (%s)\n"
                "Calibration state: %d (%s)\n"
                "Level state: %d (%s)\n"
                "Tare state: %d (%s)\n"
                "Input mode: %d (%s)\n"
                "Input state: %d (%s)\n",
                APP_TITLE,
                eepromReady ? "yes" : "no",
                calibrationFactor,
                knownWeight,
                maxPropane,
                tankTare,
                static_cast<int>(calCtx.mode),
                calModeNameFor(calCtx.mode),
                static_cast<int>(calCtx.state),
                calStateNameFor(calCtx.state),
                static_cast<int>(levelCtx.state),
                levelStateNameFor(levelCtx.state),
                static_cast<int>(tareCtx.state),
                tareStateNameFor(tareCtx.state),
                static_cast<int>(inputCtx.mode),
                inputModeNameFor(inputCtx.mode),
                static_cast<int>(inputCtx.state),
                inputStateNameFor(inputCtx.state));

  queueSerialOutput(report);
}

void eepromValues() {
  if (!eepromReady) {
    queueSerialOutput("EEPROM is not initialized; no saved values can be read.\n");
    return;
  }

  queueSerialOutput("\nEEPROM Saved Values\n");

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
    char line[96];
    snprintf(line, sizeof(line), "Runtime tare offset (HX711 counts): %.0f\n", savedRuntimeOffset);
    queueSerialOutput(line);
  } else {
    queueSerialOutput("Runtime tare offset (HX711 counts): <invalid or not set>\n");
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
    queueSerialOutput(helpText);
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
    queueSerialOutput(startupSummary);
  }
}
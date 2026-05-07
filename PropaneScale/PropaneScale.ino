/**
 * @file PropaneScale.ino
 * @author Gerald Manweiler
 * 
 * @brief Main application file for ESP32-based propane level scale using HX711 amplifier.
 * 
 * @details Implements serial command interface for calibration and weight reporting, 
 * manages HX711 interactions, and applies calibration factors to convert raw readings to weight in pounds.
 * 
 * @version 0.1
 * @date 2024-06-01
 * @copyright Copyright (c) 2024 Gerald Manweiler
 */

/**
 * @section Standard library headers
 */

#include <EEPROM.h>                                         // EEPROM library for persistent storage of calibration and tare values

/**
 * @section Third party library headers
 */
#include "HX711.h"                                          // HX711 library for interfacing with the load cell amplifier to read weight data

/** 
 * @section Local library headers
 */
#include "config.h"                                         // Configuration constants for the ESP32-based propane level scale
#include "src/eeprom_store.h"                               // EEPROM storage functions
#include "src/parsing_utils.h"                              // Utility functions for validating and parsing input values
#include "src/scale_io.h"                                   // Input/output functions for user workflows and HX711 interactions

/**
 * @section Global Class Instances
 */
HX711 scale;                                                // HX711 instance for interacting with the load cell amplifier

/** 
 * @section Global State Variables
 */

float calibrationFactor = 0.0f;                             // Calibration factor for converting raw HX711 readings to weight in pounds
bool eepromReady = false;                                   // Flag to track if EEPROM was successfully initialized
float knownWeight = 0.0f;                                   // Known weight for calibration
float maxPropane = 0.0f;                                    // Maximum legal propane weight in pounds
float tankTare = 0.0f;                                      // Tare weight of the empty propane tank in pounds

/**
 * @section Calibration State Machine Enums, Structs & variables
 */

/**
 * @enum CalMode
 * 
 * @brief Enumeration of calibration modes.
 * 
 * @details Used to manage different calibration workflows initiated by the user.
 */
enum class CalMode  : uint8_t { 
  NONE = 0,                                                 /**< No calibration mode active */
  AUTO = 1,                                                 /**< Automatic calibration mode */
  MANUAL = 2,                                               /**< Manual calibration mode */
  REZERO = 3                                                /**< Rezero calibration mode */
};

/**
 * @enum CalState
 * 
 * @brief Enumeration of states for calibration workflows.
 * 
 * @details Used to manage multi-step calibration processes and user prompts during calibration.
 */
enum class CalState : uint8_t { 
  IDLE = 0,                                                 /**< Not currently in a calibration workflow, waiting for user input to start one */
  WAIT_EMPTY = 1,                                           /**< Waiting for the scale to be empty before starting calibration */
  WAIT_LOAD = 2,                                            /**< Waiting for a known weight to be placed on the scale */
  SETTLING = 3,                                             /**< Waiting for the placed weight to mechanically settle before measuring */
  ADJUSTING = 4                                             /**< Adjusting the calibration factor based on user input or measurements */
};

/**
 * @struct CalContext
 * 
 * @brief Struct to hold state for calibration workflows.
 * 
 * @details Contains variables that allow managing complex calibration workflows.
 */
struct CalContext {
  float         adjustmentStep     = 0.0f;                  /**< manual mode: current factor nudge size */
  bool          hasManualDisplay   = false;                 /**< manual mode: whether we have a prior display snapshot to compare against */
  int           lastDirection      = 0;                     /**< manual mode: +1 = last press +, -1 = last press - */
  int           lastFactorHundredth = 0;                    /**< manual mode: last displayed factor, scaled by 100 (2 decimal places) */
  int           lastReadingTenth   = 0;                     /**< manual mode: last displayed reading, scaled by 10 (1 decimal place) */
  int           lastStepTenThousandth = 0;                  /**< manual mode: last displayed step, scaled by 10000 (4 decimal places) */
  float         loadDetectThreshold = 0.0f;                 /**< noise-derived threshold for weight detection */
  float         measuredUnits      = 0.0f;                  /**< last averaged reading in pounds */
  float         minStep            = 0.0f;                  /**< manual mode: floor for adjustmentStep */
  CalMode       mode               = CalMode::NONE;         /**< current calibration mode, or NONE when not in a calibration workflow */
  float         originalCalibrationFactor = 0.0f;           /**< manual mode: calibration factor captured at start for cancel/restore */
  int           stableEmptyChecks  = 0;                     /**< consecutive empty-scale readings in WAIT_EMPTY */
  CalState      state              = CalState::IDLE;        /**< current state within the calibration workflow, used to manage multi-step processes and user prompts */
  unsigned long stateStartMs       = 0;                     /**< millis() when current state was entered */
};

static CalContext calCtx;                                   /**< Calibration context instance to hold state for calibration workflows */

/**
 * @section Serial Input State Machine Enums, structs & variables
 */

/**
 * @enum InputMode
 *
 * @brief Active user input workflow mode.
 * 
 * @details Used to manage different non-blocking serial input workflows initiated by the user
 */
enum class InputMode : uint8_t {
  NONE = 0,                                                 /**< No active user input workflow */
  TANK_TARE = 1,                                            /**< Tank tare update workflow */
  PROPANE_WEIGHT = 2,                                       /**< Max propane weight update workflow */
  KNOWN_WEIGHT = 3                                          /**< Known calibration weight update workflow */
};

/**
 * @enum InputState
 *
 * @brief Internal parse state for non-blocking input workflows.
 * 
 * @details Used to manage the stepwise process of collecting and confirming user input during workflows that require multiple interactions
 */
enum class InputState : uint8_t {
  IDLE = 0,                                                 /**< Not collecting any input */
  ENTER_VALUE = 1,                                          /**< Collecting numeric value text */
  WAIT_SAVE_CONFIRM = 2                                     /**< Waiting for 's' save or 'q' cancel */
};

/**
 * @struct InputContext
 *
 * @brief Persistent serial parse context for non-blocking user input workflows.
 * 
 * @details Contains variables for managing the state of user input collection and confirmation across multiple loop() iterations without blocking
 */
struct InputContext {
  char      buffer[24]   = {0};                             /**< Text buffer for user-entered numeric value */
  int       index        = 0;                               /**< Current write index into buffer */
  InputMode mode         = InputMode::NONE;                 /**< Active workflow mode */
  float     parsedValue  = 0.0f;                            /**< Last parsed numeric value pending save */
  InputState state       = InputState::IDLE;                /**< Current parse state for active mode */
};

static InputContext inputCtx;                               /**< Non-blocking input context for serial workflows */

/**
 * @section Level Read State Machine Enums, Structs & variables
 */

/**
 * @enum LevelState
 *
 * @brief Enumeration of states for the liquid level read workflow.
 *
 * @details Used to manage the non-blocking tank detection and measurement sequence.
 */
enum class LevelState : uint8_t {
  IDLE      = 0,                                            /**< No active level read workflow */
  WAIT_LOAD = 1,                                            /**< Polling scale until tank weight exceeds detection threshold */
  SETTLING  = 2,                                            /**< Load detected; waiting for placement motion to settle */
  READING   = 3                                             /**< Load settled; taking final averaged measurement */
};

/**
 * @struct LevelContext
 *
 * @brief Persistent context for the non-blocking liquid level read workflow.
 *
 * @details Stores the detection threshold, start timestamp, and current state
 * so each loop() tick can advance the workflow without blocking.
 */
struct LevelContext {
  float         loadDetectThreshold = 0.0f;                 /**< noise-derived threshold used to detect tank placement */
  LevelState    state               = LevelState::IDLE;     /**< current state within the level read workflow */
  unsigned long stateStartMs        = 0;                    /**< millis() when WAIT_LOAD state was entered */
};

static LevelContext levelCtx;                               /**< Level read context instance to hold state for the level read workflow */

/**
 * @section Tare State Machine Enums, Structs & variables
 */

/**
 * @enum TareState
 *
 * @brief Enumeration of states for the startup tare workflow.
 *
 * @details Used to manage the non-blocking startup empty-scale detection and tare sequence.
 */
enum class TareState : uint8_t {
  IDLE        = 0,                                          /**< No active startup tare workflow */
  WAIT_STABLE = 1,                                          /**< Polling scale for stable empty reading, or user 'q' to skip */
  TARE        = 2,                                          /**< Stable empty confirmed; apply tare and finish */
  SKIP        = 3                                           /**< User skipped or scale unstable at timeout; skip tare */
};

/**
 * @struct TareContext
 *
 * @brief Persistent context for the non-blocking startup tare workflow.
 *
 * @details Stores the baseline reading, stable check counter, start timestamp, and current
 * state so each loop() tick can advance the workflow without blocking.
 */
struct TareContext {
  float          baseline     = 0.0f;                       /**< Initial scale reading used as the stability reference */
  int            stableChecks = 0;                          /**< Consecutive readings within tolerance of baseline */
  TareState      state        = TareState::IDLE;            /**< Current state within the startup tare workflow */
  unsigned long  stateStartMs = 0;                          /**< millis() when WAIT_STABLE state was entered */
};

static TareContext tareCtx;                                 /**< Startup tare context instance */

/**
 * @section State Machine Functions
 */

/**
 * @brief Routes a single serial character to the active calibration state machine.
 *
 * @details Called by loop() when a character arrives while calibration is in progress.
 * Handles cancel ('q'/'Q') from WAIT_EMPTY and WAIT_LOAD, and factor adjustment
 * ('+', '-', 's', 'q') from the ADJUSTING state.
 *
 * @param {char} serialchar The received serial character.
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void handleCalibrationInput(char serialchar) {
  // 
  if (calCtx.state == CalState::WAIT_EMPTY || calCtx.state == CalState::WAIT_LOAD || calCtx.state == CalState::SETTLING) {
    if (serialchar != 'q' && serialchar != 'Q') {
      if (calCtx.mode == CalMode::AUTO) {
        char buf[64];
        snprintf(buf, sizeof(buf), "Invalid automatic calibration key: '%c'. Use 'q' to cancel.\n", serialchar);
        Serial.print(buf);
      } else if (calCtx.mode == CalMode::REZERO) {
        char buf[64];
        snprintf(buf, sizeof(buf), "Invalid re-zero key: '%c'. Use 'q' to cancel.\n", serialchar);
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

/**
 * @brief Handles user input for the known calibration weight update workflow.
 *
 * @details Processes one serial character per loop() iteration, 
 * managing the stepwise collection of a new known weight value and user confirmation to save or cancel.
 *
 * @param incoming The incoming character from the serial input.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void handleKnownWeightInput(char incoming) {
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
      Serial.println("Known weight update cancelled.");
      resetInputContext();
      return;
    }

    if (incoming == 's' || incoming == 'S') {
      knownWeight = inputCtx.parsedValue;
      bool eepromSuccess = saveToEeprom(knownWeight, KNOWN_WEIGHT_EEPROM_MAGIC, KNOWN_WEIGHT_EEPROM_MAGIC_ADDR, KNOWN_WEIGHT_EEPROM_VALUE_ADDR);
      
      if (!eepromSuccess) {
        Serial.println("Failed to save known weight to EEPROM.");
      } else {
        char buf[48];
        snprintf(buf, sizeof(buf), "Known weight updated: %.2f lbs\n", knownWeight);
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
      Serial.println("Known weight update cancelled.");
      resetInputContext();
      return;
    }

    // got the newline, attempt to parse the entered text as a float value, validate it, and if valid prompt user to save or cancel
    if (incoming == '\n') {
      inputCtx.buffer[inputCtx.index] = '\0';

      // if the index is 1 and the only character is 'q', treat it as a cancel command to allow quick cancellation
      if (inputCtx.index == 1 && (inputCtx.buffer[0] == 'q' || inputCtx.buffer[0] == 'Q')) {
        Serial.println("Known weight update cancelled.");
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
        snprintf(buf, sizeof(buf), "New known weight entered: %.2f lbs\nSend 's' to save this value to EEPROM, or 'q' to cancel.\n", inputCtx.parsedValue);
        Serial.print(buf);

        inputCtx.state = InputState::WAIT_SAVE_CONFIRM;
        inputCtx.index = 0;
        inputCtx.buffer[0] = '\0';
        return;
      }

      char buf[96];
      snprintf(buf, sizeof(buf), "Invalid known weight. Enter a number from %.2f to %.2f lbs, or 'q' to cancel.\n", MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT);
      Serial.print(buf);
      inputCtx.index = 0;
      inputCtx.buffer[0] = '\0';
      return;
    }

    // user trying to save 
    if (incoming == 's' || incoming == 'S') {
      if (inputCtx.index == 0) {
        Serial.println("Enter a known weight first, then send 's' to save.");
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
        snprintf(buf, sizeof(buf), "Invalid known weight. Enter a number from %.2f to %.2f lbs, or 'q' to cancel.\n", MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT);
        Serial.print(buf);
        inputCtx.index = 0;
        inputCtx.buffer[0] = '\0';
        return;
      }

      knownWeight = inputCtx.parsedValue;
      bool eepromSuccess = saveToEeprom(knownWeight, KNOWN_WEIGHT_EEPROM_MAGIC, KNOWN_WEIGHT_EEPROM_MAGIC_ADDR, KNOWN_WEIGHT_EEPROM_VALUE_ADDR);

      if (!eepromSuccess) {
        Serial.println("Failed to save known weight to EEPROM.");
      } else {
        char buf[48];
        snprintf(buf, sizeof(buf), "Known weight updated: %.2f lbs\n", knownWeight);
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

  Serial.println("Known weight input state invalid; cancelling workflow.");
  resetInputContext();
}

/**
 * @brief Handles user input for the max propane weight update workflow.
 *
 * @details Processes one serial character per loop() iteration, 
 * managing the stepwise collection of a new max propane weight value and user confirmation to save or cancel.
 *
 * @param incoming The incoming character from the serial input.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
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

/**
 * @brief Handles user input for the tank tare update workflow.
 *
 * @details Processes one serial character per loop() iteration, 
 * managing the stepwise collection of a new tank tare value and user confirmation to save or cancel. 
 * 
 * @param incoming The incoming character from the serial input.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
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

/**
 * @brief Resets the input context to its initial state.
 *
 * @details Clears the input context, setting the mode to NONE, state to IDLE,
 * index to 0, parsed value to 0.0f, and buffer to an empty string.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
static void resetInputContext() {
  inputCtx.mode = InputMode::NONE;
  inputCtx.state = InputState::IDLE;
  inputCtx.index = 0;
  inputCtx.parsedValue = 0.0f;
  inputCtx.buffer[0] = '\0';
}

/**
 * @brief Processes calibration workflow steps and scale interactions on each loop() iteration.
 *
 * @details Called every loop() iteration when a calibration workflow is active. 
 * Manages the state machine for automatic and manual calibration workflows:
 * - IDLE: no active workflow, waiting for user input to start one 
 * - WAIT_EMPTY: checks for stable empty scale condition with a timeout for auto-confirmation
 * - WAIT_LOAD: checks for load placement with a noise-derived threshold and timeout, transitions to either:
 *    - AUTO: automatic calibration measurement
 *    - MANUAL: manual adjustment based on the active mode
 * - ADJUSTING: handles manual calibration adjustments
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
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

      if (fabsf(calCtx.measuredUnits) <= MINIMUM_LOAD_WEIGHT) {
        Serial.println("Empty scale auto-confirmed at timeout (stable scale).");
        Serial.println();
        transitionFromWaitEmpty();
      } else {
        Serial.println("Confirmation timed out: scale not empty; cancelled.");
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

    if (fabsf(calCtx.measuredUnits) <= MINIMUM_LOAD_WEIGHT) {

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

/**
 * @brief Advances the level read workflow on each loop() iteration.
 *
 * @details Called every loop() iteration when the level read workflow is active.
 * - WAIT_LOAD: polls the scale each tick until the reading exceeds the load
 *   detection threshold or the placement timeout expires.
 * - SETTLING: waits a short delay after load detection to avoid measuring while
 *   placement motion is still in progress.
 * - READING: takes a final averaged measurement, computes and prints propane
 *   weight and fill percentage, then resets to IDLE.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void tickLevelRead() {
  if (levelCtx.state == LevelState::IDLE) {
    return;
  }

  if (levelCtx.state == LevelState::WAIT_LOAD) {
    if ((millis() - levelCtx.stateStartMs) >= EMPTY_CONFIRM_TIMEOUT_MS) {
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

/**
 * @brief Advances the non-blocking startup tare workflow one iteration.
 *
 * @details Called each loop() iteration. Handles the WAIT_STABLE, TARE, and SKIP
 * states. Returns immediately when IDLE.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void tickTare() {
  if (tareCtx.state == TareState::IDLE) return;

  if (tareCtx.state == TareState::TARE) {
    Serial.println("Stable scale detected, proceeding with tare.");
    scale.tare();
    Serial.println("Scale is tared and ready.");
    tareCtx.state = TareState::IDLE;
    helpMenu();
    return;
  }

  if (tareCtx.state == TareState::SKIP) {
    Serial.println("Continuing without startup tare.");
    Serial.println("Remove propane weight and send 'r' to re-zero when ready.");
    tareCtx.state = TareState::IDLE;
    helpMenu();
    return;
  }

  // WAIT_STABLE: check for user skip or stable scale reading
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'q' || c == 'Q') {
      Serial.println("Startup tare skipped by user.");
      tareCtx.state = TareState::SKIP;
      return;
    }
  }

  if ((millis() - tareCtx.stateStartMs) >= EMPTY_CONFIRM_TIMEOUT_MS) {
    float m = readAveragedUnits(1, POLL_SAMPLES);
    if (!isfinite(m)) {
      printScaleNotReadyDiagnostic("startup tare");
      tareCtx.state = TareState::SKIP;
      return;
    }

    if (fabsf(m - tareCtx.baseline) <= SETUP_EMPTY_WEIGHT) {
      Serial.println("Startup tare auto-confirmed at timeout (stable scale).");
      tareCtx.state = TareState::TARE;
    } else {
      Serial.println("Startup tare timeout: scale unstable, skipping tare.");
      tareCtx.state = TareState::SKIP;
    }
    return;
  }

  float m = readAveragedUnits(1, POLL_SAMPLES);
  if (!isfinite(m)) {
    printScaleNotReadyDiagnostic("startup tare");
    tareCtx.state = TareState::SKIP;
    return;
  }

  if (fabsf(m - tareCtx.baseline) <= SETUP_EMPTY_WEIGHT) {
    tareCtx.stableChecks++;
    if (tareCtx.stableChecks >= UNLOAD_CHECK_COUNT) {
      tareCtx.state = TareState::TARE;
    }
  } else {
    tareCtx.stableChecks = 0;
  }
}

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
    scale.set_scale(calibrationFactor);
    Serial.println("Scale re-zero complete.");
    calCtx.state = CalState::IDLE;
    calCtx.mode  = CalMode::NONE;
    return;
  }

  Serial.println("Empty scale confirmed. Taring now...");
  scale.set_scale();
  scale.tare();
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

// /**
//  * @section Helper functions for user initiated workflows
//  */

// /**
//  * @brief Computes the load-detection threshold from measured noise.
//  *
//  * @details Reads the current unloaded noise from the scale, multiplies it by 20
//  * as a signal-to-noise margin, then clamps to minimumThresholdLbs so a very
//  * quiet scale still responds to a real load.
//  *
//  * @param {float} minimumThresholdLbs Floor value for the returned threshold in pounds.
//  * @return {float} Computed threshold in pounds: max(noise * 20, minimumThresholdLbs).
//  *
//  * @throws {none} This function does not throw exceptions.
//  */
// float computeLoadDetectThreshold(float minimumThresholdLbs) {
//   float noise = fabsf(readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES));
//   float threshold = noise * 20.0f;
//   return (threshold >= minimumThresholdLbs) ? threshold : minimumThresholdLbs;
// }

// /**
//  * @brief Ensures the HX711 is ready before attempting reads or tare.
//  *
//  * @details Checks the amplifier readiness and prints a field-diagnostic message when it is not ready so workflows can exit early instead of blocking.
//  *
//  * @param {const char*} operation Short workflow label used in the error message.
//  * @return {bool} True when HX711 is ready; false otherwise.
//  *
//  * @throws {none} This function does not throw exceptions.
//  */
// bool ensureScaleReady(const char* operation) {
//   if (scale.is_ready()) {
//     return true;
//   }

//   Serial.print("HX711 not ready");
//   if (operation != nullptr && operation[0] != '\0') {
//     Serial.print(" during ");
//     Serial.print(operation);
//   }
//   Serial.println('.');
//   Serial.println("Check HX711 wiring, power, and data pins (DOUT/CLK).");
//   Serial.println();
//   return false;
// }

// /**
//  * @brief Flushes any buffered serial input.
//  *
//  * @details Reads and discards any available serial input to ensure that subsequent serial reads start with fresh input from the user.
//  * 
//  * @return {void} No value is returned.
//  * 
//  * @throws {none} This function does not throw exceptions.
//  */
// void flushSerialInput() {
//   while (Serial.available()) {
//     Serial.read();
//   }
// }

// @todo readAveragedUnits() uses wait_ready_timeout() per iteration so it no longer spins
// indefinitely, but it still blocks loop() for up to HX711_READY_TIMEOUT_MS per reading
// (e.g. up to ~120ms per call for single-reading polling paths, more for multi-reading
// measurement calls). Acceptable for serial-only use. When adding a web interface, refactor
// callers to drive one reading per loop() tick via is_ready() and accumulate across ticks.

// /**
//  * @brief Reads the average weight from the scale over multiple readings.
//  * 
//  * @details Takes multiple readings from the scale, averages them, and returns the result in pounds.
//  * Useful for smoothing out noise in the scale readings and getting a more stable weight measurement.
//  * 
//  * @param {int} readings Number of readings to average.
//  * @param {int} samplesPerReading Number of samples per reading.
//  * @return {float} avgWeight The average weight in pounds. 
//  * 
//  * @throws {none} This function does not throw exceptions.
//  */
// float readAveragedUnits(int readings, int samplesPerReading) {
//   float avgWeight = 0.0f;               // Computed average weight in pounds to return at the end of the function.
//   int   collected  = 0;                 // Number of samples actually read (may be less than requested if HX711 not ready)
//   float totalUnits = 0.0f;              // Accumulator summing weight readings across all iterations for averaging
  
//   // Use bounded wait to avoid infinite blocking while preserving the original
//   // per-reading averaging semantics used across workflows.
//   for (int readingIndex = 0; readingIndex < readings; ++readingIndex) {
//     if (!scale.wait_ready_timeout(HX711_READY_TIMEOUT_MS)) {
//       continue;
//     }

//     totalUnits += scale.get_units(samplesPerReading);
//     collected++;
//   }

//   if (collected == 0) {
//     return 0.0f;
//   }

//   avgWeight = totalUnits / collected;
//   return avgWeight;
// }

/**
 * @section Project initiated workflows
 */

/**
 * @brief Begins the non-blocking startup tare workflow.
 *
 * @details Validates scale readiness, establishes a baseline reading, prints startup
 * prompts, and enters WAIT_STABLE state. The workflow is then advanced each loop()
 * iteration by tickTare().
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void beginTare() {
  if (!ensureScaleReady("startup tare")) {
    tareCtx.state = TareState::SKIP;
    return;
  }

  scale.set_scale(calibrationFactor);

  // Establish baseline before tare; stability is checked relative to this reading.
  tareCtx.baseline     = readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES);
  if (!isfinite(tareCtx.baseline)) {
    printScaleNotReadyDiagnostic("startup tare");
    tareCtx.state = TareState::SKIP;
    return;
  }

  tareCtx.stableChecks = 0;
  tareCtx.stateStartMs = millis();
  tareCtx.state        = TareState::WAIT_STABLE;

  printStartupSummary();

  char startupPrompt[320];
  int startupPromptLen = snprintf(startupPrompt,
                                  sizeof(startupPrompt),
                                  "Startup tare: waiting for stable scale...\n"
                                  "Auto-detect is active.\n"
                                  "Auto-detect timeout: %lu seconds.\n"
                                  "Stability tolerance: +/- %.2f lbs.\n"
                                  "Timeout expiry with stable scale values auto-confirms taring workflow.\n"
                                  "Send 'q' to skip startup tare.\n",
                                  static_cast<unsigned long>(EMPTY_CONFIRM_TIMEOUT_MS / 1000UL),
                                  SETUP_EMPTY_WEIGHT);

  if (startupPromptLen > 0) {
    Serial.print(startupPrompt);
  }
}

/**
 * @brief Prints a summary of the startup configuration and EEPROM values.
 *
 * @details Displays the application title, loaded calibration factor, known weight,
 * maximum propane weight, and tank tare from EEPROM. Also prints startup tare prompts.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
static void printStartupSummary() {
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

/**
 * @section User initiated workflows
 */

/**
 * @brief Starts the automatic calibration workflow.
 *
 * @details Validates that no calibration is already running, checks scale readiness,
 * prints the initial prompts, and sets the calibration context to begin waiting for
 * an empty scale. The workflow continues asynchronously through tickCalibration().
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
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

/**
 * @brief Prints current runtime values and workflow states.
 *
 * @details Displays the active in-memory values used by the program at runtime,
 * including calibration factor, known calibration weight, max propane weight,
 * and tank tare. Also prints the current state of each workflow context.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
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

/**
 * @brief Resets all EEPROM-persisted values to their hardcoded defaults.
 *
 * @details Overwrites calibration factor, known calibration weight, maximum legal propane
 * weight, and tank tare with the compile-time defaults defined in config.h, then persists
 * each value to EEPROM and updates the corresponding runtime globals. Reports the result
 * of each save over serial.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void defaultEeprom() {
  if (!eepromReady) {
    Serial.println("EEPROM is not initialized; cannot reset to defaults.");
    return;
  }

  Serial.println();
  Serial.println("Resetting EEPROM to hardcoded defaults...");

  calibrationFactor = DEF_CALIBRATION_FACTOR;
  if (!saveToEeprom(calibrationFactor, CAL_EEPROM_MAGIC, CAL_EEPROM_MAGIC_ADDR, CAL_EEPROM_VALUE_ADDR)) {
    Serial.println("Failed to save default calibration factor.");
  } else {
    Serial.print("Calibration factor reset to: ");
    Serial.println(calibrationFactor, 2);
  }

  knownWeight = DEF_KNOWN_WEIGHT;
  if (!saveToEeprom(knownWeight, KNOWN_WEIGHT_EEPROM_MAGIC, KNOWN_WEIGHT_EEPROM_MAGIC_ADDR, KNOWN_WEIGHT_EEPROM_VALUE_ADDR)) {
    Serial.println("Failed to save default known calibration weight.");
  } else {
    Serial.print("Known calibration weight reset to: ");
    Serial.print(knownWeight, 2);
    Serial.println(" lbs");
  }

  maxPropane = DEF_MAX_PROPANE;
  if (!saveToEeprom(maxPropane, MAX_PROPANE_EEPROM_MAGIC, MAX_PROPANE_EEPROM_MAGIC_ADDR, MAX_PROPANE_EEPROM_VALUE_ADDR)) {
    Serial.println("Failed to save default max propane weight.");
  } else {
    Serial.print("Max propane weight reset to: ");
    Serial.print(maxPropane, 2);
    Serial.println(" lbs");
  }

  tankTare = DEF_TANK_TARE;
  if (!saveToEeprom(tankTare, TARE_EEPROM_MAGIC, TARE_EEPROM_MAGIC_ADDR, TARE_EEPROM_VALUE_ADDR)) {
    Serial.println("Failed to save default tank tare.");
  } else {
    Serial.print("Tank tare reset to: ");
    Serial.print(tankTare, 2);
    Serial.println(" lbs");
  }

  scale.set_scale(calibrationFactor);
  Serial.println("EEPROM reset complete. Recalibrate before use.");
}

/**
 * @brief Displays all saved EEPROM values with validity status.
 *
 * @details Reads each persisted record (calibration factor, known calibration weight,
 * maximum legal propane weight, and tank tare), verifies its magic marker and bounds,
 * and prints the stored value or an invalid notice.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void eepromValues() {
  if (!eepromReady) {
    Serial.println("EEPROM is not initialized; no saved values can be read.");
    return;
  }

  Serial.println();
  Serial.println("EEPROM Saved Values");

  printEepromField("Calibration factor",
                   CAL_EEPROM_MAGIC_ADDR, CAL_EEPROM_MAGIC, CAL_EEPROM_VALUE_ADDR,
                   CAL_FACTOR_ABS_MIN, CAL_FACTOR_ABS_MAX, true);

  printEepromField("Known calibration weight",
                   KNOWN_WEIGHT_EEPROM_MAGIC_ADDR, KNOWN_WEIGHT_EEPROM_MAGIC, KNOWN_WEIGHT_EEPROM_VALUE_ADDR,
                   MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT, false, " lbs");

  printEepromField("Max propane weight",
                   MAX_PROPANE_EEPROM_MAGIC_ADDR, MAX_PROPANE_EEPROM_MAGIC, MAX_PROPANE_EEPROM_VALUE_ADDR,
                   MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT, false, " lbs");

  printEepromField("Tank tare",
                   TARE_EEPROM_MAGIC_ADDR, TARE_EEPROM_MAGIC, TARE_EEPROM_VALUE_ADDR,
                   MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT, false, " lbs");
}

/**
 * @brief Prints the help menu with all available commands.
 *
 * @details Lists all the runtime commands that the user can send over serial to interact with the scale.
 * 
 * @return {void} No value is returned.
 * 
 * @throws {none} This function does not throw exceptions.
 */
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

/**
 * @brief Initiates the known calibration weight update workflow.
 *
 * @details Starts a non-blocking serial workflow that prompts the user to enter a new known
 * calibration weight, validates the input, and saves it to EEPROM if confirmed.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void knownWeightUpdate() {
  if (inputCtx.mode != InputMode::NONE) {
    Serial.println("Known weight input workflow already in progress. Send 'q' to cancel first.");
    return;
  }

  flushSerialInput();

  inputCtx.mode = InputMode::KNOWN_WEIGHT;
  inputCtx.state = InputState::ENTER_VALUE;
  inputCtx.index = 0;
  inputCtx.parsedValue = 0.0f;
  inputCtx.buffer[0] = '\0';
  
  char prompt[192];
  snprintf(prompt, sizeof(prompt),
           "\nCurrent known calibration weight: %.2f lbs\n"
           "Enter new known calibration weight in lbs (%.2f to %.2f), then press Enter.\n"
           "After entry, send 's' to save or 'q' to cancel.\n",
           knownWeight, MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT);
  Serial.print(prompt);
}

/**
 * @brief Starts the liquid level read workflow.
 *
 * @details Validates scale readiness, measures the noise baseline to compute a
 * load detection threshold, prints prompts, and sets the level context to begin
 * polling for tank placement. The workflow continues asynchronously through
 * tickLevelRead().
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
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
  unsigned long loadDetectSeconds = EMPTY_CONFIRM_TIMEOUT_MS / 1000UL;
  snprintf(levelPrompt, sizeof(levelPrompt),
           "\nPlace propane tank on scale.\n"
           "Waiting for tank placement...\n"
           "Load placement timeout: %lu seconds.\n"
           "Send 'q' to cancel.\n",
           loadDetectSeconds);
  Serial.print(levelPrompt);
}

/**
 * @brief Starts the manual calibration workflow.
 *
 * @details Validates that no calibration is already running, checks scale readiness,
 * initializes the adjustment step from the current calibration factor magnitude,
 * prints the initial prompts, and sets the calibration context to begin waiting for
 * an empty scale. The workflow continues asynchronously through tickCalibration().
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
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

/**
 * @brief Starts the runtime re-zero workflow.
 *
 * @details Validates that no calibration is already running, checks scale readiness,
 * prints the initial prompts, and sets the calibration context to begin waiting for
 * an empty scale in REZERO mode. The workflow continues asynchronously through tickCalibration().
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
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

  calCtx.mode              = CalMode::REZERO;
  calCtx.state             = CalState::WAIT_EMPTY;
  calCtx.stateStartMs      = millis();
  calCtx.stableEmptyChecks = 0;
  calCtx.measuredUnits     = 0.0f;
}

/**
 * @brief Initiates the tank tare update workflow.
 *
 * @details Starts a non-blocking serial workflow that prompts the user to enter a new tank tare value,
 * validates the input, and saves it to EEPROM if confirmed.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
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

/**
 * @brief Initiates the max propane weight update workflow.
 *
 * @details Starts a non-blocking serial workflow that prompts the user to enter a new maximum legal propane weight,
 * validates the input, and saves it to EEPROM if confirmed.
 * 
 * @return {void} No value is returned.
 * 
 * @throws {none} This function does not throw exceptions.
 */
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

/** 
 * @section Project lifecycle functions
 */

/**
 * @brief Initializes serial output and the HX711 scale interface.
 *
 * @details Starts the serial port, prints the available runtime commands, initializes
 * the HX711 using the configured pins, verifies the amplifier is responding, and
 * applies the current calibration factor before normal readings begin.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void setup() {
  Serial.begin(BAUD);

  // need to check if eeprom is ready before trying to load values, and if not, use defaults and continue without eeprom functionality
  eepromReady = EEPROM.begin(EEPROM_SIZE_BYTES);

  // complete eeprom begin failure means we have to use the hard coded defaults and cannot persist any changes, 
  // but we can still operate the scale with the default calibration factor and tare values, 
  // so we should not halt execution, just warn the user and continue
  if (!eepromReady) {
    Serial.println("EEPROM init failed. Using defaults.");
    calibrationFactor = DEF_CALIBRATION_FACTOR;
    knownWeight       = DEF_KNOWN_WEIGHT;
    maxPropane        = DEF_MAX_PROPANE;
    tankTare          = DEF_TANK_TARE;
  } else {
    float loaded = 0.0f;

    // for each persisted value, use the magic marker if present and valid, 
    // otherwise use the default and save it to eeprom for next time
    // this way if one value becomes corrupted, it does not affect the others, 
    // user can still get a valid reading with defaults ,
    // and can fix the corrupted value by re-saving it

    if (loadFromEeprom(loaded, CAL_EEPROM_MAGIC_ADDR, CAL_EEPROM_MAGIC, CAL_EEPROM_VALUE_ADDR) && isValidBoundedFloat(loaded, CAL_FACTOR_ABS_MIN, CAL_FACTOR_ABS_MAX, true)) {
      calibrationFactor = loaded;
    } else {
      calibrationFactor = DEF_CALIBRATION_FACTOR;
      Serial.println("Calibration factor not found or invalid; using default.");
      if (!saveToEeprom(calibrationFactor, CAL_EEPROM_MAGIC, CAL_EEPROM_MAGIC_ADDR, CAL_EEPROM_VALUE_ADDR)) {
        Serial.println(CALIBRATION_SAVE_FAILURE_MSG);
      } else {
        Serial.println(CALIBRATION_SAVE_SUCCESS_MSG);
      }
    }

    if (loadFromEeprom(loaded, KNOWN_WEIGHT_EEPROM_MAGIC_ADDR, KNOWN_WEIGHT_EEPROM_MAGIC, KNOWN_WEIGHT_EEPROM_VALUE_ADDR) && isValidBoundedFloat(loaded, MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT)) {
      knownWeight = loaded;
    } else {
      knownWeight = DEF_KNOWN_WEIGHT;
      Serial.println("Known calibration weight not found or invalid; using default.");
      if (!saveToEeprom(knownWeight, KNOWN_WEIGHT_EEPROM_MAGIC, KNOWN_WEIGHT_EEPROM_MAGIC_ADDR, KNOWN_WEIGHT_EEPROM_VALUE_ADDR)) {
        Serial.println("Failure saving default known calibration weight to EEPROM.");
      } else {
        Serial.println("Success saving default known calibration weight to EEPROM.");
      }
    }

    if (loadFromEeprom(loaded, MAX_PROPANE_EEPROM_MAGIC_ADDR, MAX_PROPANE_EEPROM_MAGIC, MAX_PROPANE_EEPROM_VALUE_ADDR) && isValidBoundedFloat(loaded, MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT)) {
      maxPropane = loaded;
    } else {
      maxPropane = DEF_MAX_PROPANE;
      Serial.println("Max propane weight not found or invalid; using default.");
      if (!saveToEeprom(maxPropane, MAX_PROPANE_EEPROM_MAGIC, MAX_PROPANE_EEPROM_MAGIC_ADDR, MAX_PROPANE_EEPROM_VALUE_ADDR)) {
        Serial.println("Failure saving default max propane weight to EEPROM.");
      } else {
        Serial.println("Success saving default max propane weight to EEPROM.");
      }
    }

    if (loadFromEeprom(loaded, TARE_EEPROM_MAGIC_ADDR, TARE_EEPROM_MAGIC, TARE_EEPROM_VALUE_ADDR) && isValidBoundedFloat(loaded, MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT)) {
      tankTare = loaded;
    } else {
      tankTare = DEF_TANK_TARE;
      Serial.println("Tank tare not found or invalid; using default.");
      if (!saveToEeprom(tankTare, TARE_EEPROM_MAGIC, TARE_EEPROM_MAGIC_ADDR, TARE_EEPROM_VALUE_ADDR)) {
        Serial.println("Failure saving default tank tare to EEPROM.");
      } else {
        Serial.println("Success saving default tank tare to EEPROM.");
      }
    }
  }

  scale.begin(DOUT_PIN, CLK_PIN);

  // project not intended for continuous operation
  // always starting fresh avoids issues with long term stability issues
  beginTare();
}

/**
 * @brief Processes serial commands and prints the current scale reading.
 * 
 * @details Handles calibration, tare, and re-zero commands from the serial port,
 * verifies the HX711 is ready, and reports a single propane reading when requested.
 * 
 * @return {void} No value is returned.
 * 
 * @throws {none} This function does not throw exceptions. 
 */
void loop() {
  // Advance all active state machines each iteration before processing serial input
  tickCalibration();
  tickLevelRead();
  tickTare();

  // While startup tare is active, tickTare() consumes serial input; suppress all other dispatch
  if (tareCtx.state != TareState::IDLE) {
    return;
  }

  // On no serial input, need return so state machines can continue running until next loop iteration
  if (!Serial.available()) {
    return;
  }

  char temp = Serial.read();

  // Route 'q' cancel when level read is waiting for tank placement or settling
  if (levelCtx.state == LevelState::WAIT_LOAD || levelCtx.state == LevelState::SETTLING) {
    if (temp == 'q' || temp == 'Q') {
      Serial.println("Level read cancelled.");
      levelCtx.state = LevelState::IDLE;
    } else if (temp != '\r' && temp != '\n') {
      Serial.print("Invalid level read key: '");
      Serial.print(temp);
      Serial.println("'. Send 'q' to cancel.");
    }
    return;
  }

  // Route one character at a time to active tank tare input workflow
  if (inputCtx.mode == InputMode::TANK_TARE) {
    handleTankTareInput(temp);
    return;
  }

  // Route one character at a time to active propane weight input workflow
  if (inputCtx.mode == InputMode::PROPANE_WEIGHT) {
    handlePropaneWeightInput(temp);
    return;
  }

  // Route one character at a time to active known calibration weight input workflow
  if (inputCtx.mode == InputMode::KNOWN_WEIGHT) {
    handleKnownWeightInput(temp);
    return;
  }

  // ignore newline characters that may be sent by the serial monitor after commands
  // need to avoid accidentally triggering multiple commands in a row
  // MUST come after input mode handlers because user hits enter after inputting new value, 
  // so newlines need to be processed by input handlers but ignored for general command dispatch
  if (temp == '\r' || temp == '\n') {
    return;
  }

  // Route input to the active calibration state machine; 
  // ignore other commands while busy
  if (calCtx.state != CalState::IDLE) {
    handleCalibrationInput(temp);
    return;
  }

  // Fell through to here, so no active workflows, setup for user serial input
  bool handled = true;

  // by having empty lower case input cases, do not need to call tolower() on the input
  // this allows the user to send either upper or lower case commands
  // without needing to worry about case sensitivity
  switch (temp) {
    case 'a':
    case 'A':
      automaticCalibration();
      break;
    case 'c':
    case 'C':
      currentRuntimeValues();
      break;
    case 'd':
    case 'D':
      defaultEeprom();
      break;
    case 'e':
    case 'E':
      eepromValues();
      break;
    case 'h':
    case 'H':
      helpMenu();
      break;
    case 'k':
    case 'K':
      knownWeightUpdate();
      break;
    case 'l':
    case 'L':
      liquidLevel();
      break;
    case 'm':
    case 'M':
      manualCalibration();
      break;
    case 'p':
    case 'P':
      propaneWeightUpdate();
      break;
    case 'r':
    case 'R':
      reZero();
      break;
    case 't':
    case 'T':
      tankTareUpdate();
      break;
    default:
      handled = false;
      Serial.print("Unknown command: '");
      Serial.print(temp);
      Serial.println("'. Send 'h' for help.");
      break;
  }

  // flush any extra input after handling a command
  // need to prevent accidental multiple command triggers from a single line of input
  if (handled && inputCtx.mode == InputMode::NONE) {
    flushSerialInput();
  }
}

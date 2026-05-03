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
 * @copyright Copyright (c) 2024
 */

/**
 * @section Includes 
 */

/**
 * @subsection Standard library headers
 */

#include <EEPROM.h>                                         // EEPROM library for persistent storage of calibration and tare values
#include <math.h>                                           // Math helpers for fabsf and isfinite during value validation
#include <stdlib.h>                                         // Standard library for functions like strtof for parsing floats from strings 

/**
 * @subsection Third party library headers
 */
#include "HX711.h"                                          // HX711 library for interfacing with the load cell amplifier to read weight data

/** 
 * @subsection Local library headers
 */
#include "config.h"                                         // Local configuration header defining pin assignments, calibration constants, and EEPROM addresses

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
 * @section Global State Machine Variables and Constants
 */

/**
 * @subsection Calibration State Machine
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
 * @subsection Non-blocking Serial Input State Machine
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
 * @subsection Non-blocking Level Read State Machine
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
  READING   = 2                                             /**< Load detected; taking final averaged measurement */
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
 * @subsection Non-blocking Tare State Machine
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

static bool startupSummaryReplayDone = false;               /**< Emits startup summary once when startup tare resolves */
static TareContext tareCtx;                                 /**< Startup tare context instance */

// @todo move to config.h

/**
 * @section Global EEPROM Sanity Limit Constants
 */
constexpr float CAL_FACTOR_ABS_MAX = 500000.0f;             // Maximum absolute value for valid calibration factor 
constexpr float CAL_FACTOR_ABS_MIN = 100.0f;                // Minimum absolute value for valid calibration factor 
constexpr float MAX_PROJECT_WEIGHT = 60.0f;                 // Project will never measure a propane tank above nominal 60 lbs
constexpr float MIN_PLAUSIBLE_WEIGHT = 0.1f;                // Minimum plausible non-zero weight for user-entered values

// @todo move to config.h

/** 
 * @section Global UI String Constants
 */

constexpr char APP_TITLE[] = "Propane Level Scale";
constexpr char CALIBRATION_SAVE_FAILURE_MSG[] = "Failure saving default calibration to EEPROM.";
constexpr char CALIBRATION_SAVE_SUCCESS_MSG[] = "Success saving default calibration to EEPROM.";
constexpr char CMD_AUTO_CAL_MSG[] = "Send 'a' to enter automatic calibration mode";
constexpr char CMD_CURRENT_VALUES_MSG[] = "Send 'c' to print current runtime values";
constexpr char CMD_DEFAULT_EEPROM_MSG[] = "Send 'd' to reset EEPROM to default values";
constexpr char CMD_EEPROM_MSG[] = "Send 'e' to display saved EEPROM values";
constexpr char CMD_HELP_MSG[] = "Send 'h' to display this help menu";
constexpr char CMD_KNOWN_WEIGHT_MSG[] = "Send 'k' to enter known weight value for calibration mode";
constexpr char CMD_LEVEL_MSG[] = "Send 'l' to display one liquid propane percent level reading";
constexpr char CMD_MANUAL_CAL_MSG[] = "Send 'm' to enter manual calibration mode";
constexpr char CMD_PROPANE_WEIGHT_MSG[] = "Send 'p' to set maximum legal propane weight";
constexpr char CMD_REZERO_MSG[] = "Send 'r' to re-zero scale with no propane weight on it";
constexpr char CMD_TANK_TARE_MSG[] = "Send 't' to set propane tank tare";

/**
 * @section EEPROM Workflows
 */

/**
 * @brief Validates that a float value is finite and within specified bounds.
 * 
 * @details Checks if a float value is finite and within specified bounds, with an option to use absolute magnitude for the check.
 * 
 * @param value The float value to validate.
 * @param minimumValue The minimum allowable value.
 * @param maximumValue The maximum allowable value.
 * @param useAbsoluteMagnitude If true, the absolute value of the float is used for validation.
 * @return true if the value is valid, false otherwise.
 * 
 * @throws {none} This function does not throw exceptions.
 */
bool isValidBoundedFloat(float value, float minimumValue, float maximumValue, bool useAbsoluteMagnitude = false) {
  float candidate = 0.0f;                                   // Temporarily holds the value used for comparison

  if (!isfinite(value)) {
    return false;
  }

  candidate = useAbsoluteMagnitude ? fabsf(value) : value;
  return (candidate >= minimumValue) && (candidate <= maximumValue);
}

/**
 * @brief Loads a float value from EEPROM if the magic number is valid.
 * 
 * @details Reads the magic number from EEPROM to verify that a valid value has been saved.
 * If the magic number is valid, it loads the value into the provided reference variable.
 * 
 * @param value Reference to a float variable where the loaded value will be stored.
 * @param magicAddr EEPROM address of the magic number.
 * @param magicValue Expected magic number for validation.
 * @param valueAddr EEPROM address of the float value.
 * @return true if the value was successfully loaded, false otherwise.
 * 
 * @throws {none} This function does not throw exceptions.
 */
bool loadFromEeprom(float& value, uint32_t magicAddr, uint32_t magicValue, uint32_t valueAddr) {
  uint32_t magic = 0;                                       // Magic number read from EEPROM for validation

  EEPROM.get(magicAddr, magic);
  if (magic != magicValue) {
    return false;
  }

  EEPROM.get(valueAddr, value);
  return true;
}

/**
 * @brief Saves a float value to EEPROM with a magic number for validation.
 *
 * @details Writes the magic number and float value to EEPROM, and commits the changes.
 * Returns false immediately if EEPROM was not successfully initialized.
 *
 * @param {float} value The float value to save.
 * @param {uint32_t} magic The magic number for validation.
 * @param {int} magicAddr EEPROM address of the magic number.
 * @param {int} valueAddr EEPROM address of the float value.
 * @return {bool} True if the value was successfully saved, false otherwise.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool saveToEeprom(float value, uint32_t magic, int magicAddr, int valueAddr) {
  if (!eepromReady) {
    return false;
  }

  EEPROM.put(magicAddr, magic);
  EEPROM.put(valueAddr, value);
  return EEPROM.commit();
}

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
        Serial.print("Invalid automatic calibration key: '");
        Serial.print(serialchar);
        Serial.println("'. Use 'q' to cancel.");
      } else if (calCtx.mode == CalMode::REZERO) {
        Serial.print("Invalid re-zero key: '");
        Serial.print(serialchar);
        Serial.println("'. Use 'q' to cancel.");
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

      Serial.print("Manual calibration complete, computed calibration factor: ");
      Serial.println(calibrationFactor, 2);

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
      Serial.print("Invalid manual calibration key: '");
      Serial.print(serialchar);
      Serial.println("'. Use '+', '-', 's' (save), or 'q' (cancel).");
    }
  }
}

/**
 * @brief Handles user input for the known calibration weight update workflow.
 *
 * @details Processes one serial character per loop() iteration, preserving parse
 * state in inputCtx so calibration tick processing remains non-blocking.
 *
 * @param incoming The incoming character from the serial input.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void handleKnownWeightInput(char incoming) {
  if (incoming == '\r') {
    return;
  }

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
      if (!saveToEeprom(knownWeight, KNOWN_WEIGHT_EEPROM_MAGIC, KNOWN_WEIGHT_EEPROM_MAGIC_ADDR, KNOWN_WEIGHT_EEPROM_VALUE_ADDR)) {
        Serial.println("Failed to save known weight to EEPROM.");
      } else {
        Serial.print("Known weight updated: ");
        Serial.print(knownWeight, 2);
        Serial.println(" lbs");
      }
      resetInputContext();
      return;
    }

    Serial.println("Invalid response. Send 's' to save, or 'q' to cancel.");
    return;
  }

  if ((incoming == 'q' || incoming == 'Q') && inputCtx.index == 0) {
    Serial.println("Known weight update cancelled.");
    resetInputContext();
    return;
  }

  if (incoming == '\n') {
    inputCtx.buffer[inputCtx.index] = '\0';

    if (inputCtx.index == 1 && (inputCtx.buffer[0] == 'q' || inputCtx.buffer[0] == 'Q')) {
      Serial.println("Known weight update cancelled.");
      resetInputContext();
      return;
    }

    inputCtx.parsedValue = 0.0f;
    if (parseNonNegativeFloat(inputCtx.buffer, inputCtx.parsedValue) && isValidBoundedFloat(inputCtx.parsedValue, MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT)) {
      Serial.print("New known weight entered: ");
      Serial.print(inputCtx.parsedValue, 2);
      Serial.println(" lbs");
      Serial.println("Send 's' to save this value to EEPROM, or 'q' to cancel.");
      inputCtx.state = InputState::WAIT_SAVE_CONFIRM;
      inputCtx.index = 0;
      inputCtx.buffer[0] = '\0';
      return;
    }

    Serial.print("Invalid known weight. Enter a number from ");
    Serial.print(MIN_PLAUSIBLE_WEIGHT, 2);
    Serial.print(" to ");
    Serial.print(MAX_PROJECT_WEIGHT, 2);
    Serial.println(" lbs, or 'q' to cancel.");
    inputCtx.index = 0;
    inputCtx.buffer[0] = '\0';
    return;
  }

  if (incoming == 's' || incoming == 'S') {
    if (inputCtx.index == 0) {
      Serial.println("Enter a known weight first, then send 's' to save.");
      return;
    }

    inputCtx.buffer[inputCtx.index] = '\0';
    if (!parseNonNegativeFloat(inputCtx.buffer, inputCtx.parsedValue) || !isValidBoundedFloat(inputCtx.parsedValue, MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT)) {
      Serial.print("Invalid known weight. Enter a number from ");
      Serial.print(MIN_PLAUSIBLE_WEIGHT, 2);
      Serial.print(" to ");
      Serial.print(MAX_PROJECT_WEIGHT, 2);
      Serial.println(" lbs, or 'q' to cancel.");
      inputCtx.index = 0;
      inputCtx.buffer[0] = '\0';
      return;
    }

    knownWeight = inputCtx.parsedValue;
    if (!saveToEeprom(knownWeight, KNOWN_WEIGHT_EEPROM_MAGIC, KNOWN_WEIGHT_EEPROM_MAGIC_ADDR, KNOWN_WEIGHT_EEPROM_VALUE_ADDR)) {
      Serial.println("Failed to save known weight to EEPROM.");
    } else {
      Serial.print("Known weight updated: ");
      Serial.print(knownWeight, 2);
      Serial.println(" lbs");
    }
    resetInputContext();
    return;
  }

  if (inputCtx.index < static_cast<int>(sizeof(inputCtx.buffer) - 1)) {
    inputCtx.buffer[inputCtx.index++] = incoming;
  } else {
    Serial.println("Input too long. Enter a shorter number, or 'q' to cancel.");
    inputCtx.index = 0;
    inputCtx.buffer[0] = '\0';
  }
}

/**
 * @brief Handles user input for the max propane weight update workflow.
 *
 * @details Processes one serial character per loop() iteration, preserving parse
 * state in inputCtx so calibration tick processing remains non-blocking.
 *
 * @param incoming The incoming character from the serial input.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void handlePropaneWeightInput(char incoming) {
  if (incoming == '\r') {
    return;
  }

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
      if (fabsf(inputCtx.parsedValue - maxPropane) >= maxPropane * CHANGE_WARN_PCT) {
        Serial.print("Warning: Max propane weight changed by ");
        Serial.print(fabsf(inputCtx.parsedValue - maxPropane), 2);
        Serial.println(" lbs. Verify tank tare ('t') and known calibration weight ('k') are still accurate, and recalibrate ('a' or 'm') if needed.");
      }
      maxPropane = inputCtx.parsedValue;
      if (!saveToEeprom(maxPropane, MAX_PROPANE_EEPROM_MAGIC, MAX_PROPANE_EEPROM_MAGIC_ADDR, MAX_PROPANE_EEPROM_VALUE_ADDR)) {
        Serial.println("Failed to save max propane weight to EEPROM.");
      } else {
        Serial.print("Max propane weight updated: ");
        Serial.print(maxPropane, 2);
        Serial.println(" lbs");
      }
      resetInputContext();
      return;
    }

    Serial.println("Invalid response. Send 's' to save, or 'q' to cancel.");
    return;
  }

  if ((incoming == 'q' || incoming == 'Q') && inputCtx.index == 0) {
    Serial.println("Max propane weight update cancelled.");
    resetInputContext();
    return;
  }

  if (incoming == '\n') {
    inputCtx.buffer[inputCtx.index] = '\0';

    if (inputCtx.index == 1 && (inputCtx.buffer[0] == 'q' || inputCtx.buffer[0] == 'Q')) {
      Serial.println("Max propane weight update cancelled.");
      resetInputContext();
      return;
    }

    inputCtx.parsedValue = 0.0f;
    if (parseNonNegativeFloat(inputCtx.buffer, inputCtx.parsedValue) && isValidBoundedFloat(inputCtx.parsedValue, MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT)) {
      Serial.print("New max propane weight entered: ");
      Serial.print(inputCtx.parsedValue, 2);
      Serial.println(" lbs");
      Serial.println("Send 's' to save this value to EEPROM, or 'q' to cancel.");
      inputCtx.state = InputState::WAIT_SAVE_CONFIRM;
      inputCtx.index = 0;
      inputCtx.buffer[0] = '\0';
      return;
    }

    Serial.print("Invalid max propane weight. Enter a number from ");
    Serial.print(MIN_PLAUSIBLE_WEIGHT, 2);
    Serial.print(" to ");
    Serial.print(MAX_PROJECT_WEIGHT, 2);
    Serial.println(" lbs, or 'q' to cancel.");
    inputCtx.index = 0;
    inputCtx.buffer[0] = '\0';
    return;
  }

  if (incoming == 's' || incoming == 'S') {
    if (inputCtx.index == 0) {
      Serial.println("Enter a max propane weight first, then send 's' to save.");
      return;
    }

    inputCtx.buffer[inputCtx.index] = '\0';
    if (!parseNonNegativeFloat(inputCtx.buffer, inputCtx.parsedValue) || !isValidBoundedFloat(inputCtx.parsedValue, MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT)) {
      Serial.print("Invalid max propane weight. Enter a number from ");
      Serial.print(MIN_PLAUSIBLE_WEIGHT, 2);
      Serial.print(" to ");
      Serial.print(MAX_PROJECT_WEIGHT, 2);
      Serial.println(" lbs, or 'q' to cancel.");
      inputCtx.index = 0;
      inputCtx.buffer[0] = '\0';
      return;
    }

    if (fabsf(inputCtx.parsedValue - maxPropane) >= maxPropane * CHANGE_WARN_PCT) {
      Serial.print("Warning: Max propane weight changed by ");
      Serial.print(fabsf(inputCtx.parsedValue - maxPropane), 2);
      Serial.println(" lbs. Verify tank tare ('t') and known calibration weight ('k') are still accurate, and recalibrate ('a' or 'm') if needed.");
    }
    maxPropane = inputCtx.parsedValue;
    if (!saveToEeprom(maxPropane, MAX_PROPANE_EEPROM_MAGIC, MAX_PROPANE_EEPROM_MAGIC_ADDR, MAX_PROPANE_EEPROM_VALUE_ADDR)) {
      Serial.println("Failed to save max propane weight to EEPROM.");
    } else {
      Serial.print("Max propane weight updated: ");
      Serial.print(maxPropane, 2);
      Serial.println(" lbs");
    }
    resetInputContext();
    return;
  }

  if (inputCtx.index < static_cast<int>(sizeof(inputCtx.buffer) - 1)) {
    inputCtx.buffer[inputCtx.index++] = incoming;
  } else {
    Serial.println("Input too long. Enter a shorter number, or 'q' to cancel.");
    inputCtx.index = 0;
    inputCtx.buffer[0] = '\0';
  }
}

/**
 * @brief Handles user input for the tank tare update workflow.
 *
 * @details Processes incoming characters from the serial input, updating the input context,
 * validating the input, and managing the state transitions for the tank tare update workflow.
 *
 * @param incoming The incoming character from the serial input.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void handleTankTareInput(char incoming) {
  if (incoming == '\r') {
    return;
  }

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
      if (fabsf(inputCtx.parsedValue - tankTare) >= tankTare * CHANGE_WARN_PCT) {
        Serial.print("Warning: Tank tare changed by ");
        Serial.print(fabsf(inputCtx.parsedValue - tankTare), 2);
        Serial.println(" lbs.");
        Serial.println("Update max legal propane ('p') and known calibration weight ('k'), then recalibrate ('a' or 'm') to maintain accuracy.");
      }
      tankTare = inputCtx.parsedValue;
      if (!saveToEeprom(tankTare, TARE_EEPROM_MAGIC, TARE_EEPROM_MAGIC_ADDR, TARE_EEPROM_VALUE_ADDR)) {
        Serial.println("Failed to save tank tare to EEPROM.");
      } else {
        Serial.print("Tank tare saved successfully: ");
        Serial.print(tankTare, 2);
        Serial.println(" lbs");
      }
      resetInputContext();
      return;
    }

    Serial.println("Invalid response. Send 's' to save, or 'q' to cancel.");
    return;
  }

  if ((incoming == 'q' || incoming == 'Q') && inputCtx.index == 0) {
    Serial.println("Tank tare update cancelled.");
    resetInputContext();
    return;
  }

  if (incoming == '\n') {
    inputCtx.buffer[inputCtx.index] = '\0';

    if (inputCtx.index == 1 && (inputCtx.buffer[0] == 'q' || inputCtx.buffer[0] == 'Q')) {
      Serial.println("Tank tare update cancelled.");
      resetInputContext();
      return;
    }

    if (parseNonNegativeFloat(inputCtx.buffer, inputCtx.parsedValue) && isValidBoundedFloat(inputCtx.parsedValue, MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT)) {
      Serial.print("New tank tare entered: ");
      Serial.print(inputCtx.parsedValue, 2);
      Serial.println(" lbs");
      Serial.println("Send 's' to save this value to EEPROM, or 'q' to cancel.");
      inputCtx.state = InputState::WAIT_SAVE_CONFIRM;
      inputCtx.index = 0;
      inputCtx.buffer[0] = '\0';
      return;
    }

    Serial.print("Invalid tank tare. Enter a number from ");
    Serial.print(MIN_PLAUSIBLE_WEIGHT, 2);
    Serial.print(" to ");
    Serial.print(MAX_PROJECT_WEIGHT, 2);
    Serial.println(" lbs, or 'q' to cancel.");
    inputCtx.index = 0;
    inputCtx.buffer[0] = '\0';
    return;
  }

  if (incoming == 's' || incoming == 'S') {
    if (inputCtx.index == 0) {
      Serial.println("Enter a tare value first, then send 's' to save.");
      return;
    }

    inputCtx.buffer[inputCtx.index] = '\0';
    if (!parseNonNegativeFloat(inputCtx.buffer, inputCtx.parsedValue) || !isValidBoundedFloat(inputCtx.parsedValue, MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT)) {
      Serial.print("Invalid tank tare. Enter a number from ");
      Serial.print(MIN_PLAUSIBLE_WEIGHT, 2);
      Serial.print(" to ");
      Serial.print(MAX_PROJECT_WEIGHT, 2);
      Serial.println(" lbs, or 'q' to cancel.");
      inputCtx.index = 0;
      inputCtx.buffer[0] = '\0';
      return;
    }

    if (fabsf(inputCtx.parsedValue - tankTare) >= tankTare * CHANGE_WARN_PCT) {
      Serial.print("Warning: Tank tare changed by ");
      Serial.print(fabsf(inputCtx.parsedValue - tankTare), 2);
      Serial.println(" lbs.");
      Serial.println("Update max legal propane ('p') and known calibration weight ('k'), then recalibrate ('a' or 'm') to maintain accuracy.");
    }
    tankTare = inputCtx.parsedValue;
    if (!saveToEeprom(tankTare, TARE_EEPROM_MAGIC, TARE_EEPROM_MAGIC_ADDR, TARE_EEPROM_VALUE_ADDR)) {
      Serial.println("Failed to save tank tare to EEPROM.");
    } else {
      Serial.print("Tank tare saved successfully: ");
      Serial.print(tankTare, 2);
      Serial.println(" lbs");
    }
    resetInputContext();
    return;
  }

  if (inputCtx.index < static_cast<int>(sizeof(inputCtx.buffer) - 1)) {
    inputCtx.buffer[inputCtx.index++] = incoming;
  } else {
    Serial.println("Input too long. Enter a shorter number, or 'q' to cancel.");
    inputCtx.index = 0;
    inputCtx.buffer[0] = '\0';
  }
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
    
    if ((millis() - calCtx.stateStartMs) >= USER_CONFIRM_TIMEOUT_MS) {
      
      // take one final reading to decide auto-confirm vs. abort
      calCtx.measuredUnits = readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES);
      
      // Scale is empty at expiry — auto-confirm & transition state machine so user doesn't have to interact
      if (fabsf(calCtx.measuredUnits) <= MINIMUM_LOAD_WEIGHT) {
        Serial.println("Empty scale auto-confirmed at timeout (stable scale).");
        Serial.println();
        transitionFromWaitEmpty();
      } else {
        // otherwise something still on platen — unsafe to tare, reset state machine and abort
        Serial.println("Confirmation timed out: scale not empty; cancelled.");
        Serial.println();
        calCtx.state = CalState::IDLE;
        calCtx.mode  = CalMode::NONE;
      }
      return;
    }

    // Poll each tick to detect empty early (before timeout expires)
    calCtx.measuredUnits = readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES);

    if (fabsf(calCtx.measuredUnits) <= MINIMUM_LOAD_WEIGHT) {
      // require N consecutive readings to suppress transient spikes
      // gets incremented on every tick caused by loop() until it reaches the threshold for auto-confirmation,
      // but resets to 0 if a non-empty reading occurs
      calCtx.stableEmptyChecks++; 
      
      if (calCtx.stableEmptyChecks >= UNLOAD_CHECK_COUNT) {
        // Consecutive stable-empty streak met — no need to wait for timeout
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
    
    // User never placed weight in time — abort rather than calibrate with no load
    if ((millis() - calCtx.stateStartMs) >= EMPTY_CONFIRM_TIMEOUT_MS) {
      Serial.println("Weight placement timed out; calibration cancelled.");
      calCtx.state = CalState::IDLE;
      calCtx.mode  = CalMode::NONE;
      return;
    }

    calCtx.measuredUnits = readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES);
    // below noise-derived threshold — no real load yet
    if (fabsf(calCtx.measuredUnits) < calCtx.loadDetectThreshold) {
      return; 
    }

    // workflow - weight placed — settle before measuring to avoid mid-placement reads
    Serial.print("Weight detected. Settling for ");
    Serial.print(CAL_SETTLE_DELAY_MS / 1000UL);
    Serial.println(" seconds before measuring...");
    calCtx.stateStartMs = millis();
    calCtx.state        = CalState::SETTLING;
    return;
  }

  // workflow - waiting for placed weight to stop moving before taking final measurement
  if (calCtx.state == CalState::SETTLING) {
    if ((millis() - calCtx.stateStartMs) < CAL_SETTLE_DELAY_MS) {
      return;
    }

    // workflow - weight settled for automatic calibration
    if (calCtx.mode == CalMode::AUTO) {
      Serial.println("Measuring stable reading...");

      // Use more samples for the final measurement to reduce noise in the derived factor
      calCtx.measuredUnits = readAveragedUnits(CAL_SAMPLES, LIVE_SAMPLES);

      if (knownWeight == 0.0f || calCtx.measuredUnits == 0.0f) {
        // Guard against division by zero or a zero reading that would produce an unusable factor
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

      Serial.print("Initial calibration factor estimate: ");
      Serial.println(calibrationFactor, 2);
      Serial.print("Verified reading: ");
      Serial.print(verifiedUnits, 2);
      Serial.println(" lbs");
      Serial.print("Automatic calibration complete, computed calibration factor: ");
      Serial.println(calibrationFactor, 2);

      if (!saveToEeprom(calibrationFactor, CAL_EEPROM_MAGIC, CAL_EEPROM_MAGIC_ADDR, CAL_EEPROM_VALUE_ADDR)) {
        Serial.println("Failed to save calibration to EEPROM.");
      } else {
        Serial.println("Calibration saved to EEPROM.");
      }

      calCtx.state = CalState::IDLE;
      calCtx.mode  = CalMode::NONE;
    }

    // workflow - weight settled for manual calibration — enter interactive adjustment
    if (calCtx.mode == CalMode::MANUAL) {
      // serial input handled by handleCalibrationInput()
      Serial.println("Adjust calibration until the reading matches the known weight.");
      Serial.println("Send '+' to increase calibration factor");
      Serial.println("Send '-' to decrease calibration factor");
      Serial.println("(step halves on direction reversal)");
      Serial.println("Send 's' to save and finish manual calibration.");
      Serial.println("Send 'q' to cancel manual calibration without saving.");

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
    float readingLbs = scale.get_units();
    // Quantize to integers for change detection — float comparison is unreliable across loop ticks
    int readingTenth      = static_cast<int>(lroundf(readingLbs           * 10.0f));
    int factorHundredth   = static_cast<int>(lroundf(calibrationFactor    * 100.0f));
    int stepTenThousandth = static_cast<int>(lroundf(calCtx.adjustmentStep * 10000.0f));

    // always print once on first entry
    if (!calCtx.hasManualDisplay || readingTenth != calCtx.lastReadingTenth || factorHundredth != calCtx.lastFactorHundredth || stepTenThousandth != calCtx.lastStepTenThousandth) {
      Serial.print("Reading: ");
      Serial.print(readingLbs, 1);
      Serial.print(" lbs  factor: ");
      Serial.print(calibrationFactor, 2);
      Serial.print("  step: ");
      Serial.println(calCtx.adjustmentStep, 4);

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

    float measuredUnits = readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES);
    if (fabsf(measuredUnits) >= levelCtx.loadDetectThreshold) {
      levelCtx.state = LevelState::READING;
    }
    return;
  }

  if (levelCtx.state == LevelState::READING) {
    Serial.println("Tank detected. Reading weight...");
    float rawWeight = readAveragedUnits(CAL_SAMPLES, LIVE_SAMPLES);

    float propaneWeight = rawWeight - tankTare - PLATEN_TARE;
    if (propaneWeight < 0.0f) {
      propaneWeight = 0.0f;
    }

    float propaneLevel = (maxPropane > 0.0f) ? (propaneWeight / maxPropane) * 100.0f : 0.0f;

    Serial.print("Scale load: ");
    Serial.print(rawWeight, 1);
    Serial.print(" lbs, ");
    Serial.print("Calculated propane: ");
    Serial.print(propaneWeight, 1);
    Serial.print(" lbs, ");
    Serial.print("Propane level: ");
    Serial.print(propaneLevel, 1);
    Serial.println("%");

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
    if (!startupSummaryReplayDone) {
      printStartupSummary();
      startupSummaryReplayDone = true;
    }
    Serial.println("Stable scale detected, proceeding with tare.");
    scale.tare();
    Serial.println("Scale is tared and ready.");
    tareCtx.state = TareState::IDLE;
    helpMenu();
    return;
  }

  if (tareCtx.state == TareState::SKIP) {
    if (!startupSummaryReplayDone) {
      printStartupSummary();
      startupSummaryReplayDone = true;
    }
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
    float m = readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES);
    if (fabsf(m - tareCtx.baseline) <= SETUP_EMPTY_WEIGHT) {
      Serial.println("Startup tare auto-confirmed at timeout (stable scale).");
      tareCtx.state = TareState::TARE;
    } else {
      Serial.println("Startup tare timeout: scale unstable, skipping tare.");
      tareCtx.state = TareState::SKIP;
    }
    return;
  }

  float m = readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES);
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

  // workflow - auto calibration waiting on user to place known weight on scale after empty confirmation
  if (calCtx.mode == CalMode::AUTO) {
    Serial.print("Place the known weight on the scale: ");
    Serial.print(knownWeight, 2);
    Serial.println(" lbs");
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
  
  Serial.println("Waiting for weight placement on scale...");
  Serial.print("Load placement timeout: ");
  Serial.print(EMPTY_CONFIRM_TIMEOUT_MS / 1000UL);
  Serial.println(" seconds.");

  calCtx.stateStartMs = millis();
  calCtx.state        = CalState::WAIT_LOAD;
}

/**
 * @section Helper functions for user initiated workflows
 */

/**
 * @brief Computes the load-detection threshold from measured noise.
 *
 * @details Reads the current unloaded noise from the scale, multiplies it by 20
 * as a signal-to-noise margin, then clamps to minimumThresholdLbs so a very
 * quiet scale still responds to a real load.
 *
 * @param {float} minimumThresholdLbs Floor value for the returned threshold in pounds.
 * @return {float} Computed threshold in pounds: max(noise * 20, minimumThresholdLbs).
 *
 * @throws {none} This function does not throw exceptions.
 */
float computeLoadDetectThreshold(float minimumThresholdLbs) {
  float noise = fabsf(readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES));
  float threshold = noise * 20.0f;
  return (threshold >= minimumThresholdLbs) ? threshold : minimumThresholdLbs;
}

/**
 * @brief Ensures the HX711 is ready before attempting reads or tare.
 *
 * @details Checks the amplifier readiness and prints a field-diagnostic message when it is not ready so workflows can exit early instead of blocking.
 *
 * @param {const char*} operation Short workflow label used in the error message.
 * @return {bool} True when HX711 is ready; false otherwise.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool ensureScaleReady(const char* operation) {
  if (scale.is_ready()) {
    return true;
  }

  Serial.print(F("HX711 not ready"));
  if (operation != nullptr && operation[0] != '\0') {
    Serial.print(F(" during "));
    Serial.print(operation);
  }
  Serial.println('.');
  Serial.println(F("Check HX711 wiring, power, and data pins (DOUT/CLK)."));
  Serial.println();
  return false;
}

/**
 * @brief Flushes any buffered serial input.
 *
 * @details Reads and discards any available serial input to ensure that subsequent serial reads start with fresh input from the user.
 * 
 * @return {void} No value is returned.
 * 
 * @throws {none} This function does not throw exceptions.
 */
void flushSerialInput() {
  while (Serial.available()) {
    Serial.read();
  }
}

/**
 * @brief Parses a non-negative float from a null-terminated C string.
 *
 * @details Attempts to parse a float value from the input string. 
 * Validates that the entire string is a valid float representation and that the parsed value is non-negative.
 * 
 * @param {const char*} text Input text to parse.
 * @param {float&} outValue Parsed output value on success.
 * @return {bool} True if parsing succeeds and the value is non-negative.
 * 
 * @throws {none} This function does not throw exceptions.
 */
bool parseNonNegativeFloat(const char* text, float& outValue) {
  char* parseEnd = nullptr;                                 // Pointer used by strtof to indicate where parsing stopped
  float parsed;                                             // Parsed float value from the input text
  
  // strtof will set parseEnd to point to the first character after the parsed float.
  parsed = strtof(text, &parseEnd);                   

  if (parseEnd == text) {
    return false;
  }

  // loop is not a blocking concern since strtof has already parsed the float 
  // and we are just validating that the rest of the string is whitespace 
  // and that the value is non-negative, which are both very fast operations
  while (*parseEnd == ' ' || *parseEnd == '\t') {
    ++parseEnd;
  }

  if (*parseEnd != '\0' || parsed < 0.0f) {
    return false;
  }

  outValue = parsed;
  return true;
}

// @todo make non-blocking when starting web interface coding begins.

/**
 * @brief Reads the average weight from the scale over multiple readings.
 * 
 * @details Takes multiple readings from the scale, averages them, and returns the result in pounds.
 * Useful for smoothing out noise in the scale readings and getting a more stable weight measurement.
 * 
 * @param {int} readings Number of readings to average.
 * @param {int} samplesPerReading Number of samples per reading.
 * @return {float} avgWeight The average weight in pounds. 
 * 
 * @throws {none} This function does not throw exceptions.
 */
float readAveragedUnits(int readings, int samplesPerReading) {
  float avgWeight = 0.0f;               // Computed average weight in pounds to return at the end of the function.
  float totalUnits = 0.0f;              // Accumulator summing weight readings across all iterations for averaging 

  for (int readingIndex = 0; readingIndex < readings; ++readingIndex) {
    totalUnits += scale.get_units(samplesPerReading);
  }

  avgWeight = totalUnits / readings;
  return avgWeight;
}

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
  tareCtx.stableChecks = 0;
  tareCtx.stateStartMs = millis();
  tareCtx.state        = TareState::WAIT_STABLE;

  Serial.println();
  Serial.println(F("Startup tare: waiting for stable scale..."));
  Serial.println(F("Auto-detect is active."));
  Serial.print(F("Auto-detect timeout: "));
  Serial.print(EMPTY_CONFIRM_TIMEOUT_MS / 1000UL);
  Serial.println(F(" seconds."));
  Serial.print(F("Stability tolerance: +/- "));
  Serial.print(SETUP_EMPTY_WEIGHT, 2);
  Serial.println(F(" lbs."));
  Serial.println(F("Timeout expiry with stable scale values auto-confirms taring workflow."));
  Serial.println(F("Send 'q' to skip startup tare."));
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
  Serial.println();
  Serial.println(APP_TITLE);
  Serial.println();

  Serial.print("Loaded calibration factor from EEPROM: ");
  Serial.println(calibrationFactor, 2);
  Serial.print("Loaded known calibration weight from EEPROM: ");
  Serial.print(knownWeight, 2);
  Serial.println(" lbs");
  Serial.print("Loaded max propane weight from EEPROM: ");
  Serial.print(maxPropane, 2);
  Serial.println(" lbs");
  Serial.print("Loaded tank tare from EEPROM: ");
  Serial.print(tankTare, 2);
  Serial.println(" lbs");

  Serial.println();
  Serial.println("Startup tare: waiting for stable scale...");
  Serial.println("Auto-detect is active.");
  Serial.print("Auto-detect timeout: ");
  Serial.print(EMPTY_CONFIRM_TIMEOUT_MS / 1000UL);
  Serial.println(" seconds.");
  Serial.print("Stability tolerance: +/- ");
  Serial.print(SETUP_EMPTY_WEIGHT, 2);
  Serial.println(" lbs.");
  Serial.println("Timeout expiry with stable scale values auto-confirms taring workflow.");
  Serial.println("Send 'q' to skip startup tare.");
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

  Serial.println();
  Serial.println("Automatic calibration mode");
  scale.set_scale(calibrationFactor);

  Serial.println();
  Serial.println("Remove all weight from scale.");
  Serial.println("Auto-detect is active.");
  Serial.print("Empty threshold: +/- ");
  Serial.print(MINIMUM_LOAD_WEIGHT, 2);
  Serial.println(" lbs.");
  Serial.println("Send 'q' to cancel.");
  Serial.print("Confirmation timeout: ");
  Serial.print(USER_CONFIRM_TIMEOUT_MS / 1000UL);
  Serial.println(" seconds.");

  calCtx.mode              = CalMode::AUTO;
  calCtx.state             = CalState::WAIT_EMPTY;
  calCtx.stateStartMs      = millis();
  calCtx.stableEmptyChecks = 0;
  calCtx.measuredUnits     = 0.0f;
}

/**
 * @brief Displays all saved EEPROM values with validity status.
 *
 * @details Reads each persisted record (calibration factor, tank tare, and maximum
 * legal propane weight), verifies its magic marker, and prints the stored value.
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

  uint32_t magic = 0;                   // Temporary variable for reading magic numbers from EEPROM
  float storedValue = 0.0f;             // Temporary variable for reading stored float values from EEPROM

  Serial.println();
  Serial.println("EEPROM Saved Values");

  EEPROM.get(CAL_EEPROM_MAGIC_ADDR, magic);
  if (magic == CAL_EEPROM_MAGIC) {
    EEPROM.get(CAL_EEPROM_VALUE_ADDR, storedValue);
    Serial.print("Calibration factor: ");
    Serial.println(storedValue, 2);
  } else {
    Serial.println("Calibration factor: <invalid or not set>");
  }

    EEPROM.get(KNOWN_WEIGHT_EEPROM_MAGIC_ADDR, magic);
  if (magic == KNOWN_WEIGHT_EEPROM_MAGIC) {
    EEPROM.get(KNOWN_WEIGHT_EEPROM_VALUE_ADDR, storedValue);
    Serial.print("Known calibration weight: ");
    Serial.print(storedValue, 2);
    Serial.println(" lbs");
  } else {
    Serial.println("Known calibration weight: <invalid or not set>");
  }

  EEPROM.get(MAX_PROPANE_EEPROM_MAGIC_ADDR, magic);
  if (magic == MAX_PROPANE_EEPROM_MAGIC) {
    EEPROM.get(MAX_PROPANE_EEPROM_VALUE_ADDR, storedValue);
    Serial.print("Max propane weight: ");
    Serial.print(storedValue, 2);
    Serial.println(" lbs");
  } else {
    Serial.println("Max propane weight: <invalid or not set>");
  }

  EEPROM.get(TARE_EEPROM_MAGIC_ADDR, magic);
  if (magic == TARE_EEPROM_MAGIC) {
    EEPROM.get(TARE_EEPROM_VALUE_ADDR, storedValue);
    Serial.print("Tank tare: ");
    Serial.print(storedValue, 2);
    Serial.println(" lbs");
  } else {
    Serial.println("Tank tare: <invalid or not set>");
  }
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
  Serial.println();
  Serial.println(CMD_AUTO_CAL_MSG);
  Serial.println(CMD_EEPROM_MSG);
  Serial.println(CMD_KNOWN_WEIGHT_MSG);
  Serial.println(CMD_LEVEL_MSG);
  Serial.println(CMD_MANUAL_CAL_MSG);
  Serial.println(CMD_PROPANE_WEIGHT_MSG);
  Serial.println(CMD_REZERO_MSG);
  Serial.println(CMD_TANK_TARE_MSG);
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
  
  Serial.println();
  Serial.print("Current known calibration weight: ");
  Serial.print(knownWeight, 2);
  Serial.println(" lbs");
  Serial.print("Enter new known calibration weight in lbs (");
  Serial.print(MIN_PLAUSIBLE_WEIGHT, 2);
  Serial.print(" to ");
  Serial.print(MAX_PROJECT_WEIGHT, 2);
  Serial.println("), then press Enter.");
  Serial.println("After entry, send 's' to save or 'q' to cancel.");
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

  Serial.println();
  Serial.println("Place propane tank on scale.");
  Serial.println("Waiting for tank placement...");
  Serial.print("Load placement timeout: ");
  Serial.print(EMPTY_CONFIRM_TIMEOUT_MS / 1000UL);
  Serial.println(" seconds.");
  Serial.println("Send 'q' to cancel.");
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

  Serial.println();
  Serial.println("Manual calibration mode");
  scale.set_scale(calibrationFactor);

  Serial.println();
  Serial.println("Remove all weight from scale.");
  Serial.println("Auto-detect is active.");
  Serial.print("Empty threshold: +/- ");
  Serial.print(MINIMUM_LOAD_WEIGHT, 2);
  Serial.println(" lbs.");
  Serial.println("Send 'q' to cancel.");
  Serial.print("Confirmation timeout: ");
  Serial.print(USER_CONFIRM_TIMEOUT_MS / 1000UL);
  Serial.println(" seconds.");

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

  Serial.println();
  Serial.println("Runtime re-zero requested.");
  scale.set_scale(calibrationFactor);

  Serial.println();
  Serial.println("Remove all weight from scale.");
  Serial.println("Auto-detect is active.");
  Serial.print("Empty threshold: +/- ");
  Serial.print(MINIMUM_LOAD_WEIGHT, 2);
  Serial.println(" lbs.");
  Serial.println("Send 'q' to cancel.");
  Serial.print("Confirmation timeout: ");
  Serial.print(USER_CONFIRM_TIMEOUT_MS / 1000UL);
  Serial.println(" seconds.");

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

  Serial.println();
  Serial.print("Current tank tare: ");
  Serial.print(tankTare, 2);
  Serial.println(" lbs");
  Serial.print("Enter new tank tare in lbs (");
  Serial.print(MIN_PLAUSIBLE_WEIGHT, 2);
  Serial.print(" to ");
  Serial.print(MAX_PROJECT_WEIGHT, 2);
  Serial.println("), then press Enter.");
  Serial.println("After entry, send 's' to save or 'q' to cancel.");
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
  
  Serial.println();
  Serial.print("Current max propane weight: ");
  Serial.print(maxPropane, 2);
  Serial.println(" lbs");
  Serial.print("Enter new max propane weight in lbs (");
  Serial.print(MIN_PLAUSIBLE_WEIGHT, 2);
  Serial.print(" to ");
  Serial.print(MAX_PROJECT_WEIGHT, 2);
  Serial.println("), then press Enter.");
  Serial.println("After entry, send 's' to save or 'q' to cancel.");
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

  Serial.println();
  Serial.println(APP_TITLE);
  Serial.println();
  
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
      Serial.print("Loaded calibration factor from EEPROM: ");
      Serial.println(calibrationFactor, 2);
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
      Serial.print("Loaded known calibration weight from EEPROM: ");
      Serial.print(knownWeight, 2);
      Serial.println(" lbs");
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
      Serial.print("Loaded max propane weight from EEPROM: ");
      Serial.print(maxPropane, 2);
      Serial.println(" lbs");
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
      Serial.print("Loaded tank tare from EEPROM: ");
      Serial.print(tankTare, 2);
      Serial.println(" lbs");
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

  // Route 'q' cancel when level read is waiting for tank placement
  if (levelCtx.state == LevelState::WAIT_LOAD) {
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
    // @todo case 'c' print current runtime values (cal factor, tank tare, max legal propane weight, known tank weight) for debugging without needing to check EEPROM values
    // @todo case 'd' for resetting EEPROM to default cal, known tank weight, propane tank tare, max legal propane values
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
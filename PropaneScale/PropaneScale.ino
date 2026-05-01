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

// Standard library headers
#include <EEPROM.h>                     // EEPROM library for persistent storage of calibration and tare values
#include <math.h>                       // Math helpers for fabsf and isfinite during value validation
#include <stdlib.h>                     // Standard library for functions like strtof for parsing floats from strings 

// Third party library headers
#include "HX711.h"                      // HX711 library for interfacing with the load cell amplifier to read weight data

// Local library headers
#include "config.h"                     // Local configuration header defining pin assignments, calibration constants, and EEPROM addresses
#include "eeprom_init_logic.h"          // EEPROM load-or-default decision logic extracted for local unit testing

// Global class variables
HX711 scale;                            // HX711 instance for interacting with the load cell amplifier

// Global state variables
float calibrationFactor = 0.0f;         // Calibration factor for converting raw HX711 readings to weight in pounds
bool eepromReady = false;               // Flag to track if EEPROM was successfully initialized
float knownWeightLbs = 0.0f;            // Known weight for calibration
float maxPropaneLbs = 0.0f;             // Maximum legal propane weight in pounds
float tankTare = 0.0f;                  // Tare weight of the empty propane tank in pounds

// Calibration state machine variables and constants

/**
 * @enum CalMode
 * 
 * @brief Enumeration of calibration modes.
 * 
 * @details Used to manage different calibration workflows initiated by the user.
 */
enum class CalMode  : uint8_t { 
  NONE = 0,                             /**< No calibration mode active */
  AUTO = 1,                             /**< Automatic calibration mode */
  MANUAL = 2,                           /**< Manual calibration mode */
  REZERO = 3                            /**< Rezero calibration mode */
};

/**
 * @enum CalState
 * 
 * @brief Enumeration of states for calibration workflows.
 * 
 * @details Used to manage multi-step calibration processes and user prompts during calibration.
 */
enum class CalState : uint8_t { 
  IDLE = 0,                             /**< Not currently in a calibration workflow, waiting for user input to start one */
  WAIT_EMPTY = 1,                       /**< Waiting for the scale to be empty before starting calibration */
  WAIT_LOAD = 2,                        /**< Waiting for a known weight to be placed on the scale */
  ADJUSTING = 3                         /**< Adjusting the calibration factor based on user input or measurements */
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

// @todo move to config.h
// EEPROM sanity limits for persisted values
constexpr float CAL_FACTOR_ABS_MAX = 500000.0f;             // Maximum absolute value for valid calibration factor 
constexpr float CAL_FACTOR_ABS_MIN = 100.0f;                // Minimum absolute value for valid calibration factor 
constexpr float MAX_PROJECT_WEIGHT_LBS = 60.0f;             // Project will never measure a propane tank above nominal 60 lbs
constexpr float MAX_PROPANE_MIN_LBS = 0.1f;                 // Minimum plausible maximum propane weight for tank
constexpr float TANK_TARE_MIN_LBS = 0.0f;                   // Minimum plausible tare weight for empty propane tank

// @todo move to config.h
// UI strings
constexpr char APP_TITLE[] = "Propane Level Scale";
constexpr char CALIBRATION_SAVE_FAILURE_MSG[] = "Failed to save default calibration to EEPROM.";
constexpr char CALIBRATION_SAVE_SUCCESS_MSG[] = "Default calibration saved to EEPROM.";
constexpr char CMD_AUTO_CAL_MSG[] = "Send 'a' to enter automatic calibration mode";
// constexpr char CMD_CURRENT_VALUES_MSG[] = "Send 'c' to print current runtime values";
// constexpr char CMD_DEFAULT_EEPROM_MSG[] = "Send 'd' to reset EEPROM to default values";
constexpr char CMD_EEPROM_MSG[] = "Send 'e' to display saved EEPROM values";
constexpr char CMD_HELP_MSG[] = "Send 'h' to display this help menu";
// constexpr char CMD_KNOWN_WEIGHT_MSG[] = "Send 'k' to enter known weight value for calibration mode";
constexpr char CMD_LEVEL_MSG[] = "Send 'l' to display one liquid propane percent level reading";
constexpr char CMD_MANUAL_CAL_MSG[] = "Send 'm' to enter manual calibration mode";
constexpr char CMD_PROPANE_WEIGHT_MSG[] = "Send 'p' to set maximum legal propane weight";
constexpr char CMD_REZERO_MSG[] = "Send 'r' to re-zero scale with no propane weight on it";
constexpr char CMD_TANK_TARE_MSG[] = "Send 't' to set propane tank tare";

// Helper functions for EEPROM workflows

/**
 * @brief Validates a finite float against configured bounds.
 *
 * @details Optionally validates against the absolute magnitude of the value.
 * Used for calibration factor checks (absolute bounds) and direct positive-range checks.
 * 
 * @param {float} value Value to validate.
 * @param {float} minimumValue Lower bound for validation.
 * @param {float} maximumValue Upper bound for validation.
 * @param {bool} useAbsoluteMagnitude When true, validates fabs(value) instead of value.
 * @return {bool} True when finite and inside [minimumValue, maximumValue].
 * 
 * @throws {none} This function does not throw exceptions.
 */
bool isValidBoundedFloat(float value, float minimumValue, float maximumValue, bool useAbsoluteMagnitude = false) {
  if (!isfinite(value)) {
    return false;
  }

  float candidate = useAbsoluteMagnitude ? fabsf(value) : value;
  return (candidate >= minimumValue) && (candidate <= maximumValue);
}

/**
 * @brief Validates calibration factor values loaded from EEPROM.
 *
 * @details Uses absolute magnitude bounds so both positive and negative
 * calibration factors are accepted when their magnitude is plausible.
 *
 * @param {float} value Calibration factor to validate.
 * @return {bool} True when value is finite and abs(value) is within calibration bounds.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool isValidCalibrationFactorValue(float value) {
  return isValidBoundedFloat(value, CAL_FACTOR_ABS_MIN, CAL_FACTOR_ABS_MAX, true);
}

/**
 * @brief Validates tank tare values loaded from EEPROM.
 *
 * @param {float} value Tank tare value in pounds.
 * @return {bool} True when value is finite and within tank tare bounds.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool isValidTankTareValue(float value) {
  return isValidBoundedFloat(value, TANK_TARE_MIN_LBS, MAX_PROJECT_WEIGHT_LBS);
}

/**
 * @brief Validates max propane weight values loaded from EEPROM.
 *
 * @param {float} value Max propane weight value in pounds.
 * @return {bool} True when value is finite and within max propane bounds.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool isValidMaxPropaneWeightValue(float value) {
  return isValidBoundedFloat(value, MAX_PROPANE_MIN_LBS, MAX_PROJECT_WEIGHT_LBS);
}

/**
 * @brief Loads a float from EEPROM or falls back to a default and saves it.
 *
 * @details Uses extracted load-or-default logic and preserves existing serial status messages.
 *
 * @param {const char*} valueName Human-readable value label for status messages.
 * @param {float} defaultValue Default value to use when no valid EEPROM value exists.
 * @param {float&} targetValue Output variable receiving loaded or default value.
 * @param {bool (*)(float&)} loadFn Function pointer used to load from EEPROM.
 * @param {bool (*)(float)} saveFn Function pointer used to save to EEPROM.
 * @param {bool (*)(float)} validateFn Function pointer used to validate the loaded value.
 * @param {int} decimals Decimal precision for printed values.
 * @param {const char*} units Optional units suffix (for example " lbs"), can be nullptr.
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void loadOrInitializeEepromValue(
  const char* valueName, float defaultValue, float& targetValue,
  bool (*loadFn)(float&), bool (*saveFn)(float), bool (*validateFn)(float),
  int decimals, const char* units)
{
  // Call shared load-or-default logic and capture result for status reporting
  EepromLoadInitResult initResult;
  initResult = loadOrInitializeFloatValue(defaultValue, targetValue, loadFn, saveFn, validateFn);

  if (initResult.status == EEPROM_VALUE_LOADED_VALID) {
    Serial.print("Loaded ");
    Serial.print(valueName);
    Serial.print(" from EEPROM: ");
    Serial.print(targetValue, decimals);

    if (units != nullptr) {
      Serial.println(units);
    } else {
      Serial.println();
    }
    return;
  }

  if (initResult.status == EEPROM_VALUE_INITIALIZED_AFTER_INVALID_LOAD) {
    Serial.print("Loaded ");
    Serial.print(valueName);
    Serial.println(" from EEPROM is invalid. Reverting to default.");
  }

  if (!initResult.saveSucceeded) {
    Serial.print("Failed to initialize default ");
    Serial.print(valueName);
    Serial.println(" in EEPROM.");
  } else {
    Serial.print("Initialized default ");
    Serial.print(valueName);
    Serial.println(" in EEPROM.");
  }
}

// EEPROM workflows

// @todo ask agent if it is possible to write a function that condenses all of the load*FromEeprom functions,
// to reduce repeated code between the different load*FromEeprom functions, 
// since they are very similar except for the input value, specific magic number and EEPROM addresses they use.
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
 * @brief Loads the calibration factor from EEPROM if the magic number is valid.
 * 
 * @details Reads the magic number from EEPROM to verify that a calibration factor has been saved.
 * If the magic number is valid, it loads the calibration factor into the provided reference variable.
 * 
 * @param {float&} factor Reference to a float variable where the loaded calibration factor will be stored.
 * @return {bool} True if the calibration factor was successfully loaded, false if the magic number was invalid.
 * 
 * @throws {none} This function does not throw exceptions.
 */
bool loadCalibrationFromEeprom(float& factor) {
  uint32_t magic = 0;                                       // Magic number read from EEPROM for validation

  EEPROM.get(CAL_EEPROM_MAGIC_ADDR, magic);
  if (magic != CAL_EEPROM_MAGIC) {
    return false;
  }

  EEPROM.get(CAL_EEPROM_VALUE_ADDR, factor);
  return true;
}

/**
 * @brief Loads the maximum legal propane weight value from EEPROM if magic is valid.
 *
 * @details Checks for the maximum propane weight magic number in EEPROM.
 * If the magic number is valid, it loads the maximum propane weight value into the provided reference variable. 
 * 
 * @param {float&} maxPropaneLbs Reference where loaded value is stored.
 * @return {bool} True if value was loaded successfully, false otherwise.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool loadMaxPropaneWeightFromEeprom(float& maxPropaneLbs) {
  uint32_t magic = 0;                                         // Magic number read from EEPROM for validation

  EEPROM.get(MAX_PROPANE_EEPROM_MAGIC_ADDR, magic);
  if (magic != MAX_PROPANE_EEPROM_MAGIC) {
    return false;
  }

  EEPROM.get(MAX_PROPANE_EEPROM_VALUE_ADDR, maxPropaneLbs);
  return true;
}

/**
 * @brief Loads the propane tank tare value from EEPROM if tare magic is valid.
 *
 * @details Checks for the tare magic number in EEPROM.
 * If the magic number is valid, it loads the tare value into the provided reference variable.
 * 
 * @param {float&} tareLbs Reference where loaded tare value is stored.
 * @return {bool} True if tare value was loaded successfully, false otherwise.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool loadTankTareFromEeprom(float& tareLbs) {
  uint32_t magic = 0;                                           // Magic number read from EEPROM for validation

  EEPROM.get(TARE_EEPROM_MAGIC_ADDR, magic);
  if (magic != TARE_EEPROM_MAGIC) {
    return false;
  }

  EEPROM.get(TARE_EEPROM_VALUE_ADDR, tareLbs);
  return true;
}

//@todo ask agent if it is possible to write a function that do all of the save*ToEeprom operations with the magic number and value together, 
// to reduce repeated code between the different save*ToEeprom functions, 
// since they are very similar except for the specific magic number and EEPROM addresses they use. 
// For example, we could have a generic function like "saveFloatWithMagicToEeprom(float value, uint32_t magic, int magicAddr, int valueAddr)" 
// that handles writing both the magic and value to EEPROM and committing, 
// and then the specific save*ToEeprom functions would just call this generic function with the appropriate parameters for their specific value. 
// This would reduce code duplication and centralize the logic for saving values with magic numbers to EEPROM.
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
 * @brief Saves the given calibration factor to EEPROM with a magic number for validation.
 * 
 * @details Writes the calibration magic number and calibration factor to EEPROM, and commits the changes.
 * Returns false immediately if EEPROM was not successfully initialized.
 * 
 * @param {float} factor The calibration factor to save.
 * @return {bool} True if the calibration factor was successfully saved, false otherwise.
 * 
 * @throws {none} This function does not throw exceptions.
 */
bool saveCalibrationToEeprom(float factor) {
  if (!eepromReady) {
    return false;
  }

  EEPROM.put(CAL_EEPROM_MAGIC_ADDR, CAL_EEPROM_MAGIC);
  EEPROM.put(CAL_EEPROM_VALUE_ADDR, factor);
  return EEPROM.commit();
}

/**
 * @brief Saves the maximum legal propane weight value to EEPROM with a magic number for validation.
 *
 * @details Writes the maximum propane weight magic number and value to EEPROM, and commits the changes.
 * Returns false immediately if EEPROM was not successfully initialized.
 * 
 * @param {float} maxPropaneLbs The maximum propane weight value to save.
 * @return {bool} True if the value was successfully saved, false otherwise.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool saveMaxPropaneWeightToEeprom(float maxPropaneLbs) {
  if (!eepromReady) {
    return false;
  }

  EEPROM.put(MAX_PROPANE_EEPROM_MAGIC_ADDR, MAX_PROPANE_EEPROM_MAGIC);
  EEPROM.put(MAX_PROPANE_EEPROM_VALUE_ADDR, maxPropaneLbs);
  return EEPROM.commit();
}

/**
 * @brief Saves the propane tank tare value to EEPROM with a tare-specific magic number.
 *
 * @details Writes the tare magic number and tare value to EEPROM, and commits the changes.
 * Returns false immediately if EEPROM was not successfully initialized.
 * 
 * @param {float} tareLbs Tank tare in pounds to save.
 * @return {bool} True if the tare value was successfully saved, false otherwise.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool saveTankTareToEeprom(float tareLbs) {
  if (!eepromReady) {
    return false;
  }

  EEPROM.put(TARE_EEPROM_MAGIC_ADDR, TARE_EEPROM_MAGIC);
  EEPROM.put(TARE_EEPROM_VALUE_ADDR, tareLbs);
  return EEPROM.commit();
}

// Helper functions for user initiated workflows

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

  while (*parseEnd == ' ' || *parseEnd == '\t') {
    ++parseEnd;
  }

  if (*parseEnd != '\0' || parsed < 0.0f) {
    return false;
  }

  outValue = parsed;
  return true;
}

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
  if (calCtx.state == CalState::WAIT_EMPTY || calCtx.state == CalState::WAIT_LOAD) {
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
      if (!saveCalibrationToEeprom(calibrationFactor)) {
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
      if (!saveCalibrationToEeprom(calibrationFactor)) {
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
    
    if ((millis() - calCtx.stateStartMs) >= USER_CONFIRM_TIMEOUT) {
      
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
    if ((millis() - calCtx.stateStartMs) >= EMPTY_CONFIRM_TIMEOUT) {
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

    // workflow - weight placed for automatic calibration
    if (calCtx.mode == CalMode::AUTO) {
      Serial.println("Weight detected. Measuring stable reading...");

      // Use more samples for the final measurement to reduce noise in the derived factor
      calCtx.measuredUnits = readAveragedUnits(CAL_SAMPLES, LIVE_SAMPLES);

      if (DEF_KNOWN_WEIGHT == 0.0f || calCtx.measuredUnits == 0.0f) {
        // Guard against division by zero or a zero reading that would produce an unusable factor
        Serial.println("Automatic calibration failed: invalid known weight or reading.");
        calCtx.state = CalState::IDLE;
        calCtx.mode  = CalMode::NONE;
        return;
      }

      // units/lb: maps raw ADC counts to pounds
      calibrationFactor = calCtx.measuredUnits / DEF_KNOWN_WEIGHT; 
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

      if (!saveCalibrationToEeprom(calibrationFactor)) {
        Serial.println("Failed to save calibration to EEPROM.");
      } else {
        Serial.println("Calibration saved to EEPROM.");
      }

      calCtx.state = CalState::IDLE;
      calCtx.mode  = CalMode::NONE;
    } 
    
    // workflow - weight placed for manual calibration — enter interactive adjustment
    if (calCtx.mode == CalMode::MANUAL) {
      // serial input handled by handleCalibrationInput()
      Serial.println("Weight detected. Adjust calibration until the reading matches the known weight.");
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
    Serial.print(DEF_KNOWN_WEIGHT, 2);
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
  Serial.print(EMPTY_CONFIRM_TIMEOUT / 1000UL);
  Serial.println(" seconds.");

  calCtx.stateStartMs = millis();
  calCtx.state        = CalState::WAIT_LOAD;
}

/**
 * @brief Waits for a load to exceed a threshold within a timeout window.
 *
 * @details Samples the scale until the absolute reading meets or exceeds the provided threshold.
 *
 * @param {float} loadDetectThreshold Minimum absolute reading required to detect a load.
 * @param {float&} measuredUnits Output receiving the last measured reading.
 * @param {const char*} timeoutMessage Message printed if the wait times out.
 * @return {bool} True when a load is detected before timeout, false otherwise.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool waitForLoadPlacement(float loadDetectThreshold, float& measuredUnits, const char* timeoutMessage) {
  unsigned long startTimeMs = millis();                     // Timestamp marking the start of the waiting period

  while ((millis() - startTimeMs) < EMPTY_CONFIRM_TIMEOUT) {
    measuredUnits = readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES);
    if (fabsf(measuredUnits) >= loadDetectThreshold) {
      return true;
    }
  
    // @todo query agent if this should be removed 
    // or if it is necessary to have some delay here to avoid hammering the HX711 with continuous reads in a tight loop,  
    // since the HX711 may have a limited sample rate and continuous reads without delay could cause issues 
    delay(100);
  }

  Serial.println(timeoutMessage);
  return false;
}

/**
 * @brief Waits for startup empty-scale condition with optional user override.
 *
 * @details Checks repeated raw HX711 readings during setup and auto-confirms when the scale appears empty.
 * The user can also 'q' to skip startup tare.
 *
 * @return {bool} True if startup tare should run; false to skip startup tare.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool waitForStartupEmptyScale() {
  float baseline = 0.0f;                // Initial baseline reading used to check for stability during startup
  float measuredUnits = 0.0f;           // Current measured weight in pounds
  int stableEmptyChecks = 0;            // Consecutive readings considered empty
  unsigned long startTimeMs = millis(); // Start of waiting period
  char temp = '\0';                     // User input from the serial interface


  if (!ensureScaleReady("startup tare")) {
    return false;
  }

  Serial.println();
  Serial.println(F("Startup tare: waiting for stable scale..."));
  Serial.println(F("Auto-detect is active."));
  Serial.print(F("Auto-detect timeout: "));
  Serial.print(EMPTY_CONFIRM_TIMEOUT / 1000UL);
  Serial.println(F(" seconds."));
  Serial.print(F("Stability tolerance: +/- "));
  Serial.print(SETUP_EMPTY_WEIGHT, 2);
  Serial.println(F(" lbs."));
  Serial.println(F("Timeout expiry with stable scale values auto-confirms taring workflow."));
  Serial.println(F("Send 'q' to skip startup tare."));
  
  scale.set_scale(calibrationFactor);

  // Establish baseline before tare; stability is checked relative to this reading.
  baseline = readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES);
  measuredUnits = baseline;

  // Check for stable readings near the initial baseline to auto-confirm empty scale, but allow user to cancel with 'q'.
  while ((millis() - startTimeMs) < EMPTY_CONFIRM_TIMEOUT) {
    measuredUnits = readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES);

    if (fabsf(measuredUnits - baseline) <= SETUP_EMPTY_WEIGHT) {
      stableEmptyChecks++;
      if (stableEmptyChecks >= UNLOAD_CHECK_COUNT) {
        Serial.println("Stable scale detected, proceeding with tare.");
        return true;
      }
    } else {
      stableEmptyChecks = 0;
    }

    if (Serial.available()) {
      temp = Serial.read();
      if (temp == 'q' || temp == 'Q') {
        Serial.println("Startup tare skipped by user.");
        return false;
      }
    }

    // @todo query agent if this should be removed 
    // or if it is necessary to have some delay here to avoid hammering the HX711 with continuous reads in a tight loop,  
    // since the HX711 may have a limited sample rate and continuous reads without delay could cause issues
    delay(100);
  }

  // Timeout: auto-confirm if the reading remained near the initial baseline.
  if (fabsf(measuredUnits - baseline) <= SETUP_EMPTY_WEIGHT) {
    Serial.println("Startup tare auto-confirmed at timeout (stable scale).");
    return true;
  }

  Serial.println("Startup tare timeout: scale unstable, skipping tare.");
  return false;
}

// User initiated functions
// These functions are called in response to user commands over serial

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
    Serial.println("Calibration already in progress. Send 'q' to cancel first.");
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
  Serial.print(USER_CONFIRM_TIMEOUT / 1000UL);
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

  EEPROM.get(TARE_EEPROM_MAGIC_ADDR, magic);
  if (magic == TARE_EEPROM_MAGIC) {
    EEPROM.get(TARE_EEPROM_VALUE_ADDR, storedValue);
    Serial.print("Tank tare: ");
    Serial.print(storedValue, 2);
    Serial.println(" lbs");
  } else {
    Serial.println("Tank tare: <invalid or not set>");
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
}

/**
 * @brief Displays the current propane weight and fill level readings.
 *
 * @details Reads the scale, applies calibration, subtracts tares, and displays
 * the raw weight, propane weight, and current fill level percentage.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void liquidLevel() {
  float loadDetectThreshold = 0.0f;     // Threshold to detect when a load is present on the scale, based on noise measurements
  float measuredUnits = 0.0f;           // Current averaged reading from the scale in pounds
  float propaneLbs = 0.0f;              // Calculated weight of the propane in pounds after subtracting tank tare and platen tare
  float propaneLevel = 0.0f;            // Calculated fill level percentage based on propane weight and maximum legal propane weight for the tank
  float rawLbs = 0.0f;                  // Raw weight reading from the scale in pounds before subtracting tares

  if (!ensureScaleReady("level read")) {
    return;
  }

  // apply the current calibration factor to convert raw readings to weight in pounds
  scale.set_scale(calibrationFactor);

  // measure baseline noise with no load to compute a detection threshold (readings are in pounds)
  loadDetectThreshold = computeLoadDetectThreshold(MINIMUM_LOAD_WEIGHT);

  Serial.println();
  Serial.println("Place propane tank on scale.");
  Serial.println("Waiting for tank placement...");
  Serial.print("Load placement timeout: ");
  Serial.print(EMPTY_CONFIRM_TIMEOUT / 1000UL);
  Serial.println(" seconds.");

  if (!waitForLoadPlacement(loadDetectThreshold, measuredUnits, "Tank placement timed out; cancelled.")) {
    return;
  }

  Serial.println("Tank detected. Reading weight...");
  rawLbs = readAveragedUnits(CAL_SAMPLES, LIVE_SAMPLES);

  propaneLbs = rawLbs - tankTare - PLATEN_TARE;

  // if the calculated propane weight is negative, set it to zero to avoid reporting negative weight
  if (propaneLbs < 0) {
    propaneLbs = 0.0f; 
  }

  // calculate the fill level percentage based on the weight of the propane and the maximum legal propane weight of the tank
  propaneLevel = (maxPropaneLbs > 0.0f) ? (propaneLbs / maxPropaneLbs) * 100.0f : 0.0f;
  
  Serial.print("Scale load: ");
  Serial.print(rawLbs, 1);
  Serial.print(" lbs, ");
  Serial.print("Calculated propane: ");
  Serial.print(propaneLbs, 1);
  Serial.print(" lbs, "); 
  Serial.print("Propane level: ");
  Serial.print(propaneLevel, 1);
  Serial.println("%");
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
  Serial.println(CMD_LEVEL_MSG);
  Serial.println(CMD_MANUAL_CAL_MSG);
  Serial.println(CMD_PROPANE_WEIGHT_MSG);
  Serial.println(CMD_REZERO_MSG);
  Serial.println(CMD_TANK_TARE_MSG);
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
    Serial.println("Calibration already in progress. Send 'q' to cancel first.");
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
  Serial.print(USER_CONFIRM_TIMEOUT / 1000UL);
  Serial.println(" seconds.");

  calCtx.adjustmentStep    = step;
  calCtx.hasManualDisplay  = false;
  calCtx.lastDirection     = 0;
  calCtx.measuredUnits     = 0.0f;
  calCtx.minStep           = minStep;
  calCtx.mode              = CalMode::MANUAL;
  calCtx.originalCalibrationFactor = calibrationFactor;
  calCtx.stableEmptyChecks = 0;
  calCtx.state             = CalState::WAIT_EMPTY;
  calCtx.stateStartMs      = millis();
}

/**
 * @brief Starts the runtime re-zero workflow.
 *
 * @details Validates that no calibration is in progress, checks scale readiness,
 * prints prompts, and sets the calibration context to REZERO mode waiting for
 * an empty scale. The tare is applied asynchronously via tickCalibration().
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void reZero() {
  if (calCtx.state != CalState::IDLE) {
    Serial.println("Calibration already in progress. Send 'q' to cancel first.");
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
  Serial.print(USER_CONFIRM_TIMEOUT / 1000UL);
  Serial.println(" seconds.");

  calCtx.mode              = CalMode::REZERO;
  calCtx.state             = CalState::WAIT_EMPTY;
  calCtx.stateStartMs      = millis();
  calCtx.stableEmptyChecks = 0;
  calCtx.measuredUnits     = 0.0f;
}

// @todo query agent if tankTareUpdate is non-blocking
/**
 * @brief Prompts the user for a tare value and saves it to EEPROM.
 *
 * @details This function interacts with the user over serial to get a new multi-digit tare weight for the propane tank.
 * It validates the input to ensure it's a non-negative float, and allows the user to cancel the update by sending 'q'.
 * 
 * @return {void} No value is returned.
 * 
 * @throws {none} This function does not throw exceptions.
 */
void tankTareUpdate() {
  char inputBuffer[24] = {0};           // Hold user input for the new tank tare value
  int inputIndex = 0;                   // Track the current position in inputBuffer for storing incoming characters
  float parsedValue = 0.0f;             // Parsed value of the new tank tare after validating the input string
  bool waitingForSave = false;          // Indicate currently waiting for the user to confirm save

  flushSerialInput();

  Serial.println();
  Serial.print("Current tank tare: ");
  Serial.print(tankTare, 2);
  Serial.println(" lbs");
  Serial.print("Enter new tank tare in lbs (");
  Serial.print(TANK_TARE_MIN_LBS, 2);
  Serial.print(" to ");
  Serial.print(MAX_PROJECT_WEIGHT_LBS, 2);
  Serial.println("), then press Enter.");
  Serial.println("After entry, send 's' to save or 'q' to cancel.");

  while (true) {
    if (!Serial.available()) {
      delay(10);
      continue;
    }

    char incoming = Serial.read();

    if (incoming == '\r') {
      continue;
    }

    if (!waitingForSave && (incoming == 'q' || incoming == 'Q') && inputIndex == 0) {
      Serial.println("Tank tare update cancelled.");
      return;
    }

    if (waitingForSave) {
      if (incoming == '\n') {
        continue;
      }

      if (incoming == 'q' || incoming == 'Q') {
        Serial.println("Tank tare update cancelled.");
        return;
      }

      if (incoming == 's' || incoming == 'S') {
        tankTare = parsedValue;
        if (!saveTankTareToEeprom(tankTare)) {
          Serial.println("Failed to save tank tare to EEPROM.");
        } else {
          Serial.print("Tank tare saved successfully: ");
          Serial.print(tankTare, 2);
          Serial.println(" lbs");
        }
        return;
      }

      Serial.println("Invalid response. Send 's' to save, or 'q' to cancel.");
      continue;
    }

    if (incoming == '\n') {
      inputBuffer[inputIndex] = '\0';

      if (inputIndex == 1 && (inputBuffer[0] == 'q' || inputBuffer[0] == 'Q')) {
        Serial.println("Tank tare update cancelled.");
        return;
      }

      if (parseNonNegativeFloat(inputBuffer, parsedValue) && isValidBoundedFloat(parsedValue, TANK_TARE_MIN_LBS, MAX_PROJECT_WEIGHT_LBS)) {
        Serial.print("New tank tare entered: ");
        Serial.print(parsedValue, 2);
        Serial.println(" lbs");
        Serial.println("Send 's' to save this value to EEPROM, or 'q' to cancel.");
        waitingForSave = true;
        inputIndex = 0;
        inputBuffer[0] = '\0';
        continue;
      }

      Serial.print("Invalid tank tare. Enter a number from ");
      Serial.print(TANK_TARE_MIN_LBS, 2);
      Serial.print(" to ");
      Serial.print(MAX_PROJECT_WEIGHT_LBS, 2);
      Serial.println(" lbs, or 'q' to cancel.");
      inputIndex = 0;
      inputBuffer[0] = '\0';
      continue;
    }

    if (incoming == 's' || incoming == 'S') {
      if (inputIndex == 0) {
        Serial.println("Enter a tare value first, then send 's' to save.");
        continue;
      }

      inputBuffer[inputIndex] = '\0';
      if (!parseNonNegativeFloat(inputBuffer, parsedValue) || !isValidBoundedFloat(parsedValue, TANK_TARE_MIN_LBS, MAX_PROJECT_WEIGHT_LBS)) {
        Serial.print("Invalid tank tare. Enter a number from ");
        Serial.print(TANK_TARE_MIN_LBS, 2);
        Serial.print(" to ");
        Serial.print(MAX_PROJECT_WEIGHT_LBS, 2);
        Serial.println(" lbs, or 'q' to cancel.");
        inputIndex = 0;
        inputBuffer[0] = '\0';
        continue;
      }

      tankTare = parsedValue;
      if (!saveTankTareToEeprom(tankTare)) {
        Serial.println("Failed to save tank tare to EEPROM.");
      } else {
        Serial.print("Tank tare saved successfully: ");
        Serial.print(tankTare, 2);
        Serial.println(" lbs");
      }
      return;
    }

    if (inputIndex < static_cast<int>(sizeof(inputBuffer) - 1)) {
      inputBuffer[inputIndex++] = incoming;
    } else {
      Serial.println("Input too long. Enter a shorter number, or 'q' to cancel.");
      inputIndex = 0;
      inputBuffer[0] = '\0';
    }
  }
}

// @todo query agent if propaneWeightUpdate is non-blocking
/**
 * @brief Prompts the user for max propane weight and saves it to EEPROM.
 *
 * @details This function interacts with the user over serial to get a new multi-digit maximum legal propane weight.
 * It validates the input to ensure it's a positive float, and allows the user to cancel the update by sending 'q'.
 * 
 * @return {void} No value is returned.
 * 
 * @throws {none} This function does not throw exceptions.
 */
void propaneWeightUpdate() {
  char inputBuffer[24] = {0};           // Buffer to hold user input for the new maximum propane weight as a string
  float parsedValue = 0.0f;             // Variable to hold the parsed float value of the new maximum propane weight after validating the input string
  int inputIndex = 0;                   // Index to track the current position in the inputBuffer for storing incoming characters from serial input
  bool waitingForSave = false;          // Indicate currently waiting for the user to confirm save

  flushSerialInput();
  
  Serial.println();
  Serial.print("Current max propane weight: ");
  Serial.print(maxPropaneLbs, 2);
  Serial.println(" lbs");
  Serial.print("Enter new max propane weight in lbs (");
  Serial.print(MAX_PROPANE_MIN_LBS, 2);
  Serial.print(" to ");
  Serial.print(MAX_PROJECT_WEIGHT_LBS, 2);
  Serial.println("), then press Enter.");
  Serial.println("After entry, send 's' to save or 'q' to cancel.");

  while (true) {
    if (!Serial.available()) {
      delay(10);
      continue;
    }

    char incoming = Serial.read();

    if (incoming == '\r') {
      continue;
    }

    if (!waitingForSave && (incoming == 'q' || incoming == 'Q') && inputIndex == 0) {
      Serial.println("Max propane weight update cancelled.");
      return;
    }

    if (waitingForSave) {
      if (incoming == '\n') {
        continue;
      }

      if (incoming == 'q' || incoming == 'Q') {
        Serial.println("Max propane weight update cancelled.");
        return;
      }

      if (incoming == 's' || incoming == 'S') {
        maxPropaneLbs = parsedValue;
        if (!saveMaxPropaneWeightToEeprom(maxPropaneLbs)) {
          Serial.println("Failed to save max propane weight to EEPROM.");
        } else {
          Serial.print("Max propane weight updated: ");
          Serial.print(maxPropaneLbs, 2);
          Serial.println(" lbs");
        }
        return;
      }

      Serial.println("Invalid response. Send 's' to save, or 'q' to cancel.");
      continue;
    }

    if (incoming == '\n') {
      inputBuffer[inputIndex] = '\0';

      if (inputIndex == 1 && (inputBuffer[0] == 'q' || inputBuffer[0] == 'Q')) {
        Serial.println("Max propane weight update cancelled.");
        return;
      }

      parsedValue = 0.0f;
      if (parseNonNegativeFloat(inputBuffer, parsedValue) && isValidBoundedFloat(parsedValue, MAX_PROPANE_MIN_LBS, MAX_PROJECT_WEIGHT_LBS)) {
        Serial.print("New max propane weight entered: ");
        Serial.print(parsedValue, 2);
        Serial.println(" lbs");
        Serial.println("Send 's' to save this value to EEPROM, or 'q' to cancel.");
        waitingForSave = true;
        inputIndex = 0;
        inputBuffer[0] = '\0';
        continue;
      }

      Serial.print("Invalid max propane weight. Enter a number from ");
      Serial.print(MAX_PROPANE_MIN_LBS, 2);
      Serial.print(" to ");
      Serial.print(MAX_PROJECT_WEIGHT_LBS, 2);
      Serial.println(" lbs, or 'q' to cancel.");
      inputIndex = 0;
      inputBuffer[0] = '\0';
      continue;
    }

    if (incoming == 's' || incoming == 'S') {
      if (inputIndex == 0) {
        Serial.println("Enter a max propane weight first, then send 's' to save.");
        continue;
      }

      inputBuffer[inputIndex] = '\0';
      if (!parseNonNegativeFloat(inputBuffer, parsedValue) || !isValidBoundedFloat(parsedValue, MAX_PROPANE_MIN_LBS, MAX_PROJECT_WEIGHT_LBS)) {
        Serial.print("Invalid max propane weight. Enter a number from ");
        Serial.print(MAX_PROPANE_MIN_LBS, 2);
        Serial.print(" to ");
        Serial.print(MAX_PROJECT_WEIGHT_LBS, 2);
        Serial.println(" lbs, or 'q' to cancel.");
        inputIndex = 0;
        inputBuffer[0] = '\0';
        continue;
      }

      maxPropaneLbs = parsedValue;
      if (!saveMaxPropaneWeightToEeprom(maxPropaneLbs)) {
        Serial.println("Failed to save max propane weight to EEPROM.");
      } else {
        Serial.print("Max propane weight updated: ");
        Serial.print(maxPropaneLbs, 2);
        Serial.println(" lbs");
      }
      return;
    }

    if (inputIndex < static_cast<int>(sizeof(inputBuffer) - 1)) {
      inputBuffer[inputIndex++] = incoming;
    } else {
      Serial.println("Input too long. Enter a shorter number, or 'q' to cancel.");
      inputIndex = 0;
      inputBuffer[0] = '\0';
    }
  }
}

// Main setup and loop functions for initializing the scale, handling serial commands, and reporting weight readings.

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
  
  // If EEPROM initialization fails, we will continue with default values
  // If EEPROM initializes succeeds, we will attempt to load saved values
  // If load fails, we fall back to defaults and attempt to save those defaults to EEPROM for future use
  eepromReady = EEPROM.begin(EEPROM_SIZE_BYTES);

  // @todo add known tank weight to eeprom saves and loads, 
  // and add a user command for updating the known tank weight as well, 
  if (!eepromReady) {
    Serial.println("EEPROM init failed. Using default calibration factor, tank tare, and max propane weight.");
    calibrationFactor = DEF_CALIBRATION_FACTOR;
    tankTare = DEF_TANK_TARE;
    maxPropaneLbs = DEF_MAX_PROPANE;
  } else {
    loadOrInitializeEepromValue(
      "calibration factor",
      DEF_CALIBRATION_FACTOR,
      calibrationFactor,
      loadCalibrationFromEeprom,
      saveCalibrationToEeprom,
      isValidCalibrationFactorValue,
      2,
      nullptr
    );

    loadOrInitializeEepromValue(
      "tank tare",
      DEF_TANK_TARE,
      tankTare,
      loadTankTareFromEeprom,
      saveTankTareToEeprom,
      isValidTankTareValue,
      2,
      " lbs"
    );

    loadOrInitializeEepromValue(
      "max propane weight",
      DEF_MAX_PROPANE,
      maxPropaneLbs,
      loadMaxPropaneWeightFromEeprom,
      saveMaxPropaneWeightToEeprom,
      isValidMaxPropaneWeightValue,
      2,
      " lbs"
    );

  }

  scale.begin(DOUT_PIN, CLK_PIN);

  // project not intended for continuous operation
  // always starting fresh avoids issues with long term stability issues
  if (waitForStartupEmptyScale()) {
    scale.tare();
    Serial.println("Scale is tared and ready.");
  } else {
    Serial.println("Continuing without startup tare.");
    Serial.println("Remove propane weight and send 'r' to re-zero when ready.");
  }
  helpMenu();
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
  // Must call calibration tick function first to keep advancing calibration state machine,
  // even before a serial key is received
  tickCalibration();

  // On no serial input, need return so calibration tick can continue running until next loop iteration
  if (!Serial.available()) {
    return;
  }

  char temp = Serial.read();

  // ignore newline characters that may be sent by the serial monitor after commands
  // need to avoid accidentally triggering multiple commands in a row
  if (temp == '\r' || temp == '\n') {
    return;
  }

  // Route input to the active calibration state machine; 
  // ignore other commands while busy
  if (calCtx.state != CalState::IDLE) {
    handleCalibrationInput(temp);
    return;
  }

  // Fell through to here, so no active calibration workflow, 
  // time to setup for user serial input command handler
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
    // @todo case'k' for entering known weight value for calibration mode
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
      break;
  }

  // flush any extra input after handling a command
  // need to prevent accidental multiple command triggers from a single line of input
  if (handled) {
    flushSerialInput();
  }
}
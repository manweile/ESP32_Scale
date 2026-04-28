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
 */

// Standard library headers
#include <EEPROM.h>                     // EEPROM library for persistent storage of calibration and tare values
#include <math.h>                       // Math helpers for fabsf and isfinite during value validation
#include <stdlib.h>                     // Standard library for functions like strtof for parsing floats from strings 

// Third party library headers
#include "HX711.h"                      // HX711 library for interfacing with the load cell amplifier to read weight data

// Local library headers
#include "config.h"                     // Local configuration header defining pin assignments, calibration constants, and EEPROM addresses

// Global class variables
HX711 scale;                            // HX711 instance for interacting with the load cell amplifier

// Global state variables
float calibration_factor = 0.0f;        // Calibration factor for converting raw HX711 readings to weight in pounds
bool eepromReady = false;               // Flag to track if EEPROM was successfully initialized
float maxPropaneLbs = 0.0f;             // Maximum legal propane weight in pounds
float tankTare = 0.0f;                  // Tare weight of the empty propane tank in pounds

// EEPROM sanity limits for persisted values
constexpr float CAL_FACTOR_ABS_MIN = 100.0f;                // Minimum absolute value for valid calibration factor 
constexpr float CAL_FACTOR_ABS_MAX = 500000.0f;             // Maximum absolute value for valid calibration factor 
constexpr float MAX_PROJECT_WEIGHT_LBS = 60.0f;             // Project will never measure a propane tank above nominal 60 lbs
constexpr float TANK_TARE_MIN_LBS = 0.0f;                   // Minimum plausible tare weight for empty propane tank
constexpr float MAX_PROPANE_MIN_LBS = 0.1f;                 // Minimum plausible maximum propane weight for tank

// UI strings
constexpr char APP_TITLE[] = "Propane Level Scale";
constexpr char CMD_AUTO_CAL_MSG[] = "Send 'a' to enter automatic calibration mode";
constexpr char CMD_SHOW_EEPROM_MSG[] = "Send 'e' to display saved EEPROM values";
constexpr char CMD_SHOW_LEVEL_MSG[] = "Send 'l' to display one propane reading";
constexpr char CMD_MANUAL_CAL_MSG[] = "Send 'm' to enter manual calibration mode";
constexpr char CMD_REZERO_MSG[] = "Send 'r' to re-zero scale with no propane weight on it";
constexpr char CMD_TANK_TARE_MSG[] = "Send 't' to set propane tank tare";
constexpr char CMD_MAX_PROPANE_MSG[] = "Send 'w' to set maximum legal propane weight";
constexpr char DEFAULT_CALIBRATION_SAVE_FAILED_MSG[] = "Failed to save default calibration to EEPROM.";
constexpr char DEFAULT_CALIBRATION_SAVED_MSG[] = "Default calibration saved to EEPROM.";

// Helper functions for EEPROM workflows
// validating workflows, and loading/initializing from EEPROM.

/**
 * @brief Validates that an EEPROM calibration factor is finite and plausible.
 *
 * @details Checks that the calibration factor is a finite number and falls within configured absolute magnitude limits.
 * This helps to catch EEPROM corruption or invalid values that could lead to wildly incorrect weight readings.
 * 
 * @param {float} value Calibration factor to validate.
 * @return {bool} True when value is finite and within expected magnitude limits.
 * 
 * @throws {none} This function does not throw exceptions.
 */
bool isValidCalibrationFactor(float value) {
  if (!isfinite(value)) {
    return false;
  }

  float magnitude = fabsf(value);
  return (magnitude >= CAL_FACTOR_ABS_MIN) && (magnitude <= CAL_FACTOR_ABS_MAX);
}

/**
 * @brief Validates that an EEPROM max propane weight is finite and positive.
 *
 * @details Checks that the maximum propane weight value is a finite number and falls within configured bounds for plausible maximum propane weights.
 * Catches EEPROM corruption or invalid values that could lead to incorrect percentage calculations or unrealistic readings.
 * 
 * @param {float} value Maximum propane pounds value to validate.
 * @return {bool} True when value is finite and within configured propane bounds.
 * 
 * @throws {none} This function does not throw exceptions.
 */
bool isValidMaxPropaneLbs(float value) {
  if (!isfinite(value)) {
    return false;
  }

  return (value >= MAX_PROPANE_MIN_LBS) && (value <= MAX_PROJECT_WEIGHT_LBS);
}

/**
 * @brief Validates that an EEPROM tank tare value is finite and non-negative.
 *
 * @details Checks that the tank tare value is a finite number and falls within configured bounds for plausible tank tare weights.
 * Catches EEPROM corruption or invalid values that could lead to incorrect net weight calculations.
 * 
 * @param {float} value Tank tare in pounds to validate.
 * @return {bool} True when value is finite and within configured tare bounds.
 * 
 * @throws {none} This function does not throw exceptions.
 */
bool isValidTankTare(float value) {
  if (!isfinite(value)) {
    return false;
  }

  return (value >= TANK_TARE_MIN_LBS) && (value <= MAX_PROJECT_WEIGHT_LBS);
}

/**
 * @brief Loads a float from EEPROM or falls back to a default and saves it.
 *
 * @details Attempts to load a persisted value.
 * If loading fails, apply default value, then attempts to save that default value.
 * Prints status messages for both load and initialization paths.
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
  /**
   * *loadFn value is one of the "load<workflow>FromEeprom" functions: 
   * loadCalibrationFromEeprom, loadTankTareFromEeprom, or loadMaxPropaneWeightFromEeprom
   * which attempt to load the value from EEPROM and return true if successful.
   * 
   * *saveFn value is one of the "save<workflow>ToEeprom" functions:
   * saveCalibrationToEeprom, saveTankTareToEeprom, or saveMaxPropaneWeightToEeprom
   * which attempt to save the value to EEPROM and return true if successful.
   * 
   * *validateFn value is one of the "isValid<workflow>" functions:
   * isValidCalibrationFactor, isValidTankTare, or isValidMaxPropaneLbs
   * which return true if the value is valid. 
   */

  // attempt to load the value from EEPROM with the input load<workflow>FromEeprom function pointer
  if (loadFn(targetValue)) {

    // attempt to validate the loaded value with the input isValid<workflow> function pointer
    if (!validateFn(targetValue)) {
      Serial.print("Loaded ");
      Serial.print(valueName);
      Serial.println(" from EEPROM is invalid. Reverting to default.");
    } else {
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
  }

  // Attempt to save the default value with the input save<workflow>ToEeprom function pointer
  targetValue = defaultValue;
  if (!saveFn(targetValue)) {
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
// Loading and saving calibration factor, tank tare, and maximum propane weight values to EEPROM.

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
// Flushing serial input, parsing non-negative floats, reading averaged units from the scale, and handling user confirmation during calibration.

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

  while ((millis() - startTimeMs) < SETUP_EMPTY_MAX_WAIT_MS) {
    measuredUnits = readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES);
    if (fabsf(measuredUnits) >= loadDetectThreshold) {
      return true;
    }
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
  Serial.print(SETUP_EMPTY_MAX_WAIT_MS / 1000UL);
  Serial.println(F(" seconds."));
  Serial.print(F("Stability tolerance: +/- "));
  Serial.print(SETUP_EMPTY_TOLERANCE_LBS, 2);
  Serial.println(F(" lbs."));
  Serial.println(F("Timeout expiry with stable scale values auto-confirms taring workflow."));
  Serial.println(F("Send 'q' to skip startup tare."));
  
  scale.set_scale(calibration_factor);

  // Establish baseline before tare; stability is checked relative to this reading.
  baseline = readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES);
  measuredUnits = baseline;

  // Check for stable readings near the initial baseline to auto-confirm empty scale, but allow user to cancel with 'q'.
  while ((millis() - startTimeMs) < SETUP_EMPTY_MAX_WAIT_MS) {
    measuredUnits = readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES);

    if (fabsf(measuredUnits - baseline) <= SETUP_EMPTY_TOLERANCE_LBS) {
      stableEmptyChecks++;
      if (stableEmptyChecks >= SETUP_EMPTY_REQUIRED_STABLE_CHECKS) {
        Serial.println("Stable scale detected, proceeding with tare.");
        Serial.println();
        return true;
      }
    } else {
      stableEmptyChecks = 0;
    }

    if (Serial.available()) {
      temp = Serial.read();
      if (temp == 'q' || temp == 'Q') {
        Serial.println("Startup tare skipped by user.");
        Serial.println();
        return false;
      }
    }

    delay(100);
  }

  // Timeout: auto-confirm if the reading remained near the initial baseline.
  if (fabsf(measuredUnits - baseline) <= SETUP_EMPTY_TOLERANCE_LBS) {
    Serial.println("Startup tare auto-confirmed at timeout (stable scale).");
    return true;
  }

  Serial.println("Startup tare timeout: scale unstable, skipping tare.");
  return false;
}

/**
 * @brief Waits for an empty-scale condition or user cancel.
 *
 * @details Uses calibrated readings to auto-detect empty scale with a timeout, so workflows do not block.
 * User can still send 'q' to cancel immediately.
 *
 * @param {const char*} cancelMessage Text to print when the user cancels.
 * @return {bool} True when empty scale is detected, false when cancelled or timeout with non-empty scale.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool waitForCalibrationEmptyScale(const char* cancelMessage) {
  float measuredUnits = 0.0f;           // Current measured weight in pounds
  int stableEmptyChecks = 0;            // Consecutive readings considered empty
  unsigned long startTimeMs = millis(); // Start of the waiting period
  char temp = '\0';                     // User input from the serial interface

  if (!ensureScaleReady("empty scale confirmation")) {
    return false;
  }

  // Use calibrated readings and treat near-zero as empty (no propane weight present).
  scale.set_scale(calibration_factor);

  Serial.println();
  Serial.println("Remove all weight from scale.");
  Serial.println("Auto-detect is active.");
  Serial.print("Empty threshold: +/- ");
  Serial.print(PLACED_LOAD_THRESHOLD_LBS, 2);
  Serial.println(" lbs.");
  Serial.println("Send 'q' to cancel.");
  Serial.print("Confirmation timeout: ");
  Serial.print(USER_CONFIRMATION_TIMEOUT_MS / 1000UL);
  Serial.println(" seconds.");

  // Check for stable readings near zero to auto-confirm empty scale, but allow user to cancel with 'q'.
  while ((millis() - startTimeMs) < USER_CONFIRMATION_TIMEOUT_MS) {
    measuredUnits = readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES);

    if (fabsf(measuredUnits) <= PLACED_LOAD_THRESHOLD_LBS) {
      stableEmptyChecks++;
      if (stableEmptyChecks >= SETUP_EMPTY_REQUIRED_STABLE_CHECKS) {
        Serial.println("Empty scale auto-detected, continuing with calibration");
        Serial.println();
        return true;
      }
    } else {
      stableEmptyChecks = 0;
    }

    if (Serial.available()) {
      temp = Serial.read();
      if (temp == 'q' || temp == 'Q') {
        Serial.println(cancelMessage);
        Serial.println();
        return false;
      }
    }

    delay(100);
  }

  // Timeout: auto-confirm if the reading remained near zero, otherwise cancel.
  if (fabsf(measuredUnits) <= PLACED_LOAD_THRESHOLD_LBS) {
    Serial.println("Empty scale auto-confirmed at timeout (stable scale).");
    Serial.println();
    return true;
  }

  Serial.println("Confirmation timed out: scale not empty; cancelled.");
  Serial.println();
  return false;
}

// User initiated functions
// These functions are called in response to user commands over serial.
// Perform automatic calibration, display EEPROM values, display current propane readings, manual calibration, re-zero scale, update maximum propane weight value, update tank tare

/**
 * @brief Runs automatic calibration using a known reference weight.
 *
 * @details Clears any prior scale factor, tares the empty scale, waits for the known
 * reference weight to be placed on the platform, then computes a calibration factor
 * from the measured reading without any further serial input.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void automaticCalibration() {
  float loadDetectThreshold = 0.0f;     // Threshold to detect when the known weight has been placed on the scale
  float measuredUnits = 0.0f;           // Current measured weight in pounds during the calibration process
  float unloadedNoise = 0.0f;           // Measured noise level of the scale when it is unloaded

  Serial.println();
  Serial.println("Automatic calibration mode");

  if (!ensureScaleReady("automatic calibration")) {
    return;
  }

  if (!waitForCalibrationEmptyScale("Automatic calibration cancelled")) {
    calibration_factor = DEF_CALIBRATION_FACTOR;
    if (!saveCalibrationToEeprom(calibration_factor)) {
      Serial.println(DEFAULT_CALIBRATION_SAVE_FAILED_MSG);
    } else {
      Serial.println(DEFAULT_CALIBRATION_SAVED_MSG);
    }
    return;
  }

  Serial.println("Empty scale confirmed. Taring now...");

  scale.set_scale();
  scale.tare();

  unloadedNoise = fabsf(readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES));
  loadDetectThreshold = unloadedNoise * 20.0f;
  if (loadDetectThreshold < MINIMUM_LOAD_THRESHOLD) {
    loadDetectThreshold = MINIMUM_LOAD_THRESHOLD;
  }

  Serial.print("Place the known weight on the scale: ");
  Serial.print(CAL_KNOWN_WEIGHT_LBS, 2);
  Serial.println(" lbs");
  Serial.println("Waiting for weight placement on scale...");
  Serial.print("Load placement timeout: ");
  Serial.print(SETUP_EMPTY_MAX_WAIT_MS / 1000UL);
  Serial.println(" seconds.");

  if (!waitForLoadPlacement(loadDetectThreshold, measuredUnits, "Weight placement timed out; calibration cancelled.")) {
    return;
  }

  Serial.println("Weight detected. Measuring stable reading...");
  measuredUnits = readAveragedUnits(CAL_SAMPLES, LIVE_SAMPLES);

  if (CAL_KNOWN_WEIGHT_LBS == 0.0f || measuredUnits == 0.0f) {
    Serial.println("Automatic calibration failed: invalid known weight or reading.");
    return;
  }

  calibration_factor = measuredUnits / CAL_KNOWN_WEIGHT_LBS;

  Serial.print("Initial calibration factor estimate: ");
  Serial.println(calibration_factor, 2);

  scale.set_scale(calibration_factor);
  Serial.print("Verified reading: ");
  Serial.print(readAveragedUnits(CAL_SAMPLES, LIVE_SAMPLES), 2);
  Serial.println(" lbs");
  Serial.print("Automatic calibration complete, computed calibration factor: ");
  Serial.println(calibration_factor, 2);

  if (!saveCalibrationToEeprom(calibration_factor)) {
    Serial.println("Failed to save calibration to EEPROM.");
  } else {
    Serial.println("Calibration saved to EEPROM.");
  }
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
void displayEepromValues() {
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
void displayLevel() {
  float loadDetectThreshold = 0.0f;     // Threshold to detect when a load is present on the scale, based on noise measurements
  float measuredUnits = 0.0f;           // Current averaged reading from the scale in pounds
  float propaneLbs = 0.0f;              // Calculated weight of the propane in pounds after subtracting tank tare and platen tare
  float propaneLevel = 0.0f;            // Calculated fill level percentage based on propane weight and maximum legal propane weight for the tank
  float rawLbs = 0.0f;                  // Raw weight reading from the scale in pounds before subtracting tares
  float unloadedNoise = 0.0f;           // Measured noise level on the scale with no load

  if (!ensureScaleReady("level read")) {
    return;
  }

  // apply the current calibration factor to convert raw readings to weight in pounds
  scale.set_scale(calibration_factor);

  // measure baseline noise with no load to compute a detection threshold (readings are in pounds)
  unloadedNoise = fabsf(readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES));
  loadDetectThreshold = unloadedNoise * 20.0f;
  if (loadDetectThreshold < PLACED_LOAD_THRESHOLD_LBS) {
    loadDetectThreshold = PLACED_LOAD_THRESHOLD_LBS;
  }

  Serial.println();
  Serial.println("Place propane tank on scale.");
  Serial.println("Waiting for tank placement...");
  Serial.print("Load placement timeout: ");
  Serial.print(SETUP_EMPTY_MAX_WAIT_MS / 1000UL);
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
 * @brief Enters manual calibration mode for the scale.
 * 
 * @details Prompts the user to remove all weight from the scale, then place a known weight on the scale.
 * Allows the user to adjust the calibration factor using serial commands.
 * 
 * @return {void} No value is returned.
 * 
 * @throws {none} This function does not throw exceptions.
 */
void manualCalibration() {
  float adjustmentStep = fabsf(calibration_factor) * 0.01f; // Start step at 1% of current factor magnitude; floor at 0.01% to prevent stall
  int lastDirection = 0;                                    // +1 = last press increased, -1 = last press decreased, 0 = none
  float loadDetectThreshold = 0.0f;                         // Threshold to detect when the known weight has been placed on the scale
  float measuredUnits = 0.0f;                               // Current averaged reading from the scale in pounds during calibration adjustments
  float minStep = fabsf(calibration_factor) * 0.0001f;      // Minimum adjustment step to prevent infinite halving and stalling during fine-tuning
  char temp = '\0';                                         // Temporary variable to hold user input from serial for adjusting calibration factor
  float unloadedNoise = 0.0f;                               // Measured noise level on the scale with no load

  if (adjustmentStep == 0.0f) adjustmentStep = 10.0f;
  if (minStep == 0.0f) minStep = 0.001f;
  
  Serial.println();
  Serial.print("Manual calibration mode");

  if (!ensureScaleReady("manual calibration")) {
    return;
  }

  if (!waitForCalibrationEmptyScale("Manual calibration cancelled")) {
    calibration_factor = DEF_CALIBRATION_FACTOR;
    if (!saveCalibrationToEeprom(calibration_factor)) {
      Serial.println(DEFAULT_CALIBRATION_SAVE_FAILED_MSG);
    } else {
      Serial.println(DEFAULT_CALIBRATION_SAVED_MSG);
    }
    return;
  }

  Serial.println("Empty scale confirmed. Taring now...");

  // clear all calibration and tare settings to start fresh
  scale.set_scale();
  scale.tare();

  unloadedNoise = fabsf(readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES));
  loadDetectThreshold = unloadedNoise * 20.0f;
  if (loadDetectThreshold < MINIMUM_LOAD_THRESHOLD) {
    loadDetectThreshold = MINIMUM_LOAD_THRESHOLD;
  }

  Serial.println("After readings begin, place known weight on scale");
  Serial.println("Waiting for weight placement on scale...");
  Serial.print("Load placement timeout: ");
  Serial.print(SETUP_EMPTY_MAX_WAIT_MS / 1000UL);
  Serial.println(" seconds.");

  if (!waitForLoadPlacement(loadDetectThreshold, measuredUnits, "Weight placement timed out; calibration cancelled.")) {
    return;
  }

  Serial.println("Weight detected. Adjust calibration until the reading matches the known weight.");
  Serial.println("Press + (plus) increase calibration factor");
  Serial.println("Press - (minus) to decrease calibration factor");
  Serial.println("(step halves on direction reversal)");

  while (true) {
    scale.set_scale(calibration_factor);
    Serial.print("Reading: ");
    Serial.print(scale.get_units(), 1);
    Serial.print(" lbs  factor: ");
    Serial.print(calibration_factor, 2);
    Serial.print("  step: ");
    Serial.println(adjustmentStep, 4);

    if (Serial.available()) {
      temp = Serial.read();
      if (temp == '+') {
        if (lastDirection == -1) {
          adjustmentStep = max(adjustmentStep * 0.5f, minStep);
        }
        calibration_factor += adjustmentStep;
        lastDirection = 1;
      } else if (temp == '-') {
        if (lastDirection == 1) {
          adjustmentStep = max(adjustmentStep * 0.5f, minStep);
        }
        calibration_factor -= adjustmentStep;
        lastDirection = -1;
      } else if (temp == 'q' || temp == 'Q') {
        Serial.print("Manual calibration complete, computed calibration factor: ");
        Serial.println(calibration_factor, 2);

        if (!saveCalibrationToEeprom(calibration_factor)) {
          Serial.println("Failed to save calibration to EEPROM.");
        } else {
          Serial.println("Calibration saved to EEPROM.");
        }
        return;
      }
    } else {
      delay(50);
    }
  }
};

/**
 * @brief Re-zeros (tares) the scale during runtime.
 *
 * @details Prompts the user to remove all weight, confirms over serial,
 * and then runs HX711 tare so subsequent readings start from zero load.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void reZeroScale() {
  Serial.println();
  Serial.println("Runtime re-zero requested.");

  if (!ensureScaleReady("re-zero")) {
    return;
  }

  if (!waitForCalibrationEmptyScale("Runtime re-zero cancelled.")) {
    return;
  }

  scale.set_scale();
  scale.tare();
  scale.set_scale(calibration_factor);

  Serial.println("Scale re-zero complete.");
}

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
  char inputBuffer[24] = {0};           // Buffer to hold user input for the new tank tare value as a string
  float parsedValue = 0.0f;             // Variable to hold the parsed float value of the new tank tare after validating the input string
  int inputIndex = 0;                   // Index to track the current position in the inputBuffer for storing incoming characters from serial input
  bool waitingForSave = false;          // Flag to indicate whether the function is currently waiting for the user to confirm saving the new tare value

  flushSerialInput();

  Serial.println();
  Serial.print("Current tank tare: ");
  Serial.print(tankTare, 2);
  Serial.println(" lbs");
  Serial.println("Enter new tank tare in lbs (>= 0), then press Enter.");
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

      if (parseNonNegativeFloat(inputBuffer, parsedValue)) {
        Serial.print("New tank tare entered: ");
        Serial.print(parsedValue, 2);
        Serial.println(" lbs");
        Serial.println("Send 's' to save this value to EEPROM, or 'q' to cancel.");
        waitingForSave = true;
        inputIndex = 0;
        inputBuffer[0] = '\0';
        continue;
      }

      Serial.println("Invalid tank tare. Enter a number >= 0, or 'q' to cancel.");
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
      if (!parseNonNegativeFloat(inputBuffer, parsedValue)) {
        Serial.println("Invalid tank tare. Enter a number >= 0, or 'q' to cancel.");
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
void weightUpdate() {
  char inputBuffer[24] = {0};           // Buffer to hold user input for the new maximum propane weight as a string
  float parsedValue = 0.0f;             // Variable to hold the parsed float value of the new maximum propane weight after validating the input string
  int inputIndex = 0;                   // Index to track the current position in the inputBuffer for storing incoming characters from serial input

  flushSerialInput();
  
  Serial.println();
  Serial.print("Current max propane weight: ");
  Serial.print(maxPropaneLbs, 2);
  Serial.println(" lbs");
  Serial.println("Enter new max propane weight in lbs (> 0). Send 'q' to cancel.");

  while (true) {
    if (!Serial.available()) {
      delay(10);
      continue;
    }

    char incoming = Serial.read();

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      inputBuffer[inputIndex] = '\0';

      if (inputIndex == 1 && (inputBuffer[0] == 'q' || inputBuffer[0] == 'Q')) {
        Serial.println("Max propane weight update cancelled.");
        return;
      }

      parsedValue = 0.0f;
      if (parseNonNegativeFloat(inputBuffer, parsedValue) && parsedValue > 0.0f) {
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

      Serial.println("Invalid max propane weight. Enter a number > 0, or 'q' to cancel.");
      inputIndex = 0;
      inputBuffer[0] = '\0';
      continue;
    }

    if (inputIndex < static_cast<int>(sizeof(inputBuffer) - 1)) {
      inputBuffer[inputIndex++] = incoming;
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

  if (!eepromReady) {
    Serial.println("EEPROM init failed. Using default calibration factor, tank tare, and max propane weight.");
    calibration_factor = DEF_CALIBRATION_FACTOR;
    tankTare = DEF_TANK_TARE;
    maxPropaneLbs = MAX_PROPANE_LBS;
  } else {
    loadOrInitializeEepromValue(
      "calibration factor",
      DEF_CALIBRATION_FACTOR,
      calibration_factor,
      loadCalibrationFromEeprom,
      saveCalibrationToEeprom,
      isValidCalibrationFactor,
      2,
      nullptr
    );

    loadOrInitializeEepromValue(
      "tank tare",
      DEF_TANK_TARE,
      tankTare,
      loadTankTareFromEeprom,
      saveTankTareToEeprom,
      isValidTankTare,
      2,
      " lbs"
    );

    loadOrInitializeEepromValue(
      "max propane weight",
      MAX_PROPANE_LBS,
      maxPropaneLbs,
      loadMaxPropaneWeightFromEeprom,
      saveMaxPropaneWeightToEeprom,
      isValidMaxPropaneLbs,
      2,
      " lbs"
    );

    Serial.println();
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
  Serial.println();

  Serial.println(CMD_AUTO_CAL_MSG);
  Serial.println(CMD_SHOW_EEPROM_MSG);
  Serial.println(CMD_SHOW_LEVEL_MSG);
  Serial.println(CMD_MANUAL_CAL_MSG);
  Serial.println(CMD_REZERO_MSG);
  Serial.println(CMD_TANK_TARE_MSG);
  Serial.println(CMD_MAX_PROPANE_MSG);
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
  char temp = '\0';                     // Temporary variable to hold user input from the serial interface for command processing
  if (Serial.available()) {
    temp = Serial.read();
    
    // ignore newline characters that may be sent by the serial monitor after commands to avoid accidentally triggering multiple commands in a row
    if (temp == '\r' || temp == '\n') {
      return;
    }
    bool handled = true;

    // having empty lower case input cases allows not needing to call tolower() on the input
    switch (temp) {
      case 'a':
      case 'A':
        automaticCalibration();
        break;
      case 'e':
      case 'E':
        displayEepromValues();
        break;
      case 'l':
      case 'L':
        displayLevel();
        break;
      case 'm':
      case 'M':
        manualCalibration();
        break;
      case 'r':
      case 'R':
        reZeroScale();
        break;
      case 't':
      case 'T':
        tankTareUpdate();
        break;
      case 'w':
      case 'W':
        weightUpdate();
        break;
      default:
        handled = false;
        break;
    }

    // flush any extra input after handling a command to prevent accidental multiple command triggers from a single line of input
    if (handled) {
      flushSerialInput();
    }
  }
}
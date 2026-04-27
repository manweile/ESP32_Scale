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
#include <EEPROM.h>               // EEPROM library for persistent storage of calibration and tare values.
#include <stdlib.h>               // Standard library for functions like strtof for parsing floats from strings.  

// Third party library headers
#include "HX711.h"                // HX711 library for interfacing with the load cell amplifier to read weight data.

// Local library headers
#include "config.h"               // Local configuration header defining pin assignments, calibration constants, and EEPROM addresses.

// Global class variables
HX711 scale;                      // HX711 instance for interacting with the load cell amplifier.

// Global state variables
float calibration_factor = 0.0f;  // Calibration factor for converting raw HX711 readings to weight in pounds, loaded from EEPROM or set to default.
bool eepromReady = false;         // Flag to track if EEPROM was successfully initialized
float maxPropaneLbs = 0.0f;       // Maximum legal propane weight in pounds, loaded from EEPROM or set to default.
float propaneLevel = 0.0f;        // Current propane level as a percentage of maximum weight.
float propaneLbs = 0.0f;          // Current propane weight in pounds.
float rawLbs = 0.0f;              // Raw weight reading from HX711 in pounds.
float tankTare = 0.0f;            // Tare weight of the empty propane tank in pounds.
char temp = '\0';                 // Temporary variable for serial input.

// Helper functions for EEPROM workflows
// Displaying saved EEPROM values, loading/saving calibration, tare, and maximum propane weight values to EEPROM.

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
void displaySavedEepromValues() {
  if (!eepromReady) {
    Serial.println("EEPROM is not initialized; no saved values can be read.");
    return;
  }

  uint32_t magic = 0;
  float storedValue = 0.0f;

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
 * @brief Loads the calibration factor from EEPROM if the magic number is valid.
 * 
 * @details This function reads the magic number from EEPROM to verify that a calibration factor has been saved.
 *  If the magic number is valid, it loads the calibration factor into the provided reference variable.
 * 
 * @param {float&} factor Reference to a float variable where the loaded calibration factor will be stored.
 * @return {bool} True if the calibration factor was successfully loaded, false if the magic number was invalid.
 * 
 * @throws {none} This function does not throw exceptions.
 */
bool loadCalibrationFromEeprom(float& factor) {
  uint32_t magic = 0;
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
 * @details This function checks for the maximum propane weight magic number in EEPROM.
 *  If the magic number is valid, it loads the maximum propane weight value into the provided reference variable. 
 * 
 * @param {float&} maxPropaneLbs Reference where loaded value is stored.
 * @return {bool} True if value was loaded successfully, false otherwise.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool loadMaxPropaneWeightFromEeprom(float& maxPropaneLbs) {
  uint32_t magic = 0;
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
 * @details This function checks for the tare magic number in EEPROM.
 *  If the magic number is valid, it loads the tare value into the provided reference variable.
 * 
 * @param {float&} tareLbs Reference where loaded tare value is stored.
 * @return {bool} True if tare value was loaded successfully, false otherwise.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool loadTankTareFromEeprom(float& tareLbs) {
  uint32_t magic = 0;
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
 * @details This function writes the calibration magic number and calibration factor to EEPROM, and commits the changes.
 * 
 * @param {float} factor The calibration factor to save.
 * @return {bool} True if the calibration factor was successfully saved, false otherwise.
 * 
 * @throws {none} This function does not throw exceptions.
 */
bool saveCalibrationToEeprom(float factor) {
  EEPROM.put(CAL_EEPROM_MAGIC_ADDR, CAL_EEPROM_MAGIC);
  EEPROM.put(CAL_EEPROM_VALUE_ADDR, factor);
  return EEPROM.commit();
}

/**
 * @brief Loads the maximum legal propane weight value from EEPROM if magic is valid.
 *
 * @details This function checks for the maximum propane weight magic number in EEPROM.
 *  If the magic number is valid, it loads the maximum propane weight value into the provided reference variable.
 * 
 * @param {float&} maxPropaneLbs Reference where loaded value is stored.
 * @return {bool} True if value was loaded successfully, false otherwise.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool saveMaxPropaneWeightToEeprom(float maxPropaneLbs) {
  EEPROM.put(MAX_PROPANE_EEPROM_MAGIC_ADDR, MAX_PROPANE_EEPROM_MAGIC);
  EEPROM.put(MAX_PROPANE_EEPROM_VALUE_ADDR, maxPropaneLbs);
  return EEPROM.commit();
}

/**
 * @brief Saves the propane tank tare value to EEPROM with a tare-specific magic number.
 *
 * @details This function writes the tare magic number and tare value to EEPROM, and commits the changes.
 * 
 * @param {float} tareLbs Tank tare in pounds to save.
 * @return {bool} True if the tare value was successfully saved, false otherwise.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool saveTankTareToEeprom(float tareLbs) {
  EEPROM.put(TARE_EEPROM_MAGIC_ADDR, TARE_EEPROM_MAGIC);
  EEPROM.put(TARE_EEPROM_VALUE_ADDR, tareLbs);
  return EEPROM.commit();
}

// Helper functions for user initiated workflows
// Flushing serial input, parsing non-negative floats, reading averaged units from the scale, and handling user confirmation during calibration.

/**
 * @brief Flushes any buffered serial input.
 *
 * @details This function reads and discards any available serial input to ensure that subsequent serial reads start with fresh input from the user.
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
 * @details This function attempts to parse a float value from the input string. 
 * It validates that the entire string is a valid float representation and that the parsed value is non-negative.
 * 
 * @param {const char*} text Input text to parse.
 * @param {float&} value Parsed output value on success.
 * @return {bool} True if parsing succeeds and the value is non-negative.
 * 
 * @throws {none} This function does not throw exceptions.
 */
bool parseNonNegativeFloat(const char* text, float& value) {
  char* parseEnd = nullptr;
  float parsed = strtof(text, &parseEnd);

  if (parseEnd == text) {
    return false;
  }

  while (*parseEnd == ' ' || *parseEnd == '\t') {
    ++parseEnd;
  }

  if (*parseEnd != '\0' || parsed < 0.0f) {
    return false;
  }

  value = parsed;
  return true;
}

/**
 * @brief Reads the average weight from the scale over multiple readings.
 * 
 * @details This function takes multiple readings from the scale, averages them, and returns the result in pounds.
 * This is useful for smoothing out noise in the scale readings and getting a more stable weight measurement.
 * 
 * @param {int} readings Number of readings to average.
 * @param {int} samplesPerReading Number of samples per reading.
 * @return {float} avgWeight The average weight in pounds. 
 * 
 * @throws {none} This function does not throw exceptions.
 */
float readAveragedUnits(int readings, int samplesPerReading) {
  float avgWeight = 0.0f;
  float totalUnits = 0.0f;

  for (int readingIndex = 0; readingIndex < readings; ++readingIndex) {
    totalUnits += scale.get_units(samplesPerReading);
  }

  avgWeight = totalUnits / readings;
  return avgWeight;
}

/**
 * @brief Blocks until the user confirms or cancels.
 *
 * @details This function blocks until user confirms an action by sending 'y' over serial, or to cancel by sending 'q'.
 * 
 * @param {const char*} cancelMessage Text to print when the user cancels.
 * @return {bool} True if the user confirmed, false if the user cancelled.
 * 
 * @throws {none} This function does not throw exceptions.
 */
bool waitForUserConfirmation(const char* cancelMessage) {
  Serial.println();
  Serial.println("Remove all weight from scale.");
  Serial.println("Send 'y' when the scale is empty.");
  Serial.println("Send 'q' to cancel.");

  while (true) {
    if (Serial.available()) {
      temp = Serial.read();
      if (temp == 'y' || temp == 'Y') {
        while (Serial.available()) { Serial.read(); }  // flush remaining input
        return true;
      }
      if (temp == 'q' || temp == 'Q') {
        Serial.println(cancelMessage);
        return false;
      }
    } else {
      delay(10);
    }
  }
}

/**
 * @brief Waits for startup empty-scale condition with optional user override.
 *
 * @details During setup, this function checks repeated raw HX711 readings and
 * auto-confirms when the scale appears empty for multiple consecutive checks.
 * The user can also send 'y' to force tare or 'q' to skip startup tare.
 *
 * @return {bool} True if startup tare should run; false to skip startup tare.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool waitForStartupEmptyScale() {
  int stableEmptyChecks = 0;
  float measuredUnits = 0.0f;
  unsigned long startTimeMs = millis();

  Serial.println();
  Serial.println("Startup tare: waiting for stable scale...");
  Serial.println("Auto-detect is active.");
  Serial.print("Auto-detect timeout: ");
  Serial.print(SETUP_EMPTY_MAX_WAIT_MS / 1000UL);
  Serial.println(" seconds.");
  Serial.print("Stability tolerance: +/- ");
  Serial.print(SETUP_EMPTY_TOLERANCE_LBS, 2);
  Serial.println(" lbs.");
  Serial.println("Timeout with stable scale auto-confirms tare.");
  Serial.println("Send 'q' to skip startup tare.");

  scale.set_scale(calibration_factor);

  // Establish baseline before tare; stability is checked relative to this reading.
  float baseline = readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES);
  measuredUnits = baseline;

  while ((millis() - startTimeMs) < SETUP_EMPTY_MAX_WAIT_MS) {
    measuredUnits = readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES);

    if (fabsf(measuredUnits - baseline) <= SETUP_EMPTY_TOLERANCE_LBS) {
      stableEmptyChecks++;
      if (stableEmptyChecks >= SETUP_EMPTY_REQUIRED_STABLE_CHECKS) {
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

// User initiated functions
// These functions are called in response to user commands over serial.
// Perform automatic & manual calibration, updating of tank tare, maximum propane weight values, and re-zero scale.

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
  float loadDetectThreshold = 0.0f;
  float measuredUnits = 0.0f;
  float unloadedNoise = 0.0f;

  Serial.println();
  Serial.println("Automatic calibration mode");

  if (!waitForUserConfirmation("Automatic calibration cancelled")) {
    calibration_factor = DEF_CALIBRATION_FACTOR;
    if (!saveCalibrationToEeprom(calibration_factor)) {
      Serial.println("Failed to save default calibration to EEPROM.");
    } else {
      Serial.println("Default calibration saved to EEPROM.");
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

  while (true) {
    measuredUnits = readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES);
    if (fabsf(measuredUnits) >= loadDetectThreshold) {
      break;
    }
    delay(100);
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
 * @brief Displays the current propane weight and fill level readings.
 *
 * @details Reads the scale, applies calibration, subtracts tares, and displays
 * the raw weight, propane weight, and current fill level percentage.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void displayCurrentPropaneReadings() {
  float loadDetectThreshold = 0.0f;
  float measuredUnits = 0.0f;
  float unloadedNoise = 0.0f;

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

  while (true) {
    measuredUnits = readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES);
    if (fabsf(measuredUnits) >= loadDetectThreshold) {
      break;
    }
    delay(100);
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
  // Start step at 1% of current factor magnitude; floor at 0.01% to prevent stall.
  float adjustmentStep = fabsf(calibration_factor) * 0.01f;
  int lastDirection = 0;   // +1 = last press increased, -1 = last press decreased, 0 = none
  float loadDetectThreshold = 0.0f;
  float measuredUnits = 0.0f;
  float minStep = fabsf(calibration_factor) * 0.0001f;
  float unloadedNoise = 0.0f;

  if (adjustmentStep == 0.0f) adjustmentStep = 10.0f;
  if (minStep == 0.0f) minStep = 0.001f;
  
  Serial.println();
  Serial.print("Manual calibration mode");

  if (!waitForUserConfirmation("Manual calibration cancelled")) {
    calibration_factor = DEF_CALIBRATION_FACTOR;
    if (!saveCalibrationToEeprom(calibration_factor)) {
      Serial.println("Failed to save default calibration to EEPROM.");
    } else {
      Serial.println("Default calibration saved to EEPROM.");
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

  while (true) {
    measuredUnits = readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES);
    if (fabsf(measuredUnits) >= loadDetectThreshold) {
      break;
    }
    delay(100);
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
  if (!waitForUserConfirmation("Runtime re-zero cancelled.")) {
    return;
  }

  scale.set_scale();
  scale.tare();
  scale.set_scale(calibration_factor);

  Serial.println("Scale re-zero complete.");
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
void updateMaxPropaneWeight() {
  char inputBuffer[24] = {0};
  int inputIndex = 0;

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

      float parsedValue = 0.0f;
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
void updateTankTare() {
  char inputBuffer[24] = {0};
  float parsedValue = 0.0f;
  int inputIndex = 0;
  bool waitingForSave = false;

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
  Serial.println("Propane Level Scale\n");

  // If EEPROM initialization fails, we will continue with default values.
  // If EEPROM initializes succeeds, we will attempt to load saved values.
  // If load fails, we fall back to defaults and attempt to save those defaults to EEPROM for future use.
  eepromReady = EEPROM.begin(EEPROM_SIZE_BYTES);

  if (!eepromReady) {
    Serial.println("EEPROM init failed. Using default calibration factor, tank tare, and max propane weight.");
    calibration_factor = DEF_CALIBRATION_FACTOR;
    tankTare = DEF_TANK_TARE;
    maxPropaneLbs = MAX_PROPANE_LBS;
  } else {
    float storedCalibration = DEF_CALIBRATION_FACTOR;
    if (loadCalibrationFromEeprom(storedCalibration)) {
      calibration_factor = storedCalibration;
      Serial.print("Loaded calibration factor from EEPROM: ");
      Serial.println(calibration_factor, 2);
    } else {
      calibration_factor = DEF_CALIBRATION_FACTOR;
      if (!saveCalibrationToEeprom(calibration_factor)) {
        Serial.println("Failed to initialize default calibration in EEPROM.");
      } else {
        Serial.println("Initialized default calibration in EEPROM.");
      }
    }

    float storedTankTare = DEF_TANK_TARE;
    if (loadTankTareFromEeprom(storedTankTare)) {
      tankTare = storedTankTare;
      Serial.print("Loaded tank tare from EEPROM: ");
      Serial.print(tankTare, 2);
      Serial.println(" lbs");
    } else {
      tankTare = DEF_TANK_TARE;
      if (!saveTankTareToEeprom(tankTare)) {
        Serial.println("Failed to initialize default tank tare in EEPROM.");
      } else {
        Serial.println("Initialized default tank tare in EEPROM.");
      }
    }

    float storedMaxPropane = MAX_PROPANE_LBS;
    if (loadMaxPropaneWeightFromEeprom(storedMaxPropane)) {
      maxPropaneLbs = storedMaxPropane;
      Serial.print("Loaded max propane weight from EEPROM: ");
      Serial.print(maxPropaneLbs, 2);
      Serial.println(" lbs");
    } else {
      maxPropaneLbs = MAX_PROPANE_LBS;
      if (!saveMaxPropaneWeightToEeprom(maxPropaneLbs)) {
        Serial.println("Failed to initialize default max propane weight in EEPROM.");
      } else {
        Serial.println("Initialized default max propane weight in EEPROM.");
      }
    }
  }

  scale.begin(DOUT_PIN, CLK_PIN);

  // project not intended for continuous operation
  // always starting fresh avoids issues with long term stability issues
  if (waitForStartupEmptyScale()) {
    scale.tare();
    Serial.println("Scale is tared and ready.\n");
  } else {
    Serial.println("Continuing without startup tare.");
    Serial.println("Remove propane weight and send 'r' to re-zero when ready.\n");
  }

  Serial.println("Send 'a' to enter automatic calibration mode");
  Serial.println("Send 'e' to display saved EEPROM values");
  Serial.println("Send 'l' to display one propane reading");
  Serial.println("Send 'm' to enter manual calibration mode");
  Serial.println("Send 'p' to set propane tank tare");
  Serial.println("Send 'r' to re-zero scale with no propane weight on it");
  Serial.println("Send 'w' to set maximum legal propane weight");
}

/**
 * @brief  Processes serial commands and prints the current scale reading.
 * 
 * @details Handles calibration, tare, and re-zero commands from the serial port,
 * verifies the HX711 is ready, and reports a single propane reading when requested.
 * 
 * @return {void} No value is returned.
 * 
 * @throws {none} This function does not throw exceptions. 
 */
void loop() {
  if (Serial.available()) {
    temp = Serial.read();
    if (temp == '\r' || temp == '\n') {
      return;
    }

    if(temp == 'a' || temp == 'A') {
      automaticCalibration();
    }
    if(temp == 'e' || temp == 'E') {
      displaySavedEepromValues();
    }    
    if(temp == 'l' || temp == 'L') {
      displayCurrentPropaneReadings();
    }
    if(temp == 'm' || temp == 'M') {
      manualCalibration();
    }
    if(temp == 'p' || temp == 'P') {
      updateTankTare();
    }
    if(temp == 'r' || temp == 'R') {
      reZeroScale();
    }
    if(temp == 'w' || temp == 'W') {
      updateMaxPropaneWeight();
    }
  }
}
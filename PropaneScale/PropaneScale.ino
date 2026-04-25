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

#include "HX711.h"
#include <EEPROM.h>
#include "config.h"

HX711 scale;

float calibration_factor = DEF_CALIBRATION_FACTOR;
char temp = '\0';
long zero_factor = 0;
float propaneLevel = 0.0f;
float propaneLbs = 0.0f;
float rawLbs = 0.0f;
float tankTare = DEF_TWENTY_TANK_TARE;

/**
 * @brief  
 * 
 * @details This function saves the given calibration factor to EEPROM with a magic number for validation.
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
 * @brief Loads the calibration factor from EEPROM if the magic number is valid.
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
 * @brief Reads the average weight from the scale over multiple readings.
 * 
 * @details This function takes multiple readings from the scale, averages them, and returns the result in pounds.
 * This is useful for smoothing out noise in the scale readings and getting a more stable weight measurement.
 * 
 * @param {int} [in] readings Number of readings to average.
 * @param {int} [in] samplesPerReading Number of samples per reading.
 * @return {float} The average weight in pounds. 
 * 
 * @throws {none} This function does not throw exceptions.
 */
float readAveragedUnits(int readings, int samplesPerReading) {
  float totalUnits = 0.0f;

  for (int readingIndex = 0; readingIndex < readings; ++readingIndex) {
    totalUnits += scale.get_units(samplesPerReading);
  }

  return totalUnits / readings;
}

/**
 * @brief Blocks until the user confirms or cancels.
 *
 * @details This function blocks until user confirms an action by sending 'y' over serial, or to cancel by sending 'q'.
 * 
 * @param cancelMessage Text to print when the user cancels.
 * @return true if the user confirmed, false if the user cancelled.
 * 
 * @throws {none} This function does not throw exceptions.
 */
bool waitForUserConfirmation(const char* cancelMessage) {
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
    }
  }
}

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
  Serial.println("Waiting for weight placement...");

  while (true) {
    measuredUnits = readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES);
    if (fabsf(measuredUnits) >= loadDetectThreshold) {
      break;
    }
  }

  Serial.println("Weight detected. Measuring stable reading...");
  measuredUnits = readAveragedUnits(LOADED_CHECK_COUNT, LIVE_SAMPLES);

  if (CAL_KNOWN_WEIGHT_LBS == 0.0f || measuredUnits == 0.0f) {
    Serial.println("Automatic calibration failed: invalid known weight or reading.");
    return;
  }

  calibration_factor = measuredUnits / CAL_KNOWN_WEIGHT_LBS;

  Serial.print("Initial calibration_factor estimate: ");
  Serial.println(calibration_factor, 2);

  scale.set_scale(calibration_factor);
  Serial.print("Verified reading: ");
  Serial.print(readAveragedUnits(LOADED_CHECK_COUNT, LIVE_SAMPLES), 2);
  Serial.println(" lbs");
  Serial.print("Automatic calibration complete, computed calibration_factor: ");
  Serial.println(calibration_factor, 2);

  if (!saveCalibrationToEeprom(calibration_factor)) {
    Serial.println("Failed to save calibration to EEPROM.");
  } else {
    Serial.println("Calibration saved to EEPROM.");
  }
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
  Serial.println("Waiting for weight placement...");

  while (true) {
    measuredUnits = readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES);
    if (fabsf(measuredUnits) >= loadDetectThreshold) {
      break;
    }
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
        Serial.print("Manual calibration complete, computed calibration_factor: ");
        Serial.println(calibration_factor, 2);

        if (!saveCalibrationToEeprom(calibration_factor)) {
          Serial.println("Failed to save calibration to EEPROM.");
        } else {
          Serial.println("Calibration saved to EEPROM.");
        }
        return;
      }
    }
  }
};

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

  Serial.println("Propane Level Scale");

  if (!EEPROM.begin(EEPROM_SIZE_BYTES)) {
    Serial.println("EEPROM init failed. Using default calibration factor.");
    calibration_factor = DEF_CALIBRATION_FACTOR;
  } else {
    float storedCalibration = DEF_CALIBRATION_FACTOR;
    if (loadCalibrationFromEeprom(storedCalibration)) {
      calibration_factor = storedCalibration;
      Serial.print("Loaded calibration_factor from EEPROM: ");
      Serial.println(calibration_factor, 2);
    } else {
      calibration_factor = DEF_CALIBRATION_FACTOR;
      if (!saveCalibrationToEeprom(calibration_factor)) {
        Serial.println("Failed to initialize default calibration in EEPROM.");
      } else {
        Serial.println("Initialized default calibration in EEPROM.");
      }
    }
  }

  scale.begin(DOUT_PIN, CLK_PIN);

  Serial.println("Setup requires an empty scale before initial tare.");
  while (!waitForUserConfirmation("Setup confirmation cancelled.")) {
    Serial.println("Cannot continue setup tare until all weight is removed.");
  }

  // clear all calibration and tare settings to start fresh
  scale.set_scale();
  scale.tare();

  Serial.println("Scale is tared and ready.");
  Serial.println("Send 'a' to enter automatic calibration mode");
  Serial.println("Send 'm' to enter manual calibration mode");
}

/**
 * @brief  Processes serial commands and prints the current scale reading.
 * 
 * @details Handles calibration, tare, and re-zero commands from the serial port,
 * verifies the HX711 is ready, and reports the current weight in pounds at a fixed
 * interval during normal operation.
 * 
 * @return {void} No value is returned.
 * 
 * @throws {none} This function does not throw exceptions. 
 */
void loop() {

  if(Serial.available())
  {
    temp = Serial.read();
    if(temp == 'a' || temp == 'A')
      automaticCalibration();
    if(temp == 'm' || temp == 'M')
      manualCalibration();
  }

  // apply the current calibration factor to convert raw readings to weight in pounds
  scale.set_scale(calibration_factor);

  rawLbs = scale.get_units();

  propaneLbs = rawLbs - tankTare - PLATEN_TARE;

  // if the calculated propane weight is negative, set it to zero to avoid reporting negative weight
  if (propaneLbs < 0) {
    propaneLbs = 0.0f; 
  }

  // calculate the fill level percentage based on the weight of the propane and the maximum legal propane weight of the tank
  propaneLevel = (propaneLbs / MAX_TWENTY_PROPANE_LBS) * 100.0f; 
  
  Serial.print("Raw weight: ");
  Serial.print(rawLbs, 1);
  Serial.print(" lbs total, ");
  Serial.print("Propane weight: ");
  Serial.print(propaneLbs, 1);
  Serial.print(" lbs, "); 
  Serial.print("Level: ");
  Serial.print(propaneLevel, 1);
  Serial.println("%");
}
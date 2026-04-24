/*
 Example using the SparkFun HX711 breakout board with a scale
 By: Nathan Seidle
 SparkFun Electronics
 Date: November 19th, 2014
 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
 
 This is the calibration sketch. Use it to determine the calibration_factor that the main example uses. It also
 outputs the zero_factor useful for projects that have a permanent mass on the scale in between power cycles.
 
 Setup your scale and start the sketch WITHOUT a weight on the scale
 Once readings are displayed place the weight on the scale
 Press +/- or a/z to adjust the calibration_factor until the output readings match the known weight
 Use this calibration_factor on the example sketch
 
 This example assumes pounds (lbs). If you prefer kilograms, change the Serial.print(" lbs"); line to kg. The
 calibration factor will be significantly different but it will be linearly related to lbs (1 lbs = 0.453592 kg).
 
 Your calibration factor may be very positive or very negative. It all depends on the setup of your scale system
 and the direction the sensors deflect from zero state

 This example code uses bogde's excellent library: https://github.com/bogde/HX711
 bogde's library is released under a GNU GENERAL PUBLIC LICENSE

 Arduino pin 2 -> HX711 CLK
 3 -> DOUT
 5V -> VCC
 GND -> GND
 
 Most any pin on the Arduino Uno will be compatible with DOUT/CLK.
 
 The HX711 board can be powered from 2.7V to 5V so the Arduino 5V power should be fine.
 
*/
/**
 * @file PropaneScale.ino
 * @author Gerald Manweiler
 * @brief Main application file for ESP32-based propane level scale using HX711 amplifier.
 * @details Implements serial command interface for calibration and weight reporting, 
 * manages HX711 interactions, and applies calibration factors to convert raw readings to weight in pounds.
 * @version 0.1
 * @date 2024-06-01
 */

#include "HX711.h"
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
 * @brief Blocks until the user sends 'y' to confirm or 'q' to cancel.
 *
 * @param cancelMessage Text to print when the user cancels.
 * @return true if the user confirmed, false if the user cancelled.
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

  if (!waitForUserConfirmation("Automatic calibration cancelled")) return;

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

  if (!waitForUserConfirmation("Manual calibration cancelled")) return;

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
  Serial.println("Remove all weight from scale");
  Serial.println("Send 'a' to enter automatic calibration mode");
  Serial.println("Send 'm' to enter manual calibration mode");

  scale.begin(DOUT_PIN, CLK_PIN);

  // clear all calibration and tare settings to start fresh
  scale.set_scale();
  scale.tare();
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

  propaneLbs = rawLbs - tankTare;

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
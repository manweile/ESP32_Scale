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
float level = 0.0f;
float propaneLbs = 0.0f;
float rawLbs = 0.0f;
float tankTare = DEF_TWENTY_TANK_TARE;

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
  Serial.println("Remove all weight from scale");
  Serial.println("After readings begin, place known weight on scale");
  Serial.println("Press + or a to increase calibration factor");
  Serial.println("Press - or z to decrease calibration factor");

    // clear all calibration and tare settings to start fresh
  scale.set_scale();
  scale.tare();

  // get baseline reading with no weight on the scale to determine zero factor
  // This can be used to remove the need to tare the scale
  // Useful in permanent scale projects
  // zero_factor = scale.read_average(); 
  // Serial.print("Zero factor: "); 
  // Serial.println(zero_factor);

  Serial.println("After readings begin, place known weight on scale");
  Serial.println("Press + or a to increase calibration factor");
  Serial.println("Press - or z to decrease calibration factor");

  while (true) {
    scale.set_scale(calibration_factor);
    Serial.print("Reading: ");
    Serial.print(scale.get_units(), 1);
    Serial.print(" lbs");
    Serial.print(" calibration_factor: ");
    Serial.print(calibration_factor);
    Serial.println();

    if(Serial.available())
    {
      temp = Serial.read();
      if(temp == '+' || temp == 'a')
        calibration_factor += 10;
      else if(temp == '-' || temp == 'z')
        calibration_factor -= 10;
      else if(temp == 'q' || temp == 'Q')
      {
        Serial.println("Exiting manual calibration mode");
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
  level = (propaneLbs / MAX_TWENTY_PROPANE_LBS) * 100.0f; 
  
  Serial.print("Raw weight: ");
  Serial.print(rawLbs, 1);
  Serial.print(" lbs total, ");
  Serial.print("Propane weight: ");
  Serial.print(propaneLbs, 1);
  Serial.print(" lbs, "); 
  Serial.print("Level: ");
  Serial.print(level, 1);
  Serial.println("%");
}
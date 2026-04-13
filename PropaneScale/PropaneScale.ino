#include "HX711.h"
#include "config.h"

HX711 scale;

/**
 * @brief Reads the current scale value in pounds from the HX711.
 *
 * @details Averages five HX711 samples using the configured calibration factor and
 * returns the unadjusted weight value before the software zero offset is applied.
 *
 * @return {float} The raw measured weight in pounds.
 *
 * @throws {none} This function does not throw exceptions.
 */
float readRawWeightLbs() {
  return scale.get_units(5);
}

/**
 * @brief Returns the current weight with drift compensation and zero clamping applied.
 *
 * @details Subtracts the software zero offset captured during re-zero operations and
 * clamps readings below the configured deadband to 0.0 pounds to reduce visible noise.
 *
 * @return {float} The adjusted non-negative weight in pounds.
 *
 * @throws {none} This function does not throw exceptions.
 */
float getNonNegativeWeightLbs() {
  float weight = readRawWeightLbs() - zero_offset_lbs - platen_tare;
  if (weight < ZERO_DEADBAND_LBS) {
    return 0.0f;
  }
  return weight;
}

/**
 * @brief Runs the interactive serial calibration workflow.
 *
 * @details Resets the current scale calibration state, tares the scale, prints the
 * zero factor, and then allows the operator to adjust calibration_factor using serial
 * commands until the displayed weight matches a known reference load.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void runCalibrationMode() {
  Serial.println();
  Serial.println("HX711 calibration mode");
  Serial.println("Remove all weight from scale");
  Serial.println("After readings begin, place known weight on scale");
  Serial.println("Press + or a to increase calibration factor");
  Serial.println("Press - or z to decrease calibration factor");
  Serial.println("Press q to exit calibration mode");

  scale.set_scale();
  scale.tare();
  // Keep platen compensation active even after hardware tare.
  zero_offset_lbs = -platen_tare;

  long zero_factor = scale.read_average();
  Serial.print("Zero factor: ");
  Serial.println(zero_factor);

  while (true) {
    scale.set_scale(calibration_factor);
    float weightLbs = getNonNegativeWeightLbs();

    Serial.print("Reading: ");
    Serial.print(weightLbs, 1);
    Serial.print(" lbs calibration_factor: ");
    Serial.println(calibration_factor);

    if (Serial.available()) {
      char temp = Serial.read();
      if (temp == '+' || temp == 'a') {
        calibration_factor += 10;
      } else if (temp == '-' || temp == 'z') {
        calibration_factor -= 10;
      } else if (temp == 'q') {
        Serial.println("Exiting calibration mode");
        scale.set_scale(calibration_factor);
        return;
      }
    }

    delay(200);
  }
}

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
  Serial.begin(38400);
  Serial.println("HX711 scale demo");
  Serial.println("Send 'c' to enter calibration mode");
  Serial.println("Send 't' to tare when scale is empty");
  Serial.println("Send 'r' to re-zero current no-load drift");

  scale.begin(DOUT_PIN, CLK_PIN);
  if (!scale.wait_ready_timeout(2000)) {
    Serial.println("ERROR: HX711 not found. Check wiring and pins.");
    return;
  }

  scale.set_scale(calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch

  Serial.println("Readings:");
}

/**
 * @brief Processes serial commands and prints the current scale reading.
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
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'c') {
      runCalibrationMode();
    } else if (command == 't') {
      Serial.println("Taring... remove all weight from scale");
      scale.tare();
      // Keep platen compensation active even after hardware tare.
      zero_offset_lbs = -platen_tare;
      Serial.println("Tare complete");
    } else if (command == 'r') {
      Serial.println("Re-zeroing current drift...");
      // Preserve platen compensation while cancelling current no-load drift.
      zero_offset_lbs = readRawWeightLbs() - platen_tare;
      Serial.print("Zero offset set to: ");
      Serial.println(zero_offset_lbs, 3);
    }
  }

  if (!scale.is_ready()) {
    Serial.println("HX711 not ready");
    delay(500);
    return;
  }

  Serial.print("Reading: ");
  Serial.print(getNonNegativeWeightLbs(), 1); // Clamp noise-induced negatives to 0
  Serial.print(" lbs"); //You can change this to kg but you'll need to refactor the calibration_factor
  Serial.println();

  delay(500);
}

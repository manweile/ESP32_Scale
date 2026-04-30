#include "HX711.h"
#include "config.h"
#include <EEPROM.h>

HX711 scale;

float calibrationFactor = 0.0f;
float propaneLevel = 0.0f;
float propaneWeight = 0.0f;
float revertFactor = DEF_CALIBRATION_FACTOR;
String tareInput = "";
float tankTare = 0.0f;
float tareValue = 0.0f;
float totalWeight = 0.0f;

constexpr size_t EEPROM_SIZE_BYTES = 64;
constexpr int EEPROM_MAGIC_ADDR = 0;
constexpr int EEPROM_CAL_FACTOR_ADDR = 4;
constexpr uint32_t EEPROM_MAGIC_VALUE = 0xCA1B4A7Eu;

enum class CalLoadResult {
  OK,           ///< Valid calibration factor loaded.
  NO_MAGIC,     ///< EEPROM has never been written (magic mismatch).
  INVALID_VALUE ///< Stored float is not a finite number.
};

/**
 * @brief Loads the calibration factor from EEPROM when available.
 *
 * @param {float*} outFactor Destination for loaded calibration factor.
 * @return {CalLoadResult} OK on success; NO_MAGIC if EEPROM was never written;
 *         INVALID_VALUE if the stored float is not finite.
 *
 * @throws {none} This function does not throw exceptions.
 */
CalLoadResult loadCalibrationFactor(float* outFactor) {
  uint32_t magic = 0;
  float storedFactor = 0.0f;

  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  if (magic != EEPROM_MAGIC_VALUE) {
    return CalLoadResult::NO_MAGIC;
  }

  EEPROM.get(EEPROM_CAL_FACTOR_ADDR, storedFactor);
  if (!isfinite(storedFactor)) {
    return CalLoadResult::INVALID_VALUE;
  }

  *outFactor = storedFactor;
  return CalLoadResult::OK;
}

/**
 * @brief Saves the current calibration factor to EEPROM.
 *
 * @param {float} factor Calibration factor to persist.
 * @return {bool} True if write/commit succeeded, false otherwise.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool saveCalibrationFactor(float factor) {
  EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
  EEPROM.put(EEPROM_CAL_FACTOR_ADDR, factor);

  if (!EEPROM.commit()) {
    Serial.println("ERROR: Failed to commit calibration factor to EEPROM.");
    return false;
  } else {
    Serial.println("Calibration factor successfully saved to EEPROM.");
  }

  return true;
}

/**
 * @brief Runs the automatic calibration workflow.
 * 
 * @details Adjusts the calibration factor based on the ratio of the known weight
 * to the measured weight until the displayed weight matches the known reference load.
 *
 * @param {float} knownWeightLbs The known reference weight in pounds used for automatic calibration.
 * 
 * @throws {none} This function does not throw exceptions.
 */
void autoCalibrate(float knownWeightLbs) {
  char calibrationInput = '\0';
  float newCalibrationFactor = 0.0f;
  long reading = 0.0f;
  float weightLbs = 0.0f;

  newCalibrationFactor = calibrationFactor;

  Serial.println();
  Serial.println("HX711 automatic calibration mode");

  if (!promptConfirm("Ready to tare... remove any weight from the scale.")) {
    return;
  } else {
    // reset the scale's calibration factor and tare with only the platen on the load cells to zero out the platen weight so that readings reflect only the weight of the propane and tank
    scale.set_scale();
    scale.tare();
    scale.set_offset(PLATEN_TARE);
    Serial.println("Tare done...");
  }

  long zero_factor = scale.read_average(); //Get a baseline reading
  Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  Serial.println(zero_factor);

  if (!promptConfirm("Place the known weight on scale.")) {
    return;
  }

  while (true) {
    weightLbs = scale.get_units(CAL_SAMPLES);

    Serial.println();
    Serial.print("Reading: ");
    Serial.print(weightLbs, 1);
    Serial.println(" lbs");

    // serial buffer flush before waiting for input
    while (Serial.available()) {
      Serial.read();
    }

    if (Serial.available()) {
      calibrationInput = Serial.read();

      switch (calibrationInput) {
        case 'q':
        case 'Q':
          Serial.print("Exiting calibration mode, using default calibration factor: ");
          Serial.println(DEF_CALIBRATION_FACTOR);
          scale.set_scale(DEF_CALIBRATION_FACTOR);
          return;
        case 'f':
        case 'F':
          // calculate new calibration factor based on ratio of known weight to measured weight, then apply new calibration factor to scale
          newCalibrationFactor = weightLbs / DEF_KNOWN_WEIGHT_LBS;
          Serial.print("Exiting calibration mode, setting new calibration factor: ");
          Serial.println(newCalibrationFactor);
          calibrationFactor = newCalibrationFactor;
          scale.set_scale(newCalibrationFactor);

          // save the new calibration factor to EEPROM for future use
          if (saveCalibrationFactor(newCalibrationFactor)) {
            Serial.println("Calibration factor saved to EEPROM.");
          }

          return;
        default:
          break;
      }
    }

    delay(200);
  }
}

/**
 * @brief Runs the interactive serial calibration workflow.
 *
 * @details Resets the current scale calibration state, tares the scale, prints the
 * zero factor, and then allows the operator to adjust calibrationFactor using serial
 * commands until the displayed weight matches a known reference load.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void manualCalibrate() {
  char calibrationInput = '\0';
  float newCalibrationFactor = 0.0f;
  float oldCalibrationFactor = 0.0f;
  float weightLbs = 0.0f;

  newCalibrationFactor = calibrationFactor;
  oldCalibrationFactor = calibrationFactor;

  Serial.println();
  Serial.println("HX711 manual calibration mode");

  if (!promptConfirm("Ready to tare... remove any weight from the scale.")) {
    return;
  } else {
    scale.set_scale();
    scale.tare();
    Serial.println("Tare done...");
  }

  long zero_factor = scale.read_average(); //Get a baseline reading
  Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  Serial.println(zero_factor);

  Serial.println("Wait for readings to stabilize, then GENTLY place known weight on scale");
  Serial.println("Increase calibration factor: 1 = +1, 2 = +10, 3 = +100, 4 = +1,000 , 5 = +10,000");
  Serial.println("Decrease calibration factor: 6 = -1, 7 = -10, 8 = -100, 9 = -1,000, 0 = -10,000");
  Serial.println("Press q or Q to exit calibration mode");
  Serial.println("Press f or F to finish calibration and set new calibration factor");

  while (true) {
    scale.set_scale(newCalibrationFactor);
    weightLbs = scale.get_units(CAL_SAMPLES);

    Serial.print("Weight reading for manual calibration: ");
    Serial.print(weightLbs, 1);
    Serial.print(" lbs, ");
    Serial.print("Old calibration factor: ");
    Serial.print(oldCalibrationFactor);
    Serial.print(", New calibration factor: ");
    Serial.println(newCalibrationFactor);

    // serial buffer flush before waiting for input
    while (Serial.available()) {
      Serial.read();
    }

    if (Serial.available()) {
      calibrationInput = Serial.read();
      oldCalibrationFactor = newCalibrationFactor;

      switch (calibrationInput) {
        case '1':
          newCalibrationFactor += 1;
          break;
        case '2':
          newCalibrationFactor += 10;
          break;
        case '3':
          newCalibrationFactor += 100;
          break;
        case '4':
          newCalibrationFactor += 1000;
          break;
        case '5':
          newCalibrationFactor += 10000;
          break;
        case '6':
          newCalibrationFactor -= 1;
          break;
        case '7':
          newCalibrationFactor -= 10;
          break;
        case '8':
          newCalibrationFactor -= 100;
          break;
        case '9':
          newCalibrationFactor -= 1000;
          break;
        case '0':
          newCalibrationFactor -= 10000;
          break;
        // fall through lower case q to upper case Q for exit logic
        case 'q':
        case 'Q': {
          CalLoadResult result = loadCalibrationFactor(&revertFactor);
          if (result == CalLoadResult::OK) {
            Serial.print("Exiting calibration mode, reverting to saved calibration factor: ");
          } else {
            revertFactor = DEF_CALIBRATION_FACTOR;
            if (result == CalLoadResult::NO_MAGIC) {
              Serial.println("No saved calibration factor found in EEPROM.");
            } else {
              Serial.println("Saved calibration factor is invalid.");
            }
            Serial.print("Exiting calibration mode, reverting to default calibration factor: ");
          }
          calibrationFactor = revertFactor;
          Serial.println(revertFactor);
          scale.set_scale(revertFactor);
          return;
        }
        case 'f':
        case 'F':
          Serial.print("Exiting calibration mode, setting new calibration factor: ");
          Serial.println(newCalibrationFactor);
          calibrationFactor = newCalibrationFactor;
          scale.set_scale(newCalibrationFactor);

          // save the new calibration factor to EEPROM for future use
          if (saveCalibrationFactor(newCalibrationFactor)) {
            Serial.println("Calibration factor saved to EEPROM.");
          }

          return;
        default:
          break;
      }
    }

    delay(200);
  }
}

/**
 * @brief Computs the level of propane in tank
 * 
 * @details Subtracts the tare weight of the tank from the total weight reading to get the weight of the propane, 
 * then divides by the maximum legal propane weight to compute a percentage level.
 * 
 * @return {void} No value is returned.
 */
void computePropaneLevel() {
  totalWeight = scale.get_units(LIVE_SAMPLES);
  propaneWeight = totalWeight - tankTare;
  propaneLevel = propaneWeight / DEF_MAX_PROPANE_LBS * 100.0f;
  Serial.println();
  Serial.print("Reading: ");
  Serial.print(propaneLevel, 1);
  Serial.print(" %");
  Serial.println();
}

/**
 * @brief Prompts operator to confirm ready for workflow
 * 
 * @details Ensures the HX711 is ready, asks for explicit confirmation before commencing workflow.
 *
 * @param {const char*} promptMessage The message to display when prompting the operator for confirmation.
 * @return {bool} True when prompts are completed successfully; false on cancel/failure.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool promptConfirm(const char* promptMessage) {
  char confirm = '\0';

  if (!scale.is_ready()) {
    Serial.println("HX711 not ready. Check wiring and pins.");
    return false;
  } else {
    Serial.println("HX711 is ready.");
  }

  Serial.println(promptMessage);
  Serial.println("Press y to confirm scale is ready for workflow");
  Serial.println("Press q to exit");

  // serial buffer flush before waiting for input
  while (Serial.available()) {
    Serial.read();
  }

  // now wait for operator confirmation to proceed with workflow
  while (true) {
    if (Serial.available()) {
      confirm = Serial.read();
      if (confirm == 'y' || confirm == 'Y') {
        Serial.println("Scale ready confirmed.");
        return true;
      }
      if (confirm == 'q' || confirm == 'Q') {
        Serial.println("Exiting workflow.");
        return false;
      }
    }

    delay(10);
  }
}

/**
 * @brief Tare the scale with platen empty
 * 
 * @details Tares the scale with only the platen on the load cells,
 *  which sets the software zero offset to the known weight of the platen so that readings reflect only the weight of the propane and tank.
 * 
 * @return {void} No value is returned.
 * 
 * @throws {none} This function does not throw exceptions.  
 * 
 */
void tarePlaten() {
  Serial.println();
  Serial.println("Tare scale with platen weight");

  if (!promptConfirm("Ready to tare... remove any weight from the platen.")) {
    return;
  } else {
    scale.tare();
    Serial.println("Tare done...");
  }
}

/**
 * 
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

  if (!EEPROM.begin(EEPROM_SIZE_BYTES)) {
    Serial.println("ERROR: EEPROM initialization failed.");
  } else {
    Serial.println("EEPROM initialized successfully.");
  }

  Serial.println("Propane Level Scale");
  Serial.println("Send 'a' to enter automatic calibration mode");
  Serial.println("Send 'd' to display current weight reading in pounds");
  Serial.println("Send 'm' to enter manual calibration mode");
  Serial.println("Send 'l' to get percent level");
  Serial.println("Send 'r' to reset saved calibration factor and tare weight");
  Serial.println("Send 't' to tare scale with empty platen");
 
  scale.begin(DOUT_PIN, CLK_PIN);
  if (!scale.wait_ready_timeout(2000)) {
    Serial.println("ERROR: HX711 not found. Check wiring and pins.");
    return;
  } else {
    Serial.println("HX711 initialized successfully.");
  }

  CalLoadResult calResult = loadCalibrationFactor(&calibrationFactor);
  if (calResult == CalLoadResult::OK) {
    Serial.println("Saved calibration factor loaded from EEPROM.");
  } else {
    calibrationFactor = DEF_CALIBRATION_FACTOR;
    if (calResult == CalLoadResult::NO_MAGIC) {
      Serial.println("No saved calibration factor found in EEPROM. Using default.");
    } else {
      Serial.println("Saved calibration factor is invalid. Using default.");
    }
  }

  Serial.print("Using calibration factor: ");
  Serial.println(calibrationFactor);
  scale.set_scale(calibrationFactor);

  // @todo if there is a persisted propane tank tare weight, retrieve & use persisted 
  // propane tank tare weight is used to calculate tank fill percentage
  tankTare = DEF_TANK_TARE;

  // tare & zero offset to the known weight of the platen so that readings reflect only the weight of the propane and tank
  Serial.println("Taring & setting scale offset with platen weight");
  scale.tare();
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
  char command = '\0';
  float weightLbs = 0.0f;

  if (!scale.is_ready()) {
    Serial.println("HX711 not ready");
    delay(500);
    return;
  }

  // serial buffer flush before waiting for input
  while (Serial.available()) {
    Serial.read();
  }

  if (Serial.available()) {
    command = Serial.read();

    switch (command) {
      case 'a':
        autoCalibrate(DEF_KNOWN_WEIGHT_LBS);
        break;
      case 'm':
        manualCalibrate();
        break;
      case 'l':
        computePropaneLevel();
        break;
      case 'r':
        // clear calibration factor and tare
        scale.set_scale();
        scale.tare();
        Serial.println("Calibration factor and tare weight reset");
        break;
      case 'p':
        Serial.println();
        Serial.println("Enter tare weight of empty tank in pounds, then press Enter");
        while (!Serial.available()) {
          delay(10);
        }
        tareInput = Serial.readStringUntil('\n');
        tareValue = tareInput.toFloat();
        if (tareValue > 0 && tareValue < MAX_TWENTY_WEIGHT_LBS) {
          tankTare = tareValue;
          Serial.print("Tank tare weight set to: ");
          Serial.print(tankTare, 1);
          Serial.println(" lbs");
          // @todo persist tare weight to EEPROM for future use
        } else {
          Serial.println("Invalid tare weight input. Tare weight must be a positive number less than the maximum tank weight.");
        }
        break;
      case 't':
        // tare the scale with only platen on load cells
        tarePlaten();
        break;
      default:
        break;
    }
  }

  // display the current weight reading in pounds without computing level percentage
  weightLbs = scale.get_units(LIVE_SAMPLES);
  Serial.println();
  Serial.print("Weight reading: ");
  Serial.print(weightLbs, 1);
  Serial.println(" lbs");

  

  delay(500);
}

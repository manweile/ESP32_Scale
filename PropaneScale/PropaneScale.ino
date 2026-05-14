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
#include "src/commands.h"                                   // Command processing functions for serial interface
#include "src/eeprom_store.h"                               // EEPROM storage functions
#include "src/parsing_utils.h"                              // Utility functions for validating and parsing input values
#include "src/runtime_report.h"                             // Declarations for runtime reporting functions
#include "src/scale_io.h"                                   // Input/output functions for user workflows and HX711 interactions
#include "src/workflows/input_context.h"                    // Input context definitions for non-blocking user input workflows
#include "src/workflows/input_known_weight.h"               // Handlers for the known weight update workflow
#include "src/workflows/input_propane_weight.h"             // Handlers for the max propane weight update workflow
#include "src/workflows/input_tank_tare.h"                  // Handlers for the tank tare weight update workflow
#include "src/workflows/level_workflow.h"                   // Functions for the liquid level read workflow
#include "src/workflows/workflows_contexts.h"               // Context definitions for non-blocking workflows
#include "src/workflows/startup_tare_workflow.h"            // Functions for the startup tare workflow
#include "src/workflows/calibration_workflow.h"             // Functions for the calibration workflow

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
 * @section State Machine Variables
 */

CalContext calCtx;                                          /**< Calibration context instance to hold state for calibration workflows */
InputContext inputCtx;                                      /**< Non-blocking input context for serial workflows */
LevelContext levelCtx;                                      /**< Level read context instance to hold state for the level read workflow */
TareContext tareCtx;                                        /**< Startup tare context instance */

/**
 * @section State Machine Functions
 */

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
void resetInputContext() {
  inputCtx.mode = InputMode::NONE;
  inputCtx.state = InputState::IDLE;
  inputCtx.index = 0;
  inputCtx.parsedValue = 0.0f;
  inputCtx.buffer[0] = '\0';
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
  // Treat startup not-empty as configured full-tank weight plus margin.
  const float startupNotEmptyThreshold = computeStartupNotEmptyThreshold(tankTare, maxPropane);

  if (tareCtx.state == TareState::IDLE) return;

  if (tareCtx.state == TareState::TARE) {
    Serial.println("Stable scale detected, proceeding with tare.");
    scale.tare();
    saveRuntimeTareOffset();
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

  if ((millis() - tareCtx.stateStartMs) >= CONFIRM_TIMEOUT_MS) {
    float m = readAveragedUnits(1, POLL_SAMPLES);
    if (!isfinite(m)) {
      printScaleNotReadyDiagnostic("startup tare");
      tareCtx.state = TareState::SKIP;
      return;
    }

    char diag[128];
    snprintf(diag, sizeof(diag),
             "Startup tare timeout check: reading=%.2f lbs, baseline=%.2f lbs\n",
             m, tareCtx.baseline);
    Serial.print(diag);

    // a negative reading means we had a false non-empty, so auto re-zero
    if (m < -1.0f) {
      Serial.println("Negative weight detected at startup, auto re-zeroing (tare).");
      tareCtx.state = TareState::TARE;
      return;
    }

    // Not-empty check first
    if (fabsf(m) >= startupNotEmptyThreshold) {
      Serial.println("Startup tare timeout: scale not empty, skipping tare.");
      tareCtx.state = TareState::SKIP;
      return;
    }

    // Only then check stability; require near-zero and sustained stability during the wait window.
    bool nearZero = fabsf(m) <= MINIMUM_LOAD_WEIGHT;
    bool stableFromBaseline = fabsf(m - tareCtx.baseline) <= SETUP_EMPTY_WEIGHT;
    bool stableLongEnough = tareCtx.stableChecks >= UNLOAD_CHECK_COUNT;

    if (nearZero && stableFromBaseline && stableLongEnough) {
      Serial.println("Startup tare auto-confirmed empty at timeout.");
      tareCtx.state = TareState::TARE;
    } else {
      Serial.println("Startup tare timeout: scale not-empty or unstable, skipping tare.");
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

  // 1) Not-empty check first.
  if (fabsf(m) >= startupNotEmptyThreshold) {
    tareCtx.stableChecks = 0;
    return;
  }

  // 2) Only then check stability.
  if (fabsf(m - tareCtx.baseline) <= SETUP_EMPTY_WEIGHT) {
    tareCtx.stableChecks++;
  } else {
    tareCtx.stableChecks = 0;
  }
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

  // Restore previously saved runtime tare offset so startup empty/load checks use a known-empty reference.
  float savedRuntimeOffset = 0.0f;
  if (loadFromEeprom(savedRuntimeOffset,
                     HX711_OFFSET_EEPROM_MAGIC_ADDR,
                     HX711_OFFSET_EEPROM_MAGIC,
                     HX711_OFFSET_EEPROM_VALUE_ADDR)) {
    scale.set_offset(static_cast<long>(savedRuntimeOffset));
  }

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

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

// Third party library headers
#include "HX711.h"                                          // HX711 library for interfacing with the load cell amplifier to read weight data

//Local library headers
#include "config.h"                                         // Configuration constants for the ESP32-based propane level scale
#include "src/app_startup.h"                                // Application startup initialization functions
#include "src/commands.h"                                   // Command processing functions for serial interface
#include "src/eeprom_store.h"                               // EEPROM storage functions
#include "src/parsing_utils.h"                              // Utility functions for validating and parsing input values
#include "src/runtime_report.h"                             // Declarations for runtime reporting functions
#include "src/scale_io.h"                                   // Input/output functions for user workflows and HX711 interactions
#include "src/web_ble.h"                                    // BLE interface
#include "src/workflows/input_context.h"                    // Input context definitions for non-blocking user input workflows
#include "src/workflows/input_known_weight.h"               // Handlers for the known weight update workflow
#include "src/workflows/input_propane_weight.h"             // Handlers for the max propane weight update workflow
#include "src/workflows/input_tank_tare.h"                  // Handlers for the tank tare weight update workflow
#include "src/workflows/level_workflow.h"                   // Functions for the liquid level read workflow
#include "src/workflows/workflows_contexts.h"               // Context definitions for non-blocking workflows
#include "src/workflows/startup_tare_workflow.h"            // Functions for the startup tare workflow
#include "src/workflows/calibration_workflow.h"             // Functions for the calibration workflow

// Global Class Instances
HX711 scale;                                                /**< HX711 instance for interacting with the load cell amplifier */

// Global State Variables
float calibrationFactor = 0.0f;                             /**< Calibration factor for converting raw HX711 readings to weight in pounds */
bool eepromReady = false;                                   /**< Flag to track if EEPROM was successfully initialized */
float knownWeight = 0.0f;                                   /**< Known weight for calibration */
float maxPropane = 0.0f;                                    /**< Maximum legal propane weight in pounds */
float tankTare = 0.0f;                                      /**< Tare weight of the empty propane tank in pounds */

// State Machine Variables
CalContext calCtx;                                          /**< Calibration context instance to hold state for calibration workflows */
InputContext inputCtx;                                      /**< Non-blocking input context for serial workflows */
LevelContext levelCtx;                                      /**< Level read context instance to hold state for the level read workflow */
TareContext tareCtx;                                        /**< Startup tare context instance */

// Definitions & Declarations for State Machine Functions

/**
 * @brief Resets the input context to its initial state.
 *
 * @details Resets the mode & state, index, parsed value, and buffer to default values.
 * Called at the end of each input workflow to prepare for the next one.
 * Defined & declared here so accessible for input workflows without circular dependencies.
 *
 * @throws {none} This function does not throw exceptions.
 */
void resetInputContext()
{
  inputCtx.mode = InputMode::NONE;
  inputCtx.state = InputState::IDLE;
  inputCtx.index = 0;
  inputCtx.parsedValue = 0.0f;
  inputCtx.buffer[0] = '\0';
}

// Definitions & Declarations for Project lifecycle functions

/**
 * @brief Initializes the application and starts the startup tare workflow.
 *
 * @details Initializes the serial interface, sets up the HX711 scale, applies calibration from EEPROM,
 * and begins the startup tare workflow.
 *
 * @throws {none} This function does not throw exceptions.
 */
void setup()
{
  Serial.begin(BAUD);
  initializeApp();
  beginStartupTare();
}

/**
 * @brief Main application loop that processes serial input and advances workflows.
 *
 * @details Advances the calibration, level read, and tare workflows on each iteration.
 * Processes serial input for workflow interactions and command dispatch.
 *
 * @throws {none} This function does not throw exceptions.
 */
void loop()
{
  // can't have any queued serial output before processing new input or advancing workflows
  drainQueuedSerialOutput();

  // tickTare has to preempt all other workflows and user input until complete,
  // to guarantee stable tare condition before allowing any other interactions or workflows to run
  tickTare();

  if (tareCtx.state != TareState::IDLE) {
    return;
  }

  // Advance other active state machines each iteration
  tickLevelRead();
  tickCalibration();

  // On no serial input, need return so state machines can continue running until next loop iteration
  if (!Serial.available()) {
    return;
  }

  char temp = Serial.read();

  // level read is raison d'etre of this project,
  // it goes after taring is stable to ensure no interference from anything else
  if (levelCtx.state != LevelState::IDLE) {
    handleLevelReadInput(temp);
    return;
  }

  // multi-character input workflows all require user hit enter after inputting new value,
  // do not interact with the scale hardware at all, and have unique input handling requirements
  // tank tare & propane weight likely to be used in preparation for level workflow,
  // and may require a calibration workflow afterward
  // known weight likely to used in preparation for calibration workflow

  if (inputCtx.mode == InputMode::TANK_TARE) {
    handleTankTareInput(temp);
    return;
  }

  if (inputCtx.mode == InputMode::PROPANE_WEIGHT) {
    handlePropaneWeightInput(temp);
    return;
  }

  if (inputCtx.mode == InputMode::KNOWN_WEIGHT) {
    handleKnownWeightInput(temp);
    return;
  }

  // Don't want accidentally triggered multiple commands in a row
  // MUST come after all the input handlers because user hits enter somewhere in those workflows,
  // so newlines need to be processed by handlers but ignored for general command dispatch
  if (temp == '\r' || temp == '\n') {
    return;
  }

  // calibration workflows (especially manual) are special workflows that require single-character command to start,
  // do interact with the scale hardware, and have unique input handling and display logic separate from the other multi-character input workflows,
  // so they go after the input context workflow checks but before the single-character command dispatch
  if (calCtx.state != CalState::IDLE) {
    handleCalibrationInput(temp);
    return;
  }

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

  // flush any extra input after handling a command to prevent accidental multiple command triggers from a single line of input
  if (handled && inputCtx.mode == InputMode::NONE) {
    flushSerialInput();
  }
}

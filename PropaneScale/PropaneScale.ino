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
 * Needs to be accessible for workflow implementations without circular dependencies.
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
 * @details Called each loop() iteration. Handles the WAIT_STABLE, TARE, and SKIP states. Returns immediately when IDLE.
 * Needs to be accessible so the main loop can manage serial input and advance state based on timing and readings.
 *
 * @throws {none} This function does not throw exceptions.
 */
void tickTare() {
  // Treat startup not-empty as configured full-tank weight plus margin.
  const float startupNotEmptyThreshold = computeStartupNotEmptyThreshold(tankTare, maxPropane);

  // fast idle detect to save cycles when we are not in a tare workflow
  if (tareCtx.state == TareState::IDLE) return;

  // primary happy path
  if (tareCtx.state == TareState::TARE) {
    Serial.println("Stable scale detected, proceeding with tare.");
    scale.tare();
    saveRuntimeTareOffset();
    Serial.println("Scale is tared and ready.");
    tareCtx.state = TareState::IDLE;
    helpMenu();
    return;
  }

  // alternate happy path where user skipped taring
  if (tareCtx.state == TareState::SKIP) {
    Serial.println("Continuing without startup tare.");
    Serial.println("Remove propane weight and send 'r' to re-zero when ready.");
    tareCtx.state = TareState::IDLE;
    helpMenu();
    return;
  }

  // if we have gotten here, we are in WAIT_STABLE, 
  // need to always check for user cancel before doing any other processing
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'q' || c == 'Q') {
      Serial.println("Startup tare skipped by user.");
      tareCtx.state = TareState::SKIP;
      return;
    }
  }

  // still in WAIT_STABLE, need to quick check scale is ready to avoid long blocking
  if ((millis() - tareCtx.stateStartMs) >= CONFIRM_TIMEOUT_MS) {

    // Use non-blocking averaged read driven by loop() ticks
    float m;
    if (!nonBlockingAvgUnits(1, POLL_SAMPLES, m)) {
      // averaging in progress; try again on next loop tick
      return;
    }
    
    // bad scale reading, warn user to check hardware
    if (!isfinite(m)) {
      printScaleNotReadyDiagnostic("startup tare");
      tareCtx.state = TareState::SKIP;
      return;
    }

    char diag[128];
    snprintf(diag, sizeof(diag), "Startup tare timeout check: reading=%.2f lbs, baseline=%.2f lbs\n", m, tareCtx.baseline);
    Serial.print(diag);

    // a negative reading is typically a false non-empty
    if (m < -1.0f) {
      Serial.println("Negative weight detected at startup, auto re-zeroing (tare).");
      tareCtx.state = TareState::TARE;
      return;
    }

    // a true non-empty, but we require near-zero and sustained stability
    if (fabsf(m) >= startupNotEmptyThreshold) {
      Serial.println("Startup tare timeout: scale not empty, skipping tare.");
      tareCtx.state = TareState::SKIP;
      return;
    }

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

  // probably at stable point where we can update state
  // verify scale reading is valid before doing any processing
  float m;
  if (!nonBlockingAvgUnits(1, POLL_SAMPLES, m)) {
    // averaging in progress; continue on next loop tick
    return;
  }

  if (!isfinite(m)) {
    printScaleNotReadyDiagnostic("startup tare");
    tareCtx.state = TareState::SKIP;
    return;
  }

  // scale is not empty, we want to reset stability checks 
  // requires new window of stable readings below the not-empty threshold before auto-confirming
  if (fabsf(m) >= startupNotEmptyThreshold) {
    tareCtx.stableChecks = 0;
    return;
  }

  // stable relative to baseline, can increment stable check count for auto-confirm tare at timeout
  if (fabsf(m - tareCtx.baseline) <= SETUP_EMPTY_WEIGHT) {
    tareCtx.stableChecks++;
  } else {
    tareCtx.stableChecks = 0;
  }
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
void setup() {
  Serial.begin(BAUD);
  initializeApp();
  beginTare();
}

/**
 * @brief Main application loop that processes serial input and advances workflows.
 * 
 * @details Advances the calibration, level read, and tare workflows on each iteration.
 * Processes serial input for workflow interactions and command dispatch. 
 * 
 * @throws {none} This function does not throw exceptions. 
 */
void loop() {
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

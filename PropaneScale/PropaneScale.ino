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
#include "src/wifi.h"                                       // WiFi module interface for handling HTTP requests and providing telemetry
#include "src/wifi_bridge.h"                                // Wifi callbacks for WiFi handlers
#include "src/workflows/input_context.h"                    // Input context definitions for non-blocking user input workflows
#include "src/workflows/input_known_weight.h"               // Handlers for the known weight update workflow
#include "src/workflows/input_propane_weight.h"             // Handlers for the max propane weight update workflow
#include "src/workflows/input_tank_tare.h"                  // Handlers for the tank tare weight update workflow
#include "src/workflows/level_workflow.h"                   // Functions for the liquid level read workflow
#include "src/workflows/calibration_workflow.h"             // Functions for the calibration workflow
#include "src/workflows/startup_tare_workflow.h"            // Functions for the startup tare workflow
#include "src/workflows/workflows_contexts.h"               // Context definitions for non-blocking workflows

// Global Class Instances
HX711 scale;                                                /**< HX711 instance for interacting with the load cell amplifier */

// Global State Variables
float calibrationFactor = 0.0f;                             /**< Calibration factor for converting raw HX711 readings to weight in pounds */
bool  eepromReady       = false;                            /**< Flag to track if EEPROM was successfully initialized */
float knownWeight       = 0.0f;                             /**< Known weight for calibration */
float maxPropane        = 0.0f;                             /**< Maximum legal propane weight in pounds */
float tankTare          = 0.0f;                             /**< Tare weight of the empty propane tank in pounds */
bool  wifiStarted       = false;                            /**< Tracks whether WiFi has been initialized */

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
 * @brief Initializes the application, starts the startup tare workflow and initializes WiFi.
 *
 * @details Initializes the serial interface, sets up the HX711 scale, applies calibration from EEPROM,
 * and begins the startup tare workflow, and initializes WiFi.
 *
 * @throws {none} This function does not throw exceptions.
 */
void setup()
{
  // Wifi and web ui are integral to the user experience of this project,
  // so initialize those first before doing anything else
  registerCallbacks(&g_wifi_callbacks);

  // Block until WiFi (STA or AP) is successfully started.
  while (!initWifi()) {
    delay(2000);
  }

  wifiStarted = true;

  // Now that WiFi is up, can initialize the scale and start the startup tare workflow,
  // which may need to report status or errors to the web UI
  initializeApp();
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
  // tickWifi to keep the HTTP server responsive and handle incoming requests,
  // which may trigger workflow actions via callbacks
  if (wifiStarted) {
    tickWifi();
  }

  // tickTare has to preempt all other workflows and user input until complete,
  // to guarantee stable tare condition before allowing any other interactions or workflows to run
  tickTare();

  if (tareCtx.state != TareState::IDLE) {
    return;
  }

  // Advance other active state machines each iteration
  tickLevelRead();
  // tickCalibration();

  // level read is raison d'etre of this project,
  // it goes after taring is stable to ensure no interference from anything else
  if (levelCtx.state != LevelState::IDLE) {
    return;
  }
}

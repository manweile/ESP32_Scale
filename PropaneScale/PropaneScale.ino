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
#include "src/workflows/calibration_workflow.h"             // Functions for the calibration workflow
#include "src/workflows/input_context.h"                    // Input context definitions for non-blocking user input workflows
#include "src/workflows/input_known_weight.h"               // Handlers for the known weight update workflow
#include "src/workflows/input_propane_weight.h"             // Handlers for the max propane weight update workflow
#include "src/workflows/input_tank_tare.h"                  // Handlers for the tank tare weight update workflow
#include "src/workflows/level_workflow.h"                   // Functions for the liquid level read workflow
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

// Declarations for FreeRTOS Tasks

/**
 * @brief Task function for handling scale logic, including HX711 interactions and user workflows.
 *
 * @details Runs an infinite loop to manage scale readings, advance user workflows (calibration, level read, tare), and process serial input.
 *
 * @param pvParameters {void*} Unused parameter required by FreeRTOS task signature.
 *
 * @throws {none} This function does not throw exceptions.
 */
void scaleTask(void* pvParameters)
{
  (void)pvParameters;

  // Wait for WiFi to be available so startup can report status to UI
  while (!wifiStarted) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Initialize scale, start workflows, etc.
  initializeApp();

  for (;;) {
    tickTare();

    if (tareCtx.state != TareState::IDLE) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    tickLevelRead();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/**
 * @brief Task function for handling WiFi and web UI operations.
 *
 * @details Runs an infinite loop to manage WiFi connectivity, handle incoming HTTP requests, and provide telemetry data to the web UI.
 *
 * @param pvParameters {void*} Unused parameter required by FreeRTOS task signature.
 *
 * @throws {none} This function does not throw exceptions.
 */
void wifiTask(void* pvParameters)
{
  (void)pvParameters;

  // Block until WiFi (STA or AP) is successfully started.
  while (!initWifi()) {
    vTaskDelay(pdMS_TO_TICKS(2000));
  }

  wifiStarted = true;

  for (;;) {
    tickWifi();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
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
  Serial.begin(BAUD);

  // Register callbacks used by the WiFi/web UI code.
  registerCallbacks(&g_wifi_callbacks);

  // WiFi task creation on core 0
  // separate from scale task so that network activity does not delay HX711 readings and workflows
  xTaskCreatePinnedToCore(wifiTask, "WiFiTask", 4096, NULL, 1, NULL, 0);

  // Scale task creation on core 1
  // priority 5 to ensure timely processing of HX711 readings and workflows.
  xTaskCreatePinnedToCore(scaleTask, "ScaleTask", 4096, NULL, 5, NULL, 1);
}

/**
 * @brief Main application loop function, runs indefinitely after setup() completes.
 *
 * @details The main loop is unused in this application since we're running dedicated FreeRTOS tasks for WiFi and scale logic.
 * This function simply yields to reduce CPU usage.
 *
 * @throws {none} This function does not throw exceptions.
 */
void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}
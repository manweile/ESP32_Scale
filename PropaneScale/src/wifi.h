// Minimal WiFi module stubs for PropaneScale
/**
 * @file wifi.h
 * @author Gerald Manweiler
 * 
 * @brief Header file for the WiFi module of the PropaneScale project.
 * 
 * @version 0.1
 * @date 2026-05-27
 * @copyright Copyright (c) 2026 Gerald Manweiler
 */

#pragma once

// Forward Declarations
typedef struct WifiCallbacks WifiCallbacks;                 /**< Forward declaration of WifiCallbacks struct for use in function declarations */

// Declarations for WiFi module functions

/**
 * @brief Initializes the WiFi module and starts the HTTP server.
 *
 * @details Configures the ESP32 as an access point, sets up HTTP routes, and begins listening for client requests.
 *
 * @throws {none} This function does not throw exceptions.
 */
void initWifi();

/**
 * @brief Handles requests to the calibrate URL ("/api/calibrate").
 *
 * @details Expects a POST request with a "weight" parameter indicating the known weight for calibration. Enqueues a calibration operation using the provided weight by calling the appropriate callback function registered in the WifiCallbacks struct.
 * Responds with a simple "ok" message on success, or an error message if the required parameter is missing.
 * This allows the web interface to trigger a calibration without blocking the main loop or interfering with other workflows.
 * 
 * @throws {none} This function does not throw exceptions. It handles errors by responding with appropriate HTTP status codes and messages.
 */
void handleCalibrate();

/**
 * @brief Acknowledges the completion of a liquid level read workflow (POST /api/level/ack).
 *
 * @details Clears the server-side stored prompt and report so that the browser does not see stale values.
 * Responds with a JSON object indicating the acknowledgment status.
 *
 * @throws {none} This function does not throw exceptions.
 */
void handleLevelAck();

/**
 * @brief Cancels any in-progress liquid level read workflow (POST /api/level/cancel).
 * 
 * @details Stops any ongoing liquid level read operation and resets the workflow state to IDLE. Responds with a JSON object indicating the cancellation status.
 *
 * @throws {none} This function does not throw exceptions.
 */
void handleLevelCancel();

/**
 * @brief Starts a liquid level read workflow (POST /api/level).
 * 
 * @details Triggers the liquid level read workflow by calling the liquidLevel() function, which will guard against concurrent runs. Responds immediately with a simple "ok" message, while the workflow continues asynchronously.
 * This allows the web interface to trigger a level read without blocking the main loop or interfering with other workflows.
 * 
 * @throws {none} This function does not throw exceptions.
 */
void handleLevelStart();

/**
 * @brief Returns the current liquid level workflow status and last report (GET /api/level/status).
 *
 * @details Responds with a JSON object containing the current state of the liquid level workflow, whether an average calculation is pending, the last report, and the last prompt message.
 *
 * @throws {none} This function does not throw exceptions.
 */
void handleLevelStatus();

/**
 * @brief Handles requests to the root URL ("/").
 * 
 * @details Responds with the HTML content defined in ROOT_PAGE, which serves as the main web interface for the PropaneScale project.
 * This page displays telemetry data and provides buttons to trigger tare and calibration actions via the WiFi API.
 * 
 * @throws {none} This function does not throw exceptions.
 */
void handleRoot();

/**
 * @brief Handles requests to the save URL ("/api/save").
 *
 * @details Enqueues a save operation to persist the current calibration factor.
 *
 * @throws {none} This function does not throw exceptions.
 */
void handleSave();

/**
 * @brief Handles requests to the tare URL ("/api/tare").
 *
 * @details Enqueues a tare operation to zero the scale. This operation is non-blocking and
 * will be processed by the core workflow.
 *
 * @throws {none} This function does not throw exceptions.
 */
void handleTare();

/**
 * @brief Handles requests to the telemetry URL ("/api/telemetry").
 *
 * @details Responds with a JSON payload containing the current telemetry data from the scale
 * and other relevant information. Uses the registered `WifiCallbacks` to obtain
 * telemetry.
 *
 * @throws {none} This function does not throw exceptions.
 */
void handleTelemetry();

/**
 * @brief Registers callback functions for WiFi events.
 * 
 * @details Registers callback functions for WiFi events.
 * The provided callbacks allow the WiFi module to interact with core workflows by providing telemetry data and enqueuing workflow actions in response to HTTP requests.
 *
 * @param cb Pointer to a structure containing the callback functions.
 * 
 * @throws {none} This function does not throw exceptions.
 */
void registerCallbacks(const WifiCallbacks* cb);

/**
 * @brief Handles incoming HTTP requests and advances the WiFi module state.
 * 
 * @details Should be called regularly from the main loop to ensure responsive handling of HTTP requests. This function processes any pending client connections and routes requests to the appropriate handlers based on the registered routes.
 * 
 * @throws {none} This function does not throw exceptions.
 */
void tickWifi();
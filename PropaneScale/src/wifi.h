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

// Forward Declarations for WiFi module
class AsyncWebServerRequest;                                /**< AsyncWebServerRequest for use in handler function declarations */
typedef struct WifiCallbacks WifiCallbacks;                 /**< WifiCallbacks struct for use in function declarations */

// Declarations for WiFi module functions

/**
 * @brief Return application startup status (GET /api/app/status).
 *
 * @details Returns a JSON object so the web UI can display initialization diagnostics and status messages produced during app startup.
 * The function handles errors by responding with appropriate HTTP status codes and messages.
 *
 * @param request [AsyncWebServerRequest*] Pointer to the AsyncWebServerRequest object representing the incoming HTTP request.
 *
 * @throws {none} This function does not throw exceptions.
 */
void handleAppStatus(AsyncWebServerRequest* request);

/**
 * @brief Acknowledges the completion of a liquid level read workflow (POST /api/level/ack).
 *
 * @details Clears the server-side stored prompt and report so that the browser does not see stale values.
 * Responds with a JSON object indicating the acknowledgment status.
 *
 * @param request [AsyncWebServerRequest*] Pointer to the AsyncWebServerRequest object representing the incoming HTTP request.
 *
 * @throws {none} This function does not throw exceptions.
 */
void handleLevelAck(AsyncWebServerRequest* request);

/**
 * @brief Cancels any in-progress liquid level read workflow (POST /api/level/cancel).
 *
 * @details Stops any ongoing liquid level read operation and resets the workflow state to IDLE. Responds with a JSON object indicating the cancellation status.
 *
 * @param request [AsyncWebServerRequest*] Pointer to the AsyncWebServerRequest object representing the incoming HTTP request.
 *
 * @throws {none} This function does not throw exceptions.
 */
void handleLevelCancel(AsyncWebServerRequest* request);

/**
 * @brief Starts a liquid level read workflow (POST /api/level).
 *
 * @details Triggers the liquid level read workflow by calling the liquidLevel() function, which will guard against concurrent runs. Responds immediately with a simple "ok" message, while the workflow continues asynchronously.
 * This allows the web interface to trigger a level read without blocking the main loop or interfering with other workflows.
 *
 * @param request [AsyncWebServerRequest*] Pointer to the AsyncWebServerRequest object representing the incoming HTTP request.
 *
 * @throws {none} This function does not throw exceptions.
 */
void handleLevelStart(AsyncWebServerRequest* request);

/**
 * @brief Returns the current liquid level workflow status and last report (GET /api/level/status).
 *
 * @details Responds with a JSON object containing the current state of the liquid level workflow, whether an average calculation is pending, the last report, and the last prompt message.
 *
 * @param request [AsyncWebServerRequest*] Pointer to the AsyncWebServerRequest object representing the incoming HTTP request.
 *
 * @throws {none} This function does not throw exceptions.
 */
void handleLevelStatus(AsyncWebServerRequest* request);

/**
 * @brief Handles requests to the root URL ("/").
 *
 * @details Responds with the HTML content defined in ROOT_PAGE, which serves as the main web interface for the PropaneScale project.
 * This page displays telemetry data and provides buttons to trigger tare and calibration actions via the WiFi API.
 *
 * @param request [AsyncWebServerRequest*] Pointer to the AsyncWebServerRequest object representing the incoming HTTP request.
 *
 * @throws {none} This function does not throw exceptions.
 */
void handleRoot(AsyncWebServerRequest* request);

/**
 * @brief Acknowledge the startup tare report (POST /api/startup/ack).
 *
 * @details Clears any stored startup prompt and report on the server so the browser UI
 * does not display stale results on subsequent polls. Responds with a JSON success object.
 *
 * @param request [AsyncWebServerRequest*] Pointer to the AsyncWebServerRequest object representing the incoming HTTP request.
 *
 * @throws {none} This function does not throw exceptions.
 */
void handleStartupAck(AsyncWebServerRequest* request);

/**
 * @brief Return startup tare workflow status, prompt, and report (GET /api/startup/status).
 *
 * @details Returns a small JSON object describing the current startup tare state and any
 * last prompt or report written by the workflow. Intended for polling by the browser UI
 * during headless startup interactions.
 *
 * @param request [AsyncWebServerRequest*] Pointer to the AsyncWebServerRequest object representing the incoming HTTP request.
 *
 * @throws {none} This function does not throw exceptions.
 */
void handleStartupStatus(AsyncWebServerRequest* request);

/**
 * @brief Handles requests to the telemetry URL ("/api/telemetry").
 *
 * @details Responds with a JSON payload containing the current telemetry data from the scale
 * and other relevant information. Uses the registered `WifiCallbacks` to obtain
 * telemetry.
 *
 * @param request [AsyncWebServerRequest*] Pointer to the AsyncWebServerRequest object representing the incoming HTTP request.
 *
 * @throws {none} This function does not throw exceptions.
 */
void handleTelemetry(AsyncWebServerRequest* request);

/**
 * @brief Initialize WiFi and start HTTP server.
 *
 * @details Attempts to connect to a WiFi network in station mode first. If that fails, it falls back to access point mode. Route registration occurs during initialization but the function is non-blocking: it kicks off the STA connect attempt and returns immediately. The HTTP server will be started later in `tickWifi()` once the network (STA or AP) is available. Any initialization errors are recorded in `LastStartupReport`.
 *
 * @return true if WiFi init was started successfully (non-blocking); false otherwise.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool initWifi();

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
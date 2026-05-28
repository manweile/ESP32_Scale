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
 * Enqueues a calibration operation with the provided known weight (HTTP param
 * `weight`).
 */
void handleCalibrate();

/**
 * @brief Handles requests to the root URL ("/").
 * @see wifi.cpp
 */
void handleRoot();

/**
 * @brief Handles requests to the save URL ("/api/save").
 *
 * Enqueues a save operation to persist the current calibration factor.
 */
void handleSave();

/**
 * @brief Handles requests to the tare URL ("/api/tare").
 *
 * Enqueues a tare operation to zero the scale. This operation is non-blocking and
 * will be processed by the core workflow.
 */
void handleTare();

/**
 * @brief Handles requests to the telemetry URL ("/api/telemetry").
 *
 * Responds with a JSON payload containing the current telemetry data from the scale
 * and other relevant information. Uses the registered `WifiCallbacks` to obtain
 * telemetry.
 */
void handleTelemetry();

/**
 * @details Registers callback functions for WiFi events.
 * 
 * @brief Registers callback functions for WiFi events.
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
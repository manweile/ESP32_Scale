/**
 * @file web_ble.h
 * @author Gerald Manweiler
 * 
 * @brief Provides BLE interface declarations for the PropaneScale application.
 * 
 * @details Declares functions for initializing the BLE subsystem, processing BLE events, and handling tare requests.
 * Designed to be implementation-agnostic, with a full NimBLE-based implementation when available and safe stubs when not.
 * 
 * @note This module is optional and will degrade gracefully if NimBLE is not available. 
 * The main application should call webBleInit() during setup and webBleTick() in the main loop, 
 * and can use webBleRequestTare() and webBleTakeTareRequest() to interact with the BLE subsystem for tare functionality.
 * 
 * @version 0.1
 * @date 2024-06-01
 * @copyright Copyright (c) 2024 Gerald Manweiler
 */

#pragma once

// Definitions for BLE subsystem

/**
 * @brief Initializes the BLE subsystem.
 * 
 * @details This function sets up the BLE subsystem, either using a minimal stub implementation
 * or a full NimBLE-based implementation if available. It should be called during the setup phase
 * of the application.
 * 
 * @throws {none} This function does not throw exceptions.
 */
void webBleInit();

/**
 * @brief Requests a tare operation via the BLE subsystem.
 * 
 * @details This function can be called from an ISR context to request a tare operation.
 * The request will be processed by the BLE subsystem during the next tick.
 * 
 * @throws {none} This function does not throw exceptions.
 */
void webBleRequestTare();

/**
 * @brief Checks for a pending tare request.
 * 
 * @details This function returns true if a tare request is pending. 
 * The return value is consumable, meaning it will return true only once per request.
 * 
 * @returns {bool} True if a tare request is pending, false otherwise.
 * 
 * @throws {none} This function does not throw exceptions.
 */
bool webBleTakeTareRequest();

/**
 * @brief Process BLE events and handle notifications.
 * 
 * @details Call every loop when BLE is enabled.
 * In the full NimBLE implementation, it will handle connection events, notifications, and characteristic updates. 
 * In the stub implementation, it will be a no-op.
 * It is safe to call from the main loop context.
 * 
 * @throws {none} This function does not throw exceptions.
 */
void webBleTick();
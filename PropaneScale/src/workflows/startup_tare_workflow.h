/**
 * @file startup_tare_workflow.h
 * @author Gerald Manweiler
 *
 * @brief Declarations for the startup tare workflow.
 *
 * @details Declares the startup tare workflow function and its associated tick function for advancing the workflow state machine.
 *
 * @version 0.1
 * @date 2026-05-09
 *
 * @copyright Copyright (c) 2026 Gerald Manweiler
 */

#pragma once

// External Global State Variables
extern String LastStartupPrompt;                            /**< Short prompt shown to web UI during startup tare */
extern String LastStartupReport;                            /**< Final human-readable result or diagnostic for web UI */

// Declarations for startup tare workflow functions

/**
 * @brief Begins the non-blocking startup tare workflow.
 *
 * @details Starts a non-blocking probe operation to check HX711 responsiveness.
 * The workflow is then advanced each loop() iteration by tickTare().
 *
 * @throws {none} This function does not throw exceptions.
 */
void beginStartupTare();

/**
 * @brief Advances the non-blocking startup tare workflow one iteration.
 *
 * @details Called each loop iteration. Handles the WAIT_STABLE, TARE, and SKIP states.
 * Returns immediately when IDLE.
 *
 * @throws {none} This function does not throw exceptions.
 */
void tickTare();

/**
 * @brief Forces the startup tare workflow to begin immediately.
 *
 * @details This function is intended to be called from the web UI to start the startup tare workflow
 * regardless of the current state.
 *
 * @throws {none} This function does not throw exceptions.
 */
void webForceStartupTare();

/**
 * @brief Skips the startup tare workflow.
 *
 * @details This function is intended to be called from the web UI to skip the startup tare workflow
 * and proceed to the next step.
 *
 * @throws {none} This function does not throw exceptions.
 */
void webSkipStartupTare();

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

// Declarations for startup tare workflow functions

/**
 * @brief Begins the non-blocking startup tare workflow.
 *
 * @details Validates scale readiness, establishes a baseline reading, prints startup
 * prompts, and enters WAIT_STABLE state. The workflow is then advanced each loop()
 * iteration by tickTare().
 *
 * @throws {none} This function does not throw exceptions.
 */
void beginStartupTare();

/**
 * @brief Handles serial input for the startup tare workflow.
 *
 * @details Processes incoming characters while in the startup tare workflow.
 * Cancellation character is only acceptable input.
 * Newlines are ignored to prevent interference with command processing.
 *
 * @param incoming {char} The incoming character from the serial interface.
 * @return {bool} True if the input was handled by the startup tare workflow.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool handleStartupTareInput(char incoming);

/**
 * @brief Advances the non-blocking startup tare workflow one iteration.
 *
 * @details Called each loop iteration. Handles the WAIT_STABLE, TARE, and SKIP states.
 * Returns immediately when IDLE.
 *
 * @throws {none} This function does not throw exceptions.
 */
void tickTare();



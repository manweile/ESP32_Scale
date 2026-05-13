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

/**
 * @brief Begins the non-blocking startup tare workflow.
 *
 * @details Validates scale readiness, establishes a baseline reading, prints startup
 * prompts, and enters WAIT_STABLE state. The workflow is then advanced each loop()
 * iteration by tickTare().
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void beginTare();

/**
 * @brief Prints a summary of the startup configuration and EEPROM values.
 *
 * @details Displays the application title, loaded calibration factor, known weight,
 * maximum propane weight, and tank tare from EEPROM. Also prints startup tare prompts.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */

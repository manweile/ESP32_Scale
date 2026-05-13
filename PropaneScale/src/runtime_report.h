
/**
 * @file runtime_report.h
 * @author Gerald Manweiler
 * 
 * @brief Header file for runtime reporting functions in the ESP32-based propane level scale application.
 * 
 * @details Declares functions for reporting current EEPROM values, resetting EEPROM to defaults, and displaying the help menu.
 *  
 * @version 0.1
 * @date 2026-05-12
 * @copyright Copyright (c) 2026 Gerald Manweiler
 */

#pragma once

/**
 * @brief Prints current runtime values and workflow states.
 *
 * @details Displays the active in-memory values used by the program at runtime,
 * including calibration factor, known calibration weight, max propane weight,
 * and tank tare. Also prints the current state of each workflow context.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void currentRuntimeValues();

/**
 * @brief Displays all saved EEPROM values with validity status.
 *
 * @details Reads each persisted record (calibration factor, known calibration weight,
 * maximum legal propane weight, and tank tare), verifies its magic marker and bounds,
 * and prints the stored value or an invalid notice.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void eepromValues();

/**
 * @brief Prints the help menu with all available commands.
 *
 * @details Lists all the runtime commands that the user can send over serial to interact with the scale.
 * 
 * @return {void} No value is returned.
 * 
 * @throws {none} This function does not throw exceptions.
 */
void helpMenu();

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
void printStartupSummary();
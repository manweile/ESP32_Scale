/** 
 * @file commands.h
 * @author Gerald Manweiler
 * 
 * @brief Command processing for the propane scale project.
 * 
 * @details Declares functions for processing serial commands, managing startup tare workflow, and resetting EEPROM values to defaults.
 * 
 * @version 0.1
 * @date 2024-06-01
 * 
 * @copyright Copyright (c) 2024 Gerald Manweiler
 */

#pragma once

/**
 * @brief Resets all EEPROM-persisted values to their hardcoded defaults.
 *
 * @details Overwrites calibration factor, known calibration weight, maximum legal propane
 * weight, and tank tare with the compile-time defaults defined in config.h, then persists
 * each value to EEPROM and updates the corresponding runtime globals. Reports the result
 * of each save over serial.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void defaultEeprom();
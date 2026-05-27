/**
 * @file app_startup.h
 * @author Gerald Manweiler
 *
 * @brief Application startup initialization declarations for the propane scale project.
 *
 * @details Declares helper functions used to initialize the application before the main
 * startup tare workflow begins.
 *
 * @version 0.1
 * @date 2026-05-14
 *
 * @copyright Copyright (c) 2026 Gerald Manweiler
 */

#pragma once

// Declarations for application startup initialization functions

/**
 * @brief Initializes the application.
 *
 * @details Performs necessary setting up the HX711 scale interface and applies calibration from EEPROM.
 *
 * @throws {none} This function does not throw exceptions.
 */
void initializeApp();

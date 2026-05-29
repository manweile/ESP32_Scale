/**
 * @file input_tank_tare.h
 * @author Gerald Manweiler
 *
 * @brief Handles user input for the tank tare update workflow.
 *
 * @details Processes one serial character per loop() iteration,
 * managing the stepwise collection of a new tank tare value and user confirmation to save or cancel.
 *
 * @version 0.1
 * @date 2026-05-07
 * @copyright Copyright (c) 2026 Gerald Manweiler
 */

#pragma once

// Declarations for tank tare input functions

/**
 * @brief Handles user input for the tank tare update workflow.
 *
 * @details Processes one serial character per loop() iteration,
 * managing the stepwise collection of a new tank tare value and user confirmation to save or cancel.
 *
 * @param incoming {char} The incoming character from the serial input.
 *
 * @throws {none} This function does not throw exceptions.
 */
void handleTankTareInput(char incoming);

/**
 * @brief Initiates the tank tare update workflow.
 *
 * @details Starts a non-blocking serial workflow that prompts the user to enter a new tank tare value,
 * validates the input, and saves it to EEPROM if confirmed.
 *
 * @throws {none} This function does not throw exceptions.
 */
void tankTareUpdate();
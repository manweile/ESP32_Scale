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

/**
 * @brief Handles user input for the tank tare update workflow.
 *
 * @details Processes one serial character per loop() iteration, 
 * managing the stepwise collection of a new tank tare value and user confirmation to save or cancel. 
 * 
 * @param incoming The incoming character from the serial input.
 *
 * @return {void} No value is returned.
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
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void tankTareUpdate();
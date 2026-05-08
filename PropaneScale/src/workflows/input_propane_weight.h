/**
 * @file input_propane_weight.h
 * @author Gerald Manweiler
 * 
 * @brief Header file for handling user input workflow to update the maximum legal propane weight.
 * 
 * @details Declares functions to initiate the propane weight update workflow and handle user input for that workflow.
 * 
 * @version 0.1
 * @date 2026-05-08
 * 
 * @copyright Copyright (c) 2026 Gerald Manweiler
 */

#pragma once

/**
 * @brief Handles user input for the max propane weight update workflow.
 *
 * @details Processes one serial character per loop() iteration, 
 * managing the stepwise collection of a new max propane weight value and user confirmation to save or cancel.
 *
 * @param incoming The incoming character from the serial input.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void handlePropaneWeightInput(char incoming);

/**
 * @brief Initiates the max propane weight update workflow.
 *
 * @details Starts a non-blocking serial workflow that prompts the user to enter a new maximum legal propane weight,
 * validates the input, and saves it to EEPROM if confirmed.
 * 
 * @return {void} No value is returned.
 * 
 * @throws {none} This function does not throw exceptions.
 */
void propaneWeightUpdate();
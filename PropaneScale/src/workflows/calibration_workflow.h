/**   
 * @file calibration_workflow.h
 * @author Gerald Manweiler
 * 
 * @brief Declarations for the manual calibration workflow.
 * 
 * @details Declares the manual calibration workflow function and its associated tick function for advancing the workflow state machine.
 * 
 * @version 0.1
 * @date 2024-06-01 
 * @copyright Copyright (c) 2024 Gerald Manweiler
 */

#pragma once

/**
 * @brief Starts the automatic calibration workflow.
 *
 * @details Validates that no calibration is already running, checks scale readiness,
 * prints the initial prompts, and sets the calibration context to begin waiting for
 * an empty scale. The workflow continues asynchronously through tickCalibration().
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void automaticCalibration();

/**
 * @brief Routes a single serial character to the active calibration state machine.
 *
 * @details Called by loop() when a character arrives while calibration is in progress.
 * Handles cancel ('q'/'Q') from WAIT_EMPTY and WAIT_LOAD, and factor adjustment
 * ('+', '-', 's', 'q') from the ADJUSTING state.
 *
 * @param {char} serialchar The received serial character.
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void handleCalibrationInput(char serialchar);

/**
 * @brief Starts the manual calibration workflow.
 *
 * @details Validates that no calibration is already running, checks scale readiness,
 * initializes the adjustment step from the current calibration factor magnitude,
 * prints the initial prompts, and sets the calibration context to begin waiting for
 * an empty scale. The workflow continues asynchronously through tickCalibration().
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void manualCalibration();

/**
 * @brief Starts the runtime re-zero workflow.
 *
 * @details Validates that no calibration is already running, checks scale readiness,
 * prints the initial prompts, and sets the calibration context to begin waiting for
 * an empty scale in REZERO mode. The workflow continues asynchronously through tickCalibration().
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void reZero();

/**
 * @brief Processes calibration workflow steps and scale interactions on each loop() iteration.
 *
 * @details Called every loop() iteration when a calibration workflow is active. 
 * Manages the state machine for automatic and manual calibration workflows:
 * - IDLE: no active workflow, waiting for user input to start one 
 * - WAIT_EMPTY: checks for stable empty scale condition with a timeout for auto-confirmation
 * - WAIT_LOAD: checks for load placement with a noise-derived threshold and timeout, transitions to either:
 *    - AUTO: automatic calibration measurement
 *    - MANUAL: manual adjustment based on the active mode
 * - ADJUSTING: handles manual calibration adjustments
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void tickCalibration();

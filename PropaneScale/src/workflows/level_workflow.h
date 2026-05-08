/**
 * @file level_workflow.h
 * @author Gerald Manweiler
 * 
 * @brief Declarations for the liquid level read workflow.
 * 
 * @details Declares the liquid level read workflow function and its associated tick function for advancing the workflow state machine.
 *  
 * @version 0.1
 * @date 2026-05-08
 * 
 * @copyright Copyright (c) 2026 Gerald Manweiler
 */

#pragma once

/**
 * @brief Starts the liquid level read workflow.
 *
 * @details Validates scale readiness, measures the noise baseline to compute a
 * load detection threshold, prints prompts, and sets the level context to begin
 * polling for tank placement. The workflow continues asynchronously through
 * tickLevelRead().
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void liquidLevel();

/**
 * @brief Advances the level read workflow on each loop() iteration.
 *
 * @details Called every loop() iteration when the level read workflow is active.
 * - WAIT_LOAD: polls the scale each tick until the reading exceeds the load
 *   detection threshold or the placement timeout expires.
 * - SETTLING: waits a short delay after load detection to avoid measuring while
 *   placement motion is still in progress.
 * - READING: takes a final averaged measurement, computes and prints propane
 *   weight and fill percentage, then resets to IDLE.
 *
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void tickLevelRead();
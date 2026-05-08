/** 
 * @file input_context.h
 * @author Gerald Manweiler
 * 
 * @brief Input context definitions for non-blocking user input workflows in the propane scale project.
 * 
 * @details Defines enums and structs for managing the state of non-blocking serial input workflows initiated by the user.
 * 
 * @version 0.1
 * @date 2026-05-07
 * @copyright Copyright (c) 2026 Gerald Manweiler
 */

#pragma once

/**
 * @section Standard library headers
 */
#include <cstdint>

/**
 * @enum InputMode
 *
 * @brief Active user input workflow mode.
 * 
 * @details Used to manage different non-blocking serial input workflows initiated by the user
 */
enum class InputMode : uint8_t {
  NONE = 0,                                                 /**< No active user input workflow */
  TANK_TARE = 1,                                            /**< Tank tare update workflow */
  PROPANE_WEIGHT = 2,                                       /**< Max propane weight update workflow */
  KNOWN_WEIGHT = 3                                          /**< Known calibration weight update workflow */
};

/**
 * @enum InputState
 *
 * @brief Internal parse state for non-blocking input workflows.
 * 
 * @details Used to manage the stepwise process of collecting and confirming user input during workflows that require multiple interactions
 */
enum class InputState : uint8_t {
  IDLE = 0,                                                 /**< Not collecting any input */
  ENTER_VALUE = 1,                                          /**< Collecting numeric value text */
  WAIT_SAVE_CONFIRM = 2                                     /**< Waiting for 's' save or 'q' cancel */
};

/**
 * @struct InputContext
 *
 * @brief Persistent serial parse context for non-blocking user input workflows.
 * 
 * @details Contains variables for managing the state of user input collection and confirmation across multiple loop() iterations without blocking
 */
struct InputContext {
  char      buffer[24]   = {0};                             /**< Text buffer for user-entered numeric value */
  int       index        = 0;                               /**< Current write index into buffer */
  InputMode mode         = InputMode::NONE;                 /**< Active workflow mode */
  float     parsedValue  = 0.0f;                            /**< Last parsed numeric value pending save */
  InputState state       = InputState::IDLE;                /**< Current parse state for active mode */
};

extern InputContext inputCtx;                               /**< Non-blocking input context for serial workflows */
extern void resetInputContext();                            /**< Resets input context to idle state */
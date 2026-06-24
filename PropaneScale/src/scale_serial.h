/**
 * @file scale_serial.h
 * @author Gerald Manweiler
 *
 * @brief Serial/console output helpers moved out of `scale_io` for web-only UI.
 *
 * @details This header declares a small shim that replaces the legacy serial
 * queue used by the old serial-monitor UI. The implementation records the
 * last console-style output in `LastConsoleOutput` so the web UI can surface
 * the same messages without depending on a serial monitor.
 *
 * @version 0.1
 * @date 2024-06-01
 * @copyright Copyright (c) 2024 Gerald Manweiler
 */

#pragma once

// Standard library headers
#include <Arduino.h>

// External Global State Variables
extern String LastConsoleOutput;                            /**< Last console-style output suitable for embedding in web UI JSON responses. */

// Public Declarations for serial output helper functions

/**
 * @brief Flush any available serial input from the UART.
 *
 * @details Reads and discards any available serial input to ensure that subsequent serial reads start with fresh input from the user.
 * This is useful to call after a workflow cancellation or completion, to prevent leftover input from being
 * misinterpreted as a command or parameter for the next workflow that the user initiates.
 * This function returns immediately after flushing the available input, so it does not block waiting for new input.
 *
 * @throws {none} This function does not throw exceptions.
 */
void flushSerialInput();

/**
 * @brief Queue a console-style message intended for display via the web UI.
 *
 * @details Records the message in `LastConsoleOutput` for web UI consumption. Trims trailing newlines for safe JSON embedding. Does not write to UART since this is intended for web-only mode.
 * The web UI can then read `LastConsoleOutput` to display the most recent console-style message without needing a serial monitor.
 * This function returns immediately after recording the message, so it does not block waiting for UART availability.
 *
 * @param message Null-terminated message string. May contain newlines.
 * @return true if the message was accepted.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool queueSerialOutput(const char* message);


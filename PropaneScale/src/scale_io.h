/**
 * @file scale_io.h
 * @author Gerald Manweiler
 * 
 * @brief Input/output functions for user workflows and HX711 interactions.
 * 
 * @details Declares helper functions for user initiated workflows and HX711 interactions.
 *  
 * @version 0.1
 * @date 2026-05-07
 * 
 * @copyright Copyright (c) 2026 Gerald Manweiler
 */

#pragma once

/**
 * @brief Computes the load-detection threshold from measured noise.
 *
 * @details Reads the current unloaded noise from the scale, multiplies it by 20
 * as a signal-to-noise margin, then clamps to minimumThresholdLbs so a very
 * quiet scale still responds to a real load.
 *
 * @param {float} minimumThresholdLbs Floor value for the returned threshold in pounds.
 * @return {float} Computed threshold in pounds: max(noise * 20, minimumThresholdLbs).
 *
 * @throws {none} This function does not throw exceptions.
 */
float computeLoadDetectThreshold(float minimumThresholdLbs);

/**
 * @brief Ensures the HX711 is ready before attempting reads or tare.
 *
 * @details Checks the amplifier readiness and prints a field-diagnostic message when it is not ready so workflows can exit early instead of blocking.
 *
 * @param {const char*} operation Short workflow label used in the error message.
 * @return {bool} True when HX711 is ready; false otherwise.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool ensureScaleReady(const char* operation);

/**
 * @brief Flushes any buffered serial input.
 *
 * @details Reads and discards any available serial input to ensure that subsequent serial reads start with fresh input from the user.
 * 
 * @return {void} No value is returned.
 * 
 * @throws {none} This function does not throw exceptions.
 */
void flushSerialInput();

// @todo readAveragedUnits() uses wait_ready_timeout() per iteration so it no longer spins
// indefinitely, but it still blocks loop() for up to HX711_READY_TIMEOUT_MS per reading
// (e.g. up to ~120ms per call for single-reading polling paths, more for multi-reading
// measurement calls). Acceptable for serial-only use. When adding a web interface, refactor
// callers to drive one reading per loop() tick via is_ready() and accumulate across ticks.

/**
 * @brief Reads the average weight from the scale over multiple readings.
 * 
 * @details Takes multiple readings from the scale, averages them, and returns the result in pounds.
 * Useful for smoothing out noise in the scale readings and getting a more stable weight measurement.
 * 
 * @param {int} readings Number of readings to average.
 * @param {int} samplesPerReading Number of samples per reading.
 * @return {float} avgWeight The average weight in pounds. 
 * 
 * @throws {none} This function does not throw exceptions.
 */
float readAveragedUnits(int readings, int samplesPerReading);

/**
 * @brief Prints a standardized HX711 not-ready diagnostic.
 *
 * @details Used across workflows to keep timeout/not-ready messaging consistent.
 *
 * @param {const char*} operation Short workflow label used in the error message.
 * @return {void} No value is returned.
 *
 * @throws {none} This function does not throw exceptions.
 */
void printScaleNotReadyDiagnostic(const char* operation);
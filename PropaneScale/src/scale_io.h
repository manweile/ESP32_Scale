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

// Declarations for input/output functions for user workflows and HX711 interactions

/**
 * @brief Cancels an in-progress level load-detection threshold computation.
 * 
 * @details If a threshold computation is active, resets the internal state so that the pending operation is cancelled.
 * 
 * @throws {none} This function does not throw exceptions.
 */
void cancelLevelLoadDetect();

/**
 * @brief Computes the load-detection threshold from measured noise.
 *
 * @details Reads the current unloaded noise from the scale, multiplies it by 20
 * as a signal-to-noise margin, then clamps to minimumThresholdLbs so a very
 * quiet scale still responds to a real load.
 *
 * @param minimumThresholdLbs {float} Floor value for the returned threshold in pounds.
 * @return {float} Computed threshold in pounds: max(noise * 20, minimumThresholdLbs).
 *
 * @throws {none} This function does not throw exceptions.
 */
float computeLoadDetectThreshold(float minimumThresholdLbs);

/**
 * @brief Drains queued serial output without blocking.
 *
 * @details Writes at most the currently available UART buffer space from the
 * internal output queue and returns immediately.
 *
 * @throws {none} This function does not throw exceptions.
 */
void drainQueuedSerialOutput();

/**
 * @brief Ensures the HX711 is ready before attempting reads or tare.
 *
 * @details Checks the amplifier readiness and prints a field-diagnostic message when it is not ready so workflows can exit early instead of blocking.
 *
 * @param operation {const char*} Short workflow label used in the error message.
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
 * @throws {none} This function does not throw exceptions.
 */
void flushSerialInput();

/**
 * @brief Non-blocking averaged units reader driven by loop() ticks.
 *
 * @details Starts or advances a non-blocking averaging operation that polls
 * `scale.is_ready()` once per invocation. Call this from `loop()` (or a
 * workflow tick) repeatedly until it returns `true`, at which point
 * `outAvg` will contain the averaged reading in pounds. While the operation
 * is in-progress, the function returns `false` and `outAvg` is unspecified.
 * This helper is intended for use by workflows that must remain responsive
 * (for example automatic calibration) and must be polled regularly.
 *
 * @param readings {int} Number of readings to average (each reading may itself
 *   average multiple samples via `scale.get_units(samplesPerReading)`).
 * @param samplesPerReading {int} Number of samples passed to `get_units()` per
 *   reading. Typical values: `POLL_SAMPLES` or `LIVE_SAMPLES`.
 * @param outAvg {float&} Output parameter set to the computed average in pounds
 *   when the function returns `true`.
 * @return {bool} `true` when the averaged reading is complete and `outAvg` is valid;
 *   `false` when the operation is still in progress and must be called again.
 *
 * @note This implementation maintains a single internal active operation and is
 * not reentrant. Use only from the automatic calibration workflow as currently
 * implemented in this project.
 */
bool nonBlockingAvgUnits(int readings, int samplesPerReading, float &outAvg);

/**
 * @brief Polls the level load-detection threshold computation for completion.
 * 
 * @details If a threshold computation is active, advances the operation by polling the HX711 for new readings and updating the internal state. 
 *  When the requested number of readings has been collected, computes the final threshold value, stores it in `outThreshold`, and returns true. 
 * If no operation is active or the operation is still in progress, returns false and `outThreshold` is unspecified.
 * 
 * @param outThreshold {float&} Output parameter set to the computed threshold in pounds when the function returns `true`.
 * @return {bool} true when the threshold computation is complete and `outThreshold` is valid; 
 * `false` otherwise when the operation is still in progress or no operation is active.
 * 
 * @throws {none} This function does not throw exceptions.
 */
bool pollLevelLoadDetect(float &outThreshold);

/**
 * @brief Prints a standardized HX711 not-ready diagnostic.
 *
 * @details Used across workflows to keep timeout/not-ready messaging consistent.
 *
 * @param operation {const char*} Short workflow label used in the error message.
 *
 * @throws {none} This function does not throw exceptions.
 */
void printScaleNotReadyDiagnostic(const char* operation);

/**
 * @brief Queues a serial message for non-blocking transmission.
 *
 * @details Appends the provided message to an internal output queue.
 * Thin wrapper around the private queueSerialOutput implementation.
 * The queue is drained incrementally from loop() using drainQueuedSerialOutput().
 *
 * @param message {const char*} Null-terminated message to append to the queue.
 * @return {bool} True when the full message was queued; false if the queue has insufficient space.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool queueSerialOutput(const char* message);

// @todo readAveragedUnits() uses wait_ready_timeout() per iteration so it no longer spins
// indefinitely, but it still blocks loop() for up to HX711_READY_TIMEOUT_MS per reading
// (e.g. up to ~100ms per call for single-reading polling paths, more for multi-reading
// measurement calls). Acceptable for serial-only use. When adding a web interface, refactor
// callers to drive one reading per loop() tick via is_ready() and accumulate across ticks.

/**
 * @brief Reads the average weight from the scale over multiple readings.
 * 
 * @details Takes multiple readings from the scale, averages them, and returns the result in pounds.
 * Useful for smoothing out noise in the scale readings and getting a more stable weight measurement.
 * 
 * @param readings {int} Number of readings to average.
 * @param samplesPerReading {int} Number of samples per reading.
 * @return {float} avgWeight The average weight in pounds. Returns NaN on error.
 * 
 * @throws {none} This function does not throw exceptions.
 */
float readAveragedUnits(int readings, int samplesPerReading);

/**
 * @brief Saves the current runtime tare offset from the HX711 to EEPROM.
 *
 * @details Reads the current offset from the HX711, casts it to a float, and saves it to EEPROM with a magic number for validation.
 * This allows the scale to persist a runtime tare offset across power cycles, which is used for the re-zero workflow.
 *
 * @throws {none} This function does not throw exceptions.
 */
void saveRuntimeTareOffset();

/**
 * @brief Starts an asynchronous level load-detection threshold computation.
 * 
 * @details Initializes the internal state to begin collecting readings from the HX711.
 * 
 * @param minimumThresholdLbs {float} Floor value for the computed threshold in pounds.
 * 
 * @throws {none} This function does not throw exceptions.
 */
void startLevelLoadDetect(float minimumThresholdLbs);
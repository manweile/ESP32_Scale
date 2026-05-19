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
// (e.g. up to ~120ms per call for single-reading polling paths, more for multi-reading
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
 * @return {float} avgWeight The average weight in pounds. 
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

// @todo sort alphabetically
// --- Non-blocking sampler implementation
/**
 * @brief Starts a non-blocking sampling batch to read and average weight readings from the scale.
 * 
 * @details Initializes the internal state of the non-blocking sampler to begin accumulating readings across multiple loop() ticks.
 *
 * @param readings {int} Number of readings to average.
 * @param samplesPerReading {int} Number of samples per reading.
 * @return {bool} True when the batch was started successfully; false otherwise.
 * 
 * @throws {none} This function does not throw exceptions.
 */
bool startSampleBatch(int readings, int samplesPerReading);

/**
 * @brief Polls the non-blocking sampler to accumulate a single reading.
 * 
 * @details Should be called frequently from loop() or tick functions.
 * Performs at most one HX711 read per call when the scale is ready.
 *
 * @throws {none} This function does not throw exceptions.
 */
void pollSample();

/**
 * @brief Checks if the non-blocking sample batch has completed.
 * 
 * @return {bool} True if the sample batch is done; false otherwise.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool isSampleDone();

/**
 * @brief Retrieves the result of the last completed non-blocking sample batch.
 * 
 * @param outAvg {float&} Reference to a float variable where the average result will be stored if available.
 * @return {bool} True when a valid result was written to `outAvg`; false when no completed result exists.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool getSampleResult(float &outAvg);

/**
 * @brief Cancels any in-progress non-blocking sample batch.
 *
 * @throws {none} This function does not throw exceptions.
 */
void cancelSampleBatch();

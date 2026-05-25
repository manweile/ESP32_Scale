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
 * @brief Non-blocking averaged units reader driven by loop() ticks.
 *
 * @details Starts or advances a non-blocking averaging operation that polls once per invocation.
 * This helper is intended for use by workflows that must remain responsive and must be polled regularly.
 * Not reentrant - maintains a single internal active operation. 
 * Use only from the automatic calibration workflow as currently implemented in this project.
 *
 * @param readings {int} Number of readings to average.
 * @param samplesPerReading {int} Number of samples passed per reading.
 * @param outAvg {float&} Output parameter set to the computed average in pounds when the function returns `true`.
 * @return {bool} `true` when the averaged reading is complete and `outAvg` is valid;
 *   `false` when the operation is still in progress and must be called again.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool averageUnits(int readings, int samplesPerReading, float &outAvg);

/**
 * @brief Cancels any in-progress threshold computation used by level or calibration workflows.
 *
 * @details Resets the shared context so pending threshold computations are cancelled.
 * Used when the user cancels a workflow or when a workflow finishes and needs to clean up any pending threshold computation.
 * 
 * @throws {none} This function does not throw exceptions.
 */
void cancelThresholdDetect();

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
 * @brief Flushes any buffered serial input.
 *
 * @details Reads and discards any available serial input to ensure that subsequent serial reads start with fresh input from the user.
 * 
 * @throws {none} This function does not throw exceptions.
 */
void flushSerialInput();

/**
 * @brief Polls the HX711 probe operation for completion.
 *
 * @details Checks if the probe operation has completed within the specified timeout and collects the required number of samples.
 *
 * @param outResponsive {bool&} Output parameter set to true if the HX711 is responsive; false otherwise.
 * @param timeoutMs {unsigned long} Maximum duration for the probe operation in milliseconds.
 * @param targetSamples {int} Number of samples to collect before completing the probe.
 * @return {bool} True when the probe operation is complete; false if it is still in progress.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool pollProbe(bool &outResponsive, unsigned long timeoutMs, int targetSamples);

/**
 * @brief Polls the calibration load-detection threshold computation for completion.
 *
 * @details Verifies if the HX711 is ready for a new reading.
 * Computes the threshold once the requested number of readings have been collected,
 * and updates the output parameter with the computed threshold value.
 *
 * @param outThreshold {float&} Output parameter set to the computed threshold or failsafe value.
 * @return {bool} True when the threshold computation is complete.
 * 
 * @throws {none} This function does not throw exceptions.
 */
bool pollThresholdDetect(float &outThreshold);

/**
 * @brief Prints a standardized HX711 not-ready diagnostic.
 *
 * @details Used across workflows to keep timeout/not-ready messaging consistent.
 *
 * @param operation {const char*} Short workflow label used in the error message.
 *
 * @throws {none} This function does not throw exceptions.
 */
void printDiagnostic(const char* operation);

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
 * @brief Starts a non-blocking probe operation to check HX711 responsiveness.
 *
 * @details Initializes the probe context and begins collecting readings from the HX711.
 *
 * @param timeoutMs {unsigned long} Maximum duration for the probe operation in milliseconds.
 * @param targetSamples {int} Number of samples to collect before completing the probe.
 *
 * @throws {none} This function does not throw exceptions.
 */
void startProbe(unsigned long timeoutMs, int targetSamples);

/**
 * @brief Starts an asynchronous threshold computation used by calibration or level workflows.
 *
 * @details Initializes the shared context to begin collecting readings from the HX711.
 *
 * @param minimumThresholdLbs {float} Floor value for the computed threshold in pounds.
 * 
 * @throws {none} This function does not throw exceptions.
 */
void startThresholdDetect(float minimumThresholdLbs);
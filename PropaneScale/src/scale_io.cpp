/**
 * @file scale_io.cpp
 * @author Gerald Manweiler
 * 
 * @brief Definition of input/output functions for user workflows and HX711 interactions.
 * 
 * @details Implements helper functions for user initiated workflows and HX711 interactions.
 * 
 * @version 0.1
 * @date 2026-05-07
 * 
 * @copyright Copyright (c) 2026 Gerald Manweiler
 */

// Standard library headers
#include <Arduino.h>                                        // Arduino core library for Serial communication and basic types
#include <math.h>                                           // Math library for fabsf() and other mathematical functions
#include <string.h>                                         // String helpers for non-blocking serial queue management
#include <type_traits>                                      // For std::void_t and type traits used in compile-time feature detection
#include <utility>                                          // For std::declval used in compile-time feature detection

// Third party library headers
#include "HX711.h"                                          // HX711 library for interfacing with the load cell amplifier to read weight data

// Local library headers
#include "config.h"                                         // Configuration constants for the ESP32-based propane level scale
#include "eeprom_store.h"                                   // EEPROM storage functions
#include "scale_io.h"                                       // Input/output functions for user workflows and HX711 interactions

/**
 * @namespace hx711_utils
 * 
 * @brief Compile-time feature detection for HX711 readiness helpers.
 * 
 * @details Uses C++17's `std::void_t` and SFINAE (Substitution Failure Is Not An Error) 
 * to detect the presence of `is_ready()` and `wait_ready_timeout()` methods in the HX711 class.
 * Used for non-blocking readiness checks without worrying about which HX711 library version is being used.
 */
namespace hx711_utils {
  template <typename T, typename = void>                    /**< Detect presence of `is_ready()` method exists */
  struct has_is_ready : std::false_type {};

  template <typename T>                                     /**< Specialization of `has_is_ready` if `is_ready()` method exists */
  struct has_is_ready<T, std::void_t<decltype(std::declval<T&>().is_ready())>> : std::true_type {};

  template <typename T, typename = void>                    /**< Detect presence of `wait_ready_timeout()` method exists */
  struct has_wait_ready_timeout : std::false_type {};

  template <typename T>                                     /**< Specialization of `has_wait_ready_timeout` if `wait_ready_timeout()` method exists */
  struct has_wait_ready_timeout<T, std::void_t<decltype(std::declval<T&>().wait_ready_timeout(0))>> : std::true_type {};

  /**
   * @brief Portable wrapper to check if the HX711 is ready without blocking.
   * 
   * @details Uses compile-time feature detection to determine which method to call for checking HX711 readiness.
   * 
   * @param s {HX711&} HX711 instance to check for readiness
   * @return true if the HX711 is ready, false otherwise
   * 
   * @throws {none} This function does not throw exceptions.
   */
  static inline bool hx711_is_ready_nonblocking(HX711 &s) {
    if constexpr (has_is_ready<HX711>::value) {
      return s.is_ready();
    } else if constexpr (has_wait_ready_timeout<HX711>::value) {
      return s.wait_ready_timeout(0);
    } else {
      return false;
    }
  }
}

// Non-blocking Sampler Internal State Struct

/**
 * @struct NonBlockingSampler
 * 
 * @brief Struct to manage state for non-blocking sampling batches.
 * 
 * @details Used to accumulate readings across multiple loop() ticks for non-blocking workflows that require averaging multiple samples without blocking. 
 * Tracks the total number of readings requested, how many have been taken, the accumulated units, and whether a batch is currently running. 
 * Also stores the last computed average result for retrieval once the batch is complete.
 */
struct NonBlockingSampler {
  int    totalReadings     = 0;                             /**< total number of readings to take for the batch (e.g. 10 for a 10-sample average) */
  int    samplesPerReading = 0;                             /**< retained for compatibility; current implementation uses 1-per-poll */
  int    readingsTaken     = 0;                             /**< number of readings taken so far in the current batch */
  double totalUnits        = 0.0;                           /**< accumulated units from all readings in the current batch */
  bool   running           = false;                         /**< indicates if a batch is currently running */
  float  lastResult        = NAN;                           /**< last computed average result for the batch; NAN indicates no result yet */
};

// External Global State Variables
extern HX711 scale;                                         /**< HX711 instance for interacting with the load cell amplifier */

// Private Static Constants and Variables
static constexpr size_t SERIAL_CAPACITY = 2048;             /**< Capacity of the internal serial output queue in bytes */
static NonBlockingSampler sampler;                          /**< Manages state for non-blocking sampling batches across workflows */
static size_t serialLength = 0;                             /**< Current length of data in the serial output queue */
static size_t serialOffset = 0;                             /**< Current offset for reading from the serial output queue */
static char serialQueue[SERIAL_CAPACITY];                   /**< Internal buffer for queued serial output */

// Private Definitions & Declarations for input/output helper functions

/**
 * @brief Probes the HX711 with multiple reads to determine if it is producing a responsive signal.
 * 
 * @details Secondary check to detect if HX711 is powered but not properly connected.
 * Intentionally private implementation detail, only used as part of the scale ready workflow.
 * 
 * @return {bool} True if the HX711 is producing a responsive signal with variability across multiple reads; false otherwise.
 * 
 * @throws {none} This function does not throw exceptions.
 */
static bool hasResponsiveHx711Signal() {
  const int probeReads = LIVE_SAMPLES;                      // HX711 is set at 10 samples per second
  bool haveSample = false;
  long minRaw = 0;
  long maxRaw = 0;

  for (int i = 0; i < probeReads; ++i) {
    // wait ready false means the HX711 is not responding at all
    if (!scale.wait_ready_timeout(HX711_READY_TIMEOUT_MS)) {
      return false;
    }

    // instantiate in this scope to ensure clean signal path and timing for each read
    long raw = scale.read();

    // if we can read at least one sample, can check for signal variability
    if (!haveSample) {
      minRaw = raw;
      maxRaw = raw;
      haveSample = true;
      continue;
    }

    // update on each iteration to track signal variability
    if (raw < minRaw) minRaw = raw;
    if (raw > maxRaw) maxRaw = raw;
  }

  // if we couldn't get any samples, we can't confirm responsiveness
  if (!haveSample) {
    return false;
  }

  // true when at least one probe read changed
  // false when all probe reads the same, indicating flat/stuck/unresponsive signal
  return maxRaw != minRaw;
}

/**
 * @brief Queues a message for serial output, handling buffer management.
 * 
 * @details Appends the provided message to an internal output queue. 
 * The queue is drained incrementally from loop() using drainQueuedSerialOutput().
 * If the message exceeds the queue capacity, it will not be queued.
 * If the message is null or empty, it is treated as successfully queued.
 * Intentionally private implementation detail, only used as part of the scale ready workflow and user prompts
 * 
 * @param message {const char*} The message to queue for serial output.
 * @param messageLength {size_t} The length of the message in bytes.
 * @return {bool} True if the message was successfully queued; false if there was insufficient space in the queue.
 * 
 * @throws {none} This function does not throw exceptions.
 */
static bool queueSerialOutput(const char* message, size_t messageLength) {
  if (message == nullptr || messageLength == 0) {
    return true;
  }

  if (messageLength > SERIAL_CAPACITY) {
    return false;
  }

  size_t queuedBytes = serialLength - serialOffset;
  // compact the buffer when there is space at the front, 
  // else we risk fragmentation when we don't have contiguous space to queue the new message
  if (serialOffset > 0 && (queuedBytes + messageLength) <= SERIAL_CAPACITY) {
    memmove(serialQueue, serialQueue + serialOffset, queuedBytes);
    serialOffset = 0;
    serialLength = queuedBytes;
  }

  // If the message still doesn't fit after compaction, we can't queue it.
  if ((serialLength + messageLength) > SERIAL_CAPACITY) {
    return false;
  }

  memcpy(serialQueue + serialLength, message, messageLength);
  serialLength += messageLength;
  return true;
}

/**
 * @section Public Definitions for input/output functions
 */

void cancelSampleBatch() {
  sampler.running = false;
  sampler.readingsTaken = 0;
  sampler.totalUnits = 0.0;
  sampler.lastResult = NAN;
}

float computeLoadDetectThreshold(float minimumThresholdLbs) {
  float noise = fabsf(readAveragedUnits(UNLOAD_CHECK_COUNT, LIVE_SAMPLES));
  float threshold = noise * 20.0f;
  return (threshold >= minimumThresholdLbs) ? threshold : minimumThresholdLbs;
}

void drainQueuedSerialOutput() {
  // if there is no queued output, nothing to do
  if (serialOffset >= serialLength) {
    serialOffset = 0;
    serialLength = 0;
    return;
  }

  // we need space in the UART buffer before we can write
  int availableBytes = Serial.availableForWrite();
  if (availableBytes <= 0) {
    return;
  }

  size_t bytesToWrite = serialLength - serialOffset;

  // if the message exceeds the available space, we can only write part of it now
  if (bytesToWrite > static_cast<size_t>(availableBytes)) {
    bytesToWrite = static_cast<size_t>(availableBytes);
  }

  // reinterpret the char buffer for Serial.write, which expects a byte buffer
  size_t writtenBytes = Serial.write(reinterpret_cast<const uint8_t*>(serialQueue + serialOffset), bytesToWrite);
  serialOffset += writtenBytes;

  if (serialOffset >= serialLength) {
    serialOffset = 0;
    serialLength = 0;
  }
}

bool ensureScaleReady(const char* operation) {
  bool ready = false;
  
  ready = scale.wait_ready_timeout(HX711_READY_TIMEOUT_MS) && hasResponsiveHx711Signal();

  if (ready) {
    return true;
  }

  printScaleNotReadyDiagnostic(operation);
  return ready;
}

void flushSerialInput() {
  while (Serial.available()) {
    Serial.read();
  }
}

bool getSampleResult(float &outAvg) {
  if (!isSampleDone()) return false;

  outAvg = sampler.lastResult;
  sampler.readingsTaken = 0;
  // NAN indicates no valid result until next batch completes
  sampler.lastResult = NAN;

  return true;
}

bool isSampleDone() {
  return (!sampler.running && sampler.readingsTaken > 0);
}

void pollSample() {
  if (!sampler.running) return;

  // If HX711 isn't ready right now, return immediately — non-blocking.
  if (!hx711_utils::hx711_is_ready_nonblocking(scale)) return;

  // Read one sample (one averaged unit) so pollSample performs minimal blocking
  float value = scale.get_units(1);
  sampler.totalUnits += static_cast<double>(value);
  sampler.readingsTaken++;

  if (sampler.readingsTaken >= sampler.totalReadings) {
    sampler.lastResult = static_cast<float>(sampler.totalUnits / sampler.readingsTaken);
    sampler.running = false;
  }
}

void printScaleNotReadyDiagnostic(const char* operation) {
  Serial.print("HX711 not ready");
  if (operation != nullptr && operation[0] != '\0') {
    Serial.print(" during ");
    Serial.print(operation);
  }
  Serial.println('.');
  Serial.println("Check HX711 wiring, power, and data pins (DOUT/CLK).");
  Serial.println();
}

bool queueSerialOutput(const char* message) {
  // want to avoid calling strlen() on a null pointer, 
  // so treat null as empty message that is successfully queued
  if (message == nullptr) {
    return true;
  }

  return queueSerialOutput(message, strlen(message));
}

float readAveragedUnits(int readings, int samplesPerReading) {
  float avgWeight = 0.0f;               // Computed average weight in pounds to return at the end of the function.
  int   collected  = 0;                 // Number of samples actually read (may be less than requested if HX711 not ready)
  float totalUnits = 0.0f;              // Accumulator summing weight readings across all iterations for averaging
  
  // Use bounded wait to avoid infinite blocking while preserving the original
  // per-reading averaging semantics used across workflows.
  for (int readingIndex = 0; readingIndex < readings; ++readingIndex) {
    if (!scale.wait_ready_timeout(HX711_READY_TIMEOUT_MS)) {
      continue;
    }

    totalUnits += scale.get_units(samplesPerReading);
    collected++;
  }

  if (collected == 0) {
    return NAN;
  }

  avgWeight = totalUnits / collected;
  return avgWeight;
}

void saveRuntimeTareOffset() {
  float offsetToSave = static_cast<float>(scale.get_offset());
  if (!saveToEeprom(offsetToSave,
                    HX711_OFFSET_EEPROM_MAGIC,
                    HX711_OFFSET_EEPROM_MAGIC_ADDR,
                    HX711_OFFSET_EEPROM_VALUE_ADDR)) {
    Serial.println("Warning: failed to save runtime tare offset to EEPROM.");
  }
}

bool startSampleBatch(int readings, int samplesPerReading) {
  if (readings <= 0) return false;
  if (sampler.running) return false; // already running

  sampler.totalReadings = readings;
  sampler.samplesPerReading = samplesPerReading;
  sampler.readingsTaken = 0;
  sampler.totalUnits = 0.0;
  sampler.running = true;
  sampler.lastResult = NAN;
  return true;
}

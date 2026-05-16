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

/**
 * @section Standard library headers
 */

#include <Arduino.h>                                        // Arduino core library for Serial communication and basic types
#include <math.h>                                           // Math library for fabsf() and other mathematical functions
#include <string.h>                                         // String helpers for non-blocking serial queue management

/**
 * @subsection Third party library headers
 */

#include "HX711.h"                                          // HX711 library for interfacing with the load cell amplifier to read weight data

/** 
 * @section Local library headers
 */

#include "config.h"                                         // Configuration constants for the ESP32-based propane level scale
#include "eeprom_store.h"                                   // EEPROM storage functions
#include "scale_io.h"                                       // Input/output functions for user workflows and HX711 interactions

/**
 * @section External Global State Variables
 */

extern HX711 scale;                                         // HX711 instance for interacting with the load cell amplifier

/** 
 * @section Private Static Constants and Variables
 */

static constexpr size_t SERIAL_CAPACITY = 2048;              // Capacity of the internal serial output queue in bytes
static size_t serialLength = 0;                              // Current length of data in the serial output queue
static size_t serialOffset = 0;                              // Current offset for reading from the serial output queue
static char serialQueue[SERIAL_CAPACITY];                    // Internal buffer for queued serial output

/**
 * @section Private Helper Functions
 */

/**
 * @brief Probes the HX711 with multiple reads to determine if it is producing a responsive signal.
 * 
 * @details Secondary check to detect if HX711 is powered but not properly connected.
 * Intentionally private implementation detail, only used as part of the scale ready workflow.
 * 
 * @return true if the HX711 is producing a responsive signal with variability across multiple reads; false otherwise.
 * 
 * @throws none This function does not throw exceptions.
 */
static bool hasResponsiveHx711Signal() {
  const int probeReads = 10;                                // HX711 is set at 10 samples per second
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
 * @param message The message to queue for serial output.
 * @param messageLength The length of the message in bytes.
 * @return true if the message was successfully queued; false if there was insufficient space in the queue.
 * 
 * @throws none This function does not throw exceptions.
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
 * @section Definitions for public input/output functions for user workflows and HX711 interactions.
 */

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
  if (scale.wait_ready_timeout(HX711_READY_TIMEOUT_MS)) {
    if (hasResponsiveHx711Signal()) {
      return true;
    }
  }

  printScaleNotReadyDiagnostic(operation);
  return false;
}

void flushSerialInput() {
  while (Serial.available()) {
    Serial.read();
  }
}

bool queueSerialOutput(const char* message) {
  // want to avoid calling strlen() on a null pointer, 
  // so treat null as empty message that is successfully queued
  if (message == nullptr) {
    return true;
  }

  return queueSerialOutput(message, strlen(message));
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
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

// Third party library headers
#include "HX711.h"                                          // HX711 library for interfacing with the load cell amplifier to read weight data

// Local library headers
#include "config.h"                                         // Configuration constants for the ESP32-based propane level scale
#include "eeprom_store.h"                                   // EEPROM storage functions
#include "scale_io.h"                                       // Input/output functions for user workflows and HX711 interactions
#include "workflows/workflows_contexts.h"                   // Workflow context types for managing state across non-blocking workflow steps

// External Global State Variables
extern HX711 scale;                                         // HX711 instance for interacting with the load cell amplifier

// Global Averaging Context Variables
AvgContext avgCtx;                                          // Averaging context instance to hold state for non-blocking average computations
ThresholdAvgContext levelThresholdCtx;                      // Level threshold averaging context for asynchronous threshold computation
ThresholdAvgContext calThresholdCtx;                        // Calibration threshold averaging context for asynchronous threshold computation

//  Private Static Constants and Variables
static constexpr size_t SERIAL_CAPACITY = 2048;              // Capacity of the internal serial output queue in bytes
static size_t serialLength = 0;                              // Current length of data in the serial output queue
static size_t serialOffset = 0;                              // Current offset for reading from the serial output queue
static char serialQueue[SERIAL_CAPACITY];                    // Internal buffer for queued serial output

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

 bool averageUnits(int readings, int samplesPerReading, float &outAvg) {
    if (!avgCtx.active || avgCtx.requestedReadings != readings || avgCtx.samplesPerReading != samplesPerReading) {
    avgCtx.requestedReadings = readings;
    avgCtx.samplesPerReading = samplesPerReading;
    avgCtx.index = 0;
    avgCtx.collected = 0;
    avgCtx.total = 0.0f;
    avgCtx.active = true;
  }

  // If scale isn't ready right now, caller should call again later
  if (!scale.is_ready()) {
    return false;
  }

  // Take a single averaged reading (samplesPerReading) when ready
  float units = scale.get_units(samplesPerReading);
  avgCtx.total += units;
  avgCtx.collected++;
  avgCtx.index++;

  // When we've got the requested number of readings, finish and return result
  if (avgCtx.index >= avgCtx.requestedReadings) {
    if (avgCtx.collected == 0) {
      outAvg = NAN;
    } else {
      outAvg = avgCtx.total / static_cast<float>(avgCtx.collected);
    }
    avgCtx.active = false;
    return true;
  }

  // Not finished yet
  return false;
}

void cancelCalLoadDetect() {
  calThresholdCtx.active = false;
  calThresholdCtx.index = 0;
  calThresholdCtx.collected = 0;
  calThresholdCtx.total = 0.0f;
}

void cancelLevelLoadDetect() {
  levelThresholdCtx.active = false;
  levelThresholdCtx.index = 0;
  levelThresholdCtx.collected = 0;
  levelThresholdCtx.total = 0.0f;
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

bool pollCalLoadDetect(float &outThreshold) {
  if (!calThresholdCtx.active) {
    return false;
  }

  if (!scale.is_ready()) {
    return false;
  }

  float units = scale.get_units(calThresholdCtx.samplesPerReading);
  calThresholdCtx.total += units;
  calThresholdCtx.collected++;
  calThresholdCtx.index++;

  if (calThresholdCtx.index >= calThresholdCtx.requestedReadings) {
    float avg;
    if (calThresholdCtx.collected == 0) {
      avg = NAN;
    } else {
      avg = calThresholdCtx.total / static_cast<float>(calThresholdCtx.collected);
    }

    calThresholdCtx.active = false;

    if (!isfinite(avg)) {
      outThreshold = calThresholdCtx.minimumThreshold;
    } else {
      float noise = fabsf(avg);
      outThreshold = fmaxf(noise * 20.0f, calThresholdCtx.minimumThreshold);
    }
    return true;
  }

  return false;
}

bool pollLevelLoadDetect(float &outThreshold) {
  if (!levelThresholdCtx.active) {
    return false;
  }

  if (!scale.is_ready()) {
    return false;
  }

  float units = scale.get_units(levelThresholdCtx.samplesPerReading);
  levelThresholdCtx.total += units;
  levelThresholdCtx.collected++;
  levelThresholdCtx.index++;

  if (levelThresholdCtx.index >= levelThresholdCtx.requestedReadings) {
    float avg;
    if (levelThresholdCtx.collected == 0) {
      avg = NAN;
    } else {
      avg = levelThresholdCtx.total / static_cast<float>(levelThresholdCtx.collected);
    }

    levelThresholdCtx.active = false;

    if (!isfinite(avg)) {
      outThreshold = levelThresholdCtx.minimumThreshold;
    } else {
      float noise = fabsf(avg);
      outThreshold = fmaxf(noise * 20.0f, levelThresholdCtx.minimumThreshold);
    }
    return true;
  }

  return false;
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

void saveRuntimeTareOffset() {
  float offsetToSave = static_cast<float>(scale.get_offset());
  if (!saveToEeprom(offsetToSave,
                    HX711_OFFSET_EEPROM_MAGIC,
                    HX711_OFFSET_EEPROM_MAGIC_ADDR,
                    HX711_OFFSET_EEPROM_VALUE_ADDR)) {
    Serial.println("Warning: failed to save runtime tare offset to EEPROM.");
  }
}

void startCalLoadDetect(float minimumThresholdLbs) {
  calThresholdCtx.requestedReadings = UNLOAD_CHECK_COUNT;
  calThresholdCtx.samplesPerReading = LIVE_SAMPLES;
  calThresholdCtx.index = 0;
  calThresholdCtx.collected = 0;
  calThresholdCtx.total = 0.0f;
  calThresholdCtx.minimumThreshold = minimumThresholdLbs;
  calThresholdCtx.active = true;
}

void startLevelLoadDetect(float minimumThresholdLbs) {
  levelThresholdCtx.requestedReadings = UNLOAD_CHECK_COUNT;
  levelThresholdCtx.samplesPerReading = LIVE_SAMPLES;
  levelThresholdCtx.index = 0;
  levelThresholdCtx.collected = 0;
  levelThresholdCtx.total = 0.0f;
  levelThresholdCtx.minimumThreshold = minimumThresholdLbs;
  levelThresholdCtx.active = true;
}
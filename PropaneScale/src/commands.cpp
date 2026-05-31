/**
 * @file commands.cpp
 * @author Gerald Manweiler
 *
 * @brief Command processing implementation for the propane scale project.
 *
 * @details Implements functions for processing serial commands, managing startup tare workflow,
 * and resetting EEPROM values to defaults.
 *
 * @version 0.1
 * @date 2024-06-01
 *
 * @copyright Copyright (c) 2024 Gerald Manweiler
 */

// Standard library headers
#include <Arduino.h>
#include <EEPROM.h>

// Third party library headers
#include "HX711.h"                                          // HX711 library for interfacing with the load cell amplifier to read weight data

// Local library headers
#include "config.h"
#include "src/eeprom_store.h"
#include "src/scale_io.h"

// External Global State Variables and Functions
extern float calibrationFactor;
extern bool eepromReady;
extern float knownWeight;
extern float maxPropane;
extern HX711 scale;                                         // HX711 instance for interacting with the load cell amplifier
extern float tankTare;

// Definitions for command processing functions

void defaultEeprom()
{
  if (!eepromReady) {
    queueSerialOutput("EEPROM is not initialized; cannot reset to defaults.\n");
    return;
  }

  queueSerialOutput("\nResetting EEPROM to hardcoded defaults...\n");

  calibrationFactor = DEF_CALIBRATION_FACTOR;

  if (!saveToEeprom(calibrationFactor, CAL_EEPROM_MAGIC, CAL_EEPROM_MAGIC_ADDR, CAL_EEPROM_VALUE_ADDR)) {
    queueSerialOutput("Failed to save default calibration factor.\n");
  } else {
    char buf[128];
    snprintf(buf, sizeof(buf), "Calibration factor reset to: %.2f\n", calibrationFactor);
    queueSerialOutput(buf);
  }

  knownWeight = DEF_KNOWN_WEIGHT;

  if (!saveToEeprom(knownWeight, KNOWN_WEIGHT_EEPROM_MAGIC, KNOWN_WEIGHT_EEPROM_MAGIC_ADDR, KNOWN_WEIGHT_EEPROM_VALUE_ADDR)) {
    queueSerialOutput("Failed to save default known calibration weight.\n");
  } else {
    char buf[128];
    snprintf(buf, sizeof(buf), "Known calibration weight reset to: %.2f lbs\n", knownWeight);
    queueSerialOutput(buf);
  }

  maxPropane = DEF_MAX_PROPANE;

  if (!saveToEeprom(maxPropane, MAX_PROPANE_EEPROM_MAGIC, MAX_PROPANE_EEPROM_MAGIC_ADDR, MAX_PROPANE_EEPROM_VALUE_ADDR)) {
    queueSerialOutput("Failed to save default max propane weight.\n");
  } else {
    char buf[128];
    snprintf(buf, sizeof(buf), "Max propane weight reset to: %.2f lbs\n", maxPropane);
    queueSerialOutput(buf);
  }

  tankTare = DEF_TANK_TARE;

  if (!saveToEeprom(tankTare, TARE_EEPROM_MAGIC, TARE_EEPROM_MAGIC_ADDR, TARE_EEPROM_VALUE_ADDR)) {
    queueSerialOutput("Failed to save default tank tare.\n");
  } else {
    char buf[128];
    snprintf(buf, sizeof(buf), "Tank tare reset to: %.2f lbs\n", tankTare);
    queueSerialOutput(buf);
  }

  // Invalidate persisted runtime tare offset so next boot starts from a known state.
  uint32_t clearOffsetMagic = 0;
  EEPROM.put(HX711_OFFSET_EEPROM_MAGIC_ADDR, clearOffsetMagic);

  if (!EEPROM.commit()) {
    queueSerialOutput("Failed to clear saved runtime tare offset.\n");
  } else {
    queueSerialOutput("Saved runtime tare offset record cleared.\n");
  }

  scale.set_scale(calibrationFactor);
  queueSerialOutput("EEPROM reset complete. Run re-zero ('r') on an empty scale, then recalibrate before use.\n");
}
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

/**
 * @section Standard library headers
 */

#include <Arduino.h>
#include <EEPROM.h>

/** 
 * @section Local library headers
 */
#include "config.h"
#include "src/eeprom_store.h"

/**
 * @subsection Third party library headers
 */

#include "HX711.h"                                          // HX711 library for interfacing with the load cell amplifier to read weight data

/**
 * @section External Global State Variables and Functions
 */

extern float calibrationFactor;
extern bool eepromReady;
extern float knownWeight;
extern float maxPropane;
extern HX711 scale;                                         // HX711 instance for interacting with the load cell amplifier
extern float tankTare;

void defaultEeprom() {
  if (!eepromReady) {
    Serial.println("EEPROM is not initialized; cannot reset to defaults.");
    return;
  }

  Serial.println();
  Serial.println("Resetting EEPROM to hardcoded defaults...");

  calibrationFactor = DEF_CALIBRATION_FACTOR;
  if (!saveToEeprom(calibrationFactor, CAL_EEPROM_MAGIC, CAL_EEPROM_MAGIC_ADDR, CAL_EEPROM_VALUE_ADDR)) {
    Serial.println("Failed to save default calibration factor.");
  } else {
    Serial.print("Calibration factor reset to: ");
    Serial.println(calibrationFactor, 2);
  }

  knownWeight = DEF_KNOWN_WEIGHT;
  if (!saveToEeprom(knownWeight, KNOWN_WEIGHT_EEPROM_MAGIC, KNOWN_WEIGHT_EEPROM_MAGIC_ADDR, KNOWN_WEIGHT_EEPROM_VALUE_ADDR)) {
    Serial.println("Failed to save default known calibration weight.");
  } else {
    Serial.print("Known calibration weight reset to: ");
    Serial.print(knownWeight, 2);
    Serial.println(" lbs");
  }

  maxPropane = DEF_MAX_PROPANE;
  if (!saveToEeprom(maxPropane, MAX_PROPANE_EEPROM_MAGIC, MAX_PROPANE_EEPROM_MAGIC_ADDR, MAX_PROPANE_EEPROM_VALUE_ADDR)) {
    Serial.println("Failed to save default max propane weight.");
  } else {
    Serial.print("Max propane weight reset to: ");
    Serial.print(maxPropane, 2);
    Serial.println(" lbs");
  }

  tankTare = DEF_TANK_TARE;
  if (!saveToEeprom(tankTare, TARE_EEPROM_MAGIC, TARE_EEPROM_MAGIC_ADDR, TARE_EEPROM_VALUE_ADDR)) {
    Serial.println("Failed to save default tank tare.");
  } else {
    Serial.print("Tank tare reset to: ");
    Serial.print(tankTare, 2);
    Serial.println(" lbs");
  }

  // Invalidate persisted runtime tare offset so next boot starts from a known state.
  uint32_t clearOffsetMagic = 0;
  EEPROM.put(HX711_OFFSET_EEPROM_MAGIC_ADDR, clearOffsetMagic);
  if (!EEPROM.commit()) {
    Serial.println("Failed to clear saved runtime tare offset.");
  } else {
    Serial.println("Saved runtime tare offset record cleared.");
  }

  scale.set_scale(calibrationFactor);
  Serial.println("EEPROM reset complete. Run re-zero ('r') on an empty scale, then recalibrate before use.");
}
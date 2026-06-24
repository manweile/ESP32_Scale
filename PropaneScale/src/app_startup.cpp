#include "app_startup.h"
/**
 * @file app_startup.cpp
 * @author Gerald Manweiler
 *
 * @brief Application startup initialization implementation for the propane scale project.
 *
 * @details Implements helper functions used to initialize the application before the main
 * startup tare workflow begins.
 *
 * @version 0.1
 * @date 2026-05-14
 *
 * @copyright Copyright (c) 2026 Gerald Manweiler
 */

// Standard library headers
#include <Arduino.h>                                        // Arduino core library for Serial communication and basic types
#include <EEPROM.h>                                         // EEPROM library for persistent storage of calibration and tare values

// Third party library headers
#include "HX711.h"                                          // HX711 library for interfacing with the load cell amplifier to read weight data

// Local library headers
#include "config.h"                                         // Configuration constants for the ESP32-based propane level scale
#include "src/eeprom_store.h"                               // EEPROM storage functions
#include "src/parsing_utils.h"                              // Utility functions for validating and parsing input values
#include "src/runtime_report.h"                             // Runtime reporting functions
#include "src/workflows/startup_tare_workflow.h"            // Functions for the startup tare workflow

// External Global State Variables and Functions
extern float calibrationFactor;
extern bool eepromReady;
extern HX711 scale;
extern float knownWeight;
extern float maxPropane;
extern float tankTare;

// Definitions for application startup initialization functions

void initializeApp()
{
  // need to check if eeprom is ready before trying to load values, and if not, use defaults and continue without eeprom functionality
  eepromReady = EEPROM.begin(EEPROM_SIZE_BYTES);

  // complete eeprom begin failure means we have to use the hard coded defaults and cannot persist any changes,
  // but we can still operate the scale with the default calibration factor and tare values,
  // so we should not halt execution, just warn the user and continue
  if (!eepromReady) {
    LastStartupReport = String("EEPROM init failed. Using defaults.");
    calibrationFactor = DEF_CALIBRATION_FACTOR;
    knownWeight       = DEF_KNOWN_WEIGHT;
    maxPropane        = DEF_MAX_PROPANE;
    tankTare          = DEF_TANK_TARE;
  } else {
    // for each persisted value, use the magic marker if present and valid,
    // otherwise use the default and save it to eeprom for next time
    // this way if one value becomes corrupted, it does not affect the others,
    // user can still get a valid reading with defaults ,
    // and can fix the corrupted value by re-saving it

    float eepromCal = 0.0f;

    if (loadFromEeprom(eepromCal, CAL_EEPROM_MAGIC_ADDR, CAL_EEPROM_MAGIC, CAL_EEPROM_VALUE_ADDR)
        && isValidBoundedFloat(eepromCal, CAL_FACTOR_ABS_MIN, CAL_FACTOR_ABS_MAX, true)) {
      calibrationFactor = eepromCal;
    } else {
      calibrationFactor = DEF_CALIBRATION_FACTOR;
      LastStartupReport = String("Calibration factor not found or invalid; using default.");

      if (!saveToEeprom(calibrationFactor, CAL_EEPROM_MAGIC, CAL_EEPROM_MAGIC_ADDR, CAL_EEPROM_VALUE_ADDR)) {
        LastStartupReport = String(CALIBRATION_SAVE_FAILURE_MSG);
      } else {
        LastStartupReport = String(CALIBRATION_SAVE_SUCCESS_MSG);
      }
    }

    float eepromKnownWeight = 0.0f;

    if (loadFromEeprom(eepromKnownWeight, KNOWN_WEIGHT_EEPROM_MAGIC_ADDR, KNOWN_WEIGHT_EEPROM_MAGIC, KNOWN_WEIGHT_EEPROM_VALUE_ADDR)
        && isValidBoundedFloat(eepromKnownWeight, MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT)) {
      knownWeight = eepromKnownWeight;
    } else {
      knownWeight = DEF_KNOWN_WEIGHT;
      LastStartupReport = String("Known calibration weight not found or invalid; using default.");

      if (!saveToEeprom(knownWeight, KNOWN_WEIGHT_EEPROM_MAGIC, KNOWN_WEIGHT_EEPROM_MAGIC_ADDR, KNOWN_WEIGHT_EEPROM_VALUE_ADDR)) {
        LastStartupReport = String("Failure saving default known calibration weight to EEPROM.");
      } else {
        LastStartupReport = String("Success saving default known calibration weight to EEPROM.");
      }
    }

    float eepromMaxPropane = 0.0f;

    if (loadFromEeprom(eepromMaxPropane, MAX_PROPANE_EEPROM_MAGIC_ADDR, MAX_PROPANE_EEPROM_MAGIC, MAX_PROPANE_EEPROM_VALUE_ADDR)
        && isValidBoundedFloat(eepromMaxPropane, MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT)) {
      maxPropane = eepromMaxPropane;
    } else {
      maxPropane = DEF_MAX_PROPANE;
      LastStartupReport = String("Max propane weight not found or invalid; using default.");

      if (!saveToEeprom(maxPropane, MAX_PROPANE_EEPROM_MAGIC, MAX_PROPANE_EEPROM_MAGIC_ADDR, MAX_PROPANE_EEPROM_VALUE_ADDR)) {
        LastStartupReport = String("Failure saving default max propane weight to EEPROM.");
      } else {
        LastStartupReport = String("Success saving default max propane weight to EEPROM.");
      }
    }

    float eepromTankTare = 0.0f;

    if (loadFromEeprom(eepromTankTare, TARE_EEPROM_MAGIC_ADDR, TARE_EEPROM_MAGIC, TARE_EEPROM_VALUE_ADDR)
        && isValidBoundedFloat(eepromTankTare, MIN_PLAUSIBLE_WEIGHT, MAX_PROJECT_WEIGHT)) {
      tankTare = eepromTankTare;
    } else {
      tankTare = DEF_TANK_TARE;
      LastStartupReport = String("Tank tare not found or invalid; using default.");

      if (!saveToEeprom(tankTare, TARE_EEPROM_MAGIC, TARE_EEPROM_MAGIC_ADDR, TARE_EEPROM_VALUE_ADDR)) {
        LastStartupReport = String("Failure saving default tank tare to EEPROM.");
      } else {
        LastStartupReport = String("Success saving default tank tare to EEPROM.");
      }
    }
  }

  scale.begin(DOUT_PIN, CLK_PIN);

  // Restore previously saved runtime tare offset so startup empty/load checks use a known-empty reference.
  float savedRuntimeOffset = 0.0f;

  if (loadFromEeprom(savedRuntimeOffset,
                     HX711_OFFSET_EEPROM_MAGIC_ADDR,
                     HX711_OFFSET_EEPROM_MAGIC,
                     HX711_OFFSET_EEPROM_VALUE_ADDR)) {
    scale.set_offset(static_cast<long>(savedRuntimeOffset));
  }
}
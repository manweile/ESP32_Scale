/**
 * @file eeprom_init_logic.h
 * @author Gerald Manweiler
 * 
 * @brief EEPROM initialization logic for float values.
 * 
 * @details Defines logic to load a float value from EEPROM, validate the value, and handle loading failure or invalid values.
 * 
 * @version 0.1
 * @date 2026-04-30
 * @copyright Copyright (c) 2026
 */

#ifndef EEPROM_INIT_LOGIC_H
#define EEPROM_INIT_LOGIC_H

/**
 * @enum EepromLoadInitStatus
 * 
 * @brief Enum representing status of EEPROM load or initialization.
 * 
 * @details Defines possible outcomes attempting loading float value from EEPROM.
 */
enum EepromLoadInitStatus {
  EEPROM_VALUE_LOADED_VALID,                                /**< Value successfully loaded from EEPROM, passed validation. */
  EEPROM_VALUE_INITIALIZED_AFTER_LOAD_FAIL,                 /**< Value failed loading from EEPROM, default initialized, save attempted. */
  EEPROM_VALUE_INITIALIZED_AFTER_INVALID_LOAD               /**< Value loaded from EEPROM but failed validation, default initialized, save attempted. */
};

/**
 * @struct EepromLoadInitResult
 * 
 * @brief Struct representing result of EEPROM load or initialization.
 * 
 * @details Contains status of load/initialization operation and whether saving succeeded.
 */
struct EepromLoadInitResult {
  EepromLoadInitStatus status;                              /**< Status of load or initialization operation. */
  bool saveSucceeded;                                       /**< Indicates saving default value to EEPROM succeeded. */
};

/**
 * @brief Loads a float value from EEPROM or initializes it with a default value.
 * 
 * @details Attempt to load a float value using the provided load function. 
 * Loading succeeds, validates loaded value using provided validation function. 
 * Loading fails or loaded value is invalid, initialize target variable with provided default value.
 * The provided default value is drawn from hard coded values for project.
 * Attempt saving default value using provided save function. 
 * Result includes both status of load/initialization and whether saving succeeded.
 * 
 * @param {float} defaultValue Default value if loading fails or loaded value is invalid.
 * @param {float&} targetValue Variable to store loaded or initialized value.
 * @param {bool (*)(float&)} loadFn Function pointer to EEPROM load function.
 * @param {bool (*)(float)} saveFn Function pointer to EEPROM save function.
 * @param {bool (*)(float)} validateFn Function pointer to validation function.
 * @return {struct}(EepromLoadInitResult) Result of load or initialization operation.
 */
EepromLoadInitResult loadOrInitializeFloatValue(
  float defaultValue,
  float& targetValue,
  bool (*loadFn)(float&),
  bool (*saveFn)(float),
  bool (*validateFn)(float)
);

#endif

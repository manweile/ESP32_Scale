/**
 * @file eeprom_init_logic.cpp
 * @author Gerald Manweiler
 * 
 * @brief EEPROM initialization logic for float values.
 *
 * @details Implements logic to load a float value from EEPROM, validate the value, and handle loading failure or invalid values.
 * 
 * @version 0.1
 * @date 2026-04-30
 * @copyright Copyright (c) 2026
 */

#include "eeprom_init_logic.h"

EepromLoadInitResult loadOrInitializeFloatValue(
  float defaultValue,
  float& targetValue,
  bool (*loadFn)(float&),
  bool (*saveFn)(float),
  bool (*validateFn)(float)
) {
  EepromLoadInitResult result = {EEPROM_VALUE_INITIALIZED_AFTER_LOAD_FAIL, false};

  bool loadSuccess = loadFn(targetValue);
  if (loadSuccess) {

    bool isValid = validateFn(targetValue);
    if (isValid) {
      result.status = EEPROM_VALUE_LOADED_VALID;
      result.saveSucceeded = true;
      return result;
    } else {
      // loaded value is invalid, fall back to supplied default and save
      targetValue = defaultValue;
      result.status = EEPROM_VALUE_INITIALIZED_AFTER_INVALID_LOAD;
      result.saveSucceeded = saveFn(targetValue);
      return result;
    }
  } else {  
    // loading fails, fall back to supplied default and save
    targetValue = defaultValue;
    result.status = EEPROM_VALUE_INITIALIZED_AFTER_LOAD_FAIL;
    result.saveSucceeded = saveFn(targetValue);
  }
  return result;
}

#include "eeprom_init_logic.h"

EepromLoadInitResult loadOrInitializeFloatValue(
  float defaultValue,
  float& targetValue,
  bool (*loadFn)(float&),
  bool (*saveFn)(float),
  bool (*validateFn)(float)
) {
  EepromLoadInitResult result = {EEPROM_VALUE_INITIALIZED_AFTER_LOAD_FAIL, false};

  if (loadFn(targetValue)) {
    if (validateFn(targetValue)) {
      result.status = EEPROM_VALUE_LOADED_VALID;
      result.saveSucceeded = true;
      return result;
    }

    targetValue = defaultValue;
    result.status = EEPROM_VALUE_INITIALIZED_AFTER_INVALID_LOAD;
    result.saveSucceeded = saveFn(targetValue);
    return result;
  }

  targetValue = defaultValue;
  result.status = EEPROM_VALUE_INITIALIZED_AFTER_LOAD_FAIL;
  result.saveSucceeded = saveFn(targetValue);
  return result;
}

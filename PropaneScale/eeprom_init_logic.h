#ifndef EEPROM_INIT_LOGIC_H
#define EEPROM_INIT_LOGIC_H

enum EepromLoadInitStatus {
  EEPROM_VALUE_LOADED_VALID,
  EEPROM_VALUE_INITIALIZED_AFTER_LOAD_FAIL,
  EEPROM_VALUE_INITIALIZED_AFTER_INVALID_LOAD
};

struct EepromLoadInitResult {
  EepromLoadInitStatus status;
  bool saveSucceeded;
};

EepromLoadInitResult loadOrInitializeFloatValue(
  float defaultValue,
  float& targetValue,
  bool (*loadFn)(float&),
  bool (*saveFn)(float),
  bool (*validateFn)(float)
);

#endif

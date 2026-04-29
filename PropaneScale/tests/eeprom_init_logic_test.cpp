#include <cassert>
#include <iostream>

#include "../eeprom_init_logic.h"

namespace {

float gSavedValue = 0.0f;
int gSaveCallCount = 0;

bool saveSpy(float value) {
  gSavedValue = value;
  ++gSaveCallCount;
  return true;
}

bool loadValid(float& outValue) {
  outValue = 1234.0f;
  return true;
}

bool loadCorrupt(float& outValue) {
  outValue = -5.0f;
  return true;
}

bool loadMissing(float& outValue) {
  (void)outValue;
  return false;
}

bool validatePositive(float value) {
  return value > 0.0f;
}

void resetSpies() {
  gSavedValue = 0.0f;
  gSaveCallCount = 0;
}

}  // namespace

int main() {
  const float defaultValue = 42.0f;
  float targetValue = 0.0f;

  // Valid load path should not save defaults.
  resetSpies();
  EepromLoadInitResult validResult = loadOrInitializeFloatValue(
    defaultValue,
    targetValue,
    loadValid,
    saveSpy,
    validatePositive
  );
  assert(validResult.status == EEPROM_VALUE_LOADED_VALID);
  assert(targetValue == 1234.0f);
  assert(gSaveCallCount == 0);

  // Corrupt payload path should restore and save default.
  resetSpies();
  targetValue = 0.0f;
  EepromLoadInitResult corruptResult = loadOrInitializeFloatValue(
    defaultValue,
    targetValue,
    loadCorrupt,
    saveSpy,
    validatePositive
  );
  assert(corruptResult.status == EEPROM_VALUE_INITIALIZED_AFTER_INVALID_LOAD);
  assert(targetValue == defaultValue);
  assert(gSaveCallCount == 1);
  assert(gSavedValue == defaultValue);

  // Missing record path should restore and save default.
  resetSpies();
  targetValue = 0.0f;
  EepromLoadInitResult missingResult = loadOrInitializeFloatValue(
    defaultValue,
    targetValue,
    loadMissing,
    saveSpy,
    validatePositive
  );
  assert(missingResult.status == EEPROM_VALUE_INITIALIZED_AFTER_LOAD_FAIL);
  assert(targetValue == defaultValue);
  assert(gSaveCallCount == 1);
  assert(gSavedValue == defaultValue);

  std::cout << "All EEPROM init logic tests passed." << std::endl;
  return 0;
}

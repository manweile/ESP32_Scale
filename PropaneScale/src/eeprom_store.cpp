/**
 * @file eeprom_store.cpp
 * @author Gerald Manweiler
 * 
 * @brief EEPROM storage functions for the propane scale project.
 * 
 * @details Implements eeprom functions to load, save & validate calibration factors, known weights, maximum propane weight, and tank tare.
 * 
 * @version 0.1
 * @date 2026-05-06
 * 
 * @copyright Copyright (c) 2026 Gerald Manweiler
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
#include "eeprom_store.h"
#include "parsing_utils.h"


 extern bool eepromReady;                                   // Flag to track if EEPROM was successfully initialized

bool loadFromEeprom(float& value, uint32_t magicAddr, uint32_t magicValue, uint32_t valueAddr) {
    uint32_t magic = 0;                                       // Magic number read from EEPROM for validation

    EEPROM.get(magicAddr, magic);
    if (magic != magicValue) {
        return false;
    }

    EEPROM.get(valueAddr, value);
    return true;
}

bool saveToEeprom(float value, uint32_t magic, int magicAddr, int valueAddr) {
    if (!eepromReady) {
        return false;
    }

    EEPROM.put(magicAddr, magic);
    EEPROM.put(valueAddr, value);
    return EEPROM.commit();
}

bool printEepromField(const char* label, uint32_t magicAddr, uint32_t magicValue, uint32_t valueAddr, float minValue, float maxValue, bool useAbsMag, const char* unitSuffix) {
    float value = 0.0f;

    if (loadFromEeprom(value, magicAddr, magicValue, valueAddr) && isValidBoundedFloat(value, minValue, maxValue, useAbsMag)) {
        Serial.print(label);
        Serial.print(": ");
        Serial.print(value, 2);

        if (unitSuffix != nullptr && unitSuffix[0] != '\0') {
            Serial.print(unitSuffix);
        }

        Serial.println();
        return true;
    }

    Serial.print(label);
    Serial.println(": <invalid or not set>");
    return false;
}
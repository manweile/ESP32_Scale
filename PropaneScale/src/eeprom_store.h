/**
 * @file eeprom_store.h
 * @author Gerald Manweiler
 *
 * @brief EEPROM storage functions for the propane scale project.
 *
 * @details Declares eeprom functions to load, save & validate calibration factors, known weights, maximum propane weight, and tank tare.
 *
 * @version 0.1
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026 Gerald Manweiler
 */

#pragma once

// Declarations of EEPROM storage functions for the propane scale project

/**
 * @brief Loads a float value from EEPROM if the magic number is valid.
 *
 * @details Reads the magic number from EEPROM to verify that a valid value has been saved.
 * If the magic number is valid, it loads the value into the provided reference variable.
 *
 * @param value {float&} Reference to a float variable where the loaded value will be stored.
 * @param magicAddr {uint32_t} EEPROM address of the magic number.
 * @param magicValue {uint32_t} Expected magic number for validation.
 * @param valueAddr {uint32_t} EEPROM address of the float value.
 * @return {bool} True if the value was successfully loaded, false otherwise.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool loadFromEeprom(float& value, uint32_t magicAddr, uint32_t magicValue, uint32_t valueAddr);

/**
 * @brief Reads one EEPROM float and prints it.
 *
 * @details Attempts to read a float value from EEPROM at the specified address,
 * validating it against an expected magic number and bounds.
 * If valid, prints the label and value (with optional unit suffix).
 * If invalid or not set, prints the label with an invalid notice.
 *
 * @param label {const char*} Display label printed before the value.
 * @param magicAddr {uint32_t} EEPROM address of the magic number.
 * @param magicValue {uint32_t} Expected magic number for validation.
 * @param valueAddr {uint32_t} EEPROM address of the float value.
 * @param minValue {float} Minimum valid value.
 * @param maxValue {float} Maximum valid value.
 * @param useAbsMag {bool} When true, validate using absolute magnitude (for signed calibration factor).
 * @param unitSuffix {const char*} Optional unit string appended after the value (e.g. " lbs"), or nullptr.
 * @return {bool} True if the value was valid and printed; false if invalid/not set.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool printEepromField(const char* label, uint32_t magicAddr, uint32_t magicValue, uint32_t valueAddr, float minValue, float maxValue,
                      bool useAbsMag = false, const char* unitSuffix = nullptr);

/**
 * @brief Saves a float value to EEPROM with a magic number for validation.
 *
 * @details Writes the magic number and float value to EEPROM, and commits the changes.
 * Returns false immediately if EEPROM was not successfully initialized.
 *
 * @param value {float} The float value to save.
 * @param magic {uint32_t} The magic number for validation.
 * @param magicAddr {int} EEPROM address of the magic number.
 * @param valueAddr {int} EEPROM address of the float value.
 * @return {bool} True if the value was successfully saved, false otherwise.
 *
 * @throws {none} This function does not throw exceptions.
 */
bool saveToEeprom(float value, uint32_t magic, int magicAddr, int valueAddr);
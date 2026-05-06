/**
 * @file parsing_utils.h
 * @author Gerald Manweiler
 * 
 * @brief Utility functions for parsing and validating input values.
 * 
 * @details Provides functions for validating that float values are within expected bounds and for parsing non-negative floats from strings, with robust error handling and no exceptions.
 * 
 * @version 0.1
 * @date 2026-05-06
 * 
 * @copyright Copyright (c) 2026 Gerald Manweiler
 * 
 */

#pragma once

/**
 * @brief Validates that a float value is finite and within specified bounds.
 * 
 * @details Checks if a float value is finite and within specified bounds, with an option to use absolute magnitude for the check.
 * 
 * @param value The float value to validate.
 * @param minimumValue The minimum allowable value.
 * @param maximumValue The maximum allowable value.
 * @param useAbsoluteMagnitude If true, the absolute value of the float is used for validation.
 * @return true if the value is valid, false otherwise.
 * 
 * @throws {none} This function does not throw exceptions.
 */
bool isValidBoundedFloat(float value, float minimumValue, float maximumValue, bool useAbsoluteMagnitude = false);

/**
 * @brief Parses a non-negative float from a null-terminated C string.
 *
 * @details Attempts to parse a float value from the input string. 
 * Validates that the entire string is a valid float representation and that the parsed value is non-negative.
 * 
 * @param {const char*} text Input text to parse.
 * @param {float&} outValue Parsed output value on success.
 * @return {bool} True if parsing succeeds and the value is non-negative.
 * 
 * @throws {none} This function does not throw exceptions.
 */
bool parseNonNegativeFloat(const char* text, float& outValue);

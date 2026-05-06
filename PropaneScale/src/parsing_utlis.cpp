/**
 * @file parsing_utils.cpp
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
 */

/**
 * @section Standard library headers
 */
#include <math.h>
#include <stdlib.h>

/** 
 * @section Local library headers
 */
#include "parsing_utils.h"

bool isValidBoundedFloat(float value, float minimumValue, float maximumValue, bool useAbsoluteMagnitude) {
    float candidate = 0.0f;                                   // Temporarily holds the value used for comparison

    if (!isfinite(value)) {
        return false;
    }

    candidate = useAbsoluteMagnitude ? fabsf(value) : value;
    return (candidate >= minimumValue) && (candidate <= maximumValue);
}

bool parseNonNegativeFloat(const char* text, float& outValue) {
    char* parseEnd = nullptr;                                 // Pointer used by strtof to indicate where parsing stopped
    float parsed;                                             // Parsed float value from the input text

    // strtof will set parseEnd to point to the first character after the parsed float.
    parsed = strtof(text, &parseEnd);                   

    if (parseEnd == text) {
        return false;
    }

    // loop is not a blocking concern since strtof has already parsed the float 
    // and we are just validating that the rest of the string is whitespace 
    // and that the value is non-negative, which are both very fast operations
    while (*parseEnd == ' ' || *parseEnd == '\t') {
        ++parseEnd;
    }

    if (*parseEnd != '\0' || parsed < 0.0f) {
        return false;
    }

    outValue = parsed;
    return true;
}

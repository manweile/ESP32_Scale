/**
 * @file parsing_utils.cpp
 * @author Gerald Manweiler
 *
 * @brief Utility functions for parsing and validating input values.
 *
 * @details Implements functions for validating & parsing float values.
 *
 * @version 0.1
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026 Gerald Manweiler
 */

// Standard library headers
#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

// Local library headers
#include "parsing_utils.h"

// Definitions for parsing and validation utility functions

bool isValidBoundedFloat(float value, float minimumValue, float maximumValue, bool useAbsoluteMagnitude)
{
  float candidate = 0.0f;                                   // Temporarily holds the value used for comparison

  if (!isfinite(value)) {
    return false;
  }

  candidate = useAbsoluteMagnitude ? fabsf(value) : value;
  return (candidate >= minimumValue) && (candidate <= maximumValue);
}

String jsonEscape(const String& s)
{
  String out;
  out.reserve(s.length());

  for (size_t i = 0; i < s.length(); ++i) {
    char c = s.charAt(i);

    switch (c) {
    case '"':
      out += "\\\"";
      break;

    case '\\':
      out += "\\\\";
      break;

    case '\b':
      out += "\\b";
      break;

    case '\f':
      out += "\\f";
      break;

    case '\n':
      out += "\\n";
      break;

    case '\r':
      out += "\\r";
      break;

    case '\t':
      out += "\\t";
      break;

    default:
      if ((unsigned char)c < 0x20) {
        char buf[7];
        snprintf(buf, sizeof(buf), "\\u%04x", (unsigned char)c);
        out += buf;
      } else {
        out += c;
      }
    }
  }

  return out;
}

bool parseNonNegativeFloat(const char* text, float& outValue)
{
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
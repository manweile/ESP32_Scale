/**
 * @file scale_serial.cpp
 * @author Gerald Manweiler
 *
 * @brief Serial/console output helpers moved out of `scale_io` for web-only UI.
 *
 * @details This source file implements a small shim that replaces the legacy serial
 * queue used by the old serial-monitor UI. The implementation records the
 * last console-style output in `LastConsoleOutput` so the web UI can surface
 * the same messages without depending on a serial monitor.
 *
 * @version 0.1
 * @date 2024-06-01
 * @copyright Copyright (c) 2024 Gerald Manweiler
 */

// Local header provides Arduino.h and public API declarations
#include "scale_serial.h"

// Last console-style output for web UI
String LastConsoleOutput = "";                              /**< Last console-style output suitable for embedding in web UI JSON responses. */

// Public Definitions for input/output functions

void flushSerialInput()
{
  while (Serial.available()) {
    Serial.read();
  }
}

bool queueSerialOutput(const char* message)
{
  if (message == nullptr) return true;

  // Mirror the message into LastConsoleOutput for web UI consumption
  LastConsoleOutput = String(message);

  // Trim trailing newlines for safe JSON embedding
  uint8_t len = LastConsoleOutput.length() - 1;
  char lastChar = LastConsoleOutput.charAt(len);

  while (len > 0 && (lastChar == '\n' || lastChar == '\r')) {
    LastConsoleOutput.remove(len);
    --len;
    lastChar = LastConsoleOutput.charAt(len);
  }

  return true;
}



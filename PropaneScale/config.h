#ifndef CONFIG_H
#define CONFIG_H

// Calibration value used by HX711 conversion. Adjustable at runtime by calibration mode.
float calibration_factor = -10480.0f;

// HX711 wiring pins on the ESP32.
constexpr int DOUT_PIN = 16;
constexpr int CLK_PIN = 4;

// Readings below this threshold are clamped to zero.
constexpr float ZERO_DEADBAND_LBS = 0.30f;

// Weight of the always-present platen in pounds.
constexpr float platen_tare = 0.33125f;

// Software zero offset in pounds captured by tare/re-zero operations.
float zero_offset_lbs = 0.0f;

#endif // CONFIG_H

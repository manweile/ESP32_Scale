#ifndef CONFIG_H
#define CONFIG_H

// HX711 wiring pins on the ESP32.
constexpr int DOUT_PIN = 16;
constexpr int CLK_PIN = 4;

// serial connection speed
constexpr long BAUD = 115200;

// Number of samples to average for live weight readings (~1s at 10 SPS).
constexpr int LIVE_SAMPLES = 10;

// Number of samples to average during calibration reads (~500 ms at 10 SPS).
constexpr int CAL_SAMPLES = 5;

// Known reference weight used during automatic calibration mode (pounds).
// constexpr float CAL_KNOWN_WEIGHT_LBS = 31.6f;   // kitty litter jug of water 
constexpr float CAL_KNOWN_WEIGHT_LBS = 18.6f;   // wire spool 
// constexpr float CAL_KNOWN_WEIGHT_LBS = 7.20f;   // distilled jug of water

// Calibration value used by HX711 conversion. Adjustable at runtime by automatic or manual calibration mode.
// constexpr float DEF_CALIBRATION_FACTOR = -10651.67f;    // litter jug of water @ 36.6 lbs
constexpr float DEF_CALIBRATION_FACTOR = -10811.24f;    // wire spool @ 18.6 lbs
// constexpr float DEF_CALIBRATION_FACTOR = -10651.67f;    // distilled jug of water @ 10.0 lbs

// Tare weight of the empty twenty lb tank in pounds, used to subtract from readings to report only the weight of the propane.
constexpr float DEF_TWENTY_TANK_TARE = 17.0f;

// maximum legal propane weight (lbs) of 20 lb tank is 80% of tank capacity = 16.0 lbs
constexpr float MAX_TWENTY_PROPANE_LBS = 16.0f;

// Weight of the always-present platen in pounds.
constexpr float  PLATEN_TARE = 0.33125f;

#endif // CONFIG_H

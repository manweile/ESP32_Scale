#ifndef CONFIG_H
#define CONFIG_H

// HX711 wiring pins on the ESP32.
constexpr int CLK_PIN = 4;
constexpr int DOUT_PIN = 16;

// serial connection speed
constexpr long BAUD = 115200;

// Number of samples to average during calibration reads (~500 ms at 10 SPS).
constexpr int CAL_SAMPLES = 5;

// Number of samples to average for live weight readings (~1s at 10 SPS).
constexpr int LIVE_SAMPLES = 10;

// Known reference weight used during automatic calibration mode (pounds).
// constexpr float CAL_KNOWN_WEIGHT_LBS = 31.6f;   // kitty litter jug of water 
constexpr float CAL_KNOWN_WEIGHT_LBS = 18.6f;   // wire spool 
// constexpr float CAL_KNOWN_WEIGHT_LBS = 7.20f;   // distilled jug of water

// Calibration value used by HX711 conversion. Adjustable at runtime by automatic or manual calibration mode.
// constexpr float DEF_CALIBRATION_FACTOR = -10651.67f;    // litter jug of water @ 36.6 lbs
constexpr float DEF_CALIBRATION_FACTOR = -10811.24f;    // wire spool @ 18.6 lbs
// constexpr float DEF_CALIBRATION_FACTOR = -10651.67f;    // distilled jug of water @ 10.0 lbs

// Number of readings to average when checking for stable load placement during automatic calibration.
constexpr int LOADED_CHECK_COUNT = 5;

// Number of readings to average when checking for stable no-load condition during automatic/manual calibration.
constexpr float MINIMUM_LOAD_THRESHOLD = 1000.0f;

// Multiplier for noise level to set load detection threshold during automatic/manual calibration.
constexpr int UNLOAD_CHECK_COUNT = 3;

// Tare weight of the empty twenty lb tank in pounds, used to subtract from readings to report only the weight of the propane.
constexpr float DEF_TWENTY_TANK_TARE = 17.0f;

// maximum legal propane weight (lbs) of 20 lb tank is 80% of tank capacity = 16.0 lbs
constexpr float MAX_TWENTY_PROPANE_LBS = 16.0f;

// Weight of the always-present platen in pounds.
constexpr float  PLATEN_TARE = 0.33125f;

// EEPROM addresses and magic number for calibration persistence.
constexpr uint32_t CAL_EEPROM_MAGIC = 0x43414C31;  // "CAL1"

// EEPROM addresses for calibration data. Magic number is stored at CAL_EEPROM_MAGIC_ADDR, and the calibration factor is stored at CAL_EEPROM_VALUE_ADDR.
constexpr int CAL_EEPROM_MAGIC_ADDR = 0;

// HX711 instance for interacting with the load cell amplifier.
constexpr int CAL_EEPROM_VALUE_ADDR = 4;

// EEPROM size in bytes. Must be sufficient to store the calibration magic number and value.
constexpr int EEPROM_SIZE_BYTES = 64;


#endif // CONFIG_H

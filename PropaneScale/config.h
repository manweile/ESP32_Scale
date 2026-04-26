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
// constexpr float CAL_KNOWN_WEIGHT_LBS = 19.2f;   // wire spool good for 20 lb tanks
constexpr float CAL_KNOWN_WEIGHT_LBS = 36.8f;   // kitty litter jug of water good for 30 lb tanks
// constexpr float CAL_KNOWN_WEIGHT_LBS = 50.11f;   // 5 gallon camping water container

// Calibration value used by HX711 conversion. Adjustable at runtime by automatic or manual calibration mode.
// constexpr float DEF_CALIBRATION_FACTOR = -11551.08f;    // wire spool @ 17.4 lbs
constexpr float DEF_CALIBRATION_FACTOR = -10422.95f;    // litter jug of water @ 36.8 lbs
// constexpr float DEF_CALIBRATION_FACTOR = -10651.67f;    // 5 gallon camping water @ 50.11

// Number of readings to average when checking for stable load placement during automatic calibration.
constexpr int LOADED_CHECK_COUNT = 5;

// Number of readings to average when checking for stable no-load condition during automatic/manual calibration.
constexpr float MINIMUM_LOAD_THRESHOLD = 1000.0f;

// Multiplier for noise level to set load detection threshold during automatic/manual calibration.
constexpr int UNLOAD_CHECK_COUNT = 3;

// Tare weight of the empty, used to subtract from readings to report only the weight of the propane
// Tare weight of the empty twenty lb tank in pounds is approximately 16-18 lbs
// Tare weight of the empty thirty lb tank in pounds is typically 23-26 lbs
// constexpr float DEF_TANK_TARE = 17.0f;
constexpr float DEF_TANK_TARE = 23.5f;

// Maximum legal propane weight for a tank is 80% of the tank's total capacity
// maximum legal propane weight of 20 lb tank is 80% of tank capacity = 16.0 lbs
// maximum legal propane weight of 30 lb tank is 80% of tank capacity = 24.0 lbs
// constexpr float MAX_PROPANE_LBS = 16.0f;
constexpr float MAX_PROPANE_LBS = 24.0f;

// Weight of the always-present platen in pounds.
constexpr float  PLATEN_TARE = 0.33125f;

// EEPROM addresses and magic number for calibration persistence.
constexpr uint32_t CAL_EEPROM_MAGIC = 0x43414C31;  // "CAL1"
constexpr int CAL_EEPROM_MAGIC_ADDR = 0;
constexpr int CAL_EEPROM_VALUE_ADDR = 4;

// EEPROM addresses and magic number for propane tank tare persistence.
constexpr uint32_t TARE_EEPROM_MAGIC = 0x54415245;  // "TARE"
constexpr int TARE_EEPROM_MAGIC_ADDR = 8;
constexpr int TARE_EEPROM_VALUE_ADDR = 12;

// EEPROM addresses and magic number for maximum legal propane weight persistence.
constexpr uint32_t MAX_PROPANE_EEPROM_MAGIC = 0x4D415850;  // "MAXP"
constexpr int MAX_PROPANE_EEPROM_MAGIC_ADDR = 16;
constexpr int MAX_PROPANE_EEPROM_VALUE_ADDR = 20;

// EEPROM size in bytes. Must be sufficient to store all calibration and tare values with their magic numbers.
constexpr int EEPROM_SIZE_BYTES = 64;

#endif // CONFIG_H

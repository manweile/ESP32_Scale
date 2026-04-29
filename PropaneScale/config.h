#ifndef CONFIG_H
#define CONFIG_H

/**
 * ESP32 Hardware Constants
 */

// HX711 wiring pins on the ESP32.
constexpr int CLK_PIN = 4;              // Clock pin can be changed to any other GPIO pin if needed
constexpr int DOUT_PIN = 16;            // Data output pin can be changed to any other GPIO pin if needed

/**
 * Serial Communication Constants
 */

// serial connection speed
constexpr long BAUD = 115200;

/**
 * Sampling Constants
 */

// Number of samples to average during calibration reads (~500 ms at 10 SPS).
constexpr int CAL_SAMPLES = 5;

// Number of samples to average for live weight readings (~1s at 10 SPS).
constexpr int LIVE_SAMPLES = 10;

/**
 * Calibration Constants
 */

// @todo normalize name to DEF_KNOWN_WEIGHT_LBS
// Known reference weight used during automatic & manual calibration mode (pounds).
// constexpr float CAL_KNOWN_WEIGHT_LBS = 19.2f;   // wire spool good for 20 lb tanks
constexpr float CAL_KNOWN_WEIGHT_LBS = 36.8f;   // kitty litter jug of water good for 30 lb tanks
// constexpr float CAL_KNOWN_WEIGHT_LBS = 50.11f;   // 5 gallon camping water container

// Calibration value used by HX711 conversion. 
// Adjustable at runtime by automatic & manual calibration mode.
// constexpr float DEF_CALIBRATION_FACTOR = -11551.08f;    // wire spool @ 17.4 lbs
constexpr float DEF_CALIBRATION_FACTOR = -10422.95f;    // litter jug of water @ 36.8 lbs
// constexpr float DEF_CALIBRATION_FACTOR = -10651.67f;    // 5 gallon camping water @ 50.11

// Number of readings to average when checking for stable no-load condition during automatic/manual calibration.
constexpr float MINIMUM_LOAD_THRESHOLD = 1000.0f;

// Minimum load in pounds used to detect tank placement in displayCurrentPropaneReadings.
// Calibrated readings are in pounds so MINIMUM_LOAD_THRESHOLD (raw ADC units) cannot be used here.
constexpr float PLACED_LOAD_THRESHOLD_LBS = 1.0f;

// Multiplier for noise level to set load detection threshold during automatic/manual calibration.
constexpr int UNLOAD_CHECK_COUNT = 3;

/**
 * Startup empty-scale detection behavior.
 */

constexpr unsigned long SETUP_EMPTY_MAX_WAIT_MS = 15000UL;
constexpr int SETUP_EMPTY_REQUIRED_STABLE_CHECKS = 3;
constexpr float SETUP_EMPTY_TOLERANCE_LBS = 1.5f;

/**
 * User Interaction Constants
 */

constexpr unsigned long USER_CONFIRMATION_TIMEOUT_MS = 30000UL;

/**
 * Tank Constants
 */

// Tare weight of the empty, used to subtract from readings to report only the weight of the propane
// Tare weight of the empty twenty lb tank in pounds is approximately 16-18 lbs
// Tare weight of the empty thirty lb tank in pounds is typically 23-26 lbs
// constexpr float DEF_TANK_TARE = 17.0f;
constexpr float DEF_TANK_TARE = 23.5f;

// @todo normalize name to DEF_MAX_PROPANE_LBS
// Maximum legal propane weight for a tank is 80% of the tank's total capacity
// maximum legal propane weight of 20 lb tank is 80% of tank capacity = 16.0 lbs
// maximum legal propane weight of 30 lb tank is 80% of tank capacity = 24.0 lbs
// constexpr float MAX_PROPANE_LBS = 16.0f;
constexpr float MAX_PROPANE_LBS = 24.0f;

// Weight of the always-present platen in pounds.
constexpr float  PLATEN_TARE = 0.33125f;

/**
 * EEPROM constants
 */

// EEPROM addresses and magic number for calibration persistence.
constexpr uint32_t CAL_EEPROM_MAGIC = 0x43414C31;           // "CAL1" magic number is to indicate valid calibration factor stored in EEPROM
constexpr int CAL_EEPROM_MAGIC_ADDR = 0;                    // Calibration factor CAL1 is stored as 4 byte float
constexpr int CAL_EEPROM_VALUE_ADDR = 4;                    // value address starts at byte 4, immediately after the magic number

// EEPROM addresses and magic number for propane tank tare persistence.
constexpr uint32_t TARE_EEPROM_MAGIC = 0x54415245;          // "TARE" magic number is to indicate valid tank tare weight stored in EEPROM
constexpr int TARE_EEPROM_MAGIC_ADDR = 8;                   // Tank tare TARE is stored as a 4 byte float
constexpr int TARE_EEPROM_VALUE_ADDR = 12;                  // value address starts at byte 12, immediately after the magic number

// EEPROM addresses and magic number for maximum legal propane weight persistence.
constexpr uint32_t MAX_PROPANE_EEPROM_MAGIC = 0x4D415850;   // "MAXP" magic number is to indicate valid max propane weight stored in EEPROM
constexpr int MAX_PROPANE_EEPROM_MAGIC_ADDR = 16;           // Max propane weight MAXP is stored as a 4 byte float
constexpr int MAX_PROPANE_EEPROM_VALUE_ADDR = 20;           // value address starts at byte 20, immediately after the magic number

// EEPROM size in bytes. Must be sufficient to store all calibration and tare values with their magic numbers.
constexpr int EEPROM_SIZE_BYTES = 64;

#endif // CONFIG_H

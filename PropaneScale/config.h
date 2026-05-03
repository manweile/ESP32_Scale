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
 * Calibration & Startup Constants
 */

// Known reference weight used during automatic & manual calibration mode (pounds).
// wire spool @19.2 lbs good for almost empty 20 lb tank
// collapsible water jug in milk crate @ 26.0 lbs (h20 @ 23 lbs + milk crate @ 3 lbs) perfect for half full 20 lb tank
// kitty litter jug of water at 36.8 lbs perfect for half full 30 lb tank
constexpr float DEF_KNOWN_WEIGHT = 26.0f;

// Calibration value used by HX711 conversion. 
// Adjustable at runtime by automatic & manual calibration mode.
// constexpr float DEF_CALIBRATION_FACTOR = -11551.08f;     // wire spool @ 17.4 lbs
constexpr float DEF_CALIBRATION_FACTOR = -10434.32f;        // collapsible water jug in milk crate @ 26.0 lbs
// constexpr float DEF_CALIBRATION_FACTOR = -10422.95f;     // kitty litter jug of water @ 36.8 lbs

// Maximum time to wait for load placement during startup and calibration before giving up and returning to idle state.
constexpr unsigned long EMPTY_CONFIRM_TIMEOUT = 15000UL;

// Settle delay in milliseconds after load detection before taking a calibration measurement.
// Prevents mid-placement reads caused by the weight still moving when the threshold is first crossed.
constexpr unsigned long CAL_SETTLE_DELAY_MS = 2000UL;

// Number of readings to average when checking for stable no-load condition during automatic/manual calibration.
constexpr float MINIMUM_LOAD_THRESHOLD = 1000.0f;

// Minimum load in pounds used to detect tank placement in displayCurrentPropaneReadings.
// Calibrated readings are in pounds so MINIMUM_LOAD_THRESHOLD (raw ADC units) cannot be used here.
constexpr float MINIMUM_LOAD_WEIGHT = 1.0f;

// Tolerance in pounds for detecting stable empty condition during startup calibration.
constexpr float SETUP_EMPTY_WEIGHT = 1.5f;

// Shared sample/check count used for unloaded averaging and stable-empty confirmation checks.
constexpr int UNLOAD_CHECK_COUNT = 3;

// Time to wait for user confirmation during calibration workflows before auto-confirming with stable readings or cancelling if not stable.
constexpr unsigned long USER_CONFIRM_TIMEOUT = 20000UL;

/**
 * Tank Constants
 */

//  @todo if there is a 7 lb change in tank tare weight,
// warn user they will need to update max legal propane weight
// and known weight for calibration to maintain accuracy
// and do a recalibration to update the calibration factor

// Tare weight of the empty, used to subtract from readings to report only the weight of the propane
// Tare weight of the empty twenty lb tank in pounds is approximately 16-19 lbs
// Tare weight of the empty thirty lb tank in pounds is typically 23-26 lbs
constexpr float DEF_TANK_TARE = 18.6f;

// Maximum legal propane weight for a tank is 80% of the tank's total capacity
// maximum legal propane weight of 20 lb tank is 80% of tank capacity = 16.0 lbs
// maximum legal propane weight of 30 lb tank is 80% of tank capacity = 24.0 lbs
constexpr float DEF_MAX_PROPANE = 16.0f;

/**
 * Scale Physical Components Constants  
 */

// Weight of the always-present platen in pounds.
constexpr float  PLATEN_TARE = 0.33125f;

/**
 * EEPROM constants
 */

// EEPROM addresses and magic number for calibration persistence.
constexpr uint32_t CAL_EEPROM_MAGIC = 0x43414C31;           // "CAL1" magic number is to indicate valid calibration factor stored in EEPROM
constexpr int CAL_EEPROM_MAGIC_ADDR = 0;                    // Calibration factor CAL1 is stored as 4 byte float
constexpr int CAL_EEPROM_VALUE_ADDR = 4;                    // value address starts at byte 4, immediately after the magic number

// EEPROM addresses and magic number for known weight persistence
constexpr uint32_t KNOWN_WEIGHT_EEPROM_MAGIC = 0x4B4E5731;  // "KNW1" magic number is to indicate valid known weight stored in EEPROM
constexpr int KNOWN_WEIGHT_EEPROM_MAGIC_ADDR = 8;           // Known weight KNW1 is stored as 4 byte float
constexpr int KNOWN_WEIGHT_EEPROM_VALUE_ADDR = 12;          // value address starts at byte 12, immediately after the magic number

// EEPROM addresses and magic number for maximum legal propane weight persistence.
constexpr uint32_t MAX_PROPANE_EEPROM_MAGIC = 0x4D415850;   // "MAXP" magic number is to indicate valid max propane weight stored in EEPROM
constexpr int MAX_PROPANE_EEPROM_MAGIC_ADDR = 16;           // Max propane weight MAXP is stored as a 4 byte float
constexpr int MAX_PROPANE_EEPROM_VALUE_ADDR = 20;           // value address starts at byte 20, immediately after the magic number

// EEPROM addresses and magic number for propane tank tare persistence.
constexpr uint32_t TARE_EEPROM_MAGIC = 0x54415245;          // "TARE" magic number is to indicate valid tank tare weight stored in EEPROM
constexpr int TARE_EEPROM_MAGIC_ADDR = 24;                  // Tank tare TARE is stored as a 4 byte float
constexpr int TARE_EEPROM_VALUE_ADDR = 28;                  // value address starts at byte 28, immediately after the magic number

// EEPROM size in bytes. Must be sufficient to store all calibration and tare values with their magic numbers.
constexpr int EEPROM_SIZE_BYTES = 64;

#endif // CONFIG_H

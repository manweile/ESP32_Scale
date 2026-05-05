#ifndef CONFIG_H
#define CONFIG_H

/**
 * @section ESP32 Hardware Constants
 */

// HX711 wiring pins on the ESP32.
constexpr int CLK_PIN = 4;              // Clock pin can be changed to any other GPIO pin if needed
constexpr int DOUT_PIN = 16;            // Data output pin can be changed to any other GPIO pin if needed

/**
 * @section Serial Communication Constants
 */

// serial connection speed
constexpr long BAUD = 115200;

/**
 * @section Sampling Constants
 */


// Number of samples to average during calibration reads (~500 ms at 10 SPS).
constexpr int CAL_SAMPLES = 5;

// Number of samples to average for live weight readings (~1s at 10 SPS).
constexpr int LIVE_SAMPLES = 10;

// Single sample used in polling loops (WAIT_EMPTY, WAIT_LOAD, WAIT_STABLE) to keep the
// main loop responsive. Detection only — no accuracy needed.
constexpr int POLL_SAMPLES = 1;

// Shared sample/check count used for unloaded averaging and stable-empty confirmation checks.
constexpr int UNLOAD_CHECK_COUNT = 3;

/**
 * @section Calibration & Startup Constants
 */

constexpr float CHANGE_WARN_PCT = 0.25f;                    /**< Threshold for significant change in tank tare or max propane weight warning */

// wire spool at 19.2 lbs: -11551.08f;
// collapsible water jug in milk crate at 26.0 lbs: -10420.86f;
// kitty litter jug of water at 36.8 lbs: -10422.95f
constexpr float DEF_CALIBRATION_FACTOR = -10420.86f;        /**< Calibration factor used by HX711 conversion */

// wire spool  at 19.2 lbs good for almost empty 20 lb tank
// collapsible water jug in milk crate at 26.0 lbs (h20 @ 23 lbs + milk crate @ 3 lbs) perfect for half full 20 lb tank
// kitty litter jug of water at 36.8 lbs perfect for half full 30 lb tank
constexpr float DEF_KNOWN_WEIGHT = 26.0f;                   /**< Default known weight in pounds used for calibration if no valid value found in EEPROM */

// maximum legal propane weight of 20 lb tank: 20.0 x 0.80 = 16.0 lbs
// maximum legal propane weight of 30 lb tank: 30.0 x 0.80 = 24.0 lbs
constexpr float DEF_MAX_PROPANE = 16.0f;                    /**< Default maximum legal propane lbs is 80% of tank capacity */

// Tare weight of the empty twenty lb tank in pounds is approximately 16-19 lbs
// Tare weight of the empty thirty lb tank in pounds is typically 23-26 lbs
constexpr float DEF_TANK_TARE = 17.5f;                      /**< Default tare weight an empty tank in pounds, subtract from readings to report weight of propane */

constexpr float MINIMUM_LOAD_WEIGHT = 1.0f;                 /**< Minimum load in pounds to detect tank placement during level read workflow */

constexpr float MINIMUM_LOAD_THRESHOLD = 1000.0f;           /**< Minimum load threshold in raw ADC units for stable no-load detection */

/**
 * @section Timing Constants for Non-blocking Workflows
 */

// Settle delay in milliseconds after load detection before taking a calibration measurement.
// Prevents mid-placement reads caused by the weight still moving when the threshold is first crossed.
constexpr unsigned long CAL_SETTLE_DELAY_MS = 2000UL;

// Maximum time to wait for load placement during startup and calibration before giving up and returning to idle state.
constexpr unsigned long EMPTY_CONFIRM_TIMEOUT_MS = 15000UL;

// Maximum time to wait for the HX711 to become ready during blocking reads
constexpr unsigned long HX711_READY_TIMEOUT_MS = 120UL;

// Time to wait for user confirmation during calibration workflows before auto-confirming with stable readings or cancelling if not stable.
constexpr unsigned long USER_CONFIRM_TIMEOUT_MS = 20000UL;

/**
 * @section Startup Tare Constants
 */

// Tolerance in pounds for detecting stable empty condition during startup calibration.
constexpr float SETUP_EMPTY_WEIGHT = 1.5f;

/**
 * @section Scale Physical Components Constants
 */

// Weight of the always-present platen in pounds.
constexpr float  PLATEN_TARE = 0.33125f;                    /**< Tare weight of the scale platen in pounds, subtract from readings to report weight of the load */

/** 
 * @section UI String Constants
 */

constexpr char APP_TITLE[] = "Propane Level Scale";
constexpr char CALIBRATION_SAVE_FAILURE_MSG[] = "Failure saving default calibration to EEPROM.";
constexpr char CALIBRATION_SAVE_SUCCESS_MSG[] = "Success saving default calibration to EEPROM.";
constexpr char CMD_AUTO_CAL_MSG[] = "Send 'a' to enter automatic calibration mode";
constexpr char CMD_CURRENT_VALUES_MSG[] = "Send 'c' to print current runtime values";
constexpr char CMD_DEFAULT_EEPROM_MSG[] = "Send 'd' to reset EEPROM to default values";
constexpr char CMD_EEPROM_MSG[] = "Send 'e' to display saved EEPROM values";
constexpr char CMD_HELP_MSG[] = "Send 'h' to display this help menu";
constexpr char CMD_KNOWN_WEIGHT_MSG[] = "Send 'k' to enter known weight value for calibration mode";
constexpr char CMD_LEVEL_MSG[] = "Send 'l' to display one liquid propane percent level reading";
constexpr char CMD_MANUAL_CAL_MSG[] = "Send 'm' to enter manual calibration mode";
constexpr char CMD_PROPANE_WEIGHT_MSG[] = "Send 'p' to set maximum legal propane weight";
constexpr char CMD_REZERO_MSG[] = "Send 'r' to re-zero scale with no propane weight on it";
constexpr char CMD_TANK_TARE_MSG[] = "Send 't' to set propane tank tare";

/**
 * @section EEPROM constants
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

/**
 * @section EEPROM Sanity Limit Constants
 */

constexpr float CAL_FACTOR_ABS_MAX = 500000.0f;             // Maximum absolute value for valid calibration factor 
constexpr float CAL_FACTOR_ABS_MIN = 100.0f;                // Minimum absolute value for valid calibration factor 
constexpr float MAX_PROJECT_WEIGHT = 60.0f;                 // Project will never measure a propane tank above nominal 60 lbs
constexpr float MIN_PLAUSIBLE_WEIGHT = 0.1f;                // Minimum plausible non-zero weight for user-entered values

#endif // CONFIG_H

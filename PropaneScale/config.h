/**
 * @file config.h
 * @author Gerald Manweiler
 *
 * @brief Configuration constants for the ESP32-based propane level scale.
 *
 * @details Defines constants used throughout the PropaneScale application.
 *
 * @version 0.1
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026 Gerald Manweiler
 *
 */

#pragma once

// ESP32 Hardware Constants
constexpr int CLK_PIN  = 17;                                /**< Clock pin can be changed to any other GPIO pin if needed */
constexpr int DOUT_PIN = 16;                                /**< Data output pin can be changed to any other GPIO pin if needed */

// Serial Communication Constants
constexpr long BAUD = 115200;                               /**< Serial connection speed in bits per second */

// Sampling Constants
constexpr int AVG_SAMPLES        = 1;                       /**< Single sample used in polling loops for detection only */
constexpr int CAL_SAMPLES        = 5;                       /**< Number of samples to average for calibration readings */
constexpr int LIVE_SAMPLES       = 10;                      /**< Number of samples to average for live weight readings (~1s at 10 SPS) */
constexpr int UNLOAD_CHECK_COUNT = 3;                       /**< Shared sample/check count for unloaded averaging and stable-empty confirmation */

// Workflow Constants
constexpr float CHANGE_WARN_PCT = 0.25f;                    /**< Threshold for significant change in tank tare or max propane weight warning */

// wire spool @ 19.2 lbs: -11551.08f;
// half full water jug/milk crate @ 26.0 lbs: -10420.86f;
// kitty litter jug of water @ 36.8 lbs: -10422.95f
constexpr float DEF_CALIBRATION_FACTOR = -10420.86f;        /**< Calibration factor used by HX711 conversion */

// wire spool @ 19.2 lbs good for almost empty 20 lb tank
// half full water jug/milk crate @ 26.0 lbs (h20 @ 23 lbs + milk crate @ 3 lbs) perfect for half full 20 lb tank
// kitty litter jug of water @ 36.8 lbs perfect for half full 30 lb tank
constexpr float DEF_KNOWN_WEIGHT = 26.0f;                   /**< Default known weight lbs for calibration if no valid value found in EEPROM */

// 20 lb tank legal propane weight: 20.0 x 0.80 = 16.0 lbs
// 30 lb tank legal propane weight: 30.0 x 0.80 = 24.0 lbs
constexpr float DEF_MAX_PROPANE = 16.0f;                    /**< Default maximum legal propane lbs is 80% of tank capacity */

// Tare weight of the empty twenty lb tank in pounds is approximately 16-19 lbs
// Tare weight of the empty thirty lb tank in pounds is typically 23-26 lbs
constexpr float DEF_TANK_TARE = 17.5f;                      /**< Default tare lbs of empty tank, subtract from readings for weight of propane */

constexpr float MINIMUM_LOAD_WEIGHT = 1.0f;                 /**< Minimum load in lbs to detect tank placement during level read workflow */

// Non-blocking Timing Constants
constexpr unsigned long CAL_SETTLE_DELAY_MS = 5000UL;       /**< Time to wait for load to mechanically settle before taking calibration reading */
constexpr unsigned long CONFIRM_TIMEOUT_MS  = 15000UL;      /**< Wait time for user &auto confirmations during startup tare workflow/calibration */
constexpr unsigned long POLL_TIMEOUT_MS     = 1000UL;       /**< Maximum time to wait for the HX711 to become ready during polling */
constexpr unsigned long READY_TIMEOUT_MS    = 100UL;        /**< Maximum time to wait for the HX711 to become ready during blocking reads */
constexpr unsigned long STARTUP_TIMEOUT_MS  = 1500UL;       /**< Maximum time to wait for stable readings during startup tare workflow */

// Startup Tare Constants
constexpr float SETUP_EMPTY_WEIGHT           = 1.5f;        /**< Tolerance in lbs for detecting stable empty condition during startup calibration */
constexpr float STARTUP_NOT_EMPTY_MARGIN_LBS = 2.0f;        /**< Margin above configured full-tank weight when deciding startup not-empty condition */

// Scale Physical Components Constants
constexpr float  PLATEN_TARE = 0.33125f;                    /**< Tare lbs of the scale platen in pounds */

// UI String Constants
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

// EEPROM constants

// Calibration persistence
constexpr uint32_t CAL_EEPROM_MAGIC = 0x43414C31;           /**< "CAL1" magic number is to indicate valid calibration factor stored in EEPROM */
constexpr int CAL_EEPROM_MAGIC_ADDR = 0;                    /**< Calibration factor CAL1 is stored as 4 byte float */
constexpr int CAL_EEPROM_VALUE_ADDR = 4;                    /**< value address starts at byte 4, immediately after the magic number */

// Known weight persistence
constexpr uint32_t KNOWN_WEIGHT_EEPROM_MAGIC = 0x4B4E5731;  /**< "KNW1" magic number is to indicate valid known weight stored in EEPROM */
constexpr int KNOWN_WEIGHT_EEPROM_MAGIC_ADDR = 8;           /**< Known weight KNW1 is stored as 4 byte float */
constexpr int KNOWN_WEIGHT_EEPROM_VALUE_ADDR = 12;          /**< value address starts at byte 12, immediately after the magic number */

// Maximum legal propane weight persistence
constexpr uint32_t MAX_PROPANE_EEPROM_MAGIC = 0x4D415850;   /**< "MAXP" magic number is to indicate valid max propane weight stored in EEPROM */
constexpr int MAX_PROPANE_EEPROM_MAGIC_ADDR = 16;           /**< Max propane weight MAXP is stored as a 4 byte float */
constexpr int MAX_PROPANE_EEPROM_VALUE_ADDR = 20;           /**< value address starts at byte 20, immediately after the magic number */

// Propane tank tare persistence
constexpr uint32_t TARE_EEPROM_MAGIC = 0x54415245;          /**< "TARE" magic number is to indicate valid tank tare weight stored in EEPROM */
constexpr int TARE_EEPROM_MAGIC_ADDR = 24;                  /**< Tank tare TARE is stored as a 4 byte float */
constexpr int TARE_EEPROM_VALUE_ADDR = 28;                  /**< value address starts at byte 28, immediately after the magic number */

// HX711 runtime tare offset persistence
constexpr uint32_t HX711_OFFSET_EEPROM_MAGIC = 0x4F464653;  /**< "OFFS" magic number indicates saved runtime tare offset */
constexpr int HX711_OFFSET_EEPROM_MAGIC_ADDR = 32;          /**< Runtime offset magic stored as 4-byte uint32 */
constexpr int HX711_OFFSET_EEPROM_VALUE_ADDR = 36;          /**< Runtime offset value stored as 4-byte float (integer-compatible range) */

constexpr int EEPROM_SIZE_BYTES = 64;                       /**< EEPROM storage; must be >= highest value address + 4 bytes for the value */

// EEPROM Sanity Limit Constants
constexpr float CAL_FACTOR_ABS_MAX   = 500000.0f;           /**< Maximum absolute value for valid calibration factor */
constexpr float CAL_FACTOR_ABS_MIN   = 100.0f;              /**< Minimum absolute value for valid calibration factor */
constexpr float MAX_PROJECT_WEIGHT   = 60.0f;               /**< Project will never measure a propane tank above nominal 60 lbs */
constexpr float MIN_PLAUSIBLE_WEIGHT = 0.1f;                /**< Minimum plausible non-zero weight for user-entered values */
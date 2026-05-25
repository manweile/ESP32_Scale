/**
 * @file workflows_contexts.h
 * @author Gerald Manweiler
 *
 * @brief Header file for context definitions of non-blocking workflows in the propane scale project.
 *
 * @details Defines enums and structs for managing the state of non-blocking workflows such as calibration, level reading, and user input collection. Also declares global instances of these contexts used across the application.
 *
 * @version 0.1
 * @date 2026-05-07
 * @copyright Copyright (c) 2026 Gerald Manweiler
 */

#pragma once

// Standard library headers
#include <cstdint>

// Non-blocking Enums and Structs

/**
 * @enum AvgPhase
 *
 * @brief Named phases for the non-blocking averaging helper used during calibration.
 *
 * @details Used to manage the multi-step averaging process during calibration without blocking.
 */
enum class AvgPhase : uint8_t {
  NONE          = 0,                                        /**< No non-blocking average in progress */
  EMPTY_CONFIRM = 1,                                        /**< Empty-scale confirmation averaging in progress */
  LOAD_DETECT   = 2,                                        /**< Load placement detection averaging in progress */
  FINAL_MEAS    = 3,                                        /**< Final calibration measurement averaging in progress */
  VERIFICATION  = 4                                         /**< Verification averaging in progress */
};

/**
 * @struct AvgContext
 *
 * @brief Context for the non-blocking averaging helper used across workflows.
 *
 * @details Contains variables to manage the state of a non-blocking averaging operation.
 */
struct AvgContext {
  bool   active            = false;                         /**< Whether a non-blocking operation is active */
  int    collected         = 0;                             /**< Number of readings collected so far */
  int    index             = 0;                             /**< Current reading index */
  float  minimumThreshold  = 0.0f;                          /**< Minimum threshold floor (kept for layout compatibility) */
  int    requestedReadings = 0;                             /**< Number of readings requested (outer loop) */
  int    samplesPerReading = 0;                             /**< Samples per averaged reading */
  float  total             = 0.0f;                          /**< Accumulated total of readings */
};

/**
 * @struct ProbeContext
 * 
 * @brief Context for probing the HX711 signal during scale ready checks.
 * 
 * @details Contains variables to manage the state of a non-blocking response check of the HX711.
 */
struct ProbeContext {
  bool           active         = false;                    /**< Whether a probe operation is active */
  int            index          = 0;                        /**< Current probe reading index */
  long           maxRaw         = 0;                        /**< Maximum raw value observed during probe */
  long           minRaw         = 0;                        /**< Minimum raw value observed during probe */
  int            samplesTaken   = 0;                        /**< Number of probe samples taken */
  unsigned long  startMs        = 0;                        /**< millis() when probe was started */
  int            targetSamples  = 0;                        /**< configured number of samples to collect for the probe */
  unsigned long  timeoutMs      = 0;                        /**< configured timeout for the probe */
};

// Calibration Enums and Structs

/**
 * @enum CalMode
 *
 * @brief Enumeration of calibration modes.
 *
 * @details Used to manage different calibration workflows initiated by the user.
 */
enum class CalMode  : uint8_t {
  NONE   = 0,                                               /**< No calibration mode active */
  AUTO   = 1,                                               /**< Automatic calibration mode */
  MANUAL = 2,                                               /**< Manual calibration mode */
  REZERO = 3                                                /**< Rezero calibration mode */
};

/**
 * @enum CalState
 *
 * @brief Enumeration of states for calibration workflows.
 *
 * @details Used to manage multi-step calibration processes and user prompts during calibration.
 */
enum class CalState : uint8_t {
  IDLE       = 0,                                           /**< Not currently in a calibration workflow, waiting for user input to start one */
  WAIT_EMPTY = 1,                                           /**< Waiting for the scale to be empty before starting calibration */
  WAIT_LOAD  = 2,                                           /**< Waiting for a known weight to be placed on the scale */
  SETTLING   = 3,                                           /**< Waiting for the placed weight to mechanically settle before measuring */
  ADJUSTING  = 4                                            /**< Adjusting the calibration factor based on user input or measurements */
};

/**
 * @struct CalContext
 *
 * @brief Struct to hold state for calibration workflows.
 *
 * @details Contains variables that allow managing complex calibration workflows.
 */
struct CalContext {
  float         adjustmentStep            = 0.0f;           /**< manual mode: current factor nudge size */
  AvgPhase      avgPhase                  = AvgPhase::NONE; /**< internal: non-blocking averaging phase for AUTO calibration */
  bool          hasManualDisplay          = false;          /**< manual mode: whether we have a prior display snapshot to compare against */
  int           lastDirection             = 0;              /**< manual mode: +1 = last press +, -1 = last press - */
  int           lastFactorHundredth       = 0;              /**< manual mode: last displayed factor, scaled by 100 (2 decimal places) */
  int           lastReadingTenth          = 0;              /**< manual mode: last displayed reading, scaled by 10 (1 decimal place) */
  int           lastStepTenThousandth     = 0;              /**< manual mode: last displayed step, scaled by 10000 (4 decimal places) */
  int           loadDetectChecks          = 0;              /**< calibration mode: consecutive above-threshold load detection checks */
  float         loadDetectThreshold       = 0.0f;           /**< noise-derived threshold for weight detection */
  float         measuredUnits             = 0.0f;           /**< last averaged reading in pounds */
  float         minStep                   = 0.0f;           /**< manual mode: floor for adjustmentStep */
  CalMode       mode                      = CalMode::NONE;  /**< current calibration mode, or NONE when not in a calibration workflow */
  float         originalCalibrationFactor = 0.0f;           /**< manual mode: calibration factor captured at start for cancel/restore */
  CalState      state                     = CalState::IDLE; /**< current state within the calibration workflow */
  unsigned long stateStartMs              = 0;              /**< millis() when current state was entered */
  bool          thresholdPending          = false;          /**< whether an async threshold computation is pending */
  unsigned long thresholdStartMs          = 0;              /**< millis() when async threshold computation was started */
  bool          probePending              = false;          /**< whether a non-blocking HX711 probe is pending for workflow start */
};

// Level Enums and Structs


/**
 * @enum LevelState
 *
 * @brief Enumeration of states for the liquid level read workflow.
 *
 * @details Used to manage the non-blocking tank detection and measurement sequence.
 */
enum class LevelState : uint8_t {
  IDLE      = 0,                                            /**< No active level read workflow */
  WAIT_LOAD = 1,                                            /**< Polling scale until tank weight exceeds detection threshold */
  SETTLING  = 2,                                            /**< Load detected; waiting for placement motion to settle */
  READING   = 3                                             /**< Load settled; taking final averaged measurement */
};

/**
 * @struct LevelContext
 *
 * @brief Persistent context for the non-blocking liquid level read workflow.
 *
 * @details Stores the detection threshold, start timestamp, and current state
 * so each loop() tick can advance the workflow without blocking.
 */
struct LevelContext {
  bool          avgPending          = false;                /**< whether a non-blocking average is in progress for this workflow */  
  float         loadDetectThreshold = 0.0f;                 /**< noise-derived threshold used to detect tank placement */
  LevelState    state               = LevelState::IDLE;     /**< current state within the level read workflow */
  unsigned long stateStartMs        = 0;                    /**< millis() when WAIT_LOAD state was entered */
  bool          thresholdPending    = false;                /**< whether an async threshold computation is pending */
  unsigned long thresholdStartMs    = 0;                    /**< millis() when async threshold computation was started */
  bool          probePending        = false;                /**< whether a non-blocking HX711 probe is pending for workflow start */
};

// Tare Enums and Structs

/**
 * @enum TareState
 *
 * @brief Enumeration of states for the startup tare workflow.
 *
 * @details Used to manage the non-blocking startup empty-scale detection and tare sequence.
 */
enum class TareState : uint8_t {
  IDLE        = 0,                                          /**< No active startup tare workflow */
  WAIT_STABLE = 1,                                          /**< Polling scale for stable empty reading, or user 'q' to skip */
  TARE        = 2,                                          /**< Stable empty confirmed; apply tare and finish */
  SKIP        = 3                                           /**< User skipped or scale unstable at timeout; skip tare */
};

/**
 * @struct TareContext
 *
 * @brief Persistent context for the non-blocking startup tare workflow.
 *
 * @details Stores the baseline reading, stable check counter, start timestamp, and current
 * state so each loop() tick can advance the workflow without blocking.
 */
struct TareContext {
  float         baseline         = 0.0f;                    /**< Initial scale reading used as the stability reference */
  bool          baselinePending  = false;                   /**< baseline averaging requested and in progress */
  int           baselineReadings = 0;                       /**< requested outer readings for baseline */
  int           baselineSamples  = 0;                       /**< samples per reading for baseline */
  bool          probePending     = false;                   /**< whether a non-blocking HX711 probe is pending for workflow start */
  int           stableChecks     = 0;                       /**< Consecutive readings within tolerance of baseline */
  TareState     state            = TareState::IDLE;         /**< Current state within the workflow */
  unsigned long stateStartMs     = 0;                       /**< millis() when WAIT_STABLE state was entered */
};

// External global State Variables
extern AvgContext avgCtx;                                   /**< Averaging context instance to hold state for non-blocking computations */
extern CalContext calCtx;                                   /**< Calibration context instance to hold state for workflows */
extern LevelContext levelCtx;                               /**< Level read context instance to hold state for workflow */
extern TareContext tareCtx;                                 /**< Startup tare context instance to hold state for workflow */
extern AvgContext thresholdCtx;                             /**< Shared threshold averaging context instance (used by calibration and level workflows) */
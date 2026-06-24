/*
 * web_ble.cpp
 * NimBLE-based GATT service for PropaneScale with safe fallbacks when NimBLE
 * is not available. Implements characteristics for Weight, Tare, Calibrate,
 * Sampling Rate and a simple Auth char. Exposes consumable request accessors
 * for the main application.
 */

// Standard library headers
#include <Arduino.h>
#include <string.h>

// Third party library headers
#include <NimBLEDevice.h>                                   // NimBLE library for BLE functionality

// Local library headers
#include "web_ble.h"                                        // BLE interface declarations for the PropaneScale application

// UUID definitions (128-bit custom base)
static const char* SERVICE_UUID      = "0000FEED-0000-1000-8000-00805F9B34FB";  /**< UUID for PropaneScale service */
static const char* CHAR_WEIGHT_UUID  = "0000BE01-0000-1000-8000-00805F9B34FB";  /**< UUID for Weight characteristic */
static const char* CHAR_TARE_UUID    = "0000BE02-0000-1000-8000-00805F9B34FB";  /**< UUID for Tare characteristic */
static const char* CHAR_CAL_UUID     = "0000BE03-0000-1000-8000-00805F9B34FB";  /**< UUID for Calibrate characteristic */
static const char* CHAR_RAW_UUID     = "0000BE04-0000-1000-8000-00805F9B34FB";  /**< UUID for Raw data characteristic */
static const char* CHAR_RATE_UUID    = "0000BE05-0000-1000-8000-00805F9B34FB";  /**< UUID for Sampling Rate characteristic */
static const char* CHAR_BATT_UUID    = "0000BE06-0000-1000-8000-00805F9B34FB";  /**< UUID for Battery characteristic */
static const char* CHAR_DEVINFO_UUID = "0000BE07-0000-1000-8000-00805F9B34FB";  /**< UUID for Device Info characteristic */
static const char* CHAR_AUTH_UUID    = "0000BE08-0000-1000-8000-00805F9B34FB";  /**< UUID for Auth characteristic */

// Internal state
static NimBLECharacteristic* weightChar = nullptr;          /**< Weight characteristic */
static NimBLECharacteristic* tareChar   = nullptr;          /**< Tare characteristic */
static NimBLECharacteristic* calChar    = nullptr;          /**< Calibrate characteristic */
static NimBLECharacteristic* rateChar   = nullptr;          /**< Sampling Rate characteristic */
static NimBLECharacteristic* authChar   = nullptr;          /**< Auth characteristic */

static volatile bool bleConnected       = false;            /**< Flag indicating if a BLE client is currently connected */

// Command request flags (consumable by application)
static volatile bool tareRequested      = false;            /**< Flag indicating if a tare request has been made */
static volatile bool calibrateRequested = false;            /**< Flag indicating if a calibrate request has been made */
static volatile float calibrateMassLbs  = 0.0f;             /**< Mass in pounds for calibration */

// Notification state
static float lastWeightLbs               = 0.0f;            /**< Last reported weight in pounds */
static volatile bool lastWeightAvailable = false;           /**< Flag indicating if the last weight is available */
static uint16_t notifySeq                = 0;               /**< Notification sequence number */
static uint16_t samplingIntervalMs       = 100;             /**< Sampling interval in milliseconds */
static unsigned long lastNotifyMs        = 0;               /**< Timestamp of the last notification */

// Rate limit for tare (ms)
#define BLE_TARE_COOLDOWN_MS 1000                           /**< Cooldown period for tare requests in milliseconds */
static unsigned long lastTareMs = 0;                        /**< Timestamp of the last tare request */

// Simple auth flag set by writing 0xA5 to auth characteristic
static volatile bool clientAuthenticated = false;          /**< Flag indicating if the client is authenticated */


class PropaneBleCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    (void)pServer; (void)connInfo;
    bleConnected = true;
    Serial.println(F("[WEB-BLE] Client connected"));
  }

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    (void)pServer; (void)connInfo; (void)reason;
    bleConnected = false;
    clientAuthenticated = false;
    Serial.println(F("[WEB-BLE] Client disconnected"));
  }
};

class PropaneCharCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    std::string val = pChar->getValue();

    if (pChar == tareChar) {
      if (!clientAuthenticated) {
        Serial.println(F("[WEB-BLE] Tare write rejected: not authenticated"));
        return;
      }
      if (val.size() >= 1 && static_cast<uint8_t>(val[0]) == 0x01) {
        unsigned long now = millis();
        if (now - lastTareMs < BLE_TARE_COOLDOWN_MS) {
          Serial.println(F("[WEB-BLE] Tare write ignored: rate-limited"));
          return;
        }
        lastTareMs = now;
        tareRequested = true;
        Serial.println(F("[WEB-BLE] Tare requested via BLE"));
      }
      return;
    }

    if (pChar == calChar) {
      if (!clientAuthenticated) {
        Serial.println(F("[WEB-BLE] Calibrate write rejected: not authenticated"));
        return;
      }
      // Expect payload: [cmd:1][float32 mass_lbs:4][options:1]
      if (val.size() >= 5) {
        uint8_t cmd = static_cast<uint8_t>(val[0]);
        float mass = 0.0f;
        memcpy(&mass, &val[1], sizeof(float));
        if (cmd == 0x01) {
          calibrateMassLbs = mass;
          calibrateRequested = true;
          Serial.printf("[WEB-BLE] Calibrate requested via BLE: %.4f lbs\\n", mass);
        }
      } else {
        Serial.println(F("[WEB-BLE] Calibrate write malformed"));
      }
      return;
    }

    if (pChar == rateChar) {
      // Expect uint16 interval_ms
      if (val.size() >= 2) {
        uint16_t interval = (uint8_t)val[0] | ((uint8_t)val[1] << 8);
        if (interval < 20) interval = 20;
        if (interval > 2000) interval = 2000;
        samplingIntervalMs = interval;
        Serial.printf("[WEB-BLE] Sampling interval set to %u ms\\n", (unsigned)samplingIntervalMs);
      }
      return;
    }

    if (pChar == authChar) {
      // Simple auth: accept single byte 0xA5 to mark authenticated
      if (val.size() >= 1 && static_cast<uint8_t>(val[0]) == 0xA5) {
        clientAuthenticated = true;
        Serial.println(F("[WEB-BLE] Client authenticated"));
      } else {
        clientAuthenticated = false;
        Serial.println(F("[WEB-BLE] Client authentication failed"));
      }
      return;
    }
  }
};

// Helper to send weight notification payload (version + float32 + seq + reserved)
static void sendWeightNotify(float weight_lbs) {
  if (!weightChar) return;
  uint8_t buf[9];
  buf[0] = 0x01; // version
  memcpy(&buf[1], &weight_lbs, sizeof(float));
  buf[5] = static_cast<uint8_t>(notifySeq & 0xFF);
  buf[6] = static_cast<uint8_t>((notifySeq >> 8) & 0xFF);
  buf[7] = 0x00;
  buf[8] = 0x00;
  weightChar->setValue(buf, sizeof(buf));
  weightChar->notify();
  notifySeq++;
}

void webBleInit() {
  Serial.println(F("\n[WEB-BLE] Initializing NimBLE GATT service"));

  NimBLEDevice::init("PropaneScale");
  NimBLEServer* pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new PropaneBleCallbacks());

  NimBLEService* pService = pServer->createService(SERVICE_UUID);

  // Weight characteristic (Notify, Read)
  weightChar = pService->createCharacteristic(CHAR_WEIGHT_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  // Tare (Write Without Response)
  tareChar = pService->createCharacteristic(CHAR_TARE_UUID, NIMBLE_PROPERTY::WRITE_NR);
  // Calibrate (Write)
  calChar = pService->createCharacteristic(CHAR_CAL_UUID, NIMBLE_PROPERTY::WRITE);
  // Sampling rate (Read/Write)
  rateChar = pService->createCharacteristic(CHAR_RATE_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  // Auth (Write)
  authChar = pService->createCharacteristic(CHAR_AUTH_UUID, NIMBLE_PROPERTY::WRITE);

  PropaneCharCallbacks* charCb = new PropaneCharCallbacks();
  tareChar->setCallbacks(charCb);
  calChar->setCallbacks(charCb);
  rateChar->setCallbacks(charCb);
  authChar->setCallbacks(charCb);

  // Set default values for readables
  uint8_t rateBuf[2] = { static_cast<uint8_t>(samplingIntervalMs & 0xFF), static_cast<uint8_t>((samplingIntervalMs >> 8) & 0xFF) };
  rateChar->setValue(rateBuf, 2);

  pService->start();

  NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->enableScanResponse(true);
  pAdv->start();

  Serial.println(F("[WEB-BLE] Advertising started"));
}

void webBleTick() {
  // Pump: send notifications when interval elapsed and a client is connected.
  unsigned long now = millis();
  if (bleConnected && lastWeightAvailable && (now - lastNotifyMs >= samplingIntervalMs)) {
    sendWeightNotify(lastWeightLbs);
    lastNotifyMs = now;
    lastWeightAvailable = false; // consumed until next update; caller may call webBleNotifyWeight again
  }
}

void webBleNotifyWeight(float weight_lbs) {
  // Store latest weight to be sent on next tick (non-blocking)
  lastWeightLbs = weight_lbs;
  lastWeightAvailable = true;
  // Also log for visibility
  Serial.printf("[WEB-BLE] Queued weight for notify: %.3f lbs\n", weight_lbs);
}

void webBleRequestTare() {
  // Allow application to request tare via API (same as BLE write would)
  unsigned long now = millis();
  if (now - lastTareMs >= BLE_TARE_COOLDOWN_MS) {
    lastTareMs = now;
    tareRequested = true;
    Serial.println(F("[WEB-BLE] Tare requested via API"));
  } else {
    Serial.println(F("[WEB-BLE] Tare request from API rate-limited"));
  }
}

bool webBleTakeTareRequest() {
  if (!tareRequested) return false;
  noInterrupts();
  bool v = tareRequested;
  tareRequested = false;
  interrupts();
  return v;
}

bool webBleTakeCalibrateRequest(float* out_mass_lbs) {
  if (!calibrateRequested) return false;
  noInterrupts();
  float m = calibrateMassLbs;
  calibrateRequested = false;
  interrupts();
  if (out_mass_lbs) *out_mass_lbs = m;
  return true;
}

void webBleSetSamplingInterval(uint16_t interval_ms) {
  if (interval_ms < 20) interval_ms = 20;
  if (interval_ms > 2000) interval_ms = 2000;
  samplingIntervalMs = interval_ms;
  if (rateChar) {
    uint8_t buf[2] = { static_cast<uint8_t>(samplingIntervalMs & 0xFF), static_cast<uint8_t>((samplingIntervalMs >> 8) & 0xFF) };
    rateChar->setValue(buf, 2);
  }
}
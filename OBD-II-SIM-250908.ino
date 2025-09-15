/*
 * ESP32-S3 OBD-II CAN Emulator (ISO 15765-4, 11-bit, 500 kbps)
 *
 * Services / PIDs:
 * - Mode 0x01:
 *   0x01,0x04,0x05,0x0A,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x2F,0x33,0x51,0x5E,
 *   0x21 (Distance with MIL on), 0x31 (Distance since DTC clear)
 *   + optional non-standard: 0xA6 (Total Odometer km, 32-bit)
 * - Mode 0x09:
 *   0x00 (supported map), 0x02 (VIN, ISO-TP multi-frame)
 * - Service 0x22:
 *   DID 0xF123 → Odometer total km (32-bit, big-endian): response 0x62 F1 23 <AA BB CC DD>
 *
 * Features:
 * - Supported-PID bitmap auto-gen (0x00/0x20/0x40/0x60/0x80/0xA0 and Mode09 0x00)
 * - ISO-TP multi-frame TX (VIN) with simple Flow Control handling
 * - SIM (auto) / MANUAL (fixed) modes via CLI
 * - CLI: set values, VIN, MIL, DTC, debug, hex dump, trace interval
 *
 * NOTE: Lab use only. Do NOT plug into a live vehicle CAN bus.
 */

#include <Arduino.h>
extern "C" {
  #include "driver/twai.h"
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
}

// ======================= User Config =========================
#define CAN_BITRATE  500000  // 500 kbps

#include <Adafruit_NeoPixel.h>

// LED 핀
#define LED_PIN 48
#define LED_COUNT 1
Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ----- LED 상태 -----
float gammaValue = 2.2;
uint8_t baseR = 255, baseG = 20, baseB = 147; // DeepPink
enum LedMode { LED_RUN, LED_RX, LED_TX, LED_ERR };
static LedMode ledMode = LED_RUN;
static uint32_t ledEventUntil = 0;

// --- Choose your pins here ---
static const gpio_num_t TWAI_TX_GPIO = GPIO_NUM_4;  // change as needed for your board
static const gpio_num_t TWAI_RX_GPIO = GPIO_NUM_5;  // change as needed for your board
// =============================================================

// ----- Non-standard ODO (Mode01 PID 0xA6) enable -----
#define ENABLE_NONSTANDARD_ODO_PID  1  // 1=enable 0xA6

// ----- VIN (17 chars) -----
static char g_vin[18] = "WAUZZZ8K9AA000001"; // exactly 17 + '\0'

// ----- Service 0x22 custom ODO DID -----
#define CUSTOM_ODO_DID_H  0xF1
#define CUSTOM_ODO_DID_L  0x23

// ---------- Debug ----------
static bool g_debug     = true;       // master debug ON/OFF
static bool g_debug_can = true;       // HEX dump frames
static uint32_t g_trace_interval_ms = 1000; // periodic print

#ifndef TWAI_ALERT_RECOVERY_COMPLETE
  #ifdef TWAI_ALERT_RECOVERED
    #define TWAI_ALERT_RECOVERY_COMPLETE TWAI_ALERT_RECOVERED
  #elif defined(TWAI_ALERT_RECOVERY_IN_PROGRESS)
    #define TWAI_ALERT_RECOVERY_COMPLETE 0
  #else
    #define TWAI_ALERT_RECOVERY_COMPLETE 0
  #endif
#endif

static twai_general_config_t gen_config =
  TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_MODE_NORMAL);
static twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_500KBITS();
static twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

// ====== Mode (SIM / MANUAL) ======
enum EmuMode : uint8_t { MODE_SIM = 0, MODE_MANUAL = 1 };
static EmuMode g_mode = MODE_SIM;

// ====== MIL / DTC ======
static bool     g_mil_on    = false;
static uint8_t  g_dtc_count = 0;

// ====== Signals ======
static float g_speed_kph         = 0.0f;   // 0..255
static float g_rpm               = 800.0f; // 0..16383 raw/4
static float g_coolantC          = 70.0f;  // -40..215
static float g_iatC              = 30.0f;  // -40..215
static float g_throttle_pct      = 10.0f;  // 0..100
static float g_maf_gps           = 5.0f;   // 0..655.35
static float g_calc_load_pct     = 20.0f;  // 0..100
static float g_fuel_pressure_kPa = 300.0f; // kPa = 3*A
static float g_timing_adv_deg    = 2.0f;   // (A/2)-64
static float g_fuel_level_pct    = 65.0f;  // 0..100
static float g_barometric_kPa    = 100.0f; // 0..255
static float g_fuel_rate_lph     = 1.8f;   // 0..(0xFFFF*0.05)
static uint8_t g_fuel_type_enum  = 0x01;   // 0x01 Gasoline, 0x04 Diesel

// ====== Odometer-like distances ======
static float    g_odo_km             = 12345.6f; // total odometer (non-standard + DID)
static uint32_t g_dist_since_dtc_km  = 0;        // 0x31
static uint32_t g_dist_mil_on_km     = 0;        // 0x21

// ====== Supported PIDs (Mode 01) ======
static const uint8_t kSupportedPIDs_Mode01[] = {
  0x01, 0x04, 0x05, 0x0A, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
  0x2F, 0x33, 0x51, 0x5E,
  0x21, 0x31,
#if ENABLE_NONSTANDARD_ODO_PID
  0xA6,
#endif
};

// ====== Supported PIDs (Mode 09) ======
static const uint8_t kSupportedPIDs_Mode09[] = {
  0x02 // VIN
};

// ================= LED 제어 =================
void updateLedRun() {
  float phase = fmod(millis() / 1000.0f, 2.0f);
  float t = (phase < 1.0f) ? phase : (2.0f - phase);
  float brightness = pow(t, gammaValue);

  uint8_t r = (uint8_t)(baseR * brightness);
  uint8_t g = (uint8_t)(baseG * brightness);
  uint8_t b = (uint8_t)(baseB * brightness);
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}

void setLedColor(uint8_t r, uint8_t g, uint8_t b) {
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}

// ------- Debug helpers -------
static void hexdump(const uint8_t* p, uint8_t n) {
  for (uint8_t i = 0; i < n; ++i) {
    if (p[i] < 0x10) Serial.print('0');
    Serial.print(p[i], HEX);
    if (i < n - 1) Serial.print(' ');
  }
}
static void printMask(const char* tag, const uint8_t m[4]) {
  if (!g_debug) return;
  Serial.print(tag); Serial.print(" = ");
  for (int i=0;i<4;i++){
    if (m[i] < 0x10) Serial.print('0');
    Serial.print(m[i], HEX);
    if (i<3) Serial.print(' ');
  }
  Serial.println();
}

// ------- CAN TX -------
bool canSend(uint32_t can_id, const uint8_t* data8, uint8_t dlc = 8, TickType_t timeout_ticks = pdMS_TO_TICKS(10)) {
  twai_message_t msg = {};
  msg.identifier = can_id;
  msg.flags = 0; // std, data frame
  msg.data_length_code = dlc;
  for (int i = 0; i < dlc && i < 8; ++i) msg.data[i] = data8[i];
  esp_err_t r = twai_transmit(&msg, timeout_ticks);
  if (r == ESP_OK) {
    ledMode = LED_TX;
    ledEventUntil = millis() + 100;
  }
  if (g_debug) {
    Serial.print(F("[TX] ID=0x")); Serial.print(can_id, HEX);
    Serial.print(F(" DLC=")); Serial.print(dlc);
    Serial.print(F(" Data: "));
    if (g_debug_can) hexdump(data8, dlc); else Serial.print(F("(hidden)"));
    Serial.print(F(" -> ")); Serial.println((r==ESP_OK) ? F("OK") : F("ERR"));
  }
  return (r == ESP_OK);
}

// ================= ISO-TP helpers (TX only) =================
// Return true on success. If wait_fc==true, will wait for a FlowControl frame.
bool isotp_send(uint32_t resp_id, const uint8_t* payload, uint16_t len, bool wait_fc=true) {
  if (len <= 7) {
    // Single Frame
    uint8_t d[8] = {0};
    d[0] = (uint8_t)len;
    memcpy(&d[1], payload, len);
    return canSend(resp_id, d, 8, pdMS_TO_TICKS(20));
  }

  // First Frame
  uint8_t ff[8] = {0};
  ff[0] = 0x10 | ((len >> 8) & 0x0F);
  ff[1] = (uint8_t)(len & 0xFF);
  uint16_t copied = min<uint16_t>(6, len);
  memcpy(&ff[2], payload, copied);
  if (!canSend(resp_id, ff, 8, pdMS_TO_TICKS(20))) return false;

  // Defaults
  uint8_t  blockSize = 0;     // 0=unlimited
  uint16_t stmin_us  = 1000;  // 1 ms

  if (wait_fc) {
    // Wait up to ~100ms for Flow Control (0x30)
    uint32_t t0 = millis();
    while (millis() - t0 < 100) {
      twai_message_t rx;
      if (twai_receive(&rx, pdMS_TO_TICKS(5)) == ESP_OK) {
        if ((rx.data_length_code >= 3) && ((rx.data[0] & 0xF0) == 0x30)) {
          blockSize = rx.data[1];
          uint8_t st = rx.data[2];
          if (st <= 0x7F) stmin_us = st * 1000U;
          else if (st >= 0xF1 && st <= 0xF9) stmin_us = (st - 0xF0) * 100U;
          else stmin_us = 1000;
          if (g_debug) {
            Serial.print(F("[ISO-TP] FC: BS=")); Serial.print(blockSize);
            Serial.print(F(" STmin(us)=")); Serial.println(stmin_us);
          }
          break;
        }
      }
    }
  }

  // Consecutive Frames
  uint16_t offset = copied;
  uint8_t  sn = 1;
  uint8_t  sent_in_block = 0;

  while (offset < len) {
    uint8_t cf[8] = {0};
    cf[0] = 0x20 | (sn & 0x0F);
    uint8_t chunk = min<uint16_t>(7, len - offset);
    memcpy(&cf[1], &payload[offset], chunk);
    if (!canSend(resp_id, cf, 8, pdMS_TO_TICKS(20))) return false;

    offset += chunk;
    sn = (sn + 1) & 0x0F; if (sn == 0) sn = 1;

    // Block size handling
    if (blockSize) {
      sent_in_block++;
      if (sent_in_block >= blockSize) {
        // Wait for next FC
        sent_in_block = 0;
        uint32_t t0 = millis();
        bool got_fc = false;
        while (millis() - t0 < 100) {
          twai_message_t rx;
          if (twai_receive(&rx, pdMS_TO_TICKS(5)) == ESP_OK) {
            if ((rx.data_length_code >= 3) && ((rx.data[0] & 0xF0) == 0x30)) {
              uint8_t st = rx.data[2];
              if (st <= 0x7F) stmin_us = st * 1000U;
              else if (st >= 0xF1 && st <= 0xF9) stmin_us = (st - 0xF0) * 100U;
              got_fc = true;
              break;
            }
          }
        }
        if (!got_fc && g_debug) Serial.println(F("[ISO-TP] FC timeout (continue)"));
      }
    }

    if (stmin_us) delayMicroseconds(stmin_us);
  }
  return true;
}

// ====== Encoding utils ======
static inline void encodeRPM(uint16_t rpm, uint8_t& A, uint8_t& B) { uint16_t raw = (uint16_t)(rpm * 4U); A=(raw>>8)&0xFF; B=raw&0xFF; }
static inline uint8_t encodePct(float pct) { if (pct<0)pct=0; if(pct>100)pct=100; return (uint8_t)roundf(255.0f*(pct/100.0f)); }
static inline uint8_t encodeTempC(int8_t tc) { return (uint8_t)(tc + 40); }
static inline void encodeMAF(float maf_gps, uint8_t& A, uint8_t& B) { uint16_t raw=(uint16_t)roundf(maf_gps*100.0f); A=(raw>>8)&0xFF; B=raw&0xFF; }
static inline uint8_t encodeFuelPressure_A(float kPa) { float A=kPa/3.0f; if(A<0)A=0; if(A>255)A=255; return (uint8_t)roundf(A); }
static inline uint8_t encodeBaro_A(float kPa) { if (kPa<0) kPa=0; if (kPa>255) kPa=255; return (uint8_t)roundf(kPa); }
static inline uint8_t encodeTimingA(float adv_deg) { float A=(adv_deg+64.0f)*2.0f; if(A<0)A=0; if(A>255)A=255; return (uint8_t)roundf(A); }
static inline void encodeFuelRate_Lph(float lph, uint8_t& A, uint8_t& B) { if(lph<0)lph=0; if(lph>(0xFFFF*0.05f)) lph=(0xFFFF*0.05f); uint16_t raw=(uint16_t)roundf(lph/0.05f); A=(raw>>8)&0xFF; B=raw&0xFF; }
static inline void encodeKM16(uint32_t km, uint8_t& A, uint8_t& B) { if (km > 65535) km = 65535; A=(km>>8)&0xFF; B=km&0xFF; }
static inline void encodeKM32(uint32_t km, uint8_t out[4]) { out[0]=(km>>24)&0xFF; out[1]=(km>>16)&0xFF; out[2]=(km>>8)&0xFF; out[3]=km&0xFF; }

// ====== Supported PIDs bitmap ======
void buildSupportedMask(const uint8_t* list, size_t n, uint8_t requestPid, uint8_t out[4]) {
  uint8_t groupStart = (requestPid == 0x00) ? 0x01 : (uint8_t)(requestPid + 1);
  memset(out, 0, 4);
  for (size_t i = 0; i < n; ++i) {
    uint8_t pid = list[i];
    if (pid >= groupStart && pid <= (uint8_t)(groupStart + 0x1F)) {
      uint8_t idx = pid - groupStart;      // 0..31
      uint8_t byteIndex = idx / 8;         // 0..3
      uint8_t bitInByte = 7 - (idx % 8);   // MSB first
      out[byteIndex] |= (1u << bitInByte);
    }
  }
  printMask("[MAP]", out);
}

// ===================== Request Handlers ======================

// ---- Mode 01 ----
void sendMode1Positive(uint32_t resp_id, uint8_t pid, const uint8_t* payload, uint8_t payload_len) {
  uint8_t len = 2 + payload_len; if (len > 7) len = 7;
  uint8_t d[8] = {0};
  d[0] = len; d[1] = 0x41; d[2] = pid;
  for (int i=0; i<payload_len && (3+i)<8; ++i) d[3+i] = payload[i];
  canSend(resp_id, d, 8, pdMS_TO_TICKS(10));
}

void handle_mode01(uint32_t req_id, uint32_t resp_id, const twai_message_t& rx) {
  uint8_t len = rx.data[0] & 0x0F; //if (len < 3) return;
  uint8_t pid = rx.data[2];

  if (pid == 0x00 || pid == 0x20 || pid == 0x40 || pid == 0x60 || pid == 0x80 || pid == 0xA0) {
    uint8_t payload[4]; buildSupportedMask(kSupportedPIDs_Mode01, sizeof(kSupportedPIDs_Mode01), pid, payload);
    sendMode1Positive(resp_id, pid, payload, 4);
    return;
  }

  switch (pid) {
    case 0x01: { // Monitor status since DTCs cleared
      uint8_t A = (g_mil_on ? 0x80 : 0x00) | (g_dtc_count & 0x7F);
      uint8_t payload[4] = {A,0x00,0x00,0x00};
      sendMode1Positive(resp_id, 0x01, payload, 4); break;
    }
    case 0x04: { uint8_t A=encodePct(g_calc_load_pct); uint8_t payload[1]={A}; sendMode1Positive(resp_id,0x04,payload,1); break; }
    case 0x05: { int8_t t=(int8_t)roundf(constrain(g_coolantC,-40.0f,215.0f)); uint8_t payload[1]={encodeTempC(t)}; sendMode1Positive(resp_id,0x05,payload,1); break; }
    case 0x0A: { uint8_t A=encodeFuelPressure_A(g_fuel_pressure_kPa); uint8_t payload[1]={A}; sendMode1Positive(resp_id,0x0A,payload,1); break; }
    case 0x0C: { uint16_t rpm=(uint16_t)roundf(g_rpm); if(rpm>16383) rpm=16383; uint8_t A,B; encodeRPM(rpm,A,B); uint8_t payload[2]={A,B}; sendMode1Positive(resp_id,0x0C,payload,2); break; }
    case 0x0D: { uint8_t v=(uint8_t)roundf(constrain(g_speed_kph,0.0f,255.0f)); uint8_t payload[1]={v}; sendMode1Positive(resp_id,0x0D,payload,1); break; }
    case 0x0E: { uint8_t A=encodeTimingA(g_timing_adv_deg); uint8_t payload[1]={A}; sendMode1Positive(resp_id,0x0E,payload,1); break; }
    case 0x0F: { int8_t t=(int8_t)roundf(constrain(g_iatC,-40.0f,215.0f)); uint8_t payload[1]={encodeTempC(t)}; sendMode1Positive(resp_id,0x0F,payload,1); break; }
    case 0x10: { float maf=constrain(g_maf_gps,0.0f,655.35f); uint8_t A,B; encodeMAF(maf,A,B); uint8_t payload[2]={A,B}; sendMode1Positive(resp_id,0x10,payload,2); break; }
    case 0x11: { uint8_t A=encodePct(g_throttle_pct); uint8_t payload[1]={A}; sendMode1Positive(resp_id,0x11,payload,1); break; }
    case 0x2F: { uint8_t A=encodePct(g_fuel_level_pct); uint8_t payload[1]={A}; sendMode1Positive(resp_id,0x2F,payload,1); break; }
    case 0x33: { uint8_t A=encodeBaro_A(g_barometric_kPa); uint8_t payload[1]={A}; sendMode1Positive(resp_id,0x33,payload,1); break; }
    case 0x51: { uint8_t payload[1]={g_fuel_type_enum}; sendMode1Positive(resp_id,0x51,payload,1); break; }
    case 0x5E: { uint8_t A,B; encodeFuelRate_Lph(g_fuel_rate_lph,A,B); uint8_t payload[2]={A,B}; sendMode1Positive(resp_id,0x5E,payload,2); break; }
    case 0x21: { uint8_t A,B; encodeKM16(g_dist_mil_on_km, A, B); uint8_t payload[2]={A,B}; sendMode1Positive(resp_id,0x21,payload,2); break; }
    case 0x31: { uint8_t A,B; encodeKM16(g_dist_since_dtc_km, A, B); uint8_t payload[2]={A,B}; sendMode1Positive(resp_id,0x31,payload,2); break; }
#if ENABLE_NONSTANDARD_ODO_PID
    case 0xA6: { uint8_t km4[4]; encodeKM32((uint32_t)roundf(g_odo_km), km4); sendMode1Positive(resp_id,0xA6,km4,4); break; }
#endif
    default: { /* sendNegative(resp_id,0x01,0x12); */ break; }
  }
}

// ---- Mode 09 (VIN) ----
void handle_mode09(uint32_t req_id, uint32_t resp_id, const twai_message_t& rx) {
  uint8_t len = rx.data[0] & 0x0F; //if (len < 3) return;
  uint8_t pid = rx.data[2];

  if (pid == 0x00) {
    uint8_t payload[4]; buildSupportedMask(kSupportedPIDs_Mode09, sizeof(kSupportedPIDs_Mode09), 0x00, payload);
    uint8_t d[8] = {0};
    d[0] = 0x06;  // len for [49 00 A B C D]
    d[1] = 0x49; d[2] = 0x00;
    d[3] = payload[0]; d[4] = payload[1]; d[5] = payload[2]; d[6] = payload[3];
    canSend(resp_id, d, 8, pdMS_TO_TICKS(10));
    return;
  }

  if (pid == 0x02) { // VIN (17 chars)
    uint8_t full[2 + 17];
    full[0] = 0x49; full[1] = 0x02;
    memcpy(&full[2], g_vin, 17);
    if (g_debug) {
      Serial.print(F("[MODE09] VIN="));
      for (int i=0;i<17;i++) Serial.print((char)full[2+i]);
      Serial.println();
    }
    isotp_send(resp_id, full, sizeof(full), true);
    return;
  }

  // sendNegative(resp_id, 0x09, 0x12);
}

// ---- Service 0x22 (custom DID for ODO) ----
void handle_service22(uint32_t req_id, uint32_t resp_id, const twai_message_t& rx) {
  uint8_t len = rx.data[0] & 0x0F; //if (len < 4) return; // 22 + DID(H) + DID(L)
  uint8_t did_h = rx.data[2];
  uint8_t did_l = rx.data[3];

  if (did_h == CUSTOM_ODO_DID_H && did_l == CUSTOM_ODO_DID_L) {
    uint8_t data4[4]; encodeKM32((uint32_t)roundf(g_odo_km), data4);
    uint8_t d[8] = {0};
    d[0] = 0x07;      // payload len 7: 62 DIDH DIDL A B C D
    d[1] = 0x62; d[2] = did_h; d[3] = did_l;
    d[4] = data4[0]; d[5] = data4[1]; d[6] = data4[2]; d[7] = data4[3];
    canSend(resp_id, d, 8, pdMS_TO_TICKS(10));
    if (g_debug) Serial.println(F("[SVC22] Sent ODO via DID 0xF123"));
    return;
  }

  // sendNegative(resp_id, 0x22, 0x31); // request out of range
}

// ===================== Dispatcher ============================
void handleObdRequest(const twai_message_t& rx) {
  uint32_t req_id = rx.identifier;
  uint32_t resp_id = 0x7E8; // default ECU A
  if (req_id >= 0x7E0 && req_id <= 0x7E7) resp_id = 0x7E8 + (req_id - 0x7E0);

  if (g_debug) {
    Serial.print(F("[RX] ID=0x")); Serial.print(req_id, HEX);
    Serial.print(F(" DLC=")); Serial.print(rx.data_length_code);
    Serial.print(F(" Data: "));
    if (g_debug_can) hexdump(rx.data, rx.data_length_code); else Serial.print(F("(hidden)"));
    Serial.println();
  }
  ledMode = LED_RX;
  ledEventUntil = millis() + 100;

  uint8_t len = rx.data[0] & 0x0F; if (len < 2) return;
  uint8_t service = rx.data[1];

  switch (service) {
    case 0x01: handle_mode01(req_id, resp_id, rx); break;
    case 0x09: handle_mode09(req_id, resp_id, rx); break;
    case 0x22: handle_service22(req_id, resp_id, rx); break;
    default:
      // sendNegative(resp_id, service, 0x11); // service not supported
      break;
  }
}

// ===================== Simulation / Distances =================
void updateSimValues_SIM() {
  const float t = millis() / 1000.0f;

  g_speed_kph = 60.0f + 60.0f * sinf(2.0f * PI * 0.05f * t);
  if (g_speed_kph < 0) g_speed_kph = 0;
  g_rpm = 700.0f + (g_speed_kph * 45.0f);
  if (g_rpm > 4500.0f) g_rpm = 4500.0f;

  float targetCoolant = 90.0f - 5.0f * cosf(2.0f * PI * 0.01f * t);
  g_coolantC += (targetCoolant - g_coolantC) * 0.02f;

  g_iatC = 34.0f + 6.0f * sinf(2.0f * PI * 0.02f * t + 1.0f);

  g_throttle_pct = 30.0f + 25.0f * sinf(2.0f * PI * 0.07f * t + 0.5f);
  if (g_throttle_pct < 0) g_throttle_pct = 0; if (g_throttle_pct > 100) g_throttle_pct = 100;
  g_calc_load_pct = 20.0f + 60.0f * fabsf(sinf(2.0f * PI * 0.04f * t));

  g_maf_gps = (g_rpm / 100.0f) + (g_throttle_pct * 0.1f);
  g_timing_adv_deg = 8.0f + 6.0f * sinf(2.0f * PI * 0.03f * t);
  g_fuel_pressure_kPa = 280.0f + 60.0f * fabsf(sinf(2.0f * PI * 0.02f * t));

  g_fuel_level_pct += -0.0015f; if (g_fuel_level_pct < 10.0f) g_fuel_level_pct = 65.0f;
  g_barometric_kPa = 99.0f + 2.0f * sinf(2.0f * PI * 0.005f * t);

  float base_lph = 0.6f + 0.002f * g_rpm + 0.01f * g_calc_load_pct;
  g_fuel_rate_lph = base_lph * (0.6f + 0.4f * (g_throttle_pct/100.0f));
}

void updateDistances(float dt_sec) {
  g_odo_km += (g_speed_kph * (dt_sec / 3600.0f));

  float add_km = g_speed_kph * (dt_sec / 3600.0f);
  float new31 = (float)g_dist_since_dtc_km + add_km; if (new31 > 65535.0f) new31 = 65535.0f;
  g_dist_since_dtc_km = (uint32_t)new31;

  if (g_mil_on) {
    float new21 = (float)g_dist_mil_on_km + add_km; if (new21 > 65535.0f) new21 = 65535.0f;
    g_dist_mil_on_km = (uint32_t)new21;
  }
}

// ===================== CLI =====================
float clamp(float v, float lo, float hi){ if(v<lo) return lo; if(v>hi) return hi; return v; }

void printHelp() {
  Serial.println(F("=== CLI ==="));
  Serial.println(F("m0 : SIM mode (auto), m1 : MANUAL mode (fixed)"));
  Serial.println(F("p  : print current values"));
  Serial.println(F("d0/d1 (debug OFF/ON), c0/c1 (CAN hexdump OFF/ON), t<ms> (trace)"));
  Serial.println(F("mil0/mil1 : MIL OFF/ON, dtc=<0..127> : DTC count"));
  Serial.println(F("set <key> <val>  (speed,rpm,coolant,iat,throttle,maf,load,fuelp,timing,level,baro,rate,fueltype,odo,dclr,dmil)"));
  Serial.println(F("short: V= R= Tc= Ti= Th= MAF= Ld= FP= Adv= FL= BP= FR= FT= ODO= DCLR= DMIL="));
  Serial.println(F("VIN set: set vin <17chars>"));
}

void printValues() {
  Serial.print(F("[MODE] ")); Serial.println(g_mode==MODE_SIM?F("SIM"):F("MANUAL"));
  Serial.print(F(" V=")); Serial.print((int)roundf(g_speed_kph));
  Serial.print(F(" km/h  RPM=")); Serial.print((int)roundf(g_rpm));
  Serial.print(F("  Cool=")); Serial.print((int)roundf(g_coolantC));
  Serial.print(F("C  IAT=")); Serial.print((int)roundf(g_iatC));
  Serial.print(F("C  Th=")); Serial.print((int)roundf(g_throttle_pct));
  Serial.print(F("%  Load=")); Serial.print((int)roundf(g_calc_load_pct));
  Serial.print(F("%  FuelP=")); Serial.print((int)roundf(g_fuel_pressure_kPa));
  Serial.print(F("kPa  Level=")); Serial.print((int)roundf(g_fuel_level_pct));
  Serial.print(F("%  Baro=")); Serial.print((int)roundf(g_barometric_kPa));
  Serial.print(F("kPa  Rate=")); Serial.print(g_fuel_rate_lph, 2);
  Serial.print(F(" L/h  FuelType=0x")); Serial.print(g_fuel_type_enum, HEX);
  Serial.print(F("  MIL=")); Serial.print(g_mil_on?F("ON"):F("OFF"));
  Serial.print(F("  DTC=")); Serial.print(g_dtc_count);
  Serial.print(F("  ODO=")); Serial.print(g_odo_km, 1); Serial.print(F(" km"));
  Serial.print(F("  DCLR=")); Serial.print(g_dist_since_dtc_km); Serial.print(F(" km"));
  Serial.print(F("  DMIL=")); Serial.print(g_dist_mil_on_km); Serial.println(F(" km"));
  Serial.print(F(" VIN=")); for(int i=0;i<17;i++) Serial.print(g_vin[i]); Serial.println();
}

bool setByKeyVal(const String& key, float val) {
  String k = key; k.toLowerCase();
  if (k=="speed"||k=="speed_kph"||k=="v") { g_speed_kph = clamp(val,0,255); return true; }
  if (k=="rpm"||k=="r") { g_rpm = clamp(val,0,16383); return true; }
  if (k=="coolant"||k=="tc") { g_coolantC = clamp(val,-40,215); return true; }
  if (k=="iat"||k=="ti") { g_iatC = clamp(val,-40,215); return true; }
  if (k=="throttle"||k=="th") { g_throttle_pct = clamp(val,0,100); return true; }
  if (k=="maf") { g_maf_gps = clamp(val,0,655.35f); return true; }
  if (k=="load"||k=="ld") { g_calc_load_pct = clamp(val,0,100); return true; }
  if (k=="fuelp"||k=="fp") { g_fuel_pressure_kPa = clamp(val,0,765); return true; } // 3*A, A<=255
  if (k=="timing"||k=="adv") { g_timing_adv_deg = clamp(val,-64,191); return true; }
  if (k=="level"||k=="fl") { g_fuel_level_pct = clamp(val,0,100); return true; }
  if (k=="baro"||k=="bp"||k=="bar") { g_barometric_kPa = clamp(val,0,255); return true; }
  if (k=="rate"||k=="fr") { g_fuel_rate_lph = clamp(val,0,(float)(0xFFFF*0.05)); return true; }
  if (k=="fueltype"||k=="ft") { g_fuel_type_enum = (uint8_t)constrain((int)val,0,255); return true; }
  if (k=="odo") { g_odo_km = clamp(val, 0, 9999999); return true; }
  if (k=="dclr"||k=="d_since_clear") { g_dist_since_dtc_km = (uint32_t)clamp(val,0,65535); return true; }
  if (k=="dmil"||k=="d_mil_on") { g_dist_mil_on_km = (uint32_t)clamp(val,0,65535); return true; }
  return false;
}

void handleSerialCmd() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c=='\n' || c=='\r') continue;

    if (c=='m') { while(!Serial.available()){} char v=Serial.read(); g_mode = (v=='1')?MODE_MANUAL:MODE_SIM; Serial.print(F("[MODE] ")); Serial.println(g_mode==MODE_SIM?F("SIM"):F("MANUAL")); continue; }
    if (c=='d') { while(!Serial.available()){} g_debug = (Serial.read()=='1'); Serial.print(F("[DBG] debug=")); Serial.println(g_debug?F("ON"):F("OFF")); continue; }
    if (c=='c') { while(!Serial.available()){} g_debug_can = (Serial.read()=='1'); Serial.print(F("[DBG] can-dump=")); Serial.println(g_debug_can?F("ON"):F("OFF")); continue; }
    if (c=='t') { String num = Serial.readStringUntil('\n'); uint32_t v=num.toInt(); if(v>=100) g_trace_interval_ms=v; Serial.print(F("[DBG] trace interval(ms)=")); Serial.println(g_trace_interval_ms); continue; }
    if (c=='p') { printValues(); continue; }
    if (c=='?') { printHelp(); continue; }

    String rest = Serial.readStringUntil('\n'); String cmd; cmd += c; cmd += rest; cmd.trim();

    if (cmd=="mil1") { g_mil_on=true; Serial.println(F("[DBG] MIL=ON")); continue; }
    if (cmd=="mil0") { g_mil_on=false; Serial.println(F("[DBG] MIL=OFF")); continue; }
    if (cmd.startsWith("dtc=")) { int v = cmd.substring(4).toInt(); g_dtc_count = (uint8_t)constrain(v,0,127); Serial.print(F("[DBG] DTC=")); Serial.println(g_dtc_count); continue; }

    // VIN setter
    if (cmd.startsWith("set vin ")) {
      String v = cmd.substring(8);
      v.trim();
      if ((int)v.length()==17) {
        for (int i=0;i<17;i++) g_vin[i] = v[i];
        g_vin[17] = '\0';
        Serial.println(F("[SET] VIN updated"));
      } else {
        Serial.println(F("[ERR] VIN must be exactly 17 chars"));
      }
      continue;
    }

    // Short setters KEY=VALUE
    int eq = cmd.indexOf('=');
    if (eq > 0) {
      String key = cmd.substring(0, eq); key.trim();
      String sval = cmd.substring(eq+1); sval.trim();
      float val = sval.toFloat();

      if (key=="V") key="speed";
      else if (key=="R") key="rpm";
      else if (key=="Tc") key="coolant";
      else if (key=="Ti") key="iat";
      else if (key=="Th") key="throttle";
      else if (key=="MAF") key="maf";
      else if (key=="Ld") key="load";
      else if (key=="FP") key="fuelp";
      else if (key=="Adv") key="timing";
      else if (key=="FL") key="level";
      else if (key=="BP") key="baro";
      else if (key=="FR") key="rate";
      else if (key=="FT") key="fueltype";
      else if (key=="ODO") key="odo";
      else if (key=="DCLR") key="dclr";
      else if (key=="DMIL") key="dmil";

      if (setByKeyVal(key, val)) { Serial.print(F("[SET] ")); Serial.print(key); Serial.print(F("=")); Serial.println(val); }
      else { Serial.println(F("[ERR] Unknown key (use '?')")); }
      continue;
    }

    // Verbose: set <key> <val>
    if (cmd.startsWith("set ")) {
      String line = cmd.substring(4); line.trim();
      int sp = line.indexOf(' ');
      if (sp>0) {
        String key = line.substring(0, sp); key.trim();
        float val = line.substring(sp+1).toFloat();
        if (setByKeyVal(key, val)) { Serial.print(F("[SET] ")); Serial.print(key); Serial.print(F("=")); Serial.println(val); }
        else { Serial.println(F("[ERR] Unknown key (see '?')")); }
      } else {
        Serial.println(F("[ERR] Usage: set <key> <val>"));
      }
      continue;
    }

    Serial.println(F("[ERR] Unknown command (use '?')"));
  }
}

void rgb_show()
{
  // 0~2초 주기 위상
  float phase = fmod(millis() / 1000.0f, 2.0f);
  float t = (phase < 1.0f) ? phase : (2.0f - phase);  // 삼각파 0~1

  // gamma 보정
  float brightness = pow(t, gammaValue);

  // 최종 RGB 적용
  uint8_t r = (uint8_t)(baseR * brightness);
  uint8_t g = (uint8_t)(baseG * brightness);
  uint8_t b = (uint8_t)(baseB * brightness);

  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();  
}

// ===================== Setup/Loop =====================
void setup() {
  pixels.begin();
  pixels.clear();
  pixels.show();

  Serial.begin(115200);
  delay(2000);

  view_boot_message();
  gen_config.tx_queue_len = 20;
  gen_config.rx_queue_len = 20;
  gen_config.alerts_enabled =
      TWAI_ALERT_RX_DATA |
      TWAI_ALERT_TX_SUCCESS |
      TWAI_ALERT_TX_FAILED |
      TWAI_ALERT_BUS_OFF |
      TWAI_ALERT_ERR_PASS |
      TWAI_ALERT_RECOVERY_COMPLETE;

  timing_config = TWAI_TIMING_CONFIG_500KBITS();
  filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&gen_config, &timing_config, &filter_config) != ESP_OK) {
    Serial.println(F("[TWAI] driver_install failed")); while(true){delay(1000);}
  }
  if (twai_start() != ESP_OK) {
    Serial.println(F("[TWAI] start failed")); while(true){delay(1000);}
  }
  Serial.print(F("[TWAI] Started at 500 kbps, TX="));
  Serial.print((int)TWAI_TX_GPIO);
  Serial.print(F(" RX="));
  Serial.println((int)TWAI_RX_GPIO);

  Serial.println(F("[TWAI] Waiting for OBD requests (0x7DF / 0x7E0..0x7E7)..."));
  Serial.println(F("[CMD] m0/m1, p, d0/d1, c0/c1, t<ms>, mil0/mil1, dtc=<n>, set <key> <val>, KEY=VAL, set vin <17chars>, ?"));

  printValues();
}

void loop() {
  static uint32_t t_lastTrace = 0;
  static uint32_t g_prev_ms = 0;

  // Serial CLI
  handleSerialCmd();

  // Simulation
  if (g_mode == MODE_SIM) updateSimValues_SIM();

  // Distance integration
  uint32_t now = millis();
  if (g_prev_ms == 0) g_prev_ms = now;
  float dt = (now - g_prev_ms) / 1000.0f;
  g_prev_ms = now;
  updateDistances(dt);

  // CAN receive
  twai_message_t rx_msg;
  if (twai_receive(&rx_msg, pdMS_TO_TICKS(5)) == ESP_OK) {
    bool is_ext = false;
    #ifdef TWAI_MSG_FLAG_EXTD
      is_ext = (rx_msg.flags & TWAI_MSG_FLAG_EXTD) != 0;
    #endif
    if (!is_ext) {
      if (rx_msg.identifier == 0x7DF || (rx_msg.identifier >= 0x7E0 && rx_msg.identifier <= 0x7E7)) {
        // Dispatch OBD style requests
        uint8_t service = (rx_msg.data_length_code>=2) ? rx_msg.data[1] : 0;
        (void)service; // ignored here; handleObdRequest parses again
        handleObdRequest(rx_msg);
      } else if (g_debug) {
        Serial.print(F("[RX] Non-OBD std frame ID=0x")); Serial.println(rx_msg.identifier, HEX);
      }
    } else if (g_debug) {
      Serial.print(F("[RX] Extended frame ignored ID=0x")); Serial.println(rx_msg.identifier, HEX);
    }
  }

  // periodic trace
  if (g_debug && now - t_lastTrace >= g_trace_interval_ms) {
    t_lastTrace = now;
    printValues();
  }

  // Alerts → 에러시 빨강
  uint32_t alerts;
  if (twai_read_alerts(&alerts, 0) == ESP_OK) {
    if (alerts & (TWAI_ALERT_BUS_OFF | TWAI_ALERT_ERR_PASS)) {
      ledMode = LED_ERR;
      ledEventUntil = millis() + 100;
    }
  }

  // LED update
  if ((ledMode==LED_RX || ledMode==LED_TX || ledMode == LED_ERR) && now<ledEventUntil) {
    if (ledMode==LED_RX) setLedColor(0,255,0);
    if (ledMode==LED_TX) setLedColor(0,0,255);
    if (ledMode==LED_ERR) setLedColor(255,0,0);
  } else {
    ledMode = LED_RUN;
    updateLedRun();
  }
  delay(10); // ~100 Hz loop
}


void view_boot_message() {
  char tempbuf[50];

  Serial.printf("\r\n****************************************");
  Serial.printf("\r\n*        OBD-II CAN Emulator           *");
  Serial.printf("\r\n*    (ISO 15765-4, 11-bit, 500 kbps)   *");
  Serial.printf("\r\n****************************************");
  Serial.printf("\r\n*                                      *");
  sprintf(tempbuf, "\r\n*  Compiled : %s %s     *", __DATE__, __TIME__);
  Serial.printf(tempbuf);
  Serial.printf("\r\n*  Copyright(c) 2023 Woo Hyuk Joon     *");
  Serial.printf("\r\n*                                      *");
  Serial.printf("\r\n*  Phone : +82-10-9925-6913            *");
  Serial.printf("\r\n*  Mail : uhj0305@gmail.com            *");
  //printf("\r\n*                                      *");
  Serial.printf("\r\n*                                      *");
  Serial.printf("\r\n****************************************\r\n");
}
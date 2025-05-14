#ifndef WIRELESS_MANAGER_V2_H
#define WIRELESS_MANAGER_V2_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

typedef struct {
  char header[5];               // e.g., "[TEL]" or "[LOG]"
  uint8_t payload[32];          // Could be TelemetryPacket, log text, etc.
} FramedMessage;
// --- Telemetry Struct to Send ---
typedef struct {
  uint16_t pwm;
  int16_t pitch;
  int16_t roll;
  int16_t yaw;
  uint16_t rpm;
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  uint16_t distance;       // ✅ NEW FIELD
  char event[24];  // ← This must be 24 exactly
} TelemetryPacket;
//}; // <-- Replace with Car MAC Address
void logTelemetryEvent(const char* eventTag);
void startWirelessManager();

// Optional: provide access to latest packet if we want to display or debug
const TelemetryPacket& getLastSentTelemetry();

#endif
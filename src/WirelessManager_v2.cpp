#include "WirelessManager_v2.h"
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "ToneModule.h"
#include "ConfigManager.h"

#define PWM_NOISE_THRESHOLD 10 // Good as-is

#define PWM_NOISE_THRESHOLD 10
#define IMU_NOISE_THRESHOLD 5
#define ACCEL_NOISE_THRESHOLD 25 /

extern volatile unsigned int currentPWM;
extern int getPitch();
extern int getRoll();
extern int getYaw();
extern int getRPM();
extern int getAccelX();
extern int getAccelZ();
extern int getAccelY();

static TelemetryPacket lastTelemetry;
static TelemetryPacket lastSent = {0};
static char pendingEvent[24] = ""; // Shared event buffer
extern int lastDistance;
uint8_t peerAddress1[] = {0xDC, 0xDA, 0x0C, 0x21, 0x5F, 0xD4};

uint8_t peerAddress[] = {0xF0, 0xF5, 0xBD, 0x4A, 0xAE, 0x24};
// 0xDC, 0xDA, 0x0C, 0x21, 0x5F, 0xD4};


uint8_t camMac[] = {0xFC, 0xE8, 0xC0, 0xCD, 0xA4, 0xA8};

// DC:DA:0C:21:5F:D4
TaskHandle_t wirelessTaskHandle;

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");



}

bool isSignificantChange(const TelemetryPacket &a, const TelemetryPacket &b)
{
  bool eventChanged = strncmp(a.event, b.event, sizeof(a.event)) != 0;

  const int pwmThreshold = 15;
  const int rpmNoiseThreshold = (a.rpm > 1000) ? 400 : 150;

  bool pwmChanged = abs(a.pwm - b.pwm) > pwmThreshold;
  bool rpmChanged = abs(a.rpm - b.rpm) > rpmNoiseThreshold;

  return pwmChanged || rpmChanged || eventChanged; // ✅ IMU data removed from decision!
}
bool isSignificantChangeAll(const TelemetryPacket &a, const TelemetryPacket &b)
{
  bool eventChanged = strncmp(a.event, b.event, sizeof(a.event)) != 0;

  // --- Noise Filtering Thresholds ---
  const int pwmThreshold = 15; // Raised from 10 → 15
  const int imuThreshold = 10; // Raised from 5 → 10
  const int accelThreshold = 30;

  // --- Dynamic RPM threshold ---
  int rpmDelta = abs(a.rpm - b.rpm);
  int rpmNoiseThreshold = (a.rpm > 1000) ? 300 : 100;
  bool rpmChanged = rpmDelta > rpmNoiseThreshold;

  bool pwmChanged = abs(a.pwm - b.pwm) > pwmThreshold;
  bool pitchChanged = abs(a.pitch - b.pitch) > imuThreshold;
  bool rollChanged = abs(a.roll - b.roll) > imuThreshold;
  bool yawChanged = abs(a.yaw - b.yaw) > imuThreshold;
  bool accelXChanged = abs(a.accelX - b.accelX) > accelThreshold;
  bool accelYChanged = abs(a.accelY - b.accelY) > accelThreshold;
  bool accelZChanged = abs(a.accelZ - b.accelZ) > accelThreshold;
  if (pwmChanged || rpmChanged || pitchChanged || rollChanged || yawChanged ||
      accelXChanged || accelYChanged || accelZChanged || eventChanged)
  {
    Serial.printf("[WHY SENDING] pwm:%d rpm:%d pitch:%d roll:%d yaw:%d accelX:%d accelY:%d accelZ:%d event:%d\n",
                  pwmChanged, rpmChanged, pitchChanged, rollChanged, yawChanged,
                  accelXChanged, accelYChanged, accelZChanged, eventChanged);
  }
  if (pwmChanged || rpmChanged || pitchChanged || rollChanged || yawChanged ||
      accelXChanged || accelYChanged || accelZChanged || eventChanged)
  {
    Serial.printf("[CHANGE DETECTED] pwm:%d rpm:%d pitch:%d roll:%d yaw:%d accelX:%d accelY:%d accelZ:%d event:%d\n",
                  pwmChanged, rpmChanged, pitchChanged, rollChanged, yawChanged,
                  accelXChanged, accelYChanged, accelZChanged, eventChanged);
  }

  return pwmChanged || rpmChanged || pitchChanged || rollChanged || yawChanged ||
         accelXChanged || accelYChanged || accelZChanged || eventChanged;
}
void wirelessTask(void *parameter)
{
  Serial.println("[WirelessManager] Task running on Core 0.");

  static unsigned long lastSendTime = 0;
  static int quietCounter = 0;
  const int quietLimit = 3;                  // ✅ Require 3 quiet loops before sending again
  const unsigned long minSendInterval = 100; // ✅ 250ms throttle interval

  for (;;)
  {
    TelemetryPacket packet;

    packet.pwm = currentPWM;
    packet.rpm = getRPM();
    // packet.pitch = getPitch() * 100;
    // packet.roll = getRoll() * 100;
    packet.pitch = constrain((int16_t)getPitch(), -180, 180);
    packet.roll = constrain((int16_t)getRoll(), -180, 180);

    packet.yaw = getYaw() * 100;
    packet.accelX = getAccelX();
    packet.accelY = getAccelY();
    packet.accelZ = getAccelZ();
    packet.distance = lastDistance;

    strncpy(packet.event, pendingEvent, sizeof(packet.event));
    packet.event[sizeof(packet.event) - 1] = '\0';

    lastTelemetry = packet;

    unsigned long now = millis();

    const int arrLen = sizeof(pendingEvent) / sizeof(pendingEvent[0]);
    String strn = String(packet.event);
    strn.trim();

    
    bool isEvent = strn.length() >= 3;

    if (isEvent || (now - lastSendTime >= minSendInterval))
    {
      /// Serial.println(now - lastSendTime );
      bool significantChange = isSignificantChange(packet, lastSent);
      esp_err_t result1 = esp_now_send(peerAddress, (uint8_t *)&packet, sizeof(packet));
      //  //result = esp_now_send(camMac, (uint8_t *)&packet, sizeof(packet));
      // //esp_err_t result = esp_now_send(camMac, (uint8_t *)&packet, sizeof(packet));
      // // Serial.print("[WirelessManager] Sending packet result:");
      // //      Serial.println(result);
      if (result1 == ESP_OK)
       {
         lastSent = packet;
           lastSendTime = now;
         pendingEvent[0] = '\0'; // ✅ Clear event after successful send
         quietCounter = 0;       // ✅ Reset quiet counter after sending
       }
      esp_err_t result = esp_now_send(camMac, (uint8_t *)&packet, sizeof(packet));
      //   //result = esp_now_send(loggerMAC, (uint8_t*)&packet, sizeof(packet));
      // Serial.print("ESP-NOW send: ");
      //Serial.println(result == ESP_OK ? "Success" : "Failed");
      //  Serial.print("[WirelessManager] Sendingto logger packet result:");
      //       Serial.println(result);
      if (result == ESP_OK)
      {
        lastSent = packet;
        //   lastSendTime = now;
        pendingEvent[0] = '\0'; // ✅ Clear event after successful send
        quietCounter = 0;       // ✅ Reset quiet counter after sending
      }

      lastSendTime = now;
    }

    vTaskDelay(50 / portTICK_PERIOD_MS); // ✅ Smooth loop timing, avoids tight spin
  }
}

void processCommand(const char *command)
{

  Serial.print("[CMD] Processing: ");
  Serial.println(command);
  if (strcmp(command, "TONE:WARNING") == 0)
  {
    playTone(TONE_WARNING);
  }
  // else
  // {
  //   Serial.print("[CMD] Unknown command: ");
  //   Serial.println(command);
  // }
  else if (strncmp(command, "CONFIG:REV_LIMIT:", 17) == 0)
  {
    float value = atof(command + 17);
    currentConfig.reverseLimitPercent = constrain(value, 0.0f, 1.0f);
    saveConfig();
    Serial.println("[CMD] ✅ Reverse limit updated.");
  }
  else if (strncmp(command, "CONFIG:WHEELIE_SUPPRESS:", 24) == 0)
  {
    float value = atof(command + 24);
    currentConfig.wheelieSuppressRatio = constrain(value, 0.0f, 1.0f);
    saveConfig();
    Serial.println("[CMD] ✅ Wheelie suppression updated.");
  }
  else if (strcmp(command, "CONFIG:INDOOR:ON") == 0)
  {
    currentConfig.indoorModeEnabled = true;
    saveConfig();
    Serial.println("[CMD] ✅ Indoor mode ENABLED.");
  }
  else if (strcmp(command, "CONFIG:INDOOR:OFF") == 0)
  {
    currentConfig.indoorModeEnabled = false;
    saveConfig();
    Serial.println("[CMD] ✅ Indoor mode DISABLED.");
  }
  else if (strcmp(command, "CONFIG:RESET") == 0)
  {
    resetConfigToDefaults();
    Serial.println("[CMD] ✅ Configuration reset via remote command.");
  }
  else if (strcmp(command, "CONFIG:PRINT") == 0)
  {
    printConfig();
  }
  else if (strcmp(command, "CONFIG:SEND") == 0)
  {
    char buffer[128];
    snprintf(buffer, sizeof(buffer),
             "[CFG]REV:%.2f|WHEELIE:%.2f|INDOOR:%d",
             currentConfig.reverseLimitPercent,
             currentConfig.wheelieSuppressRatio,
             currentConfig.indoorModeEnabled ? 1 : 0);

    esp_now_send(peerAddress, (uint8_t *)buffer, strlen(buffer)); // Send back to middleman
  }
}

void onWirelessReceive(const uint8_t *mac, const uint8_t *data, int len)
{


  static uint8_t selfMac[6];
  static bool macInitialized = false;

  if (!macInitialized) {
    esp_read_mac(selfMac, ESP_MAC_WIFI_STA);  // Reads MAC of this device
    macInitialized = true;
  }

  // Skip messages sent by ourselves
  if (memcmp(mac, selfMac, 6) == 0) return;

  if (len >= 5 && strncmp((const char *)data, "[CMD]", 5) == 0)
  {
    char command[64];
    int cmdLen = len - 5;
    if (cmdLen >= sizeof(command))
      cmdLen = sizeof(command) - 1;

    memcpy(command, data + 5, cmdLen);
    command[cmdLen] = '\0';

    Serial.print("[Wireless] 📬 Command received: ");
    Serial.println(command);
    processCommand(command);
  }
}


bool addPeerToESPNow(const uint8_t *mac) {
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_now_del_peer(mac);  // clear stale entry if it exists

  esp_err_t result = esp_now_add_peer(&peerInfo);
  Serial.printf("[ESP-NOW] Add peer result: %d → %s\n", result, esp_err_to_name(result));
  return result == ESP_OK;
}

void startWirelessManager()
{
  Serial.println("[WirelessManager] Initializing...");

  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  Serial.print("car MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("[WirelessManager] ESP-NOW init failed.");
    return;
  }
  WiFi.mode(WIFI_STA);  
  esp_now_register_send_cb(onDataSent);

  // esp_now_peer_info_t peerInfo = {};
  // memcpy(peerInfo.peer_addr, peerAddress, 6); // ✅ 6 bytes for MAC address
  // peerInfo.channel = 0;                       // 0 = current WiFi channel
  // peerInfo.encrypt = false;

  // if (!esp_now_is_peer_exist(peerAddress))
  // {
  //   if (esp_now_add_peer(&peerInfo) != ESP_OK)
  //   {
  //     Serial.println("[WirelessManager] ❌ Failed to add peer1.");
  //     return;
  //   }
  //   else
  //   {
  //     Serial.println("[WirelessManager] ✅ Peer1 added.");
  //   }
  // }

//  uint8_t camMac[] = {0xFC, 0xE8, 0xC0, 0xCD, 0xA4, 0xA8};
//   esp_now_peer_info_t camPeer={};
//   memcpy(camPeer.peer_addr, camMac, 6);
//   camPeer.channel = 0;
//   camPeer.encrypt = false;
// //  esp_now_add_peer(&camPeer);
// esp_now_del_peer(camMac);  // Clean slate, even if it doesn't exist
//   if (!esp_now_is_peer_exist(camMac))
//   {

//     int peerRslt =esp_now_add_peer(&camPeer);
//     Serial.print("Adding rslt: ");
//     Serial.println(peerRslt);
//     Serial.printf("Adding peer result: %d -> %s\n", peerRslt, esp_err_to_name(peerRslt));
//     if (//esp_now_add_peer(&camPeer) 
//       peerRslt
//           != ESP_OK)
//     {
//       Serial.println("[WirelessManager] ❌ Failed to add peer2.");
//       //return;
//     }
//     else
//     {
//       Serial.println("[WirelessManager] ✅ Peer2 added.");
//     }
//   }
if (!addPeerToESPNow(peerAddress)) {
  Serial.println("❌ Failed to add ESP32-CAM as peer!");
}

  if (!addPeerToESPNow(camMac)) {
    Serial.println("❌ Failed to add ESP32-CAM as peer!");
  }


  xTaskCreatePinnedToCore(
      wirelessTask,
      "WirelessTask",
      4096,
      NULL,
      1,
      &wirelessTaskHandle,
      0);

  esp_now_register_recv_cb(onWirelessReceive);
}

const TelemetryPacket &getLastSentTelemetry()
{
  return lastTelemetry;
}

#include "BluetoothRPM.h"


bool bleConnected = false; 



BluetoothRPM::BluetoothRPM() {
  targetName = "HW_BLE7881798";
  rpmCallback = nullptr;
}

void BluetoothRPM::setTargetName(const char* name) {
  targetName = name;
}

void BluetoothRPM::onRPM(void (*callback)(int rpm)) {
  rpmCallback = callback;
}

void onBLEConnected()
{
    Serial.println("[BLE] Connected");
    bleConnected = true;
    Serial.println("[BLE] Connected — enabling PWM output.");
  // enablePWMOutput();
}

void onBLEDisconnected()
{
    Serial.println("[BLE] Disconnected");
    bleConnected = false;
    Serial.println("[BLE] Disconnected — disabling PWM output.");
    //disablePWMOutput();
}
void BluetoothRPM::begin() {
  if (!BLE.begin()) {
    Serial.println("BLE failed to start.");
    while (1);  // Stop everything if BLE fails
  }

  Serial.println("BLE started, scanning for devices...");
  BLE.scan();   // 👈 Starts the scan!
}
// void BluetoothRPM::begin() {
//   if (!BLE.begin()) {
//     Serial.println("BLE failed to start.");
//     while (1);
//   }

//   Serial.println("BLE started, scanning for devices...");
//   BLE.scan();
// }
void BluetoothRPM::update() {
  switch (btState) {
    case BT_SCAN: {
          //   Serial.println("scan");

      peripheral = BLE.available();
      if (peripheral && peripheral.localName() && strcmp(peripheral.localName().c_str(), targetName) == 0) {
        BLE.stopScan();
        Serial.println("Target device found. Connecting...");
        btState = BT_CONNECT;
      }
      break;
    }

    case BT_CONNECT: {
       //Serial.println("connect");

      if (peripheral.connect()) {
        Serial.println("Connected!");
        btState = BT_DISCOVER;
      } else {
        Serial.println("Connect failed, rescanning...");
        BLE.scan();
        btState = BT_SCAN;
      }
      break;
    }

    case BT_DISCOVER: {
      // Serial.println("Discover");
      if (peripheral.discoverAttributes()) {
        targetChar = peripheral.characteristic("f1f2");
        if (targetChar && targetChar.canSubscribe()) {
          Serial.println("Discovered target characteristic. Subscribing...");
          btState = BT_SUBSCRIBE;
        } else {
          Serial.println("Invalid characteristic. Disconnecting...");
          peripheral.disconnect();
          BLE.scan();
          btState = BT_SCAN;
        }
      } else {
        Serial.println("Attribute discovery failed. Disconnecting...");
        peripheral.disconnect();
        BLE.scan();
        btState = BT_SCAN;
      }
      break;
    }

    case BT_SUBSCRIBE: {
      //       Serial.println("scribe");

      if (targetChar.subscribe()) {
        Serial.println("Subscribed. Waiting for updates...");
        btState = BT_READ;
      } else {
        Serial.println("Subscription failed. Disconnecting...");
        peripheral.disconnect();
        BLE.scan();
        btState = BT_SCAN;
      }
      break;
    }

    case BT_READ: {
       //      Serial.println("read");

      if (!peripheral.connected()) {
        Serial.println("Disconnected. Rescanning...");
        BLE.scan();
        btState = BT_SCAN;
        break;
      }

      if (targetChar.valueUpdated()) {
        uint8_t buffer[36];
        targetChar.readValue(buffer, sizeof(buffer));
        uint16_t rawThrottle = (buffer[14] << 8) | buffer[15];
        int rpm = rawThrottle - 68;

//         Serial.print("[BluetoothRPM] New RPM raw: ");
// Serial.println(rpm);
// Serial.print("[BluetoothRPM] Callback pointer valid? ");
// Serial.println(rpmCallback != nullptr ? "YES" : "NO");
        if (rpm < 10) rpm = 0;

        if (rpmCallback) {
          rpmCallback(rpm);
        }
      }
      break;
    }
  }
}
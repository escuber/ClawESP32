#ifndef BLUETOOTH_RPM_H
#define BLUETOOTH_RPM_H

#include <ArduinoBLE.h>

class BluetoothRPM {
public:

//extern bool bleConnected;  
  BluetoothRPM();
  void begin();
  void update();

  // Set the name of the target BLE device
  void setTargetName(const char* name);

  // Hook to provide callback when new RPM is available
  void onRPM(void (*callback)(int rpm));

private:
  const char* targetName;
  void (*rpmCallback)(int rpm);

  enum BluetoothState {
    BT_SCAN,
    BT_CONNECT,
    BT_DISCOVER,
    BT_SUBSCRIBE,
    BT_READ
  };

  BluetoothState btState = BT_SCAN;
  BLEDevice peripheral;
  BLECharacteristic targetChar;
};

#endif
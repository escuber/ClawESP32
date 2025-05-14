#include <Adafruit_VL53L1X.h>
#include <Wire.h>

#define I2C2_SDA 18
#define I2C2_SCL 19

TwoWire I2C_VL53 = TwoWire(1);
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();

int lastDistance = -1;
int lastRate = 0;

void distanceManagerBegin() {
  Serial.println(F("🔍 VL53L1X Initializing..."));
  I2C_VL53.begin(I2C2_SDA, I2C2_SCL);
  delay(100);  // Settle time

  if (!vl53.begin(0x29, &I2C_VL53)) {
    Serial.print(F("❌ Sensor init failed: "));
    Serial.println(vl53.vl_status);
    while (1);
  }

  vl53.stopRanging();  // Ensure we're in CONFIG mode

  VL53L1X_ERROR err = vl53.startRanging();
  Serial.print(F("StartRanging: ")); Serial.println(err);

  err = vl53.setTimingBudget(80);  // Safe tested value
  Serial.print(F("TimingBudget: ")); Serial.println(err);

  if (err != 0) {
    Serial.println(F("❌ Failed to start ranging"));
    while (1);
  }

  Serial.println(F("✅ VL53L1X Ready"));
}

void distanceManagerUpdate() {
  static unsigned long lastTime = 0;
  static int lastDist = -1;

  if (vl53.dataReady()) {
    int16_t dist = vl53.distance();

    if (dist == -1 || dist < 0) {
      Serial.print(F("[DistanceManager] Read error: "));
      Serial.println(vl53.vl_status);
    } else {
      unsigned long now = millis();
      unsigned long dt = now - lastTime;
      lastTime = now;

      int rate = 0;
      if (lastDistance > 0 && dt > 0) {
        rate = (lastDistance - dist) * 1000 / dt;
      }

      lastDistance = dist;
      lastRate = rate;

      // Serial.print("Distance: ");
      // Serial.print(dist);
      // Serial.print(" mm | Rate: ");
      // Serial.print(rate);
      // Serial.println(" mm/s");
    }

    vl53.clearInterrupt();
  }

  static unsigned long lastDebug = 0;
  // if (millis() - lastDebug > 5000) {
  //   Serial.println("[DistanceManager] still alive...");
  //   lastDebug = millis();
  // }
}

int getLatestDistance() {
  return lastDistance;
}

int getLatestRate() {
  return lastRate;
}
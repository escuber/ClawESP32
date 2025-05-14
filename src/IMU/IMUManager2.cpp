#include "IMUManager2.h"
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <deque>
#include <string.h>
#include "ToneModule.h"

#define EEPROM_SIZE 64
#define CALIBRATION_ADDRESS 0

std::deque<float> accelHistory;
const size_t accelWindowSize = 10;
const int FLIP_CONFIRM_COUNT = 5;
int flipDetectionCounter = 0;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
volatile float lastKnownPitch;
const float TILT_THRESHOLD_DEG = 60.0;
const float CRASH_ACCEL_THRESHOLD_G = 2.0;
const unsigned long CRASH_DEBOUNCE_MS = 1000;

static volatile int latestPitch = 0;
static volatile int latestRoll = 0;
static volatile int latestYaw = 0;

static volatile int16_t latestAccelX = 0;
static volatile int16_t latestAccelY = 0;
static volatile int16_t latestAccelZ = 0;
static volatile float pitchRate = 0;
static unsigned long lastOffsetPrint = 0;
float idlePitchOffset = 0;
unsigned long lastZeroCheck = 0;
bool offsetConfirmed = false;

unsigned long lastCrashTime = 0;
volatile float lastKnownAccelerationG;
const unsigned long offsetCheckInterval = 3000; // 3 sec of idle required

extern volatile unsigned int currentPWM;
extern int getRPM();

int getPitch() { return latestPitch; }
int getRoll() { return latestRoll; }
int getYaw() { return latestYaw; }

int getAccelX() { return latestAccelX; }
int getAccelZ() { return latestAccelZ; }
int getAccelY() { return latestAccelY; }
//float getSmoothPitchRate();
float getSmoothedAccel()
{
  if (accelHistory.empty())
    return 1.0;
  float sum = 0;
  for (float a : accelHistory)
  {
    sum += a;
  }
  return sum / accelHistory.size();
}

float safeReadPitchRateDegPerSec()
{
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  float rateRad = gyro.y();

  if (!isfinite(rateRad) || fabs(rateRad) > 50.0f)
  {
    Serial.println("[IMU] ⚠️ Rejected raw gyro.y() read.");
    return 0.0f;
  }

  float rateDeg = rateRad * 180.0f / M_PI;
  if (!isfinite(rateDeg) || fabs(rateDeg) > 2000.0f)
  {
    Serial.println("[IMU] ⚠️ Rejected gyro deg/sec conversion.");
    return 0.0f;
  }

  return rateDeg;
}
void saveCalibrationData()
{
  adafruit_bno055_offsets_t calData;
  if (!bno.getSensorOffsets(calData))
  {
    Serial.println("[IMU] Failed to get sensor offsets.");
    return;
  }

  EEPROM.put(CALIBRATION_ADDRESS, calData);
  if (!EEPROM.commit())
  {
    Serial.println("[IMU] EEPROM commit failed!");
    return;
  }

  Serial.println("[IMU] Calibration saved to EEPROM.");

  adafruit_bno055_offsets_t verifyData;
  EEPROM.get(CALIBRATION_ADDRESS, verifyData);

  if (memcmp(&calData, &verifyData, sizeof(calData)) == 0)
  {
    bno.setSensorOffsets(verifyData);
    Serial.println("[IMU] Calibration reloaded and applied.");
  }
  else
  {
    Serial.println("[IMU] Calibration verification failed!");
  }

  static unsigned long lastIMUPrint = 0;

  if (millis() - lastIMUPrint > 1000)
  {
    lastIMUPrint = millis();

    Serial.print("[IMU] Pitch: ");
    Serial.print(getPitch());
    Serial.print(" | Roll: ");
    Serial.print(getRoll());
    Serial.print(" | Yaw: ");
    Serial.print(getYaw());

    Serial.print(" | AccelX: ");
    Serial.print(getAccelX());
    Serial.print(" | AccelY: ");
    Serial.print(getAccelY());
    Serial.print(" | AccelZ: ");
    Serial.println(getAccelZ());
  }
}
bool loadCalibrationData()
{

  adafruit_bno055_offsets_t calData;
  EEPROM.get(CALIBRATION_ADDRESS, calData);

  Serial.println("[IMU] 🔍 Retrieved stored calibration:");
  Serial.printf("  Accel Offset X: %d\n", calData.accel_offset_x);
  Serial.printf("  Gyro Offset X:  %d\n", calData.gyro_offset_x);
  Serial.printf("  Mag Offset X:   %d\n", calData.mag_offset_x);

  if (calData.accel_offset_x == 0 && calData.mag_offset_x == 0)
  {
    Serial.println("[IMU] ❌ No valid calibration data found.");
    return false;
  }
// Set external crystal if you're using it
bno.setExtCrystalUse(true);
delay(10);

// ✅ Go into FUSION MODE FIRST
bno.setMode(OPERATION_MODE_NDOF);
delay(50);  // Give it time to engage fusion mode

// ✅ Now apply calibration (with fusion running)
bno.setSensorOffsets(calData);
delay(10);

Serial.println("[IMU] Calibration data loaded.");

// Optional: Read back calibration state
uint8_t sys, gyro, accelStatus, mag;
bno.getCalibration(&sys, &gyro, &accelStatus, &mag);
  // bno.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_CONFIG);
  // // adafruit_bno055_opmode_t::OPERATION_MODE_CONFIG
  // delay(25);
  // bno.setSensorOffsets(calData);
  // delay(10);
  // bno.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_NDOF);
  // delay(20);
  // Serial.println("[IMU] Calibration data loaded.");

  // uint8_t sys, gyro, accelStatus, mag;
  // bno.getCalibration(&sys, &gyro, &accelStatus, &mag);
//  Serial.printf(" | Calib: SYS=%d GYRO=%d ACC=%d MAG=%d\n", sys, gyro, accelStatus, mag);

  return true;
}

float itchRate_zero_offset = 0;
void setPitchRateOffsets()
{
  float sum = 0;
  for (int x = 0; x < 20; x++)
  {
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    float pitch_rate_rad_per_sec = gyro.y();
    float rate = pitch_rate_rad_per_sec * 180.0 / M_PI;
    if (!isfinite(rate))
      rate = 0;
    sum += rate;
    delay(50);
  }
  itchRate_zero_offset = sum / 20.0f;

  Serial.print("[IMU] Pitch rate offset initialized to: ");
  Serial.println(itchRate_zero_offset);
}

float getSmoothPitchRate()
{
  static float previous_filtered_rate = 0.0f;

  float adjustedRate = pitchRate - itchRate_zero_offset;
  if (!isfinite(pitchRate))
  {
    Serial.println("[IMU] 🚫 pitchRate was NaN or inf. Reset to 0.");
    pitchRate = 0;
  }
  float filtered_pitch_rate = 0.9f * previous_filtered_rate + 0.1f * adjustedRate;

  previous_filtered_rate = filtered_pitch_rate;
  if (fabs(adjustedRate) > 500.0f)
  {
    Serial.println("[IMU] ⚠️ Spike in pitch rate filtered.");
    adjustedRate = 0;
  }
  return filtered_pitch_rate;
}
void imuManagerBegin()
{
  EEPROM.begin(EEPROM_SIZE);
  Wire.begin();

  if (!bno.begin())
  {
    Serial.println("[IMU] BNO055 Connection Failed!");
    return;
  }
  else
  {
    Serial.println("[IMU] BNO055 Connected!");
  }

  //bno.setExtCrystalUse(true);

  // bno.setAxisRemap(1, 0, 2);  // Y, X, Z
  // bno.setAxisSign(0, 1, 0);
  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1); // for Y,X,Z
  bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P1);    // for X+,Y-,Z+

  Serial.println("[IMU] AXSIS REMAPPING done.");
  //   bno.setAxisRemap(Adafruit_BNO055::REMAP_Y,
  //     Adafruit_BNO055::REMAP_X,
  //     Adafruit_BNO055::REMAP_Z);
  // bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_POSITIVE,
  //    Adafruit_BNO055::REMAP_SIGN_NEGATIVE,
  //    Adafruit_BNO055::REMAP_SIGN_POSITIVE);

  if (loadCalibrationData())
  {
    Serial.println("[IMU] Calibration applied.");
  }
  else
  {
    Serial.println("[IMU] Running without saved calibration.");
  }
  // bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);
//  bno.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_NDOF);
  delay(50); // Give it time to switch mode

  delay(1000); // Give IMU time to stabilize
  // setPitchRateOffsets();
  uint8_t sys, gyro, accelStatus, mag;
  bno.getCalibration(&sys, &gyro, &accelStatus, &mag);
  if (gyro < 1 || accelStatus < 1)
  {
    Serial.println("[IMU] ⚠️ Calibration too low for stable offset. Skipping.");
    itchRate_zero_offset = 0;
  }
  else
  {
    setPitchRateOffsets();
  }
  // setPitchRateOffsets();
}

void maybeUpdateIdlePitchOffset()
{
  static unsigned long idleStart = 0;

  if (currentPWM == 1500 && getRPM() == 0)
  {
    if (idleStart == 0)
      idleStart = millis();

    if (millis() - idleStart >= offsetCheckInterval && !offsetConfirmed)
    {
      float sum = 0;
      for (int i = 0; i < 10; i++)
      {
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        // sum += euler.yx();
        sum += euler.y();
        delay(10);
      }
      float newOffset = sum / 10.0;

      // 🔐 Reasonableness check
      if (fabs(newOffset) < 5.0)
      {
        idlePitchOffset = newOffset;
        Serial.print("[IMU] Idle pitch offset set to: ");
        Serial.println(idlePitchOffset);

        // 🔔 Audible confirmation
        tone(8, 880, 300); // Use a valid tone pin
        offsetConfirmed = true;
      }
      else if (millis() - lastOffsetPrint > 3000)
      {
        if (millis() - lastOffsetPrint > 3000)
        {
          Serial.print("[IMU] Ignored pitch offset (too large): ");
          Serial.println(newOffset);
          lastOffsetPrint = millis();
        }

        lastOffsetPrint = millis();
      }
    }
  }
  else
  {
    idleStart = 0;
    offsetConfirmed = false;
  }
}
void imuManagerUpdate()
{

  maybeUpdateIdlePitchOffset();

  // --- Euler angles (Pitch, Roll, Yaw)
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  if (!isfinite(euler.y()) || fabs(euler.y()) > 180.0f)
  {
    Serial.println("[IMU] 🚫 Invalid pitch read. Skipping update.");
    latestPitch = 0;
  }
  else
  {
    latestPitch = (int16_t)(euler.y() - idlePitchOffset);
  }

  latestRoll = (int16_t)(euler.x());
  latestYaw = (int16_t)(euler.z());

  // --- Linear acceleration
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  latestAccelX = (int16_t)(accel.x() * 100);
  latestAccelY = (int16_t)(accel.y() * 100);
  latestAccelZ = (int16_t)(accel.z() * 100);

  // --- Gyroscope-based pitch rate
  uint8_t sys, gyroCal, accelCal, magCal;
  bno.getCalibration(&sys, &gyroCal, &accelCal, &magCal);

  if (gyroCal < 1)
  {
    pitchRate = 0;
    static unsigned long lastWarn = 0;
    if (millis() - lastWarn > 2000)
    {
      Serial.println("[IMU] ⚠️ Skipping pitchRate update — gyro not calibrated.");
      lastWarn = millis();
    }
    return;
  }

  sensors_event_t gyroEvent;
  bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);
  float rateRad = gyroEvent.gyro.y;

  if (!isfinite(rateRad) || fabs(rateRad) > 50.0f)
  {
    pitchRate = 0;
    Serial.println("[IMU] 🚫 Invalid gyro.y — reset pitchRate to 0.");
  }
  else
  {
    pitchRate = rateRad * 180.0f / M_PI;
  }

  // --- Debug calibration level every second
  static unsigned long lastCalPrint = 0;
  if (millis() - lastCalPrint > 1000)
  {
    lastCalPrint = millis();
    //Serial.printf("[CAL] SYS=%d GYRO=%d ACC=%d MAG=%d\n", sys, gyroCal, accelCal, magCal);
    static unsigned long lastIMUPrint = 0;

    if (millis() - lastIMUPrint > 1000)
    {
      lastIMUPrint = millis();

      // Serial.print("[IMU] Pitch: ");
      // Serial.print(getPitch());
      // Serial.print(" | Roll: ");
      // Serial.print(getRoll());
      // Serial.print(" | Yaw: ");
      // Serial.print(getYaw());

      // Serial.print(" | AccelX: ");
      // Serial.print(getAccelX());
      // Serial.print(" | AccelY: ");
      // Serial.print(getAccelY());
      // Serial.print(" | AccelZ: ");
      // Serial.println(getAccelZ());
    }
  }
}

// void imuManagerUpdate()
// {
//   maybeUpdateIdlePitchOffset();

//   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//   // latestPitch = (int16_t)(euler.x());
//   latestPitch = (int16_t)(euler.y() - idlePitchOffset);

//   latestRoll = (int16_t)(euler.x());
//   latestYaw = (int16_t)(euler.z());

//   imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
//   latestAccelX = (int16_t)(accel.x() * 100);
//   latestAccelY = (int16_t)(accel.y() * 100);
//   latestAccelZ = (int16_t)(accel.z() * 100);

//   imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

//   // Check for sanity before using
//   if (!isfinite(gyro.y()) || fabs(gyro.y()) > 100.0f)
//   {
//     pitchRate = 0;
//     Serial.println("[IMU] ⚠️ Invalid gyro reading. Resetting pitchRate to 0.");
//   }
//   else
//   {
//     float pitch_rate_rad_per_sec = gyro.y();
//     pitchRate = pitch_rate_rad_per_sec * 180.0 / M_PI;
//     // pitchRate = pitch_rate_rad_per_sec * 180.0 / M_PI;
//     pitchRate = safeReadPitchRateDegPerSec();

//   }
// }
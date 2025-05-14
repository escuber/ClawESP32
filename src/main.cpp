#define BLE_OFF 0

#include <ArduinoBLE.h> // ✅ ArduinoBLE for safe BLE operations
#include "DistanceManager.h"
#include <ESP32Servo.h>
#ifdef BLE_OFF
#include "BluetoothRPM.h"
BluetoothRPM rpmReader;
#endif
#include "ToneModule.h"
#include "IMU/IMUManager2.h"
#include "WirelessManager_v2.h"
#include "AdjustOutput.h"
#include "ConfigManager.h"

#include "MotionTracker.h"

// #include "WheelieLogic.h"
#define MUX_PRESENT false

#define PWM_INPUT_PIN 2
#define PWM_OUTPUT_PIN 3
#define MUX_SELECT_PIN 6 // ✅ Adjust this pin to match your wiring

Servo throttleServo;
// unsigned int currentPWM;
volatile unsigned int currentPWM;
int currentRPM = -1;
volatile unsigned long measuredPulseWidth = 1500;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
unsigned long pulseStartTime = 0;
MotionTracker motionTracker;
TaskHandle_t echoTaskHandle;

BLEDevice peripheral;
BLECharacteristic rpmCharacteristic;
const char *targetDeviceName = "HW_BLE7881798"; // ✅ Correct BLE device
const char *targetCharacteristicUUID = "f1f2";  // ✅ Correct short characteristic UUID

const int ESC_PWM_CHANNEL = 0;
const int ESC_PWM_FREQ = 50;
const int ESC_PWM_RES = 16;
const int ESC_PWM_PIN = 3;

int lastRPM = -1;
bool deviceConnected = false;

extern int getPitch();
extern int getRoll();
extern int getYaw();
extern int getRPM();
extern int getAccelX();
extern int getAccelZ();
extern int getAccelY();
extern float getSmoothPitchRate();
void disablePWMOutput()
{
  throttleServo.detach();      // Fully detach — ESC sees no signal
  pinMode(ESC_PWM_PIN, INPUT); // Float the pin
}

void enablePWMOutput()
{
  throttleServo.attach(ESC_PWM_PIN, 1000, 2000);
  throttleServo.writeMicroseconds(1500); // Safe neutral on enable
}
void setupMuxControl()
{
#if MUX_PRESENT
  pinMode(MUX_SELECT_PIN, OUTPUT);
  digitalWrite(MUX_SELECT_PIN, LOW); // ✅ Safe default: Receiver controls ESC at startup
#endif
}

void processCommand(const char *command);

void checkESCConnectionAndFlipMux()
{
  static bool lastEscState = false;
  static unsigned long lastFlipTime = 0;
  const unsigned long debounceDelay = 500; // 500ms debounce time

  bool escAlive = currentRPM > 100; // ✅ Use your RPM logic here, adjust threshold if needed

  // if (escAlive != lastEscState && millis() - lastFlipTime > debounceDelay)
  // {
  //   lastEscState = escAlive;
  //   // digitalWrite(MUX_SELECT_PIN, escAlive ? HIGH : LOW);
  //   Serial.printf("[MUX] Switched control to: %s\n", escAlive ? "MCU" : "Receiver");
  //   lastFlipTime = millis();
  // }
}
int lastMicros = 0;
void setThrottlePWM(int micros)
{
  static int lastRPM = -1;
  throttleServo.writeMicroseconds(micros);

  if (lastMicros != lastMicros)
  {
    Serial.print("                                     ActualPWM:");
    Serial.println(micros);
    lastMicros = micros;
  }

  // int duty = map(micros, 1000, 2000, 3276, 6553);
  // ledcWrite(ESC_PWM_CHANNEL, duty);
}
static bool escInitialized = false;

void rpmCallback(int rpm)
{
  // Serial.print("[RPM CALLBACK] Got RPM: ");
  // Serial.println(rpm);

  if (rpm > 8000)
    rpm = 0;
  currentRPM = rpm;

  if (currentRPM > 10000)
  {
    Serial.println("[RPM WARNING] Spike detected — ignored");
    currentRPM = lastRPM;
  }
  // Serial.print("[RPM] ");
  // Serial.println(rpm);

  if (abs(rpm - lastRPM) > 25)
  { // Only print if RPM changes more than 25
    // Serial.print("[RPM] ");
    // Serial.println(rpm);
  }
  lastRPM = rpm;
  if (!escInitialized && rpm == 0)
  {
    enablePWMOutput();
    escInitialized = true; // ✅ Only initialize once per power cycle
    Serial.println("[ESC INIT] ESC initialized via RPM callback.");
  }
}
int getRPM() { return currentRPM; }
void IRAM_ATTR throttleInterrupt()
{
  if (digitalRead(PWM_INPUT_PIN) == HIGH)
  {
    pulseStartTime = micros();
  }
  else
  {
    unsigned long endTime = micros();
    portENTER_CRITICAL_ISR(&mux);
    measuredPulseWidth = endTime - pulseStartTime;
    portEXIT_CRITICAL_ISR(&mux);

    // Serial.println(measuredPulseWidth);
  }
}

unsigned long lastPulseLog = 0;
void echoTask(void *parameter)
{
  unsigned long lastLoggedPulse = 1500;
  lastPulseLog = millis();
  for (;;)
  {
    portENTER_CRITICAL(&mux);
    currentPWM = measuredPulseWidth;
    portEXIT_CRITICAL(&mux);

    if (currentPWM < 900 || currentPWM > 2100)
    {
      currentPWM = 1500;
    }

    distanceManagerUpdate();
    motionTracker.update(getAccelX(), getPitch(), getSmoothPitchRate(), currentPWM);

    int adjustedPWM = AdjustOutputEvaluate(currentPWM);

    if (millis() - lastPulseLog > 1000)
    {
      //   Serial.print("[Echo] PWM: ");
      //   Serial.print(currentPWM);
      //   Serial.print(" | Adjusted: ");
      //   Serial.print(adjustedPWM);
      //Serial.print(" | Main RPM: ");
      //Serial.println(currentRPM);
      lastPulseLog = millis();
    }
    setThrottlePWM(adjustedPWM);
    checkESCConnectionAndFlipMux(); // ✅ Add mux check after setting throttle
    imuManagerUpdate();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

unsigned long bleStartTime = 0;
bool bleStarted = false;
void setup()
{
  Serial.begin(115200);
  setupMuxControl();
  loadConfig();
  printConfig();

  pinMode(PWM_INPUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_INPUT_PIN), throttleInterrupt, CHANGE);

  AdjustOutputInit();
  // initToneModule(6); // ✅ Add your buzzer pin here

  delay(1000); // Let IMU power up before BLE starts
  imuManagerBegin();

  delay(500);
  distanceManagerBegin();
  // I2C buffer settle time
  // rpmReader.begin();

  rpmReader.onRPM(rpmCallback);
  bleStartTime = millis();

  Serial.println("[ESC] Sending init PWM...");

  xTaskCreatePinnedToCore(
      echoTask, "EchoTask", 4192, NULL, 1, &echoTaskHandle, 1);

  startWirelessManager(); // ✅ Only one call now!
}
// BLEDevice peripheral;

void loop()
{
  // all for debugging

#if !BLE_OFF
  if (!bleStarted && millis() - bleStartTime > 5000)
  {
    Serial.println("🧠 Starting BLE...");
    rpmReader.begin(); // Or BLE.begin(), depending on your code
    bleStarted = true;
  }

  if (bleStarted)
  {

    rpmReader.update();
  }

#endif
  // 🛠️ ADD THIS:
  if (Serial.available())
  {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0)
    {
      processCommand(input.c_str());
    }
  }

  static unsigned long bleTimeoutStart = millis();
  if (!escInitialized && millis() - bleTimeoutStart > 7000)
  {
    Serial.println("[ESC INIT] Fallback triggered — no RPM after 7s");
    enablePWMOutput();
    escInitialized = true;
  }
  //  rpmReader.update(); // Drive the BLE state machine!

  // if (Serial.available()) {
  //   String input = Serial.readStringUntil('\n');
  //   input.trim();  // Clean up newline characters

  //   if (input == "save") {
  //     saveCalibrationData();  // 🔥 this is the one
  //   }
  // }
}
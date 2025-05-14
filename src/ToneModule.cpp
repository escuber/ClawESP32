#include <Arduino.h>
 #include "ToneModule.h"
const int BUZZER_PIN = 6;
const int BUZZER_CHANNEL = 1;    // ✅ MUST be different from your throttle PWM channel
const int BUZZER_TIMER = 1;      // ✅ Separate timer group if possible (avoid throttle timer!)

void setupBuzzer()
{
    ledcSetup(BUZZER_CHANNEL, 1000, 8);  // 1kHz, 8-bit resolution
    ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
    ledcWrite(BUZZER_CHANNEL, 0);        // Start OFF
}

void playBuzzerTone(int frequency)
{
    ledcSetup(BUZZER_CHANNEL, frequency, 8);  // Update frequency if needed
    ledcWrite(BUZZER_CHANNEL, 128);          // 50% duty cycle for square wave
}

void stopBuzzerTone()
{
    ledcWrite(BUZZER_CHANNEL, 0);            // Fully off
}

void playTone(ToneEvent event) {
    playBuzzerTone(1000);   // 1kHz tone
    delay(200);             // ✅ Blocking is fine here at startup or alert tone
    stopBuzzerTone();
}
unsigned long toneStartTime = 0;
bool toneActive = false;

void playToneNonBlocking(int frequency, int durationMs)
{
    playBuzzerTone(frequency);
    toneStartTime = millis();
    toneActive = true;
}

void updateTone()
{
    if (toneActive && millis() - toneStartTime > 200)  // Example 200ms
    {
        stopBuzzerTone();
        toneActive = false;
    }
}
// #include "ToneModule.h"
// #include <Arduino.h>

// static int tonePin = 6;

void initToneModule(int pin) {
    setupBuzzer();
 // tonePin = pin;
  //pinMode(tonePin, OUTPUT);
}

// void playTone(ToneEvent event) {
//   if (tonePin == -1) return;  // Not initialized

//   switch (event) {
//     case TONE_IDLE_OFFSET_SET:
//       tone(tonePin, 880, 300);  // High short beep
//       break;

//     case TONE_BLUETOOTH_NOT_CONNECTED:
//       tone(tonePin, 440, 150);
//       delay(200);
//       tone(tonePin, 440, 150);
//       break;

//     case TONE_ERROR:
//       tone(tonePin, 200, 400);
//       break;

//     case TONE_WARNING:
//       tone(tonePin, 600, 200);
//       delay(100);
//       tone(tonePin, 600, 200);
//       break;

//     default:
//       // No tone
//       break;
//   }
// }
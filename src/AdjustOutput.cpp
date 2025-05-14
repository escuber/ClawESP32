
#include "AdjustOutput.h"
#include "BluetoothRPM.h"
#include "IMU/IMUManager2.h"
#include <Arduino.h>
#include "DistanceManager.h"
#include "ConfigManager.h"
// #include "MotionTracker.h"

#include "MotionTracker.h"
extern MotionTracker motionTracker;
extern bool bleConnected;
extern volatile unsigned int currentPWM;
// Degrees as int
extern float getSmoothPitchRate(); // Degrees/sec as int
// extern int getRPM();
extern int currentRPM;
extern int getPitch();
extern int getRoll();
extern int getYaw();
extern int getRPM();
extern int getAccelX();
extern int getAccelZ();
extern int getAccelY();

const int NEUTRAL_PWM = 1500;
const int MAX_PWM = 2000;
const int MIN_PWM = 1000;
const float MAX_SAFE_PITCH = 5.0f;
const float CRITICAL_PITCH = 8.0f;
const float PITCH_RATE_LIMIT = 10.0f;
const float Kp = 0.8f;
const float Ki = 0.1f;
const float Kd = 0.4f;
const int REVERSE_SOFT_START_THRESHOLD = 1520; // where ESC starts responding
const int REVERSE_SOFT_CLAMP = 1550;

extern ConfigSettings currentConfig;
static float integratedError = 0;
static float previousError = 0;
static float previousThrottle = NEUTRAL_PWM;
static unsigned long lastTime = 0;

void AdjustOutputInit()
{
    integratedError = 0;
    previousError = 0;
    previousThrottle = NEUTRAL_PWM;
    lastTime = millis();
}

static unsigned long lastPrintLog = 0;

int wheelieThrottlePIDold(int basePWM)
{
    float pitch = getPitch();
    float pitchRate = getSmoothPitchRate();

    if (pitch < MAX_SAFE_PITCH)
        return currentPWM;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;

    float error = pitch;
    float derivative = (error - previousError) / dt;

    if (pitch > CRITICAL_PITCH || pitchRate > PITCH_RATE_LIMIT)
    {
        integratedError = 0;
        previousError = error;
        previousThrottle = NEUTRAL_PWM + (basePWM - NEUTRAL_PWM) * currentConfig.wheelieSuppressRatio;
        return (int)previousThrottle;
    }

    if (abs(error) < MAX_SAFE_PITCH)
        integratedError += error * dt;
    else
        integratedError = 0;

    float adjustment = Kp * error + Ki * integratedError + Kd * derivative;
    float newThrottle = basePWM - adjustment;

    newThrottle = constrain(newThrottle, MIN_PWM, MAX_PWM);
    newThrottle = 0.7f * newThrottle + 0.3f * previousThrottle;

    previousThrottle = newThrottle;
    previousError = error;

    if (millis() - lastPrintLog > 500)
    {
        Serial.print("[AdjustEval] P: ");
        Serial.print(pitch);
        Serial.print(" PR: ");
        Serial.print(pitchRate);
        Serial.print(" E: ");
        Serial.print(error);
        Serial.print(" Der: ");
        Serial.print(derivative);
        Serial.print(" Out: ");
        Serial.print(newThrottle);
        Serial.print(" CurPWM: ");
        Serial.println(currentPWM);
        Serial.print("            BSEPWM: ");
        Serial.print(basePWM);
        Serial.print(" Adj: ");
        Serial.println(adjustment);
        lastPrintLog = millis();
    }

    return (int)(currentPWM * 0.7f + newThrottle * 0.3f);
}

int wheelieThrottlePIDnotes(int basePWM)
{
    float pitch = getPitch();
    float pitchRate = getSmoothPitchRate();
    int rpm = getRPM();

    if (pitch > 5.0f)
    {
        int suppressed = 1300;
        Serial.printf("[WHEELIE TEST] 🔥 Triggered! Pitch=%.2f → PWM=%d\n", pitch, suppressed);
        return suppressed;
    }

    if (pitch > 3)
        Serial.printf("[WHEELIE TEST] Pitch=%.2f → No action\n", pitch);

    if (millis() - lastPrintLog > 500)
    {
        Serial.print("[AdjustEval] P: ");
        Serial.print(pitch);
        Serial.print(" PR: ");
        Serial.println(pitchRate);
        //       Serial.print(" E: ");
        // Serial.print(error);
        // Serial.print(" Der: ");
        // Serial.print(derivative);
        // Serial.print(" Out: ");
        // Serial.print(newThrottle);
        // Serial.print(" CurPWM: ");
        // Serial.println(currentPWM);
        // Serial.print("            BSEPWM: ");
        // Serial.print(basePWM);
        // Serial.print(" Adj: ");
        // Serial.println(adjustment);
        lastPrintLog = millis();
    }

    return basePWM;
}
int wheelieThrottlePID(int basePWM)
{
    float pitch = getPitch();
    float pitchRate = getSmoothPitchRate();
    int accelZ = getAccelZ(); // in centi-g

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;

    float error = pitch;
    float derivative = (error - previousError) / dt;

    // 🌟 Primary trigger: Pitch rate
    bool earlyTrigger = (pitchRate > 10.0f);
    bool fullTrigger = (pitch > CRITICAL_PITCH || pitchRate > 25.0f);

    float suppressionRatio = 0.0f;

    if (fullTrigger)
    {
        // Full suppression when things get wild
        suppressionRatio = currentConfig.wheelieSuppressRatio; // e.g. 0.3
    }
    else if (earlyTrigger)
    {
        // Gradual suppression based on how "off the ground" we are
        if (accelZ < 600)
            suppressionRatio = 0.25f; // Light, floating
        else if (accelZ < 900)
            suppressionRatio = 0.15f; // Softer bounce
        else
            suppressionRatio = 0.05f; // Still planted
    }

    if (suppressionRatio > 0.0f)
    {
        integratedError = 0;
        previousError = error;

        float suppressed = NEUTRAL_PWM + (basePWM - NEUTRAL_PWM) * suppressionRatio;
        previousThrottle = suppressed;
        if (suppressionRatio > 0)
        {
            Serial.printf("[WHEELIE] Suppressed to %d from %d (%.2f)\n", (int)suppressed, basePWM, suppressionRatio);
        }
        return (int)suppressed;
    }

    // 🌱 PID fallback (normal riding)
    if (abs(error) < MAX_SAFE_PITCH)
        integratedError += error * dt;
    else
        integratedError = 0;

    float adjustment = Kp * error + Ki * integratedError + Kd * derivative;
    float newThrottle = basePWM - adjustment;

    newThrottle = constrain(newThrottle, MIN_PWM, MAX_PWM);
    newThrottle = 0.7f * newThrottle + 0.3f * previousThrottle;

    previousThrottle = newThrottle;
    previousError = error;

    return (int)(basePWM * 0.7f + newThrottle * 0.3f);
}

int AdjustOutputEvaluate(int inputPWM)
{
    static unsigned long brakeStartTime = 0;
    static bool brakingActive = false;
    static bool manualBreaking = false;
    static int lastPWM = 1500;

    int adjustedPWM = currentPWM;
    int rpm = currentRPM;
    int distance = getLatestDistance();

    // Manual braking override logic
    if (manualBreaking)
    {
        if (currentPWM < 1500 || rpm < 500)
        {
            lastPWM = adjustedPWM;
            manualBreaking = false;
        }

        if (currentPWM > 1500)
        {
            lastPWM = adjustedPWM;
            return currentPWM;
        }
    }

    if (lastPWM < 1500 && currentPWM > 1500)
    {
        manualBreaking = true;
        return currentPWM;
    }

    // ✅ Apply wheelie suppression
    int wheeliePWM = wheelieThrottlePID(currentPWM);
    if (wheeliePWM != currentPWM)
    {
        adjustedPWM = min(adjustedPWM, wheeliePWM);
    }

    // Obstacle braking
    adjustedPWM = motionTracker.applyObstacleBraking(adjustedPWM, rpm, distance);

    // 🧤 Clamp reverse if needed
    if (motionTracker.shouldClampReverse(adjustedPWM, rpm, distance, brakingActive))
    {
        adjustedPWM = NEUTRAL_PWM + (MAX_PWM - NEUTRAL_PWM) * currentConfig.reverseLimitPercent;
        Serial.println("[AdjustOutput] Reverse Max 10% limiting");
    }

    // 🛠️ 👇 INSERT THIS RIGHT HERE 👇
    // Reverse soft entry smoothing
    static int reverseRampPWM = 1500;
    if (adjustedPWM > 1510 && adjustedPWM < 1550)
    {
        reverseRampPWM += 3;
        reverseRampPWM = min(reverseRampPWM, adjustedPWM);
        adjustedPWM = reverseRampPWM;
        Serial.printf("[ReverseSoft] Ramping to %d\n", adjustedPWM);
    }
    else
    {
        reverseRampPWM = 1500;
    }

    // Record for ramping comparison
    lastPWM = adjustedPWM;

    // ✅ Obstacle braking logic
    // adjustedPWM = motionTracker.applyObstacleBraking(adjustedPWM, rpm, distance);

    // if (distance < 100)
    // {
    //     Serial.printf("[PRE BRAKE] PWM=%d RPM=%d DIST=%d\n", inputPWM, rpm, distance);
    // }

    // adjustedPWM = motionTracker.applyObstacleBraking(inputPWM, rpm, distance);

    // if (distance < 100)
    // {
    //     Serial.printf("[POST BRAKE] PWM=%d\n", adjustedPWM);
    // }

    // // ✅ Clamp reverse speed
    // if (motionTracker.shouldClampReverse(adjustedPWM, rpm, distance, brakingActive))
    // {
    //     adjustedPWM = NEUTRAL_PWM + (MAX_PWM - NEUTRAL_PWM) * currentConfig.reverseLimitPercent;
    //     Serial.println("[AdjustOutput] Reverse Max 10% limiting");
    // }

    // ✅ Indoor mode forward limit
    if (currentConfig.indoorModeEnabled && adjustedPWM < 1500)
    {
        const int indoorMinPWM = 1470;
        if (adjustedPWM < indoorMinPWM)
        {
            adjustedPWM = indoorMinPWM;
            Serial.println("[AdjustOutput] Indoor mode forward limit active");
        }
    }

    lastPWM = adjustedPWM;

    if (distance < 100)
    {
        // Serial.printf("[FINAL PWM OUT] = %d\n", adjustedPWM);
    }
    return adjustedPWM;
}

// 5/9
// int AdjustOutputEvaluate(int inputPWM)
// {

//     static unsigned long brakeStartTime = 0;
//     static bool brakingActive = false;
//     static bool manualBreaking = false;
//   int adjustedPWM = currentPWM;
//    int rpm = currentRPM;
//     static int lastPWM = 1500;

//     if (manualBreaking)
//     {
//         if (currentPWM < 1500 || rpm < 500)
//         {
//             lastPWM = adjustedPWM;
//             manualBreaking = false;
//         }

//         if (currentPWM > 1500)
//         {
//             lastPWM = adjustedPWM;
//             return currentPWM;
//         }
//     }

//     if (lastPWM < 1500 && currentPWM > 1500)
//     {
//         manualBreaking = true;
//         return currentPWM;
//     }

//     static unsigned long lastPulseLog = 0;

//     int wheeliePWM = wheelieThrottlePID(currentPWM);
//     if (wheeliePWM != currentPWM) {
//         adjustedPWM = min(adjustedPWM, wheeliePWM);
//     }
// }
// //    if (wheeliePWM > adjustedPWM && wheeliePWM < 1500)
//   //       adjustedPWM = wheeliePWM;

//     int distance = getLatestDistance();

//     // static unsigned long brakeStartTime = 0;
//     // static bool brakingActive = false;
//     // static bool manualBreaking = false;

//     // if (manualBreaking)
//     // {
//     //     if (currentPWM < 1500 || rpm < 500)
//     //     {
//     //         lastPWM = adjustedPWM;
//     //         manualBreaking = false;
//     //     }

//     //     if (currentPWM > 1500)
//     //     {
//     //         lastPWM = adjustedPWM;
//     //         return currentPWM;
//     //     }
//     // }

//     // if (lastPWM < 1500 && currentPWM > 1500)
//     // {
//     //     manualBreaking = true;
//     //     return currentPWM;
//     // }

//     // 🔁 Obstacle-aware motion-based braking

//     adjustedPWM = motionTracker.applyObstacleBraking(adjustedPWM,  rpm, distance);
//     //adjustedPWM = motionTracker.applyObstacleBraking(currentPWM, rpm, distance);

//     // 🧤 Clamp reverse speed if not braking and no obstacle
//     if (motionTracker.shouldClampReverse(adjustedPWM, rpm, distance, brakingActive)) {
//         adjustedPWM = NEUTRAL_PWM + (MAX_PWM - NEUTRAL_PWM) * currentConfig.reverseLimitPercent;
//         Serial.println("[AdjustOutput] Reverse Max 10% limiting");
//     }

//     lastPWM = adjustedPWM;

//     // 🏠 Indoor mode limit for forward speed
//     if (currentConfig.indoorModeEnabled && adjustedPWM < 1500) {
//         const int indoorMinPWM = 1470;
//         if (adjustedPWM < indoorMinPWM) {
//             adjustedPWM = indoorMinPWM;
//             Serial.println("[AdjustOutput] Indoor mode forward limit active");
//         }
//     }

//     return adjustedPWM;
// }
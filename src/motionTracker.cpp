


#include "motionTracker.h"
//#include <Arduino.h>

MotionTracker::MotionTracker() : historyIndex(0), lastMotionTime(0), lastClampTime(0), brakeHoldUntil(0), lastBrakingPWM(1500) {
    for (int i = 0; i < windowSize; i++) {
        accelXHist[i] = 0;
        pitchHist[i] = 0;
        rateHist[i] = 0;
    }
}

void MotionTracker::update(float accelX, float pitch, float pitchRate, int pwm) {
    accelXHist[historyIndex] = accelX;
    pitchHist[historyIndex] = pitch;
    rateHist[historyIndex] = pitchRate;
    historyIndex = (historyIndex + 1) % windowSize;

    if (abs(accelX) > accelThreshold || abs(pitchRate) > rateThreshold || abs(pwm - 1500) > pwmThreshold) {
        lastMotionTime = millis();
    }
}

bool MotionTracker::wasMovingRecently() const {
    return (millis() - lastMotionTime) < motionMemoryMs;
}

bool MotionTracker::isStopped() const {
    for (int i = 0; i < windowSize; i++) {
        if (abs(accelXHist[i]) > accelThreshold) return false;
        if (abs(pitchHist[i]) > pitchThreshold) return false;
        if (abs(rateHist[i]) > rateThreshold) return false;
    }
    return true;
}

MotionTracker::MotionState MotionTracker::getMotionState() const {
    if (isStopped()) return STOPPED;
    if (wasMovingRecently()) return MOVING;
    return COASTING;
}

bool MotionTracker::isDirectionMismatch(int currentPWM) const {
    float accelDelta = accelXHist[(historyIndex - 1 + windowSize) % windowSize] - accelXHist[(historyIndex - 2 + windowSize) % windowSize];
    bool brakingIntent = abs(currentPWM - 1500) < pwmThreshold;
    return brakingIntent && abs(accelDelta) > 5 && ((accelXHist[(historyIndex - 1 + windowSize) % windowSize] > 0 && accelDelta > 0) ||
                                                   (accelXHist[(historyIndex - 1 + windowSize) % windowSize] < 0 && accelDelta < 0));
}

bool MotionTracker::shouldClampReverse(int adjustedPWM, int rpm, int distance, bool brakingActive) {
    if (brakingActive) return false;
    if (distance < 300) return false;
    if (adjustedPWM <= 1500 || rpm < 1000) return false;
    unsigned long now = millis();
    if (now - lastClampTime < clampCooldownMs) return false;
    lastClampTime = now;
    return true;
}

int MotionTracker::applyObstacleBraking(int inputPWM, int rpm, int distance) {
    static bool brakingActive = false;
    static unsigned long brakeStart = 0;
    static unsigned long lastBrakeEnd = 0;
    static int brakePWM = 1670;
    static int lastRPM = 0;
    static unsigned long postBrakeHoldUntil = 0;

    const unsigned long BRAKE_TIMEOUT_MS = 2000;
    const unsigned long REARM_DELAY_MS = 1000;
    const unsigned long POST_BRAKE_HOLD_MS = 250;
    const int OBSTACLE_DIST = 300;

    unsigned long now = millis();
    int absRPM = abs(rpm);

    if (!brakingActive && now < postBrakeHoldUntil && distance < OBSTACLE_DIST && inputPWM < 1500) {
        Serial.println("[⛔ POST-BRAKE LOCKOUT] Still blocked → Holding Neutral");
        return 1500;
    }

    if (!brakingActive && now - lastBrakeEnd > REARM_DELAY_MS &&
        distance < OBSTACLE_DIST && inputPWM < 1500 && absRPM > 400) {
        brakingActive = true;
        brakeStart = now;
        brakePWM = (distance < 150 ? 1800 : 1670);
        lastRPM = absRPM;
        Serial.printf("[💥 BRAKE] Triggered: dist=%d rpm=%d → brakePWM=%d\n", distance, rpm, brakePWM);
        return brakePWM;
    }

    if (brakingActive) {
        if (inputPWM > 1510) {
            brakingActive = false;
            lastBrakeEnd = now;
            postBrakeHoldUntil = now + POST_BRAKE_HOLD_MS;
            Serial.println("[❌ BRAKE] Override PWM → Neutral");
            return 1500;
        }

        if (absRPM > 0) {
            lastRPM = absRPM;
            Serial.printf("[BRAKE HOLD] RPM=%d dist=%d → Brake PWM=%d\n", rpm, distance, brakePWM);
            return brakePWM;
        }

        brakingActive = false;
        lastBrakeEnd = now;
        postBrakeHoldUntil = now + POST_BRAKE_HOLD_MS;
        Serial.println("[BRAKE] Released: RPM zero");
        return 1500;
    }

    if (!brakingActive && distance < OBSTACLE_DIST && inputPWM < 1500) {
        Serial.printf("[❌ FORWARD BLOCKED] Obstacle dist=%d → Neutral\n", distance);
        return 1500;
    }

    return inputPWM;
}

#ifndef MOTION_TRACKER_H
#define MOTION_TRACKER_H

#include <Arduino.h>

class MotionTracker {
public:
    enum MotionState {
        STOPPED,
        COASTING,
        MOVING
    };
    unsigned long brakeHoldUntil;
    int lastBrakingPWM;
    MotionTracker();
    void update(float accelX, float pitch, float pitchRate, int pwm);
    bool wasMovingRecently() const;
    bool isStopped() const;
    MotionState getMotionState() const;
    //bool isDirectionMismatch(int currentPWM) const;
    bool isDirectionMismatch(int currentPWM)const ;
    //bool shouldClampReverse(int adjustedPWM, int rpm, int distance, bool brakingActive) const;
    int applyObstacleBraking(int currentPWM, int rpm, int distance);
    bool shouldClampReverse(int adjustedPWM, int rpm, int distance, bool brakingActive);
private:
    bool brakingActive;
    static const int windowSize = 5;
    float accelXHist[windowSize];
    float pitchHist[windowSize];
    float rateHist[windowSize];
    int historyIndex;

    unsigned long lastMotionTime;
    unsigned long lastClampTime;
    const int accelThreshold = 10;
    const int pitchThreshold = 2;
    const int rateThreshold = 3;
    const int pwmThreshold = 10;
    const unsigned long motionMemoryMs = 800;
    const unsigned long clampCooldownMs = 1000;
};

#endif // MOTION_TRACKER_H

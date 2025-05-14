#include "WheelieLogic.h"

float evaluateThrottle(int inputPWM, float pitch, float accelZ, int rpm) {
    const float MAX_SAFE_PITCH = 20.0;
    const float MIN_VERTICAL_ACCEL_G = 0.2;

    float suppression = 1.0;
    if (pitch > MAX_SAFE_PITCH && accelZ < MIN_VERTICAL_ACCEL_G && rpm > 400) {
        suppression = 0.2;
    }

    int neutral = 1500;
    float output = neutral + (inputPWM - neutral) * suppression;
    return output;
}

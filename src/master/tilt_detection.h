#ifndef TILT_DETECTION_H
#define TILT_DETECTION_H

#include <Arduino.h>
#include <cmath>

// --- TILT DETECTION STRUCTURE ---
struct TiltData {
    float roll;           // Tilt angle left/right (degrees)
    float pitch;          // Tilt angle forward/backward (degrees)
    bool isAbnormal;      // True if tilt > 30°
    bool isCritical;      // True if tilt > 45°
};

// --- TILT DETECTION FUNCTION (COMPLEMENTARY FILTER) ---
// Combines gyroscope (short-term rotation) + accelerometer (long-term correction)
TiltData detect_abnormal_tilt(float accel_x, float accel_y, float accel_z,
                               float gyro_x, float gyro_y, float gyro_z);

#endif // TILT_DETECTION_H

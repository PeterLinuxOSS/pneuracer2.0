#ifndef TILT_DETECTION_H
#define TILT_DETECTION_H

#include <Arduino.h>
#include <cmath>

// --- TILT DETEKOVANIE ŠTRUKTÚRA ---
struct TiltData {
    float roll;           // Uhol náklonu doľava/doprava (degrees)
    float pitch;          // Uhol náklonu dopredu/dozadu (degrees)
    bool isAbnormal;      // True ak je náklón > 30°
    bool isCritical;      // True ak je náklón > 45°
};

// --- TILT DETEKCIA FUNKCIE (COMPLEMENTARY FILTER) ---
// Kombinuje gyroscope (krátkoterm rotácia) + accelerometer (dlhodobá korekcija)
TiltData detect_abnormal_tilt(float accel_x, float accel_y, float accel_z,
                               float gyro_x, float gyro_y, float gyro_z);

#endif // TILT_DETECTION_H

#include "tilt_detection.h"
#include "globals.h"
#include "errors.h"
#include <Adafruit_NeoPixel.h>

// --- COMPLEMENTARY FILTER STATE ---
static float filtered_roll = 0.0f;
static float filtered_pitch = 0.0f;
static unsigned long last_filter_time = 0;
static bool first_run = true;

// --- TILT DETECTION IMPLEMENTATION (COMPLEMENTARY FILTER) ---
TiltData detect_abnormal_tilt(float accel_x, float accel_y, float accel_z,
                               float gyro_x, float gyro_y, float gyro_z) {
    TiltData tilt = {0, 0, false, false};
    
    // --- TIME INTERVAL BETWEEN READS ---
    unsigned long now = millis();
    
    float dt = 0.01f;  // Approximately 100Hz sampling
    if (!first_run) {
        dt = (now - last_filter_time) / 1000.0f;
        if (dt < 0.001f || dt > 1.0f) dt = 0.01f;  // Protect against outliers
    } else {
        first_run = false;
    }
    last_filter_time = now;
    
    // --- ANGLES FROM ACCELEROMETER (ABSOLUTE TILT) ---
    float roll_accel = atan2(accel_y, accel_z) * 180.0f / M_PI;
    float pitch_accel = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0f / M_PI;
    
    // --- INTEGRATE GYRO (ROTATION RATE) ---
    // Gyro is in rad/s, integrate to degrees
    float roll_rate = gyro_x * 180.0f / M_PI;  // rad/s -> deg/s
    float pitch_rate = gyro_y * 180.0f / M_PI;
    
    float roll_gyro = filtered_roll + (roll_rate * dt);
    float pitch_gyro = filtered_pitch + (pitch_rate * dt);
    
    // --- COMPLEMENTARY FILTER (95% GYRO + 5% ACCEL) ---
    // Gyroscope: accurate for rotation, but drifts over time
    // Accelerometer: not used for vibration detection, only drift correction
    const float GYRO_WEIGHT = 0.95f;
    const float ACCEL_WEIGHT = 0.05f;
    
    filtered_roll = (GYRO_WEIGHT * roll_gyro) + (ACCEL_WEIGHT * roll_accel);
    filtered_pitch = (GYRO_WEIGHT * pitch_gyro) + (ACCEL_WEIGHT * pitch_accel);
    
    // Prevent angle values from blowing up during rapid motion
    if (filtered_roll > 180.0f) filtered_roll = 180.0f;
    if (filtered_roll < -180.0f) filtered_roll = -180.0f;
    if (filtered_pitch > 180.0f) filtered_pitch = 180.0f;
    if (filtered_pitch < -180.0f) filtered_pitch = -180.0f;
    
    tilt.roll = filtered_roll;
    tilt.pitch = filtered_pitch;
    
    // --- THRESHOLD CHECK ---
    float max_tilt = max(abs(tilt.roll), abs(tilt.pitch));
    
    if (max_tilt > TILT_WARNING_THRESHOLD) {
        tilt.isAbnormal = true;
        
    } else {
        tilt.isAbnormal = false;
        
    }
    
    return tilt;
}

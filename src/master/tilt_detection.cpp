#include "tilt_detection.h"
#include "globals.h"
#include "errors.h"
#include <Adafruit_NeoPixel.h>

// --- COMPLEMENTARY FILTER STATE ---
static float filtered_roll = 0.0f;
static float filtered_pitch = 0.0f;
static unsigned long last_filter_time = 0;
static bool first_run = true;
static int abnormal_confirm_count = 0;
static int normal_confirm_count = 0;

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

    // --- DYNAMIC ACCELEROMETER WEIGHTING ---
    float accel_magnitude = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    float accel_weight = 0.05f;
    if (accel_magnitude < 7.5f || accel_magnitude > 11.5f) {
        // During rapid acceleration or heavy vibration, trust gyro more and use very little accel correction.
        accel_weight = 0.01f;
    }
    
    // --- INTEGRATE GYRO (ROTATION RATE) ---
    // Gyro is in rad/s, integrate to degrees
    float roll_rate = gyro_x * 180.0f / M_PI;  // rad/s -> deg/s
    float pitch_rate = gyro_y * 180.0f / M_PI;
    
    float roll_gyro = filtered_roll + (roll_rate * dt);
    float pitch_gyro = filtered_pitch + (pitch_rate * dt);
    
    // --- COMPLEMENTARY FILTER ---
    const float GYRO_WEIGHT = 1.0f - accel_weight;
    filtered_roll = (GYRO_WEIGHT * roll_gyro) + (accel_weight * roll_accel);
    filtered_pitch = (GYRO_WEIGHT * pitch_gyro) + (accel_weight * pitch_accel);
    
    // Prevent angle values from blowing up during rapid motion
    filtered_roll = constrain(filtered_roll, -180.0f, 180.0f);
    filtered_pitch = constrain(filtered_pitch, -180.0f, 180.0f);
    
    tilt.roll = filtered_roll;
    tilt.pitch = filtered_pitch;
    
    // --- THRESHOLD + DEBOUNCE ---
    float max_tilt = max(abs(tilt.roll), abs(tilt.pitch));
    const int ABNORMAL_CONFIRM_COUNT = 3;
    const int NORMAL_CONFIRM_COUNT = 3;

    if (max_tilt > TILT_WARNING_THRESHOLD) {
        abnormal_confirm_count++;
        normal_confirm_count = 0;
    } else {
        normal_confirm_count++;
        abnormal_confirm_count = 0;
    }

    if (abnormal_confirm_count >= ABNORMAL_CONFIRM_COUNT) {
        tilt.isAbnormal = true;
        tilt.isCritical = max_tilt > TILT_CRITICAL_THRESHOLD;
    } else if (normal_confirm_count >= NORMAL_CONFIRM_COUNT) {
        tilt.isAbnormal = false;
        tilt.isCritical = false;
    } else {
        // Not enough samples yet to change state.
        tilt.isAbnormal = false;
        tilt.isCritical = false;
    }
    
    return tilt;
}

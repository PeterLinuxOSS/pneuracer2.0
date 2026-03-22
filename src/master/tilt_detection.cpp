#include "tilt_detection.h"
#include "globals.h"
#include "errors.h"
#include <Adafruit_NeoPixel.h>

// --- COMPLEMENTARY FILTER STAV ---
static float filtered_roll = 0.0f;
static float filtered_pitch = 0.0f;
static unsigned long last_filter_time = 0;
static bool first_run = true;

// --- TILT DETEKCIA IMPLEMENTÁCIA (COMPLEMENTARY FILTER) ---
TiltData detect_abnormal_tilt(float accel_x, float accel_y, float accel_z,
                               float gyro_x, float gyro_y, float gyro_z) {
    TiltData tilt = {0, 0, false, false};
    
    // --- ČASOVÝ INTERVAL MEDZI ČÍTANIAMI ---
    unsigned long now = millis();
    
    float dt = 0.01f;  // Približne 100Hz odber
    if (!first_run) {
        dt = (now - last_filter_time) / 1000.0f;
        if (dt < 0.001f || dt > 1.0f) dt = 0.01f;  // Ochrana pred outliers
    } else {
        first_run = false;
    }
    last_filter_time = now;
    
    // --- UHLY Z AKCELEROMETRU (ABSOLÚTNY SKLON) ---
    float roll_accel = atan2(accel_y, accel_z) * 180.0f / M_PI;
    float pitch_accel = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0f / M_PI;
    
    // --- INTEGROVAŤ GYRÓ (ROTAČNÁ RÝCHLOSŤ) ---
    // Gyro je v rad/s, integrujeme na stupne
    float roll_rate = gyro_x * 180.0f / M_PI;  // rad/s -> deg/s
    float pitch_rate = gyro_y * 180.0f / M_PI;
    
    float roll_gyro = filtered_roll + (roll_rate * dt);
    float pitch_gyro = filtered_pitch + (pitch_rate * dt);
    
    // --- COMPLEMENTARY FILTER (95% GYRO + 5% ACCEL) ---
    // Gyroscope: presný na rotácie, ale driftuje dlhodobě
    // Accelerometer: nepoužívame na detekciu vibrácií, iba na korekciju driftu
    const float GYRO_WEIGHT = 0.95f;
    const float ACCEL_WEIGHT = 0.05f;
    
    filtered_roll = (GYRO_WEIGHT * roll_gyro) + (ACCEL_WEIGHT * roll_accel);
    filtered_pitch = (GYRO_WEIGHT * pitch_gyro) + (ACCEL_WEIGHT * pitch_accel);
    
    // Zabranit "vylietnutiu" uhlov pri prudkých pohyboch
    if (filtered_roll > 180.0f) filtered_roll = 180.0f;
    if (filtered_roll < -180.0f) filtered_roll = -180.0f;
    if (filtered_pitch > 180.0f) filtered_pitch = 180.0f;
    if (filtered_pitch < -180.0f) filtered_pitch = -180.0f;
    
    tilt.roll = filtered_roll;
    tilt.pitch = filtered_pitch;
    
    // --- KONTROLA PRAHOV ---
    float max_tilt = max(abs(tilt.roll), abs(tilt.pitch));
    
    if (max_tilt > TILT_WARNING_THRESHOLD) {
        tilt.isAbnormal = true;
        
    } else {
        tilt.isAbnormal = false;
        
    }
    
    return tilt;
}

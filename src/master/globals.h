#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>
#include <Adafruit_LSM6DS.h>
#include "ELRS.h"
#include "class.h"
#include <CRSFforArduino.hpp>
#include "errors.h"

// --- ADC VOLTAGE CONVERSION ---
const float ADC_REF_VOLTAGE = 3.3;      // ESP32 reference voltage
const int ADC_RESOLUTION = 4095;        // 12-bit ADC (0-4095)
const float CONVERSION_FACTOR = 0.62;   // Result (10uA/A * 62kOhm)

const float BATTERY_VOLTAGE_MIN = 9.0f;          // Minimum for 3S battery
const float BATTERY_VOLTAGE_MAX = 12.6f;         // Full charge for 3S battery
const float BATTERY_VOLTAGE_CRITICAL = 9.0f;     // Critical low-voltage threshold
const float BATTERY_DISCONNECTED_THRESHOLD = 1.0f; // Below this, battery is considered unplugged

// --- SERVO POSITIONS (PWM MICROSECONDS) ---
 
const int SERVO_ATTACH_MIN = 600;      // Minimum PWM for servo
const int SERVO_ATTACH_MAX = 2400;      // Maximum PWM for servo
const int SERVO_CENTER = 1400;          // Center servo position

// --- TILT DETECTION THRESHOLDS ---
const float TILT_WARNING_THRESHOLD = 30.0f;   // Warning at 30° (gyro + accel hybrid)
const float TILT_CRITICAL_THRESHOLD = 45.0f;  // Critical at 45°
const unsigned long ALERT_COOLDOWN = 1000;    // ms - max 1 alert per second

// --- GLOBAL OBJECTS ---
extern Servo steering;
extern Adafruit_NeoPixel pixels;
extern Adafruit_LSM6DS lsm6ds;
extern ELRSManager elrsManager;
extern CRSFforArduino *crsf;
extern SlavePowerManager slavePower;
extern ServosPowerManager servosPower;

// --- SHARED VARIABLES (RC CHANNELS) ---
extern volatile int currentSteerPWM;
extern volatile int currentThrottlePWM;
extern volatile int16_t gearSwitch;
extern volatile int16_t gearServoRaw;
extern volatile bool button7;
extern volatile int16_t button;
extern volatile bool Automatic;
extern volatile int16_t AutomaticSpeed;
// --- SHARED VARIABLES (COMMUNICATION) ---
extern volatile bool isFailsafeActive;
extern volatile bool isLinkUp;


// --- SHARED VARIABLES (SENSORS) ---
extern volatile float batteryVoltage;
extern volatile float imonCurrent;

// --- TIMING ---
extern uint32_t timeNow;
extern unsigned long lastBatteryUpdate;

// --- SERVO MAPPING ---
extern uint8_t batteryPercent;
extern bool imuInitialized;
extern uint8_t imuAddress;
extern volatile bool batteryConnected;
extern volatile bool usbPower;
extern volatile bool errorActive;
extern volatile ErrorCode activeErrorCode;
extern volatile bool errorCritical;

// --- NON-BLOCKING BEEPER ---
enum BeepPatternType {
    BEEP_NONE = 0,
    BEEP_BATTERY_DISCONNECT,
    BEEP_BATTERY_CONNECT,
    BEEP_ERROR,
    BEEP_SUCCESS,
};
extern volatile bool beepRequestPending;
extern volatile BeepPatternType beepRequestType;
extern volatile uint8_t beepRequestCount;
extern volatile uint16_t beepRequestFrequency;
extern volatile uint16_t beepRequestDuration;
extern volatile uint16_t beepRequestGap;

// --- SERVO MAPPING ---
extern int mappedSteer;

// --- IMU SENSOR - CALIBRATION ---
extern float gyroXoffset, gyroYoffset, gyroZoffset;
extern float accelXoffset, accelYoffset, accelZoffset;

// --- TILT DETECTION ---
extern volatile bool haltIMU;
extern unsigned long lastAlertTime;
extern volatile bool disableIMU;

// --- BATTERY LOW DETECTION ---
extern uint8_t lowBatteryCounter;
extern float previousBatteryVoltage;
extern const float BATTERY_DROP_THRESHOLD;
extern unsigned long ignoreLowBatteryUntil;

#endif // GLOBALS_H


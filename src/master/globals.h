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
#include "config.h"

// --- GLOBAL OBJECTS ---
extern Servo steering;
extern Servo servo_break;
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
extern volatile bool brakeActive;
extern volatile bool launchControl;
extern volatile bool lcStagingActive;
extern volatile int16_t AutomaticSpeed;
// --- SHARED VARIABLES (COMMUNICATION) ---
extern volatile bool isFailsafeActive;
extern volatile bool isLinkUp;


// --- SHARED VARIABLES (SENSORS) ---
extern volatile float batteryVoltage;
extern volatile float imonCurrent;

// --- TIMING ---
extern uint32_t timeNow;

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
extern unsigned long ignoreLowBatteryUntil;

// --- CROSS-CORE SYNCHRONIZATION ---
extern portMUX_TYPE sharedMux;

// --- LED DIRTY FLAG ---
extern volatile bool ledsNeedUpdate;

// --- STROBE TRANSITION TRACKING ---
extern volatile bool motorWasEnabled;

#endif // GLOBALS_H


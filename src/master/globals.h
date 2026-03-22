#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>
#include <Adafruit_LSM6DS.h>
#include "ELRS.h"
#include "class.h"
#include <CRSFforArduino.hpp>

// --- ADC KONVERTOVANIE NAPÄTIA ---
const float ADC_REF_VOLTAGE = 3.3;      // Referenčné napätie ESP32
const int ADC_RESOLUTION = 4095;        // 12-bitové ADC (0-4095)
const float CONVERSION_FACTOR = 0.62;   // Výsledok (10uA/A * 62kOhm)

// --- SERVO POZÍCIE (PWM MICROSEKÚNDY) ---
const int SERVO_ATTACH_MIN = 1050;      // Minimální PWM pre servo
const int SERVO_ATTACH_MAX = 2500;      // Maximálny PWM pre servo
const int SERVO_CENTER = 2000;          // Stredná pozícia serva

// --- TILT DETEKCIA PRAHY ---
const float TILT_WARNING_THRESHOLD = 30.0f;   // Varovanie pri 30° (gyro + accel hybrid)
const float TILT_CRITICAL_THRESHOLD = 45.0f;  // Kritické pri 45°
const unsigned long ALERT_COOLDOWN = 1000;    // ms - max 1 alert za sekundu

// --- GLOBÁLNE OBJEKTY ---
extern Servo steering;
extern Adafruit_NeoPixel pixels;
extern Adafruit_LSM6DS lsm6ds;
extern ELRSManager elrsManager;
extern CRSFforArduino *crsf;
extern SlavePowerManager slavePower;
extern ServosPowerManager servosPower;

// --- ZDIEĽANÉ PREMENNÉ (RC KANÁLY) ---
extern volatile int currentSteerPWM;
extern volatile int currentThrottlePWM;
extern volatile bool gearSwitch;
extern volatile bool button7;
extern volatile int16_t button;

// --- ZDIEĽANÉ PREMENNÉ (KOMUNIKÁCIA) ---
extern volatile bool isFailsafeActive;
extern volatile bool isLinkUp;


// --- ZDIEĽANÉ PREMENNÉ (SENZORY) ---
extern volatile float batteryVoltage;
extern volatile float imonCurrent;

// --- ČASOVANIE ---
extern uint32_t timeNow;
extern unsigned long lastBatteryUpdate;

// --- STAV APLIKÁCIE ---
extern uint8_t batteryPercent;
extern bool imuInitialized;
extern uint8_t imuAddress;

// --- SERVO MAPOVANIE ---
extern int mappedSteer;

// --- IMU SENZOR - KALIÁCIA ---
extern float gyroXoffset, gyroYoffset, gyroZoffset;
extern float accelXoffset, accelYoffset, accelZoffset;

// --- TILT DETEKCIA ---
extern volatile bool haltIMU;
extern unsigned long lastAlertTime;
extern volatile bool disableIMU;

#endif // GLOBALS_H


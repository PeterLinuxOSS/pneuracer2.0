#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>
#include <Adafruit_LSM6DS.h>
#include "ELRS.h"
#include "class.h"
#include <CRSFforArduino.hpp>
#include "globals.h"

// --- GLOBÁLNE OBJEKTY - DEFINÍCIE ---
Servo steering;
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, MASTER_STATUS_LED, NEO_GRB + NEO_KHZ800);
Adafruit_LSM6DS lsm6ds;
ELRSManager elrsManager;
CRSFforArduino *crsf = nullptr;
SlavePowerManager slavePower;
ServosPowerManager servosPower;

// --- ZDIEĽANÉ PREMENNÉ (RC KANÁLY) - DEFINÍCIE ---
volatile int currentSteerPWM = 2000;
volatile int currentThrottlePWM = 0;
volatile bool gearSwitch = false;
volatile bool button7 = false;
volatile int16_t button = 0;

// --- ZDIEĽANÉ PREMENNÉ (KOMUNIKÁCIA) - DEFINÍCIE ---
volatile bool isFailsafeActive = true;
volatile bool isLinkUp = false;
volatile bool haltIMU = false; 
volatile bool disableIMU = false;

// --- ZDIEĽANÉ PREMENNÉ (SENZORY) - DEFINÍCIE ---
volatile float batteryVoltage = 0.0;
volatile float imonCurrent = 0;


// --- ČASOVANIE - DEFINÍCIE ---
uint32_t timeNow = 0;
unsigned long lastBatteryUpdate = 0;
unsigned long lastAlertTime = 0;

// --- STAV APLIKÁCIE - DEFINÍCIE ---
uint8_t batteryPercent = 0;
bool imuInitialized = false;
uint8_t imuAddress = 0x6A; // LSM6DSOTR default I2C address

// --- SERVO MAPOVANIE - DEFINÍCIE ---
int mappedSteer;

// --- IMU SENZOR - KALIÁCIA - DEFINÍCIE ---
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;
float accelXoffset = 0, accelYoffset = 0, accelZoffset = 0;

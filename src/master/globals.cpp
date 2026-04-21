#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>
#include <Adafruit_LSM6DS.h>
#include "ELRS.h"
#include "class.h"
#include <CRSFforArduino.hpp>
#include "globals.h"

// --- GLOBAL OBJECTS - DEF ---
Servo steering;
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, MASTER_STATUS_LED, NEO_GRB + NEO_KHZ800);
Adafruit_LSM6DS lsm6ds;
ELRSManager elrsManager;
CRSFforArduino *crsf = nullptr;
SlavePowerManager slavePower;
ServosPowerManager servosPower;

// --- SHARED VARIABLES (RC CHANNELS) - DEF ---
volatile int currentSteerPWM = 2000;
volatile int currentThrottlePWM = 0;
volatile int16_t gearSwitch = 2;
volatile int16_t gearServoRaw = 1500;
volatile bool button7 = false;
volatile int16_t button = 0;
volatile bool Automatic = false;
volatile int16_t AutomaticSpeed;

// --- SHARED VARIABLES (COMMUNICATION) - DEF ---
volatile bool isFailsafeActive = true;
volatile bool isLinkUp = false;
volatile bool haltIMU = false; 
volatile bool disableIMU = false;

// --- SHARED VARIABLES (SENSORS) - DEF ---
volatile float batteryVoltage = 0.0;
volatile float imonCurrent = 0;
volatile bool batteryConnected = false;
volatile bool usbPower = true;
volatile bool errorActive = false;
volatile ErrorCode activeErrorCode = ERR_NONE;
volatile bool errorCritical = false;
volatile bool beepRequestPending = false;
volatile BeepPatternType beepRequestType = BEEP_NONE;
volatile uint8_t beepRequestCount = 0;
volatile uint16_t beepRequestFrequency = 0;
volatile uint16_t beepRequestDuration = 0;
volatile uint16_t beepRequestGap = 0;


// --- TIMING - DEF ---
uint32_t timeNow = 0;
unsigned long lastBatteryUpdate = 0;
unsigned long lastAlertTime = 0;

// --- STATE APP - DEF ---
uint8_t batteryPercent = 0;
bool imuInitialized = false;
uint8_t imuAddress = 0x6A; // LSM6DSOTR default I2C address

// --- SERVO MAPPING - DEF ---
int mappedSteer;

// --- IMU SENSOR - CALIBRATION - DEF ---
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;
float accelXoffset = 0, accelYoffset = 0, accelZoffset = 0;

// --- BATTERY LOW DETECTION ---
uint8_t lowBatteryCounter = 0;
float previousBatteryVoltage = 0.0;
const float BATTERY_DROP_THRESHOLD = 2.0f; // V - rapid drop indicates disconnection
unsigned long ignoreLowBatteryUntil = 0;
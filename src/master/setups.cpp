#include <Arduino.h>
#include "pins.h"
#include "ELRS.h"            // ELRS Manager
#include "globals.h"         // Global variables and extern declarations
#include "functions.h"       // Function declarations
#include "setups.h"          // Setup function declarations
#include "errors.h"          // Error handling

// --- SETUP FUNCTION IMPLEMENTATIONS ---

void setup_buzzer() {
    // Setup PWM for buzzer (frequency 200 Hz, 8-bit resolution)
    ledcSetup(0, 2000, 8);
    ledcAttachPin(BUZZER27, 0);
    
    ledcWrite(0, 0); // Ensure buzzer is off
    Serial.println("Buzzer initialized");
}

void setup_CRSF() {
    // Connect ELRS with proper power sequencing
    elrsManager.init();      // Initialize pins
    beep(100, 1000);
    elrsManager.connect();   // Power on, then initialize UART
    
    // Initialize CRSF on the now-powered ELRS
    crsf = new CRSFforArduino(&Serial2,ELRS_RX_PIN,ELRS_TX_PIN);
    if (!crsf->begin()) {
        Serial.println("ERROR: CRSF init failed!");
        elrsManager.reset(); // Try a soft reset
        signalError(ERR_CRSF_INIT, false);  // Non-critical - can continue
        return;
    }
    
    crsf->setRcChannelsCallback(onReceiveRcChannels);
    crsf->setLinkDownCallback([]() {
        if (!isLinkUp) return;
        isLinkUp = false;
        Serial.println("[CRSF] Link is DOWN");
        beep(100, 200);
    });

    
    Serial.println("✓ ELRS/CRSF initialized successfully");
}

void setup_ServoPWM() {
    servosPower.init();
    steering.setPeriodHertz(333);
    Serial.println("Attaching servo...");
    
    if (!steering.attach(SERVO_STEER, SERVO_ATTACH_MIN, SERVO_ATTACH_MAX)) {
        Serial.println("ERROR: Servo attach FAILED!");
        signalError(ERR_SERVO_INIT, false);  // Non-critical
        return;
    }
    
    Serial.println("✓ Servo OK");
    servosPower.enablePower();
    steering.writeMicroseconds(currentSteerPWM);
    Serial.println("✓ Steering servo initialized");
}

void setup_NeoPixel() {
    pinMode(MASTER_STATUS_PWR, OUTPUT);
    digitalWrite(MASTER_STATUS_PWR, HIGH);
    pixels.begin();
    pixels.clear();
    pixels.setBrightness(50); // Increased from 50 to 150 for better visibility

    // Test pattern: light up all LEDs one by one
    for(int i = 0; i < NEOPIXEL_COUNT; i++) {
        pixels.setPixelColor(i, pixels.Color(255, 255, 255)); // White
        pixels.show();
        delay(100);
    }
    delay(500);
    pixels.clear();
    pixels.show();
}

void setup_valves() {
    pinMode(VAL1_A, OUTPUT);
    pinMode(VAL1_B, OUTPUT);
    digitalWrite(VAL1_A, HIGH);
    digitalWrite(VAL1_B, HIGH);
    delay(1000); // Short pause for stabilization
    digitalWrite(VAL1_A, LOW);
    digitalWrite(VAL1_B, LOW);
    Serial.println("Valves initialized and set to default state");
}

void setup_battery() {
    pinMode(VBAT_REF, INPUT);
    analogReadResolution(12); // 12-bit resolution for better accuracy
    Serial.println("Battery monitoring initialized");
}

void setup_button() {
    pinMode(BUTTON, INPUT_PULLUP); // Button active-low with internal pullup
}

void setup_imon(){
    pinMode(IMON_CURRENT, INPUT);
}

void setup_imu() {
    Serial.println("\n--- Initializing LSM6DSOTR IMU ---");
    
    if (!Wire.begin(SDA_PIN, SCL_PIN, 400000)) {
        Serial.println("ERROR: Failed to initialize I2C bus!");
        signalError(ERR_I2C_INIT, true);
        return;
    }
    
    delay(200);  
    
    uint8_t imuAddress = 0x6A;  // Default address
    
    Serial.printf("Attempting to find LSM6DSOTR at address 0x%02X...\n", imuAddress);
    
    Wire.beginTransmission(imuAddress);
    int result = Wire.endTransmission();
    
    if (result == 0) {
        Serial.printf("Device found at address 0x%02X!\n", imuAddress);
    } else {
        Serial.printf("No response at 0x%02X (error: %d)\n", imuAddress, result);
        Serial.println("ERROR: LSM6DSOTR not found!");
        signalError(ERR_I2C_INIT, true);
        return;
    }
    delay(100);  
    lsm6ds.begin_I2C(imuAddress, &Wire);
    
    Serial.println("✓ LSM6DSOTR initialized successfully!");
    
    // Configure accelerometer range (±2G, ±4G, ±8G, ±16G)
    lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    Serial.println("✓ Accel range set to ±2G");
    
    // Configure gyroscope range (±125, ±250, ±500, ±1000, ±2000 DPS)
    lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    Serial.println("✓ Gyro range set to ±250 DPS");
    
    // Set accelerometer and gyroscope data rates
    lsm6ds.setAccelDataRate(LSM6DS_RATE_104_HZ);
    lsm6ds.setGyroDataRate(LSM6DS_RATE_104_HZ);
    Serial.println("✓ IMU data rate set to 104 Hz");
    
    imuInitialized = true;  // Mark as successfully initialized
}

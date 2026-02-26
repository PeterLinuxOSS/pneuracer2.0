#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "pins.h"          
#include "ELRS.h"            // ELRS Manager
#include <CRSFforArduino.hpp>
#include <ESP32Servo.h>
#include <ESP32PWM.h>
#include <Adafruit_LSM6DS.h> // 6-axis IMU (accel + gyro)
#include "shared/SharedData.h" 
#include "class.h"         


// --- OBJEKTY ---
Servo steering;
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, MASTER_STATUS_LED, NEO_GRB + NEO_KHZ800);
Adafruit_LSM6DS lsm6ds; // 6-axis IMU sensor
ELRSManager elrsManager;
CRSFforArduino *crsf = nullptr;
SlavePowerManager slavePower;

// --- ZDIEĽANÉ PREMENNÉ (THREAD SAFE) ---
volatile int currentSteerPWM = 2000; 
volatile int currentThrottlePWM = 0;
volatile bool gearSwitch = false;
volatile bool isFailsafeActive = true;
volatile bool isLinkUp = false;
volatile float batteryVoltage = 0.0;
volatile bool button7 = false;
volatile int16_t button = 0;


uint32_t timeNow = 0;
static unsigned long lastBatteryUpdate = 0;
static uint8_t batteryPercent = 0;
static bool imuInitialized = false;  // Track if IMU initialized successfully

const int attachMin = 1050;
const int attachMax = 2500;
const int desiredCenter = 2000;
int mappedSteer;

// --- DEFINÍCIE TASKOV ---
void TaskSlaveComms(void *pvParameters);
void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels);
void beep(int duration_ms, int frequency = 2000);

// --- SETUP FUNKCIE ---
void setup_buzzer() {
    // Setup PWM for buzzer (frequency 200 Hz, 8-bit resolution)
    ledcSetup(0, 2000, 8);
    ledcAttachPin(BUZZER27, 0);
    
    ledcWrite(0, 0); // Ensure buzzer is off
    Serial.println("Buzzer initialized");
}

void beep(int duration_ms, int frequency) {
    ledcSetup(0, frequency, 8);
    ledcWrite(0, 180); 
    delay(duration_ms);
    ledcWrite(0, 0);
}
;
void setup_CRSF() {
    // Connect ELRS with proper power sequencing
    elrsManager.init();      // Initialize pins
    beep(100, 1000);
    elrsManager.connect();   // Power on, then initialize UART
    
    // Initialize CRSF on the now-powered ELRS
    crsf = new CRSFforArduino(&Serial2, ELRS_TX_PIN,ELRS_RX_PIN);
    if (!crsf->begin()) {
        Serial.println("CRSF init failed!");
        elrsManager.reset(); // Try a soft reset
        while (1) delay(10);
    }
    crsf->setRcChannelsCallback(onReceiveRcChannels);
    /*crsf->setLinkUpCallback([]() {
        if (isLinkUp) return;
        isLinkUp = true;
        Serial.println("[CRSF] Link is UP");
        beep(100, 500);
        
    });
    */
    crsf->setLinkDownCallback([]() {
        if (!isLinkUp) return;
        isLinkUp = false;
        Serial.println("[CRSF] Link is DOWN");
        beep(100, 200);
    });

    
    Serial.println("ELRS/CRSF initialized successfully");
}

void setup_ServoPWM() {
    steering.setPeriodHertz(333);
    Serial.println("Pripajam servo na pin 21...");
    // Keep attach bounds, we'll offset the mapped value so neutral becomes 2000us
    if (!steering.attach(21, 1050, 2350)) {
        Serial.println("Chyba: Servo sa nepodarilo pripojit!");
    } else {
        Serial.println("Servo OK");
    }
    steering.writeMicroseconds(currentSteerPWM);
    Serial.println("Steering servo initialized ");
}

void setup_NeoPixel() {
    pixels.begin();
    pixels.clear();
    pixels.setBrightness(50); 
    
    pixels.show();
}

void setup_valves() {
    pinMode(VAL1_A, OUTPUT);
    pinMode(VAL1_B, OUTPUT);
    digitalWrite(VAL1_A, HIGH);
    digitalWrite(VAL1_B, HIGH);
    delay(1000); // Krátka pauza pre stabilizáciu
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

float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;
float accelXoffset = 0, accelYoffset = 0, accelZoffset = 0;

void calibrate_imu() {
    
    int numSamples = 500;
    
    for (int i = 0; i < numSamples; i++) {
        sensors_event_t accel, gyro, temp;
        lsm6ds.getEvent(&accel, &gyro, &temp);

        gyroXoffset += gyro.gyro.x;
        gyroYoffset += gyro.gyro.y;
        gyroZoffset += gyro.gyro.z;
        
        // For Accel: We assume Z is pointing up (1G), so we calibrate X and Y to 0
        accelXoffset += accel.acceleration.x;
        accelYoffset += accel.acceleration.y;
        accelZoffset += (accel.acceleration.z - 9.806); // Subtract Earth's gravity

        delay(10); // Match your data rate (10ms = 100Hz)
    }

    gyroXoffset /= numSamples;
    gyroYoffset /= numSamples;
    gyroZoffset /= numSamples;
    
    accelXoffset /= numSamples;
    accelYoffset /= numSamples;
    accelZoffset /= numSamples;

    Serial.println("Calibration Complete!");
}

void setup_imu() {
    Serial.println("\n--- Initializing LSM6DSOTR IMU ---");
    
    if (!Wire.begin(SDA_PIN, SCL_PIN, 400000)) {
        Serial.println("ERROR: Failed to initialize I2C bus!");
        return;
    }
    
    delay(200);  
    
    uint8_t imuAddress = 0;
    for (int attempt = 0; attempt < 2; attempt++) {
        uint8_t addressToTry = (attempt == 0) ? 0x6B : 0x6A;
        
        Serial.printf("Attempting to find LSM6DSOTR at address 0x%02X...\n", addressToTry);
        
        Wire.beginTransmission(addressToTry);
        int result = Wire.endTransmission();
        
        if (result == 0) {
            Serial.printf("Device found at address 0x%02X!\n", addressToTry);
            imuAddress = addressToTry;
            break;
        } else {
            Serial.printf("No response at 0x%02X (error: %d)\n", addressToTry, result);
        }
        delay(100);
    }
    
    if (imuAddress == 0) {
        Serial.println("ERROR: LSM6DSOTR not found at any known address!");
        Serial.println("Check I2C connections:");
        Serial.println("  SDA: GPIO 2 (IO2)");
        Serial.println("  SCL: GPIO 1 (IO1)");
        Serial.println("  Add 10kΩ pull-up resistors if not present!");
        return;
    }
    if (!lsm6ds.begin_I2C(imuAddress, &Wire)) {
        Serial.println("CHYBA: Kniznica LSM6DS sa nespustila!");
        return; 
    }
    delay(100);  
    
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



void setup() {
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    setup_NeoPixel();
    slavePower.init();
    Serial.begin(921600); // USB Debug
    Serial1.begin(921600, SERIAL_8N1, INTER_TX_S, INTER_RX_S);
    pixels.fill(pixels.Color(255, 100, 0)); 
    pixels.show();

    Serial.println("Starting MASTER setup...");
    setup_valves();
    setup_ServoPWM();
    setup_buzzer();
    beep(200, 1500); 
    setup_battery();
    setup_button();
    setup_imu();  
    delay(300); 
    Serial.println("IMU initialized, calibrating...");
    beep(200, 3000); 
    
    calibrate_imu();
    Serial.println("IMU calibration complete!");
  
    beep(150, 2000); 
    delay(100);
    beep(150, 2500); 
    delay(100);
    beep(200, 3000); 
    Serial.println("CRSF setup starting...");
    setup_CRSF();
    Serial.println("CRSF setup complete!");


    xTaskCreatePinnedToCore(
        TaskSlaveComms,   
        "SlaveComms",    
        4096,             
        NULL,            
        1,                
        NULL,           
        0                 
    );
    slavePower.enablePower();
    Serial.println("MASTER Setup Complete. Running Multithreaded.");
    
}

void gear_change(bool value = false) {
        if (value) {
            digitalWrite(VAL1_A, HIGH);
            digitalWrite(VAL1_B, LOW);
        } else {
            digitalWrite(VAL1_A, LOW);
            digitalWrite(VAL1_B, HIGH);
        }
}

// --- IMU SENSOR READING ---
void read_and_display_imu() {

    
    sensors_event_t accel, gyro, temp;
    lsm6ds.getEvent(&accel, &gyro, &temp);

    // Subtract the offsets
    float corrected_gx = gyro.gyro.x - gyroXoffset;
    float corrected_gy = gyro.gyro.y - gyroYoffset;
    float corrected_gz = gyro.gyro.z - gyroZoffset;

    float corrected_ax = accel.acceleration.x - accelXoffset;
    float corrected_ay = accel.acceleration.y - accelYoffset;
    float corrected_az = accel.acceleration.z - accelZoffset;

    Serial.printf("Gyro (dps): X=%.2f Y=%.2f Z=%.2f | Accel (m/s²): X=%.2f Y=%.2f Z=%.2f | Temp: %.2f°C\n",
        corrected_gx, corrected_gy, corrected_gz,
        corrected_ax, corrected_ay, corrected_az,
         temp.temperature);
   
}
// --- HLAVNÁ SLUČKA (CORE 1) ---
void loop() {
    crsf->update();
}

// --- CRSF CALLBACK (Volané z Loopu) ---
void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels) {
    bool fs = rcChannels->failsafe;
    
    isFailsafeActive = fs;

    if (!fs) {
        int rawSteer = rcChannels->value[0];
       
        int baseSteer = map(rawSteer, 172, 1811, 1050, 2350);
        const int baseCenter = (attachMin + attachMax) / 2; 
        mappedSteer = baseSteer + (desiredCenter - baseCenter);
        mappedSteer = constrain(mappedSteer, attachMin, attachMax);
        
        steering.writeMicroseconds(mappedSteer);
        Serial.println(mappedSteer);
        currentThrottlePWM = rcChannels->value[2];
        gearSwitch = rcChannels->value[4] > 1000; 
        gear_change(gearSwitch);
        isLinkUp = true;
        button7 = rcChannels->value[7] > 1000; 
        
        if (button7) {
            slavePower.disablePower();
            
            
        }else {
            slavePower.enablePower();
            
        }

        if (rcChannels->value[5] < 250 ) {
            button = 0;
        }else if (rcChannels->value[5] < 1000) {
            button = 250;
            
        }else{
            button = 750;
        }
        
        timeNow = millis();
        
        if (timeNow - lastBatteryUpdate >= 100)
        {
            int rawADC = analogRead(VBAT_REF);
            // Voltage divider: (R28 + R29) / R29 = 118K / 18K = 6.556 (adjusted to 6.60 for calibration)
            batteryVoltage = (rawADC / 4095.0) * 3.3 * 6.60;
            lastBatteryUpdate = timeNow;
            if (batteryVoltage < 9.0) {
                batteryPercent = 0;
            } else if (batteryVoltage > 12.6) {
                batteryPercent = 100;
            } else {
                batteryPercent = (uint8_t)(((batteryVoltage - 9.0) / 3.6) * 100.0);
            }
            crsf->telemetryWriteBattery(batteryVoltage, 0, 0, batteryPercent);
        }
    } else {
        isLinkUp = false;
        
    }
}

// --- TASK NA POZADÍ (CORE 0) ---
void TaskSlaveComms(void *pvParameters) {
    ControlPacket packetToSend;
    packetToSend.header = PACKET_HEADER; // 0xBEEF
    uint32_t imuReadCounter = 0;

    for (;;) {
        if (isFailsafeActive) {
            pixels.fill(pixels.Color(255, 0, 255)); // Červená - Failsafe
        } else {
            pixels.fill(pixels.Color(0, 255, 0)); // Zelená - OK
        }
        pixels.show();
        
        packetToSend.throttle = currentThrottlePWM; 
        packetToSend.elrsActive = isLinkUp;
        packetToSend.button = button;
        
        

        // D. ODOSLANIE CEZ UART
        packetToSend.checksum = calculateChecksum(&packetToSend);
        Serial1.write((uint8_t*)&packetToSend, sizeof(ControlPacket));

        // Obnovovacia frekvencia pre Slave (napr. 50Hz = 20ms)
        vTaskDelay(20 / portTICK_PERIOD_MS); 
    }
}



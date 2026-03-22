#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "pins.h"
#include "ELRS.h" // ELRS Manager
#include <CRSFforArduino.hpp>
#include <ESP32Servo.h>
#include <ESP32PWM.h>
#include <Adafruit_LSM6DS.h> // 6-axis IMU (accel + gyro)
#include "shared/SharedData.h"
#include "globals.h"
#include "functions.h"
#include "setups.h"
#include "errors.h"
#include "tilt_detection.h"

// --- FUNKT CIA IMPLEMENTÁCIE V MAIN.CPP ---

void beep(int duration_ms, int frequency)
{
    ledcSetup(0, frequency, 8);
    ledcWrite(0, 180);
    delay(duration_ms);
    ledcWrite(0, 0);
};

void calibrate_imu()
{
    int numSamples = 500;
    sensors_event_t accel, gyro, temp;

    for (int i = 0; i < numSamples; i++)
    {
        // Deklaruj MIMO slučky!
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

    lsm6ds.getEvent(&accel, &gyro, &temp);
    Serial.println("Calibration Complete!");
    Serial.printf("Gyro Offsets: X=%.2f Y=%.2f Z=%.2f\n", gyroXoffset, gyroYoffset, gyroZoffset);
    Serial.printf("Temp: %.2f C\n", temp.temperature);
    beep(150, 2000);
    delay(100);
    beep(150, 2500);
    delay(100);
    beep(200, 3000);
}

void setup()
{
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    analogReadResolution(12);
    setup_NeoPixel();
    slavePower.init();
    Serial.begin(921600); // USB Debug
    Serial1.begin(921600, SERIAL_8N1, INTER_TX_S, INTER_RX_S);
    pixels.fill(pixels.Color(255, 100, 0));
    pixels.show();
    Serial.println("Starting MASTER setup...");
    setup_valves();

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

    Serial.println("CRSF setup starting...");
    setup_CRSF();
    Serial.println("CRSF setup complete!");
    setup_ServoPWM();

    xTaskCreatePinnedToCore(
        TaskSlaveComms,
        "SlaveComms",
        4096,
        NULL,
        1,
        NULL,
        0);
    slavePower.enablePower();

    // ✓ Setup OK - Signal success
    signalSuccess();
    Serial.println("MASTER Setup Complete. Running Multithreaded.");
}

void gear_change(bool value)
{
    if (value)
    {
        digitalWrite(VAL1_A, HIGH);
        digitalWrite(VAL1_B, LOW);
    }
    else
    {
        digitalWrite(VAL1_A, LOW);
        digitalWrite(VAL1_B, HIGH);
    }
}

// --- IMU SENSOR READING ---
void read_and_display_imu()
{

    sensors_event_t accel, gyro, temp;
    lsm6ds.getEvent(&accel, &gyro, &temp);

    // Subtract the offsets
    float corrected_gx = gyro.gyro.x - gyroXoffset;
    float corrected_gy = gyro.gyro.y - gyroYoffset;
    float corrected_gz = gyro.gyro.z - gyroZoffset;

    float corrected_ax = accel.acceleration.x - accelXoffset;
    float corrected_ay = accel.acceleration.y - accelYoffset;
    float corrected_az = accel.acceleration.z - accelZoffset;

    // --- DEBUG: Zobraž raw hodnoty ---
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 500) {  // Každých 500ms
        Serial.printf("[DEBUG] Gyro: X=%.2f Y=%.2f Z=%.2f | Accel: X=%.2f Y=%.2f Z=%.2f\n",
                      corrected_gx, corrected_gy, corrected_gz,
                      corrected_ax, corrected_ay, corrected_az);
        lastDebugTime = millis();
    }

    // --- DETEKTUJ ABNORMÁLNY SKLON (COMPLEMENTARY FILTER: GYRO + ACCEL) ---
    TiltData tilt = detect_abnormal_tilt(corrected_ax, corrected_ay, corrected_az,
                                          corrected_gx, corrected_gy, corrected_gz);
    
    // --- DEBUG: Vždy zobraž tilt uhly ---
    static unsigned long lastTiltTime = 0;
    if (millis() - lastTiltTime > 500) {  // Každých 500ms
        Serial.printf("[TILT] Roll: %.1f° | Pitch: %.1f° | Threshold: %.1f°\n",
                      tilt.roll, tilt.pitch, TILT_WARNING_THRESHOLD);
        lastTiltTime = millis();
    }
    
    if (tilt.isAbnormal)
    {
        haltIMU = true;  // Nastav flag - beepovať bude TaskSlaveComms
        Serial.printf("  🚨 Roll: %.1f° | Pitch: %.1f° %s\n",
                      tilt.roll, tilt.pitch, tilt.isCritical ? "[CRITICAL]" : "[WARNING]");
    }
    
    
}
// --- HLAVNÁ SLUČKA (CORE 1) ---
void loop()
{
    crsf->update();
}

// --- CRSF CALLBACK (Volané z Loopu) ---
void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels)
{
    bool fs = rcChannels->failsafe;

    isFailsafeActive = fs;

    if (!fs)
    {
        int rawSteer = rcChannels->value[0];

        int baseSteer = map(rawSteer, 172, 1811, SERVO_ATTACH_MIN, SERVO_ATTACH_MAX);
        const int baseCenter = (SERVO_ATTACH_MIN + SERVO_ATTACH_MAX) / 2;
        mappedSteer = baseSteer + (SERVO_CENTER - baseCenter);
        mappedSteer = constrain(mappedSteer, SERVO_ATTACH_MIN, SERVO_ATTACH_MAX);

        steering.writeMicroseconds(mappedSteer);

        currentThrottlePWM = rcChannels->value[2];
        gearSwitch = rcChannels->value[4] > 1000;
        gear_change(gearSwitch);
        isLinkUp = true;
        button7 = rcChannels->value[7] > 1000;

        if (button7)
        {
            slavePower.disablePower();
        }
        else
        {
            slavePower.enablePower();
        }

        if (rcChannels->value[5] < 250)
        {
            button = 0;
        }
        else if (rcChannels->value[5] < 1000)
        {
            button = 250;
        }
        else
        {
            button = 750;
        }
        if (rcChannels->value[8] > 500)
        {
            haltIMU = false;
        }
        if (rcChannels->value[6] > 1000)
        {
            disableIMU  = true;
            haltIMU = false;
        }else{
            disableIMU = false;
        }

        timeNow = millis();

        if (timeNow - lastBatteryUpdate >= 100)
        {
            int rawADC = analogRead(VBAT_REF);
            int rawImon = analogRead(IMON_CURRENT);
            float voltageImon = (rawImon * ADC_REF_VOLTAGE) / ADC_RESOLUTION;
            imonCurrent = voltageImon / CONVERSION_FACTOR;
            // Voltage divider: (R28 + R29) / R29 = 118K / 18K = 6.556 (adjusted to 6.60 for calibration)
            batteryVoltage = (rawADC / (float)ADC_RESOLUTION) * ADC_REF_VOLTAGE * 6.60;
            lastBatteryUpdate = timeNow;
            if (batteryVoltage < 9.0)
            {
                batteryPercent = 0;
            }
            else if (batteryVoltage > 12.6)
            {
                batteryPercent = 100;
            }
            else
            {
                batteryPercent = (uint8_t)(((batteryVoltage - 9.0) / 3.6) * 100.0);
            }
            if (batteryVoltage < 5.0)
            {
                batteryPercent = 100;
            }
            crsf->telemetryWriteBattery(batteryVoltage, 0, 0, batteryPercent);
            if (!disableIMU)
            {
                read_and_display_imu();
            }
        }
    }
    else
    {
        isLinkUp = false;
    }
}

// --- TASK NA POZADÍ (CORE 0) ---
void TaskSlaveComms(void *pvParameters)
{
    ControlPacket packetToSend;
    packetToSend.header = PACKET_HEADER; // 0xBEEF
    uint32_t imuReadCounter = 0;
    
    // Non-blocking beep timing
    static unsigned long lastBeepTime = 0;
    static bool beepActive = false;
    static unsigned long beepStartTime = 0;
    const unsigned long BEEP_DURATION = 150;  // ms
    const unsigned long BEEP_INTERVAL = 1000; // ms

    for (;;)
    {
        // --- FARBA LED ---
        if (isFailsafeActive)
        {
            pixels.fill(pixels.Color(255, 0, 255)); // Červená - Failsafe
        }
        else if (haltIMU)
        {
            pixels.fill(pixels.Color(255, 0, 0));   // Červená - IMU halt
        }
        else
        {
            pixels.fill(pixels.Color(0, 255, 0));   // Zelená - OK
        }
        pixels.show();

        // --- NON-BLOCKING BEEP (keď haltIMU = true) ---
        unsigned long now = millis();
        
        if (haltIMU)
        {
            // Beep každú sekundu
            if ((now - lastBeepTime) >= BEEP_INTERVAL)
            {
                beepActive = true;
                beepStartTime = now;
                lastBeepTime = now;
                
                // Zapni buzzer
                ledcSetup(0, 1500, 8);
                ledcWrite(0, 180);
            }
        }
        
        // Vypni buzzer keď uplynula doba
        if (beepActive && (now - beepStartTime) >= BEEP_DURATION)
        {
            ledcWrite(0, 0);
            beepActive = false;
        }

        // --- ODOSLANIE PACKETU ---
        packetToSend.throttle = currentThrottlePWM;
        packetToSend.elrsActive = isLinkUp;
        packetToSend.button = button;
        packetToSend.haltIMU = haltIMU;

        packetToSend.checksum = calculateChecksum(&packetToSend);
        Serial1.write((uint8_t *)&packetToSend, sizeof(ControlPacket));

        // Obnovovacia frekvencia pre Slave (napr. 50Hz = 20ms) - NON-BLOCKING!
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

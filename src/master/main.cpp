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

// --- KNIGHT RIDER (KITT) SCANNER ANIMATION ---
// Physical LEDs 0-16: individual. LEDs 17-28 (12 pcs): grouped as one virtual LED.
// Positions and timing are configured in config.h.
static int8_t        krPos = 0;
static int8_t        krDir = 1;
static unsigned long krLast = 0;
static uint8_t       krR = 255, krG = 0, krB = 0; // current color — change via setKnightRiderColor()

void setKnightRiderColor(uint8_t r, uint8_t g, uint8_t b) {
    krR = r; krG = g; krB = b;
}

static void krSetVirt(int virt, uint8_t r, uint8_t g, uint8_t b) {
    if (virt < KR_GROUP_START) {
        pixels.setPixelColor(virt, pixels.Color(r, g, b));
    } else {
        for (int p = KR_GROUP_START; p < NEOPIXEL_COUNT; p++)
            pixels.setPixelColor(p, pixels.Color(r, g, b));
    }
}

bool knightRiderStep() {
    unsigned long now = millis();
    if (now - krLast < KR_STEP_MS) return false;
    krLast = now;

    pixels.clear();

    // Fading tail — scales all color channels
    for (uint8_t t = 1; t <= KR_TAIL; t++) {
        int tidx = krPos - krDir * (int)t;
        if (tidx >= 0 && tidx < KR_VIRT_COUNT) {
            uint8_t scale = 255 - (255 / (KR_TAIL + 1)) * t;
            krSetVirt(tidx,
                      (uint8_t)((krR * scale) >> 8),
                      (uint8_t)((krG * scale) >> 8),
                      (uint8_t)((krB * scale) >> 8));
        }
    }
    // Full-bright head
    krSetVirt(krPos, krR, krG, krB);

    krPos += krDir;
    if (krPos >= KR_VIRT_COUNT) { krPos = KR_VIRT_COUNT - 2; krDir = -1; }
    if (krPos < 0)              { krPos = 1;                  krDir =  1; }

    return true;
}

void bootKnightRider(unsigned long durationMs, uint8_t r, uint8_t g, uint8_t b) {
    setKnightRiderColor(r, g, b); 
    krPos = 0; krDir = 1; krLast = 0; // reset animation state
    unsigned long start = millis();
    while (millis() - start < durationMs) {
        if (knightRiderStep()) pixels.show();
        delay(1);
    }
    pixels.clear();
    pixels.show();
    setKnightRiderColor(255,0,255); // purple
}

// --- FUNCTION IMPLEMENTATIONS IN MAIN.CPP ---

void beep(int duration_ms, int frequency)
{
    ledcSetup(0, frequency, 8);
    ledcWrite(0, 180);
    delay(duration_ms);
    ledcWrite(0, 0);
};

void requestBeepPattern(uint8_t type, uint8_t count, uint16_t frequency, uint16_t duration_ms, uint16_t gap_ms)
{
    if (beepRequestPending)
    {
        // Existing beep has priority if it is an error tone.
        if (beepRequestType == BEEP_ERROR)
        {
            return;
        }
    }

    beepRequestType = (BeepPatternType)type;
    beepRequestCount = count;
    beepRequestFrequency = frequency;
    beepRequestDuration = duration_ms;
    beepRequestGap = gap_ms;
    beepRequestPending = true;
}

void calibrate_imu()
{
    int numSamples = 500;
    sensors_event_t accel, gyro, temp;

    for (int i = 0; i < numSamples; i++)
    {
        // Use a non-blocking loop structure for sampling
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
    bootKnightRider(BOOT_ANIM_DURATION_MS, 255, 255, 255);

    slavePower.init();

    Serial.begin(921600); // USB Debug
    Serial1.begin(921600, SERIAL_8N1, INTER_TX_S, INTER_RX_S);
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

void gear_change(bool value) //unused
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

    // --- DEBUG: Show raw values ---
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 500)
    { // Every 500ms
        Serial.printf("[DEBUG] Gyro: X=%.2f Y=%.2f Z=%.2f | Accel: X=%.2f Y=%.2f Z=%.2f\n",
                      corrected_gx, corrected_gy, corrected_gz,
                      corrected_ax, corrected_ay, corrected_az);
        lastDebugTime = millis();
    }

    // --- DETECT ABNORMAL TILT (COMPLEMENTARY FILTER: GYRO + ACCEL) ---
    TiltData tilt = detect_abnormal_tilt(corrected_ax, corrected_ay, corrected_az,
                                         corrected_gx, corrected_gy, corrected_gz);

    // --- DEBUG: Always show tilt angles ---
    static unsigned long lastTiltTime = 0;
    if (millis() - lastTiltTime > 500)
    { // Every 500ms
        Serial.printf("[TILT] Roll: %.1f° | Pitch: %.1f° | Threshold: %.1f°\n",
                      tilt.roll, tilt.pitch, TILT_WARNING_THRESHOLD);
        lastTiltTime = millis();
    }

    if (tilt.isAbnormal)
    {
        haltIMU = true; // Set flag - TaskSlaveComms will handle beeping
        Serial.printf("  🚨 Roll: %.1f° | Pitch: %.1f° %s\n",
                      tilt.roll, tilt.pitch, tilt.isCritical ? "[CRITICAL]" : "[WARNING]");
    }
}


void check_battery()
{
    unsigned long now = millis();
    if (now - lastBatteryUpdate < BATTERY_READ_INTERVAL_MS)
    {
        return;
    }

    int rawADC = analogRead(VBAT_REF);
    int rawImon = analogRead(IMON_CURRENT);

    float voltageImon = (rawImon * ADC_REF_VOLTAGE) / ADC_RESOLUTION;
    imonCurrent = voltageImon / IMON_CONVERSION_FACTOR;

    batteryVoltage = (rawADC / (float)ADC_RESOLUTION) * ADC_REF_VOLTAGE * 6.60;
    bool currentlyConnected = batteryVoltage >= BATTERY_DISCONNECTED_THRESHOLD;

    // Check for rapid voltage drop (indicates battery disconnection)
    float voltageDrop = previousBatteryVoltage - batteryVoltage;
    bool rapidDrop = (previousBatteryVoltage > 0) && (voltageDrop > BATTERY_DROP_THRESHOLD);
    if (rapidDrop)
    {
        ignoreLowBatteryUntil = millis() + BATTERY_LOW_IGNORE_MS;
        Serial.println("RAPID VOLTAGE DROP DETECTED - ignoring low battery for 5s");
    }
    previousBatteryVoltage = batteryVoltage;

    if (!currentlyConnected)
    {
        if (batteryConnected)
        {
            Serial.println("BATTERY DISCONNECTED - switching to USB power");
            requestBeepPattern(BEEP_BATTERY_DISCONNECT, 2, 1200, 120, 80);
        }
        batteryConnected = false;
        usbPower = true;
        batteryPercent = 100;

        if (activeErrorCode == ERR_BATTERY_LOW)
        {
            errorActive = false;
            activeErrorCode = ERR_NONE;
            errorCritical = false;
            haltIMU = false;
        }

        lastBatteryUpdate = now;
        if (crsf)
        {
            crsf->telemetryWriteBattery(BATTERY_VOLTAGE_MAX, 0, 0, batteryPercent);
        }
        return;
    }

    if (!batteryConnected)
    {
        Serial.printf("BATTERY CONNECTED: %.2fV - switching to battery power\n", batteryVoltage);
        requestBeepPattern(BEEP_BATTERY_CONNECT, 3, 2000, 100, 80);
        batteryConnected = true;
        usbPower = false;
    }

    if (batteryVoltage <= BATTERY_VOLTAGE_MIN)
    {
        batteryPercent = 0;
    }
    else if (batteryVoltage >= BATTERY_VOLTAGE_MAX)
    {
        batteryPercent = 100;
    }
    else
    {
        batteryPercent = (uint8_t)(((batteryVoltage - BATTERY_VOLTAGE_MIN) /
                                    (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN)) * 100.0);
    }

    lastBatteryUpdate = now;

    if (crsf)
    {
        crsf->telemetryWriteBattery(batteryVoltage, 0, 0, batteryPercent);
    }

    if (batteryVoltage < BATTERY_VOLTAGE_CRITICAL && currentlyConnected && !rapidDrop && millis() > ignoreLowBatteryUntil)
    {
        lowBatteryCounter++;
        if (lowBatteryCounter >= BATTERY_LOW_CONFIRM_COUNT)
        {
            Serial.printf("CRITICAL BATTERY: %.2fV < %.2fV\n", batteryVoltage, BATTERY_VOLTAGE_CRITICAL);
            signalError(ERR_BATTERY_LOW, true);
            lowBatteryCounter = 0; // Reset counter after triggering error
        }
    }
    else
    {
        lowBatteryCounter = 0; // Reset counter if voltage recovers or rapid drop
        if (activeErrorCode == ERR_BATTERY_LOW)
        {
            // Battery recovered or current power is USB, clear stale low-battery error
            errorActive = false;
            activeErrorCode = ERR_NONE;
            errorCritical = false;
            haltIMU = false;
        }
    }
}
// --- MAIN LOOP (CORE 1) ---
void loop()
{
    crsf->update();
}

// --- CRSF CALLBACK (Called from loop) ---
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
        //Serial.printf("raw Steer: %d Base Steer: %d | Mapped Steer: %d\n",rawSteer, baseSteer, mappedSteer);
        steering.writeMicroseconds(mappedSteer);

        currentThrottlePWM = rcChannels->value[2];
        Automatic = rcChannels->value[4] > 1000;
        //
        isLinkUp = true;
        button7 = rcChannels->value[7] > 1000;

        if (button7)
        {
            slavePower.disableMotor();
        }
        else
        {
            slavePower.enableMotor();
        }

        if (rcChannels->value[6] < 250)
        {
            button = 0;
        }
        else if (rcChannels->value[6] < 1000)
        {
            button = 250;
        }
        else
        {
            button = 750;
        }
        
        
        if (rcChannels->value[10] < 1000)
        {
            disableIMU  = true;
            haltIMU = false;
            
        }else{
            disableIMU = false;
            
        }
        if (rcChannels->value[8] < 1000){  // break button
            brakeActive = false;
            servo_break.writeMicroseconds(BRAKE_SERVO_CENTER_US);
        }else{
            brakeActive = true;
            servo_break.writeMicroseconds(BRAKE_SERVO_PRESSED_US);
        }
        
        
        if (rcChannels->value[5] < 250)
        {
            gearSwitch = 1;
        }
        else if (rcChannels->value[5] < 1200)
        {
            gearSwitch = 2;
        }
        else
        {
            gearSwitch = 3;
        }

        // ch[3] stick → servo microseconds 1200–1800, posielané priamo na slave
        gearServoRaw = (int16_t)constrain(map(rcChannels->value[3], 172, 1811, 1200, 1800), 1200, 1800);
        
        AutomaticSpeed = map(rcChannels->value[9], 191, 1811, 1500, 3000);

        crsf->telemetryWriteBaroAltitude(AutomaticSpeed, 0);
        if (!disableIMU)
        {
            read_and_display_imu();
        }
    }
    else
    {
        isLinkUp = false;
    }
}

// --- BACKGROUND TASK (CORE 0) ---
void TaskSlaveComms(void *pvParameters)
{
    ControlPacket packetToSend;
    packetToSend.header = PACKET_HEADER; // 0xBEEF
    uint32_t imuReadCounter = 0;

    // Non-blocking beep timing
    static unsigned long lastBeepTime = 0;
    static bool beepActive = false;
    static unsigned long beepStartTime = 0;

    // Battery check timing
    static unsigned long lastBatteryCheckTime = 0;

    for (;;)
    {
        unsigned long now = millis();

        // --- LED COLOR ---
        if (errorActive)
        {
            static unsigned long lastErrorBlink = 0;
            static bool errorLedOn = true;

            if ((now - lastErrorBlink) >= ERROR_BLINK_INTERVAL_MS)
            {
                errorLedOn = !errorLedOn;
                lastErrorBlink = now;
            }

            if (errorLedOn)
            {
                pixels.fill(pixels.Color(255, 0, 0)); // Red - Error
            }
            else
            {
                pixels.clear();
            }
        }
        else if (isFailsafeActive)
        {
            knightRiderStep(); // KITT scanner — pixels.show() called below
        }
        else if (haltIMU)
        {
            pixels.fill(pixels.Color(255, 0, 0)); // Red - IMU halt
        }
        else if (Automatic)
        {
            if (slavePower.isMotorEnabled())
            {
                static unsigned long lastAutoPulse = 0;
                uint32_t pulseColor = !disableIMU
                    ? pixels.Color(128, 0, 128)
                    : pixels.Color(255, 200, 0);

                if ((now - lastAutoPulse) >= AUTO_PULSE_INTERVAL_MS)
                {
                    pixels.fill(pulseColor);
                    lastAutoPulse = now;
                }
                else if ((now - lastAutoPulse) >= AUTO_PULSE_DURATION_MS)
                {
                    pixels.fill(pixels.Color(0, 255, 0));
                }
                else
                {
                    pixels.fill(pulseColor);
                }
            }
            else
            {
                pixels.fill(pixels.Color(0, 255, 0)); // Green - motor stopped
            }
        }
        else if (!disableIMU)
        {
            if (slavePower.isMotorEnabled())
            {
                static unsigned long lastBluePulse = 0;

                if ((now - lastBluePulse) >= BLUE_PULSE_INTERVAL_MS)
                {
                    pixels.fill(pixels.Color(0, 0, 255));
                    lastBluePulse = now;
                }
                else if ((now - lastBluePulse) >= BLUE_PULSE_DURATION_MS)
                {
                    pixels.fill(pixels.Color(0, 255, 0)); // Back to green
                }
                else
                {
                    pixels.fill(pixels.Color(0, 0, 255)); // Blue during pulse
                }
            }
            else
            {
                pixels.fill(pixels.Color(0, 255, 0)); // Green - motor stopped
            }
        }
        else
        {
            pixels.fill(pixels.Color(0, 255, 0)); // Green - OK
        }
        // Airplane strobe overlay when motor is disabled (not in error/failsafe/halt)
        if (!slavePower.isMotorEnabled() && !isFailsafeActive && !errorActive && !haltIMU)
        {
            static unsigned long strobeBase = 0;
            if (strobeBase == 0) strobeBase = now;
            unsigned long phase = (now - strobeBase) % STROBE_PERIOD_MS;
            // Two quick flashes: 0-60ms and 120-180ms
            if (phase < 60 || (phase >= 120 && phase < 180))
                pixels.fill(pixels.Color(255, 255, 255));
        }

        // Brake light overlay — always last, so it overrides everything on the rear LEDs
        if (brakeActive)
        {
            for (int i = BRAKE_LIGHT_START; i < NEOPIXEL_COUNT; i++)
                pixels.setPixelColor(i, pixels.Color(255, 0, 0));
        }

        pixels.show();

        // --- NON-BLOCKING BEEP MACHINE ---

        static uint8_t beepCountLeft = 0;

        if (beepRequestPending && !beepActive && beepCountLeft == 0)
        {
            beepCountLeft = beepRequestCount;
            beepRequestPending = false;
        }

        if (beepCountLeft > 0 && !beepActive && (now - lastBeepTime) >= beepRequestGap)
        {
            beepActive = true;
            beepStartTime = now;
            lastBeepTime = now;
            ledcSetup(0, beepRequestFrequency, 8);
            ledcWrite(0, 180);
        }

        if (haltIMU && !beepActive && beepCountLeft == 0 && (now - lastBeepTime) >= HALT_BEEP_INTERVAL_MS)
        {
            beepActive = true;
            beepStartTime = now;
            lastBeepTime = now;
            ledcSetup(0, HALT_BEEP_FREQUENCY, 8);
            ledcWrite(0, 180);
        }

        if (beepActive)
        {
            unsigned long activeDuration = (beepRequestType == BEEP_NONE) ? HALT_BEEP_DURATION_MS : beepRequestDuration;
            if ((now - beepStartTime) >= activeDuration)
            {
                ledcWrite(0, 0);
                beepActive = false;
                if (beepCountLeft > 0)
                {
                    beepCountLeft--;
                    if (beepCountLeft == 0)
                    {
                        beepRequestType = BEEP_NONE;
                    }
                }
            }
        }

        if (now - lastBatteryCheckTime > BATTERY_CHECK_TASK_INTERVAL_MS)
        {
            check_battery();
            lastBatteryCheckTime = now;
        }

        // --- SENDING PACKET ---
        packetToSend.throttle = currentThrottlePWM;
        packetToSend.elrsActive = isLinkUp;
        packetToSend.button = button;
        packetToSend.haltIMU = haltIMU;
        packetToSend.brake = brakeActive;
        packetToSend.Automatic = Automatic;
        packetToSend.AutomaticSpeed = AutomaticSpeed;
        packetToSend.Gear = gearSwitch;
        packetToSend.motorEnable = slavePower.isMotorEnabled();

        packetToSend.checksum = calculateChecksum(&packetToSend);
        Serial1.write((uint8_t *)&packetToSend, sizeof(ControlPacket));

        // Obnovovacia frekvencia pre Slave (napr. 50Hz = 20ms) - NON-BLOCKING!
        vTaskDelay(SLAVE_COMMS_LOOP_MS / portTICK_PERIOD_MS);
    }
}

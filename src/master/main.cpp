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
    portENTER_CRITICAL(&sharedMux);
    if (beepRequestPending && beepRequestType == BEEP_ERROR) {
        portEXIT_CRITICAL(&sharedMux);
        return;
    }
    beepRequestType = (BeepPatternType)type;
    beepRequestCount = count;
    beepRequestFrequency = frequency;
    beepRequestDuration = duration_ms;
    beepRequestGap = gap_ms;
    beepRequestPending = true;
    portEXIT_CRITICAL(&sharedMux);
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
    setup_imon();
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

    unsigned long imuNow = millis();

    // --- DEBUG: Show raw values ---
    static unsigned long lastDebugTime = 0;
    if (imuNow - lastDebugTime > 500) {
        Serial.printf("[DEBUG] Gyro: X=%.2f Y=%.2f Z=%.2f | Accel: X=%.2f Y=%.2f Z=%.2f\n",
                      corrected_gx, corrected_gy, corrected_gz,
                      corrected_ax, corrected_ay, corrected_az);
        lastDebugTime = imuNow;
    }

    // --- DETECT ABNORMAL TILT (COMPLEMENTARY FILTER: GYRO + ACCEL) ---
    TiltData tilt = detect_abnormal_tilt(corrected_ax, corrected_ay, corrected_az,
                                         corrected_gx, corrected_gy, corrected_gz);

    // --- DEBUG: Always show tilt angles ---
    static unsigned long lastTiltTime = 0;
    if (imuNow - lastTiltTime > 500) {
        Serial.printf("[TILT] Roll: %.1f° | Pitch: %.1f° | Threshold: %.1f°\n",
                      tilt.roll, tilt.pitch, TILT_WARNING_THRESHOLD);
        lastTiltTime = imuNow;
    }

    static unsigned long haltClearStart = 0;
    float maxTilt = max(abs(tilt.roll), abs(tilt.pitch));
    const float TILT_RECOVERY_THRESHOLD = TILT_WARNING_THRESHOLD - 5.0f;

    if (tilt.isAbnormal)
    {
        haltIMU = true; // Set flag - TaskSlaveComms will handle beeping
        haltClearStart = 0;
        Serial.printf("  🚨 Roll: %.1f° | Pitch: %.1f° %s\n",
                      tilt.roll, tilt.pitch, tilt.isCritical ? "[CRITICAL]" : "[WARNING]");
    }
    else if (haltIMU)
    {
        if (maxTilt < TILT_RECOVERY_THRESHOLD)
        {
            if (haltClearStart == 0)
            {
                haltClearStart = millis();
            }
            else if (millis() - haltClearStart >= 2000)
            {
                haltIMU = false;
                haltClearStart = 0;
                Serial.println("  ✅ Tilt returned to normal, clearing haltIMU");
            }
        }
        else
        {
            haltClearStart = 0;
        }
    }
}


static uint8_t batteryVoltageToPercent(float v) {
    // 3S LiPo discharge curve (non-linear approximation)
    static const float voltages[] = {9.0f, 9.6f, 10.2f, 10.8f, 11.1f, 11.4f, 11.7f, 12.0f, 12.3f, 12.6f};
    static const uint8_t percents[] = {0, 5, 15, 30, 50, 65, 78, 88, 95, 100};
    const int n = sizeof(voltages) / sizeof(voltages[0]);
    if (v <= voltages[0]) return 0;
    if (v >= voltages[n - 1]) return 100;
    for (int i = 1; i < n; i++) {
        if (v <= voltages[i]) {
            float t = (v - voltages[i - 1]) / (voltages[i] - voltages[i - 1]);
            return (uint8_t)(percents[i - 1] + t * (percents[i] - percents[i - 1]));
        }
    }
    return 100;
}

void check_battery()
{
    unsigned long now = millis();

    int rawADC = analogRead(VBAT_REF);
    int rawImon = analogRead(IMON_CURRENT);

    float voltageImon = (rawImon * ADC_REF_VOLTAGE) / ADC_RESOLUTION;
    imonCurrent = voltageImon / IMON_CONVERSION_FACTOR;

    batteryVoltage = (rawADC / (float)ADC_RESOLUTION) * ADC_REF_VOLTAGE * VBAT_DIVIDER_RATIO;
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

        if (crsf)
        {
            crsf->telemetryWriteBattery(BATTERY_VOLTAGE_MAX, 0, (uint32_t)(AutomaticSpeed / 100), batteryPercent);
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

    batteryPercent = batteryVoltageToPercent(batteryVoltage);

    if (crsf)
    {
        crsf->telemetryWriteBattery(batteryVoltage, 0, (uint32_t)(AutomaticSpeed / 100), batteryPercent);
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
        launchControl = rcChannels->value[11] > 1000;
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
        static bool lcStagingReady = false;
        static bool lcLaunchPhase = false;
        static unsigned long lcLaunchTimer = 0;
        if (rcChannels->value[8] < 1000) {
            brakeActive = false;
            lcStagingReady = false;
            lcStagingActive = false;
            lcLaunchPhase = false;
            lcLaunchTimer = 0;
            servo_break.writeMicroseconds(BRAKE_SERVO_CENTER_US);
        } else if (!launchControl || Automatic) {
            brakeActive = true;
            lcStagingReady = false;
            lcStagingActive = false;
            lcLaunchPhase = false;
            lcLaunchTimer = 0;
            servo_break.writeMicroseconds(BRAKE_SERVO_PRESSED_US);
        } else if (!slavePower.isMotorEnabled() && gearSwitch == 1 && button == 0) {
            // LC staging: brake held, motor off, gear 1, button at minimum
            brakeActive = true;
            lcStagingReady = true;
            lcStagingActive = true;
            lcLaunchPhase = false;
            lcLaunchTimer = 0;
            servo_break.writeMicroseconds(BRAKE_SERVO_PRESSED_US);
        } else if (lcStagingReady) {
            // LC launch: motor enabled after staging — hold brake 1s then release
            if (!lcLaunchPhase) {
                lcLaunchPhase = true;
                lcLaunchTimer = millis();
            }
            lcStagingActive = false;
            brakeActive = false;
            if (millis() - lcLaunchTimer < LAUNCH_CONTROL_BRAKE_HOLD_MS) {
                servo_break.writeMicroseconds(BRAKE_SERVO_PRESSED_US);
            } else {
                servo_break.writeMicroseconds(BRAKE_SERVO_CENTER_US);
            }
        } else {
            // motor was already on when brake was pressed — normal brake
            brakeActive = true;
            lcStagingActive = false;
            lcLaunchPhase = false;
            lcLaunchTimer = 0;
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

        
        gearServoRaw = (int16_t)constrain(map(rcChannels->value[3], 172, 1811, 1200, 1800), 1200, 1800);
        
        AutomaticSpeed = map(rcChannels->value[9], 191, 1811, AUTOMATIC_MIN_SPEED, AUTOMATIC_MAX_SPEED);

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
        static unsigned long lastLedShow = 0;

        if (errorActive)
        {
            static unsigned long lastErrorBlink = 0;
            static bool errorLedOn = true;

            if ((now - lastErrorBlink) >= ERROR_BLINK_INTERVAL_MS)
            {
                errorLedOn = !errorLedOn;
                lastErrorBlink = now;
                ledsNeedUpdate = true;
            }

            if (errorLedOn)
                pixels.fill(pixels.Color(255, 0, 0));
            else
                pixels.clear();
        }
        else if (isFailsafeActive)
        {
            if (knightRiderStep()) ledsNeedUpdate = true;
        }
        else if (haltIMU)
        {
            pixels.fill(pixels.Color(255, 0, 0));
            ledsNeedUpdate = true;
        }
        else if (lcStagingActive)
        {
            // LC staging: front LEDs magenta, brake overlay keeps rear LEDs red
            for (int i = 0; i < BRAKE_LIGHT_START; i++)
                pixels.setPixelColor(i, pixels.Color(255, 0, 128));
            ledsNeedUpdate = true;
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
                    ledsNeedUpdate = true;
                }
                else if ((now - lastAutoPulse) >= AUTO_PULSE_DURATION_MS)
                {
                    pixels.fill(pixels.Color(0, 255, 0));
                    ledsNeedUpdate = true;
                }
                else
                {
                    pixels.fill(pulseColor);
                }
            }
            else
            {
                pixels.fill(pixels.Color(0, 255, 0));
                ledsNeedUpdate = true;
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
                    ledsNeedUpdate = true;
                }
                else if ((now - lastBluePulse) >= BLUE_PULSE_DURATION_MS)
                {
                    pixels.fill(pixels.Color(0, 255, 0));
                    ledsNeedUpdate = true;
                }
                else
                {
                    pixels.fill(pixels.Color(0, 0, 255));
                }
            }
            else
            {
                pixels.fill(pixels.Color(0, 255, 0));
                ledsNeedUpdate = true;
            }
        }
        else
        {
            pixels.fill(pixels.Color(0, 255, 0));
            ledsNeedUpdate = true;
        }

        // Airplane strobe overlay — uses absolute time to stay phase-consistent
        bool motorNowEnabled = slavePower.isMotorEnabled();
        if (!motorNowEnabled && !isFailsafeActive && !errorActive && !haltIMU)
        {
            unsigned long phase = now % STROBE_PERIOD_MS;
            if (phase < 60 || (phase >= 120 && phase < 180))
            {
                pixels.fill(pixels.Color(255, 255, 255));
                ledsNeedUpdate = true;
            }
        }
        motorWasEnabled = motorNowEnabled;

        // Brake light overlay — always last, so it overrides everything on the rear LEDs
        if (brakeActive)
        {
            for (int i = BRAKE_LIGHT_START; i < NEOPIXEL_COUNT; i++)
                pixels.setPixelColor(i, pixels.Color(255, 0, 0));
            ledsNeedUpdate = true;
        }

        if (ledsNeedUpdate && (now - lastLedShow) >= 33)
        {
            pixels.show();
            ledsNeedUpdate = false;
            lastLedShow = now;
        }

        // --- NON-BLOCKING BEEP MACHINE ---

        static uint8_t beepCountLeft = 0;

        if (!beepActive && beepCountLeft == 0)
        {
            portENTER_CRITICAL(&sharedMux);
            bool pending = beepRequestPending;
            uint8_t cnt = beepRequestCount;
            if (pending) beepRequestPending = false;
            portEXIT_CRITICAL(&sharedMux);

            if (pending) beepCountLeft = cnt;
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
        packetToSend.launchControl = lcStagingActive;
        packetToSend.motorEnable = slavePower.isMotorEnabled();

        packetToSend.checksum = calculateChecksum(&packetToSend);
        Serial1.write((uint8_t *)&packetToSend, sizeof(ControlPacket));

        // Obnovovacia frekvencia pre Slave (napr. 50Hz = 20ms) - NON-BLOCKING!
        vTaskDelay(SLAVE_COMMS_LOOP_MS / portTICK_PERIOD_MS);
    }
}

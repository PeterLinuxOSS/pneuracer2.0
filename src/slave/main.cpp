#include <Arduino.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <AS5600.h>
#include "shared/SharedData.h"
#include "pins.h"
#include "config.h"

Servo airServo;
Adafruit_NeoPixel StatusLed(NEOPIXEL_COUNT, STATUS_LED, NEO_GRB + NEO_KHZ800);
AS5600 as5600;

// --- SHARED DATA (Thread Safe) ---
volatile ControlPacket currentData;
volatile bool isConnectionActive = false;
SemaphoreHandle_t dataMutex;

// --- PROTOTYPES ---
void TaskComms(void *pvParameters);
void setFailsafe();
void updateServo(int targetGear, bool motorEnabled, bool isMoving, bool pistonMoving);

unsigned long lastSwitchTime = 0;
bool ledBlinkOn = false;
unsigned long lastBlinkTime = 0;
bool directionForward = true;
int delayMin = 0;

uint16_t prev_angle = 0;
unsigned long prev_time = 0;
float angular_speed = 0.0; // degrees per second

bool automaticPrev = false;
bool manualMotorPrev = false;
float automaticTargetSpeed = 0.0;
int servoPosition = SERVO_NEUTRAL_US;
long currentDelayTarget = DELAY_MAX_MS;
int currentGear = 2;
int lastServoGear = -1;
unsigned long gearShiftTimer = 0;
bool gearUpCondition = false;
bool gearDownCondition = false;
unsigned long lastStatusLogTime = 0;

void setup_NeoPixel()
{
    pinMode(STATUS_LED_PWR, OUTPUT);
    digitalWrite(STATUS_LED_PWR, HIGH);
    StatusLed.begin();
    StatusLed.clear();
    StatusLed.setBrightness(LED_BRIGHTNESS);
    StatusLed.fill(StatusLed.Color(255, 100, 0));
    StatusLed.show();
}

void setup()
{
    setup_NeoPixel();
    Serial.begin(921600);

    Wire.begin(SDA_PIN, SCL_PIN);
    as5600.begin();

    Serial1.begin(921600, SERIAL_8N1, UART_RX, UART_TX);

    pinMode(VALVE_A, OUTPUT);
    pinMode(VALVE_B, OUTPUT);
    digitalWrite(VALVE_A, LOW);
    digitalWrite(VALVE_B, LOW);
    pinMode(HALL_START, INPUT);
    pinMode(HALL_END, INPUT);
    delay(1000);

    ESP32PWM::allocateTimer(0);
    airServo.setPeriodHertz(333);
    airServo.attach(SERVO_REG, 500, 2500);
    airServo.write(servoPosition);

    dataMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(
        TaskComms,
        "Comms",
        4096,
        NULL,
        1,
        NULL,
        0);

    Serial.println("SLAVE ESP32-S3 Ready");
}

void loop()
{
    ControlPacket localData = {};
    bool connectionOK = false;

    if (xSemaphoreTake(dataMutex, (TickType_t)5) == pdTRUE)
    {
        memcpy(&localData, (const void *)&currentData, sizeof(ControlPacket));
        connectionOK = isConnectionActive;
        xSemaphoreGive(dataMutex);
    }

    // Read AS5600 raw angle and compute angular speed.
    uint16_t current_angle = as5600.rawAngle();
    unsigned long current_time = millis();
    if (prev_time != 0)
    {
        unsigned long delta_t = current_time - prev_time;
        if (delta_t > 0)
        {
            int16_t delta_angle = current_angle - prev_angle;
            if (delta_angle > 2048)
                delta_angle -= 4096;
            else if (delta_angle < -2048)
                delta_angle += 4096;
            angular_speed = (delta_angle * 360.0f / 4096.0f) / (delta_t / 1000.0f);
        }
    }
    prev_angle = current_angle;
    prev_time = current_time;
    bool isMoving = abs(angular_speed) > MOVING_SPEED_THRESHOLD;

    uint8_t targetR = 0;
    uint8_t targetG = 0;
    uint8_t targetB = 0;
    bool pistonMoving = false;
    bool motorActive = false;
    int targetGear = localData.Gear;

    if (connectionOK)
    {
        if (!localData.elrsActive)
        {
            setFailsafe();
            unsigned long now = millis();
            if (now - lastBlinkTime >= BLINK_INTERVAL_MS)
            {
                ledBlinkOn = !ledBlinkOn;
                lastBlinkTime = now;
            }
            if (ledBlinkOn)
            {
                targetR = 0;
                targetG = 255;
                targetB = 255;
            }
        }
        else if (localData.haltIMU)
        {
            setFailsafe();
            targetR = 0;
            targetG = 0;
            targetB = 255;
        }
        else if (localData.brake && !localData.launchControl)
        {
            setFailsafe();
            targetR = 255;
            targetG = 255;
            targetB = 0;
        }
        else
        {
            targetR = localData.motorEnable ? 0 : 255;
            targetG = localData.motorEnable ? 255 : 80;
            targetB = 0;

            if (localData.Automatic)
            {
                delayMin = localData.button;
                if (!automaticPrev)
                {
                    automaticPrev = true;
                    automaticTargetSpeed = (localData.AutomaticSpeed > 0) ? localData.AutomaticSpeed : AUTO_DEFAULT_SPEED;
                    currentDelayTarget = delayMin;
                    currentGear = 1;
                }
                else if (localData.AutomaticSpeed > 0 && localData.AutomaticSpeed != automaticTargetSpeed)
                {
                    automaticTargetSpeed = localData.AutomaticSpeed;
                }

                int speedTolerance = constrain(map(localData.throttle, 190, 1800, 1, 500), 20, 500);
                float targetLow = automaticTargetSpeed - speedTolerance;
                int maxAdjustment = AUTO_MAX_ADJUSTMENT;

                if (angular_speed > automaticTargetSpeed)
                {
                    int adjustment = constrain((int)((angular_speed - automaticTargetSpeed) * AUTO_KP), 1, maxAdjustment);
                    currentDelayTarget += adjustment;
                }
                else if (angular_speed < targetLow)
                {
                    int adjustment = constrain((int)((targetLow - angular_speed) * AUTO_KP), 1, maxAdjustment);
                    currentDelayTarget -= adjustment;
                }
                else
                {
                    currentDelayTarget += 1;
                }
                currentDelayTarget = constrain(currentDelayTarget, delayMin, DELAY_MAX_MS);
                motorActive = true;
                targetGear = currentGear;

                float gearUpThresh = min(automaticTargetSpeed * GEAR_UP_RATIO, GEAR_UP_SPEED);
                if (angular_speed >= gearUpThresh && currentGear == 1)
                {
                    if (!gearUpCondition)
                    {
                        gearUpCondition = true;
                        gearShiftTimer = millis();
                    }
                    else if (millis() - gearShiftTimer > GEAR_SHIFT_DELAY_MS)
                    {
                        currentGear = 3;
                        gearUpCondition = false;
                    }
                }
                else
                {
                    gearUpCondition = false;
                }

                float gearDownThresh = min(automaticTargetSpeed * GEAR_DOWN_RATIO, GEAR_DOWN_SPEED);
                if (angular_speed < gearDownThresh && currentGear == 3)
                {
                    if (!gearDownCondition)
                    {
                        gearDownCondition = true;
                        gearShiftTimer = millis();
                    }
                    else if (millis() - gearShiftTimer > GEAR_SHIFT_DELAY_MS)
                    {
                        currentGear = 1;
                        gearDownCondition = false;
                    }
                }
                else
                {
                    gearDownCondition = false;
                }

                if (millis() - lastStatusLogTime >= 100)
                {
                    lastStatusLogTime = millis();
                    Serial.printf("Auto target: %.1f, actual: %.1f, delay: %ld, gear: %d, up@%.0f, dn@%.0f\n",
                                  automaticTargetSpeed, angular_speed, currentDelayTarget, currentGear,
                                  gearUpThresh, gearDownThresh);
                }
            }
            else
            {

                automaticPrev = false;
                motorActive = (localData.throttle > THROTTLE_DEADZONE);
                if (motorActive)
                {
                    delayMin = localData.button;
                    float manualTargetSpeed = constrain((float)map(localData.throttle, THROTTLE_DEADZONE, 1800, MANUAL_MIN_SPEED, MANUAL_MAX_SPEED), (float)MANUAL_MIN_SPEED, (float)MANUAL_MAX_SPEED);
                    if (millis() - lastStatusLogTime >= 100)
                    {
                        lastStatusLogTime = millis();
                        Serial.printf("Manual target: %.1f, actual: %.1f, delay: %ld, gear: %d\n",
                                      manualTargetSpeed, angular_speed, currentDelayTarget, currentGear);
                    }
                    if (!manualMotorPrev)
                    {
                        manualMotorPrev = true;
                        currentDelayTarget = DELAY_MAX_MS;
                    }
                    int speedTolerance = constrain(map(localData.throttle, THROTTLE_DEADZONE, 1800, 1, 500), 20, 500);
                    float targetLow = manualTargetSpeed - speedTolerance;
                    if (angular_speed > manualTargetSpeed)
                    {
                        int adjustment = constrain((int)((angular_speed - manualTargetSpeed) * AUTO_KP), 1, AUTO_MAX_ADJUSTMENT);
                        currentDelayTarget += adjustment;
                    }
                    else if (angular_speed < targetLow)
                    {
                        int adjustment = constrain((int)((targetLow - angular_speed) * AUTO_KP), 1, AUTO_MAX_ADJUSTMENT);
                        currentDelayTarget -= adjustment;
                    }
                    else
                    {
                        currentDelayTarget += 1;
                    }
                    currentDelayTarget = constrain(currentDelayTarget, delayMin, DELAY_MAX_MS);
                }
                else
                {
                    manualMotorPrev = false;
                }
            }

            if (motorActive && localData.motorEnable)
            {
                int h_start = analogRead(HALL_START);
                int h_end = analogRead(HALL_END);
                bool justHitSensor = false;

                if (h_start < HALL_THRESHOLD && !directionForward)
                {
                    directionForward = true;
                    justHitSensor = true;
                }
                else if (h_end < HALL_THRESHOLD && directionForward)
                {
                    directionForward = false;
                    justHitSensor = true;
                }

                if (justHitSensor)
                {
                    lastSwitchTime = millis();
                }

                unsigned long timeSinceHit = millis() - lastSwitchTime;
                if (timeSinceHit < (unsigned long)currentDelayTarget)
                {
                    digitalWrite(VALVE_A, LOW);
                    digitalWrite(VALVE_B, LOW);
                }
                else
                {
                    pistonMoving = true;
                    if (directionForward)
                    {
                        digitalWrite(VALVE_A, HIGH);
                        digitalWrite(VALVE_B, LOW);
                    }
                    else
                    {
                        digitalWrite(VALVE_A, LOW);
                        digitalWrite(VALVE_B, HIGH);
                    }
                }
            }
            else
            {
                lastSwitchTime = 0;
                digitalWrite(VALVE_A, LOW);
                digitalWrite(VALVE_B, LOW);
                if (localData.brake && localData.launchControl && !localData.motorEnable) {
                    int h_start = analogRead(HALL_START);
                    if (h_start > HALL_THRESHOLD) {
                        digitalWrite(VALVE_B, HIGH);
                    } else {
                        directionForward = true;
                    }
                }
            }

            if (localData.brake && localData.launchControl) {
                // LC staging: force gear 1 regardless of piston/movement state
                if (lastServoGear != 1) {
                    servoPosition = SERVO_GEAR1_US;
                    airServo.writeMicroseconds(servoPosition);
                    lastServoGear = 1;
                }
            } else {
                updateServo(targetGear, localData.motorEnable, isMoving, pistonMoving);
            }
        }
    }
    else
    {
        setFailsafe();
        unsigned long now = millis();
        if (now - lastBlinkTime >= BLINK_INTERVAL_MS)
        {
            ledBlinkOn = !ledBlinkOn;
            lastBlinkTime = now;
        }
        if (ledBlinkOn)
        {
            targetR = 255;
            targetG = 0;
            targetB = 255;
        }
    }

    StatusLed.fill(StatusLed.Color(targetR, targetG, targetB));
    StatusLed.show();
    delay(10);
}

// --- TASK: COMMUNICATION (CORE 0) ---
void TaskComms(void *pvParameters)
{
    ControlPacket tempPacket;
    unsigned long lastPacketTime = 0;

    for (;;)
    {
        if (Serial1.available() >= sizeof(ControlPacket))
        {
            if (Serial1.peek() != (PACKET_HEADER & 0xFF))
            {
                Serial1.read();
                continue;
            }

            Serial1.readBytes((char *)&tempPacket, sizeof(ControlPacket));

            if (tempPacket.header == PACKET_HEADER &&
                tempPacket.checksum == calculateChecksum(&tempPacket))
            {
                if (xSemaphoreTake(dataMutex, (TickType_t)5) == pdTRUE)
                {
                    memcpy((void *)&currentData, &tempPacket, sizeof(ControlPacket));
                    isConnectionActive = true;
                    xSemaphoreGive(dataMutex);
                }
                lastPacketTime = millis();
            }
        }

        if (millis() - lastPacketTime > CONNECTION_TIMEOUT_MS)
        {
            if (xSemaphoreTake(dataMutex, (TickType_t)5) == pdTRUE)
            {
                isConnectionActive = false;
                xSemaphoreGive(dataMutex);
            }
        }

        vTaskDelay(5);
    }
}

void setFailsafe()
{
    digitalWrite(VALVE_A, LOW);
    digitalWrite(VALVE_B, LOW);
}

void updateServo(int targetGear, bool motorEnabled, bool isMoving, bool pistonMoving)
{
    if (!motorEnabled && !isMoving)
    {
        servoPosition = SERVO_NEUTRAL_US;
        airServo.writeMicroseconds(servoPosition);
        lastServoGear = 2;
        return;
    }

    if (!pistonMoving)
    {
        return;
    }

    if (targetGear == lastServoGear)
    {
        return;
    }

    if (targetGear == 1)
    {
        servoPosition = SERVO_GEAR1_US;
    }
    else if (targetGear == 3)
    {
        servoPosition = SERVO_GEAR3_US;
    }
    else
    {
        servoPosition = SERVO_NEUTRAL_US;
    }

    airServo.writeMicroseconds(servoPosition);
    lastServoGear = targetGear;
}

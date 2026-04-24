#include <Arduino.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <AS5600.h>
#include "shared/SharedData.h" // Your shared structure
#include "pins.h"
#include "config.h"

Servo airServo;
Adafruit_NeoPixel StatusLed(NEOPIXEL_COUNT, STATUS_LED, NEO_GRB + NEO_KHZ800);
AS5600 as5600;

// --- SHARED DATA (Thread Safe) ---
volatile ControlPacket currentData;
volatile bool isConnectionActive = false;
SemaphoreHandle_t dataMutex;

// --- PROTOTYPY ---
void TaskComms(void *pvParameters);
void setFailsafe();

// Simple valve switching state (no Hall sensors)
int currentValveState = 0;
unsigned long lastSwitchTime = 0;
int hall_start = 0;
int hall_end = 0;
bool ledBlinkOn = false;
unsigned long lastBlinkTime = 0;
bool directionForward = true;
bool valveA = LOW;
bool valveB = LOW;
int delayAfterSwitch = 0;
unsigned long forwardTimer = 0;
unsigned long reverseTimer = 0;
unsigned long movementStartTime = 0;
int counter = 0;
unsigned long nowBlink = 0;
int delayMin = 0;

// AS5600 variables
uint16_t prev_angle = 0;
unsigned long prev_time = 0;
float angular_speed = 0.0; // degrees per second

bool automaticPrev = false;
float automaticTargetSpeed = 0.0;
int servoPosition = SERVO_NEUTRAL_US;
long currentDelayTarget = DELAY_MAX_MS;
int currentGear = 2;
int lastServoGear = -1;
unsigned long gearShiftTimer = 0;
bool gearUpCondition = false;
bool gearDownCondition = false;

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
        0
    );

    Serial.println("SLAVE ESP32-S3 Ready");
}

void loop()
{
    ControlPacket localData;
    bool connectionOK;

    if (xSemaphoreTake(dataMutex, (TickType_t)5) == pdTRUE)
    {
        localData = *(const ControlPacket *)&currentData;
        connectionOK = isConnectionActive;
        xSemaphoreGive(dataMutex);
    }

    // Read AS5600 raw angle
    uint16_t current_angle = as5600.rawAngle();
    unsigned long current_time = millis();
    if (prev_time != 0)
    {
        unsigned long delta_t = current_time - prev_time;
        if (delta_t > 0)
        {
            int16_t delta_angle = current_angle - prev_angle;
            // Handle wrap around (12-bit, 0-4095)
            if (delta_angle > 2048)
                delta_angle -= 4096;
            if (delta_angle < -2048)
                delta_angle += 4096;
            angular_speed = (delta_angle * 360.0 / 4096.0) / (delta_t / 1000.0);
        }
    }
    prev_angle = current_angle;
    prev_time = current_time;
    bool isMoving = abs(angular_speed) > MOVING_SPEED_THRESHOLD;

    if (connectionOK)
    {
        int val1 = analogRead(HALL_START);
        int val2 = analogRead(HALL_END);

        if (localData.elrsActive)
        {
            if (localData.haltIMU == false)
            {
                if (localData.brake)
                {
                    digitalWrite(VALVE_A, LOW);
                    digitalWrite(VALVE_B, LOW);
                    lastSwitchTime = 0;
                    if (!isMoving && lastServoGear != 1)
                    {
                        airServo.writeMicroseconds(SERVO_GEAR1_US);
                        lastServoGear = 1;
                        currentGear = 1;
                    }
                    StatusLed.fill(StatusLed.Color(255, 255, 0));
                    StatusLed.show();
                    delay(10);
                    return;
                }

                if (localData.motorEnable)
                    StatusLed.fill(StatusLed.Color(0, 255, 0));
                else
                    StatusLed.fill(StatusLed.Color(255, 80, 0));

                // --- MODE-SPECIFIC DELAY CALCULATION ---
                bool motorActive = false;

                if (localData.Automatic == true)
                {
                    delayMin = localData.button;

                    if (!automaticPrev)
                    {
                        automaticPrev = true;
                        automaticTargetSpeed = (localData.AutomaticSpeed > 0) ? localData.AutomaticSpeed : AUTO_DEFAULT_SPEED;
                        currentDelayTarget = delayMin;
                        currentGear = 1;
                    }
                    else if (localData.AutomaticSpeed != automaticTargetSpeed && localData.AutomaticSpeed > 0)
                    {
                        automaticTargetSpeed = localData.AutomaticSpeed;
                    }

                    int speedTolerance = map(localData.throttle, 190, 1800, 1, 500);
                    speedTolerance = constrain(speedTolerance, 20, 500);

                    float targetLow = automaticTargetSpeed - speedTolerance;
                    float kp = AUTO_KP;
                    int maxAdjustment = AUTO_MAX_ADJUSTMENT;

                    if (angular_speed > automaticTargetSpeed)
                    {
                        int adjustment = (int)((angular_speed - automaticTargetSpeed) * kp);
                        adjustment = constrain(adjustment, 1, maxAdjustment);
                        currentDelayTarget += adjustment;
                    }
                    else if (angular_speed < targetLow)
                    {
                        int adjustment = (int)((targetLow - angular_speed) * kp);
                        adjustment = constrain(adjustment, 1, maxAdjustment);
                        currentDelayTarget -= adjustment;
                    }
                    else
                    {
                        currentDelayTarget += 1;
                    }
                    currentDelayTarget = constrain(currentDelayTarget, delayMin, DELAY_MAX_MS);
                    motorActive = true;

                    // Automatic gear shifting
                    if (angular_speed >= GEAR_UP_SPEED && currentGear == 1)
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

                    if (angular_speed < GEAR_DOWN_SPEED && currentGear == 3)
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

                    if (millis() % 100 == 0)
                    {
                        Serial.printf("Auto target: %.1f, actual: %.1f, delay: %ld, gear: %d\n",
                                      automaticTargetSpeed, angular_speed, currentDelayTarget, currentGear);
                    }
                }
                else
                {
                    automaticPrev = false;
                    motorActive = (localData.throttle > THROTTLE_DEADZONE);

                    if (motorActive)
                    {
                        currentDelayTarget = map(localData.throttle, THROTTLE_DEADZONE, 1800, DELAY_MAX_MS, delayMin);
                        if (currentDelayTarget < delayMin)
                            currentDelayTarget = delayMin;
                        delayMin = localData.button;
                    }
                }

                // --- SHARED VALVE CONTROL LOGIC ---
                bool pistonMoving = false;

                if (motorActive && localData.motorEnable)
                {
                    int h_start = analogRead(HALL_START);
                    int h_end = analogRead(HALL_END);

                    bool justHitSensor = false;

                    if (h_start < HALL_THRESHOLD && !directionForward)
                    {
                        directionForward = true;
                        justHitSensor = true;
                        counter++;
                    }
                    else if (h_end < HALL_THRESHOLD && directionForward)
                    {
                        directionForward = false;
                        justHitSensor = true;
                        counter++;
                    }

                    if (justHitSensor)
                    {
                        lastSwitchTime = millis();
                        movementStartTime = 0;
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
                        if (movementStartTime == 0)
                            movementStartTime = millis();

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
                }

                // --- SERVO GEAR CONTROL ---
                int targetGear = (localData.Automatic && localData.motorEnable) ? currentGear : localData.Gear;
                if (!localData.motorEnable && !isMoving)
                {
                    servoPosition = SERVO_NEUTRAL_US;
                    airServo.writeMicroseconds(servoPosition);
                    lastServoGear = 2;
                }
                else if (pistonMoving)
                {
                    if (targetGear != lastServoGear)
                    {
                        if (targetGear == 1)
                            servoPosition = SERVO_GEAR1_US;
                        else if (targetGear == 2)
                            servoPosition = SERVO_NEUTRAL_US;
                        else if (targetGear == 3)
                            servoPosition = SERVO_GEAR3_US;
                        airServo.writeMicroseconds(servoPosition);
                        lastServoGear = targetGear;
                    }
                }
            }
            else
            {
                StatusLed.fill(StatusLed.Color(0, 0, 255));
                StatusLed.show();
                setFailsafe();
            }
        }
        else
        {
            setFailsafe();
            nowBlink = millis();
            if (nowBlink - lastBlinkTime >= BLINK_INTERVAL_MS)
            {
                ledBlinkOn = !ledBlinkOn;
                lastBlinkTime = nowBlink;
            }
            StatusLed.fill(ledBlinkOn ? StatusLed.Color(0, 255, 255) : StatusLed.Color(0, 0, 0));
        }
    }
    else
    {
        setFailsafe();
        nowBlink = millis();
        if (nowBlink - lastBlinkTime >= BLINK_INTERVAL_MS)
        {
            ledBlinkOn = !ledBlinkOn;
            lastBlinkTime = nowBlink;
        }
        StatusLed.fill(ledBlinkOn ? StatusLed.Color(255, 0, 255) : StatusLed.Color(0, 0, 0));
    }

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
    StatusLed.show();
}

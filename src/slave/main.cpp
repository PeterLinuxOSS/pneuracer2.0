#include <Arduino.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <AS5600.h>
#include "shared/SharedData.h" // Your shared structure
#include "pins.h"

Servo airServo;
Adafruit_NeoPixel StatusLed(NEOPIXEL_COUNT, STATUS_LED, NEO_GRB + NEO_KHZ800);
AS5600 as5600;

// --- SHARED DATA (Thread Safe) ---
volatile ControlPacket currentData; // Data received from Master
volatile bool isConnectionActive = false;
SemaphoreHandle_t dataMutex;

// --- PROTOTYPY ---
void TaskComms(void *pvParameters);
void setFailsafe();

// Simple valve switching state (no Hall sensors)
int currentValveState = 0; // 0=both off, 1=VALVE_A, 2=VALVE_B
unsigned long lastSwitchTime = 0;
int hall_start = 0;
int hall_end = 0;
bool ledBlinkOn = false;
unsigned long lastBlinkTime = 0;
bool directionForward = true;
bool valveA = LOW;
bool valveB = LOW;
int delayAfterSwitch = 0; // ms
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
float automaticTargetSpeed = 0.0; // target speed in deg/s for Automatic mode
int servoPosition = 90;           // default servo position (0-180)
long currentDelayTarget = 3000;   // delay between valve switch in ms
int currentGear = 2;              // default gear (neutral)
int lastServoGear = -1;           // tracks what gear servo is actually set to
unsigned long gearShiftTimer = 0;
bool gearUpCondition = false;
bool gearDownCondition = false;

void setup_NeoPixel()
{
    pinMode(STATUS_LED_PWR, OUTPUT);
    digitalWrite(STATUS_LED_PWR, HIGH);
    StatusLed.begin();
    StatusLed.clear();
    StatusLed.setBrightness(50);
    StatusLed.fill(StatusLed.Color(255, 100, 0));
    StatusLed.show();
}

void setup()
{
    setup_NeoPixel();
    Serial.begin(921600); // USB Debug

    // I2C for AS5600
    Wire.begin(SDA_PIN, SCL_PIN);
    as5600.begin();

    // UART ku Masterovi
    Serial1.begin(921600, SERIAL_8N1, UART_RX, UART_TX);

    // Nastavenie pinov
    pinMode(VALVE_A, OUTPUT);
    pinMode(VALVE_B, OUTPUT);
    digitalWrite(VALVE_A, LOW);
    digitalWrite(VALVE_B, LOW);
    pinMode(HALL_START, INPUT); // Or INPUT_PULLUP if needed
    pinMode(HALL_END, INPUT);
    delay(1000); // Short pause for stabilization

    // Servo setup
    ESP32PWM::allocateTimer(0);
    airServo.setPeriodHertz(333);
    airServo.attach(SERVO_REG, 500, 2500);
    airServo.write(servoPosition); // Set default position

    // Mutex
    dataMutex = xSemaphoreCreateMutex();

    // Start communication task (Core 0)
    xTaskCreatePinnedToCore(
        TaskComms,
        "Comms",
        4096,
        NULL,
        1,
        NULL,
        0 // Runs on core 0
    );

    Serial.println("SLAVE ESP32-S3 Ready");
}

void loop()
{
    // --- MAIN LOOP (CORE 1) - CONTROL ---
    // This runs fast and manages physical pins

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
            angular_speed = (delta_angle * 360.0 / 4096.0) / (delta_t / 1000.0); // degrees per second
        }
    }
    prev_angle = current_angle;
    prev_time = current_time;
    // Print raw angle and speed

    if (connectionOK)
    {
        int val1 = analogRead(HALL_START);
        int val2 = analogRead(HALL_END);

        if (localData.elrsActive)
        {
            if (localData.haltIMU == false)
            {
                if (localData.motorEnable)
                    StatusLed.fill(StatusLed.Color(0, 255, 0));   // Green - motor active
                else
                    StatusLed.fill(StatusLed.Color(255, 80, 0));  // Orange - motor disabled, shifting OK

                // --- MODE-SPECIFIC DELAY CALCULATION ---
                bool motorActive = false;

                if (localData.Automatic == true)
                {
                    delayMin = localData.button; // button controls minimum delay in automatic mode

                    if (!automaticPrev)
                    {
                        automaticPrev = true;
                        automaticTargetSpeed = (localData.AutomaticSpeed > 0) ? localData.AutomaticSpeed : 2000.0;
                        currentDelayTarget = delayMin; // start with button value
                        currentGear = 1;               // start with gear 1 in automatic mode (servo: 900µs)
                    }
                    else if (localData.AutomaticSpeed != automaticTargetSpeed && localData.AutomaticSpeed > 0)
                    {
                        automaticTargetSpeed = localData.AutomaticSpeed;
                    }

                    // throttle controls speed tolerance from 20 to 500
                    int speedTolerance = map(localData.throttle, 190, 1800, 1, 500);
                    speedTolerance = constrain(speedTolerance, 20, 500);

                    float targetLow = automaticTargetSpeed - speedTolerance;
                    float kp = 0.04;        // proportional gain for delay adjustment
                    int maxAdjustment = 15; // limit how much delay changes each cycle

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
                        currentDelayTarget += 1; // coast
                    }
                    currentDelayTarget = constrain(currentDelayTarget, delayMin, 3000);
                    motorActive = true;

                    // Automatic gear shifting based on speed with delay
                    // Gear values: 1=gear1 (900µs), 3=gear2 (2200µs) — 2 is neutral, not used in auto
                    if (angular_speed >= automaticTargetSpeed * 0.75 && currentGear == 1)
                    {
                        if (!gearUpCondition)
                        {
                            gearUpCondition = true;
                            gearShiftTimer = millis();
                        }
                        else if (millis() - gearShiftTimer > 1000)
                        {
                            currentGear = 3;
                            gearUpCondition = false;
                        }
                    }
                    else
                    {
                        gearUpCondition = false;
                    }

                    if (angular_speed < automaticTargetSpeed * 0.7 && currentGear == 3)
                    {
                        if (!gearDownCondition)
                        {
                            gearDownCondition = true;
                            gearShiftTimer = millis();
                        }
                        else if (millis() - gearShiftTimer > 1000)
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
                        Serial.printf("Auto target: %.1f, actual: %.1f, delay: %ld, gear: %d\n", automaticTargetSpeed, angular_speed, currentDelayTarget, currentGear);
                    }
                }
                else
                {
                    automaticPrev = false;
                    motorActive = (localData.throttle > 200);

                    if (motorActive)
                    {
                        currentDelayTarget = map(localData.throttle, 200, 1800, 3000, delayMin);
                        if (currentDelayTarget < delayMin)
                            currentDelayTarget = delayMin;
                        delayMin = localData.button; // button sets min delay
                    }
                }

                // --- SHARED VALVE CONTROL LOGIC ---
                bool pistonMoving = false;

                if (motorActive && localData.motorEnable)
                {
                    int h_start = analogRead(HALL_START);
                    int h_end = analogRead(HALL_END);

                    // Detect sensor hit and change direction
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
                        // PAUSE
                        digitalWrite(VALVE_A, LOW);
                        digitalWrite(VALVE_B, LOW);
                    }
                    else
                    {
                        // MOVEMENT
                        pistonMoving = true;
                        if (movementStartTime == 0)
                        {
                            movementStartTime = millis();
                        }

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
                // Gear change only when piston is moving; always neutral otherwise
                int targetGear = (localData.Automatic && localData.motorEnable) ? currentGear : localData.Gear;
                if (!localData.motorEnable)
                {
                    // Motor disabled - always shift to neutral
                    servoPosition = 1500;
                    airServo.writeMicroseconds(servoPosition);
                    lastServoGear = 2;
                }
                else if (pistonMoving)
                {
                    if (targetGear != lastServoGear)
                    {
                        if (targetGear == 1)
                            servoPosition = 1800;
                        else if (targetGear == 2)
                            servoPosition = 1500;
                        else if (targetGear == 3)
                            servoPosition = 1200;
                        airServo.writeMicroseconds(servoPosition);
                        lastServoGear = targetGear;
                    }
                }
                else if (localData.Automatic == false)
                {
                    servoPosition = 1500;
                    airServo.writeMicroseconds(servoPosition);
                    lastServoGear = 2;
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
            if (nowBlink - lastBlinkTime >= 1000UL)
            {
                ledBlinkOn = !ledBlinkOn;
                lastBlinkTime = nowBlink;
            }
            if (ledBlinkOn)
            {
                StatusLed.fill(StatusLed.Color(0, 255, 255));
            }
            else
            {
                StatusLed.fill(StatusLed.Color(0, 0, 0));
            }
        }
    }
    else
    {
        // FAILSAFE MODE
        setFailsafe();
        nowBlink = millis();
        if (nowBlink - lastBlinkTime >= 1000UL)
        {
            ledBlinkOn = !ledBlinkOn;
            lastBlinkTime = nowBlink;
        }
        if (ledBlinkOn)
        {
            StatusLed.fill(StatusLed.Color(255, 0, 255));
        }
        else
        {
            StatusLed.fill(StatusLed.Color(0, 0, 0));
        }
    }

    StatusLed.show();

    // Here you can read Hall sensors and send back to Master if needed
    // bool pistonStart = digitalRead(HALL_START);

    delay(10); // 100Hz refresh rate is enough for valves
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

            // Integrity check (Checksum)
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

        if (millis() - lastPacketTime > 500)
        { // 500ms without signal
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
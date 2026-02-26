#include <Arduino.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include "shared/SharedData.h" // Tvoja zdieľaná štruktúra
#include "pins.h"

Servo airServo;
Adafruit_NeoPixel StatusLed(NEOPIXEL_COUNT, STATUS_LED, NEO_GRB + NEO_KHZ800);

// --- ZDIEĽANÉ DÁTA (Thread Safe) ---
volatile ControlPacket currentData; // Dáta prijaté od Mastera
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
void setup_NeoPixel()
{
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

    // UART ku Masterovi
    Serial1.begin(921600, SERIAL_8N1, UART_RX, UART_TX);

    // Nastavenie pinov
    pinMode(VALVE_A, OUTPUT);
    pinMode(VALVE_B, OUTPUT);
    digitalWrite(VALVE_A, LOW);
    digitalWrite(VALVE_B, LOW);
    pinMode(HALL_START, INPUT); // Alebo INPUT_PULLUP ak treba
    pinMode(HALL_END, INPUT);
    delay(1000); // Krátka pauza pro stabilizáciu

    // Servo setup
    ESP32PWM::allocateTimer(0);
    airServo.setPeriodHertz(333);
    airServo.attach(SERVO_REG, 500, 2500);

    // Mutex
    dataMutex = xSemaphoreCreateMutex();

    // Spustenie Tasku pre komunikáciu (Core 0)
    xTaskCreatePinnedToCore(
        TaskComms,
        "Comms",
        4096,
        NULL,
        1,
        NULL,
        0 // Beží na jadre 0
    );

    Serial.println("SLAVE ESP32-S3 Ready");
}

void loop()
{
    // --- HLAVNÁ SLUČKA (CORE 1) - OVLÁDANIE ---
    // Toto beží rýchlo a stará sa o fyzické piny

    ControlPacket localData;
    bool connectionOK;

    if (xSemaphoreTake(dataMutex, (TickType_t)5) == pdTRUE)
    {
        localData = *(const ControlPacket *)&currentData;
        connectionOK = isConnectionActive;
        xSemaphoreGive(dataMutex);
    }

    if (connectionOK)
    {
        int val1 = analogRead(HALL_START);
        int val2 = analogRead(HALL_END);

        if (localData.elrsActive)
        {
            StatusLed.fill(StatusLed.Color(0, 255, 0)); // Zelená - ELRS OK

            if (localData.throttle > 190)
            {
                // 1. Výpočet delay (nepriama úmera)
                // Plyn 185 -> 3000ms | Plyn 1800 -> 0ms

                long currentDelayTarget = map(localData.throttle, 190, 1800, 3000, delayMin); // Ak je button6 stlačený, znížime max delay na 500ms
                if (currentDelayTarget < delayMin)
                    currentDelayTarget = delayMin; // ochrana

                int h_start = analogRead(HALL_START);
                int h_end = analogRead(HALL_END);

                // 2. Detekcia nárazu na senzor a zmena smeru
                bool justHitSensor = false;

                if (h_start < HALL_THRESHOLD && !directionForward)
                {
                    directionForward = true;
                    justHitSensor = true;
                    counter++;
                    // Čas od posledného štartu pohybu po náraz na tento senzor
                    if (movementStartTime != 0)
                    {
                        Serial.printf("Reverse movement duration: %lu ms\n", millis() - movementStartTime);
                    }
                }
                else if (h_end < HALL_THRESHOLD && directionForward)
                {
                    directionForward = false;
                    justHitSensor = true;
                    counter++;
                    // Čas od posledného štartu pohybu po náraz na tento senzor
                    if (movementStartTime != 0)
                    {
                        Serial.printf("Forward movement duration: %lu ms\n", millis() - movementStartTime);
                    }
                }

                if (justHitSensor)
                {
                    lastSwitchTime = millis();
                    movementStartTime = 0; // Resetujeme, kým neskončí pauza
                    delayMin = localData.button; // Aktualizujeme minimálny delay podľa tlačidla
                }

                // 3. Rozhodnutie: Bežíme alebo čakáme?
                unsigned long timeSinceHit = millis() - lastSwitchTime;

                if (timeSinceHit < (unsigned long)currentDelayTarget)
                {
                    // SME V PAUZE
                    digitalWrite(VALVE_A, LOW);
                    digitalWrite(VALVE_B, LOW);
                }
                else
                {
                    // POHYB SA PRÁVE ZAČAL alebo TRVÁ
                    if (movementStartTime == 0)
                    {
                        movementStartTime = millis(); // Zaznamenáme presný moment štartu pohybu
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
                if (millis() % 100 == 0)
                { // Loguj len každých 100ms nech to nespamuje
                    Serial.printf("Throttle: %d | Delay: %ld ms | Active: %s | Counter: %d\n",
                                  localData.throttle, currentDelayTarget,
                                  (millis() - lastSwitchTime < currentDelayTarget) ? "PAUSE" : "MOVING", counter);
                }
            }
            else
            {
                lastSwitchTime = 0; // Reset timer
                // Throttle pod 185 - OFF
                digitalWrite(VALVE_A, LOW);
                digitalWrite(VALVE_B, LOW);
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
        // FAILSAFE REŽIM
        setFailsafe();
        nowBlink = millis();
        if (nowBlink - lastBlinkTime >= 1000UL)
        {
            ledBlinkOn = !ledBlinkOn;
            lastBlinkTime = nowBlink;
        }
        if (ledBlinkOn)
        {
            StatusLed.fill(StatusLed.Color(255,0 , 255));
        }
        else
        {
            StatusLed.fill(StatusLed.Color(0, 0, 0));}
        
    }

    StatusLed.show();

    // Tu môžeš čítať Hall senzory a posielať späť Masterovi (ak to bude treba)
    // bool pistonStart = digitalRead(HALL_START);

    delay(10); // Stačí 100Hz refresh rate pre ventily
}

// --- TASK: KOMUNIKÁCIA (CORE 0) ---
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

            // Kontrola integrity (Checksum)
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
        { // 500ms bez signálu
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
    StatusLed.fill(StatusLed.Color(255, 0, 255));
    StatusLed.show();
}
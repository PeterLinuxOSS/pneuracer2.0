#include <Arduino.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include "shared/SharedData.h" // Tvoja zdieľaná štruktúra
#include "pins.h"


// --- OBJEKTY ---
Servo airServo;
Adafruit_NeoPixel StatusLed(1, STATUS_LED, NEO_GRB + NEO_KHZ800);

// --- ZDIEĽANÉ DÁTA (Thread Safe) ---
volatile ControlPacket currentData; // Dáta prijaté od Mastera
volatile bool isConnectionActive = false;
int hall_start = 0;
int hall_end = 0;
SemaphoreHandle_t dataMutex;

// --- PROTOTYPY ---
void TaskComms(void *pvParameters);
void setFailsafe();

void setup() {
    Serial.begin(921600); // USB Debug
    
    // UART ku Masterovi
    Serial1.begin(921600, SERIAL_8N1, UART_RX, UART_TX);

    // Nastavenie pinov
    pinMode(VALVE_A, OUTPUT);
    pinMode(VALVE_B, OUTPUT);
    pinMode(HALL_START, INPUT); // Alebo INPUT_PULLUP ak treba
    pinMode(HALL_END, INPUT);

    // Servo setup
    ESP32PWM::allocateTimer(0);
    airServo.setPeriodHertz(333);
    airServo.attach(SERVO_REG, 500, 2500);

    // LED
    StatusLed.begin();
    StatusLed.setBrightness(50);
    StatusLed.fill(StatusLed.Color(255, 100, 0)); // Oranžová (Boot)
    StatusLed.show();

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

void loop() {
    // --- HLAVNÁ SLUČKA (CORE 1) - OVLÁDANIE ---
    // Toto beží rýchlo a stará sa o fyzické piny
    
    ControlPacket localData;
    bool connectionOK;

    // 1. Bezpečné načítanie dát z globálnej premennej
    if (xSemaphoreTake(dataMutex, (TickType_t)5) == pdTRUE) {
        localData = *(const ControlPacket*)&currentData; // Kópia dát
        connectionOK = isConnectionActive;
        xSemaphoreGive(dataMutex);
    }

    // 2. Logika ovládania
    if (connectionOK) {
        hall_start = analogRead(HALL_START);
        hall_end = analogRead(HALL_END);
        if (localData.throttle > 0) {
            airServo.writeMicroseconds(localData.throttle);
            if (hall_start < HALL_THRESHOLD) {
                digitalWrite(VALVE_A, HIGH);
                digitalWrite(VALVE_B, LOW);

            } else if (hall_end < HALL_THRESHOLD) {
                digitalWrite(VALVE_A, LOW);
                digitalWrite(VALVE_B, HIGH);
            } 
        } else{
            digitalWrite(VALVE_A, LOW);
            digitalWrite(VALVE_B, LOW);
            airServo.write(0); // Bezpečná poloha
        }
        
        
        StatusLed.fill(StatusLed.Color(0, 255, 0));
    } else {
        // FAILSAFE REŽIM (Master neodpovedá)
        setFailsafe();
        StatusLed.fill(StatusLed.Color(255, 0, 0)); // Červená
    }
    
    StatusLed.show();
    
    // Tu môžeš čítať Hall senzory a posielať späť Masterovi (ak to bude treba)
    // bool pistonStart = digitalRead(HALL_START);
    
    delay(10); // Stačí 100Hz refresh rate pre ventily
}

// --- TASK: KOMUNIKÁCIA (CORE 0) ---
void TaskComms(void *pvParameters) {
    ControlPacket tempPacket;
    unsigned long lastPacketTime = 0;

    for (;;) {
        // Čítame UART
        if (Serial1.available() >= sizeof(ControlPacket)) {
            
            // Kontrola hlavičky (Sync byte)
            if (Serial1.peek() != (PACKET_HEADER & 0xFF)) {
                Serial1.read(); // Zahodíme smeti
                continue;
            }

            Serial1.readBytes((char*)&tempPacket, sizeof(ControlPacket));

            // Kontrola integrity (Checksum)
            if (tempPacket.header == PACKET_HEADER && 
                tempPacket.checksum == calculateChecksum(&tempPacket)) {
                
                // Zápis do zdieľanej premennej
                if (xSemaphoreTake(dataMutex, (TickType_t)5) == pdTRUE) {
                    memcpy((void*)&currentData, &tempPacket, sizeof(ControlPacket));
                    isConnectionActive = true;
                    xSemaphoreGive(dataMutex);
                }
                lastPacketTime = millis();
            }
        }

        // WATCHDOG / FAILSAFE TIMER
        if (millis() - lastPacketTime > 500) { // 500ms bez signálu
            if (xSemaphoreTake(dataMutex, (TickType_t)5) == pdTRUE) {
                isConnectionActive = false; // Trigger failsafe v hlavnom loope
                xSemaphoreGive(dataMutex);
            }
        }

        vTaskDelay(5); 
    }
}

// Bezpečnostná funkcia
void setFailsafe() {
    digitalWrite(VALVE_A, LOW); // Zatvor ventily
    digitalWrite(VALVE_B, LOW);
    StatusLed.fill(StatusLed.Color(255, 0, 0)); // Červená
    StatusLed.show();
    
}
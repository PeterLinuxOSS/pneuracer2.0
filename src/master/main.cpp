#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "pins.h"            // Tvoje definície pinov
#include <CRSFforArduino.hpp>
#include <ESP32Servo.h>
#include <ESP32PWM.h>

// DÔLEŽITÉ: Musíš mať tento súbor v src/shared/SharedData.h
#include "shared/SharedData.h" 

// --- KONFIGURÁCIA PINOV PRE SLAVE UART ---
// Uprav podľa tvojej schémy! (Napr. IO17/IO18)
#define PIN_SLAVE_RX 18  
#define PIN_SLAVE_TX 17 

// --- OBJEKTY ---
Servo steering;
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, MASTER_STATUS_LED, NEO_GRB + NEO_KHZ800);
CRSFforArduino *crsf = nullptr;

// --- ZDIEĽANÉ PREMENNÉ (THREAD SAFE) ---
// volatile = premenná sa môže zmeniť kedykoľvek iným procesom
volatile int currentSteerPWM = 1500; 
volatile int currentThrottlePWM = 1500;
volatile bool isFailsafeActive = true;
volatile bool isLinkUp = false;

// --- DEFINÍCIE TASKOV ---
void TaskSlaveComms(void *pvParameters);
void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels);

// --- SETUP FUNKCIE ---

void setup_CRSF() {
    crsf = new CRSFforArduino(&Serial1, ELRS_RX_PIN, ELRS_TX_PIN);
    if (!crsf->begin()) {
        Serial.println("CRSF init failed!");
        while (1) delay(10);
    }
    crsf->setRcChannelsCallback(onReceiveRcChannels);
}

void setup_ServoPWM() {
    ESP32PWM::allocateTimer(0);
    steering.setPeriodHertz(50);
    steering.attach(16, 900, 2200); // Pozor: Pin 16 nesmie kolidovať s UARTom!
    steering.writeMicroseconds(1500);
}

void setup_NeoPixel() {
    pixels.begin();
    pixels.clear();
    pixels.setBrightness(50); // Šetríme oči aj prúd
    pixels.show();
}

void setup() {
    Serial.begin(921600); // USB Debug
    
    Serial2.begin(921600, SERIAL_8N1, PIN_SLAVE_RX, PIN_SLAVE_TX);

    setup_ServoPWM();
    setup_NeoPixel();
    setup_CRSF();

    // --- SPUSTENIE MULTITASKINGU (CORE 0) ---
    // Toto vlákno bude riešiť LEDky a posielanie dát Slave-u
    xTaskCreatePinnedToCore(
        TaskSlaveComms,   // Názov funkcie
        "SlaveComms",     // Názov pre debug
        4096,             // Veľkosť stacku
        NULL,             // Parametre
        1,                // Priorita (1 = nízka, nech nebrzdí CRSF)
        NULL,             // Handle
        0                 // Beží na jadre 0 (Core 0)
    );

    Serial.println("MASTER Setup Complete. Running Multithreaded.");
}

// --- HLAVNÁ SLUČKA (CORE 1) ---
// Toto jadro sa venuje LEN rýchlemu čítaniu rádia a riadeniu serva
void loop() {
    crsf->update();
}

// --- CRSF CALLBACK (Volané z Loopu) ---
// Tu nesmú byť žiadne delaye ani Serial.printy!
void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels) {
    bool fs = rcChannels->failsafe;
    
    // 1. Aktualizácia globálnych stavov (pre druhý task)
    isFailsafeActive = fs;

    if (!fs) {
        // 2. Rýchle riadenie serva (Steering)
        int rawSteer = rcChannels->value[0];
        // Mapovanie z CRSF (cca 172-1811) na PWM (900-2200)
        int mappedSteer = map(rawSteer, 172, 1811, 900, 2200);
        
        steering.writeMicroseconds(mappedSteer);
        currentThrottlePWM = rcChannels->value[1];
        isLinkUp = true;
    } else {
        isLinkUp = false;
    }
}

// --- TASK NA POZADÍ (CORE 0) ---
// Rieši LEDky, Serial výpisy a komunikáciu so Slave
void TaskSlaveComms(void *pvParameters) {
    ControlPacket packetToSend;
    packetToSend.header = PACKET_HEADER; // 0xBEEF

    for (;;) {
        // A. OVLÁDANIE LED (Pomalé, preto je to tu)
        if (isFailsafeActive) {
            pixels.fill(pixels.Color(255, 0, 0)); // Červená - Failsafe
        } else {
            pixels.fill(pixels.Color(0, 255, 0)); // Zelená - OK
        }
        pixels.show();

        // B. PRÍPRAVA DÁT PRE SLAVE
        // Naplň štruktúru aktuálnymi dátami
        // Príklad: Pošleme Slave-u uhol serva (ak by ho ovládal on) 
        // alebo stav solenoidov (ktoré pridáš neskôr)
        Serial.print("Throttle PWM: ");
        Serial.println(currentThrottlePWM);
        packetToSend.throttle = currentThrottlePWM; 
        


        // C. ODOSLANIE CEZ UART
        packetToSend.checksum = calculateChecksum(&packetToSend);
        Serial2.write((uint8_t*)&packetToSend, sizeof(ControlPacket));

        // D. DEBUG VÝPIS (Voliteľné)
        // Serial.printf("Steer: %d | FS: %d\n", currentSteerPWM, isFailsafeActive);

        // Obnovovacia frekvencia pre Slave (napr. 50Hz = 20ms)
        vTaskDelay(20 / portTICK_PERIOD_MS); 
    }
}
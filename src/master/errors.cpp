#include <Arduino.h>
#include "errors.h"
#include "globals.h"
#include "functions.h"

// --- SIGNALIZÁCIA CHÝB BUZZEROM A LED ---

void signalError(ErrorCode code, bool critical) {
    Serial.printf("ERROR CODE: %d\n", code);
    
    // Červená LED - blikajúca
    for (int i = 0; i < 5; i++) {
        pixels.fill(pixels.Color(255, 0, 0));  // Červená
        pixels.show();
        delay(200);
        pixels.clear();
        pixels.show();
        delay(200);
    }
    
    // Beepy - počet = error code
    for (int i = 0; i < code; i++) {
        beep(200, 1000);  // Nízky zvuk pre chybu
        delay(300);
    }
    
    // Čakanie - aby sa dal troubleshoot
    delay(1000);
    
    if (critical) {
        // Kritická chyba - zastav program
        Serial.println("CRITICAL ERROR - HALTED!");
        while (1) {
            // Opakovane blikaj červenou
            pixels.fill(pixels.Color(255, 0, 0));
            pixels.show();
            delay(500);
            pixels.clear();
            pixels.show();
            delay(500);
        }
    }
}

void signalSuccess() {
    Serial.println("SETUP OK!");
    
    // Zelená LED - stála
    pixels.fill(pixels.Color(0, 255, 0));
    pixels.show();
    
    // 3x úspešný beep - vysoký tón
    for (int i = 0; i < 3; i++) {
        beep(150, 2500);  // Vysoký zvuk pre úspech
        delay(200);
    }
    
    delay(500);
}

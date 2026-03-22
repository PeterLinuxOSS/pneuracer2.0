#ifndef CLASS_H
#define CLASS_H

#include <Arduino.h>
#include "pins.h"

class SlavePowerManager {
public:
    void init() {
        disablePower();
    }

    void disablePower() {
        pinMode(SLAVE_RESET_BTN, OUTPUT);
        digitalWrite(SLAVE_RESET_BTN, LOW);
    }

    void enablePower() {
        pinMode(SLAVE_RESET_BTN, INPUT);
    }
};

class ServosPowerManager {
public:
    void init() {
        disablePower();
    }

    void disablePower() {
        pinMode(SERVO_BUFFER, INPUT);
        
    }

    void enablePower() {
        pinMode(SERVO_BUFFER, OUTPUT);
        digitalWrite(SERVO_BUFFER, LOW);
    }
};

#endif // CLASS_H
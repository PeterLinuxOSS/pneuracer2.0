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
        disableMotor();
        pinMode(SLAVE_RESET_BTN, OUTPUT);
        digitalWrite(SLAVE_RESET_BTN, LOW);
    }

    void enablePower() {
        pinMode(SLAVE_RESET_BTN, INPUT);
        enableMotor();
    }

    void disableMotor() { _motorEnabled = false; }
    void enableMotor()  { _motorEnabled = true;  }
    bool isMotorEnabled() const { return _motorEnabled; }

private:
    bool _motorEnabled = false;
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
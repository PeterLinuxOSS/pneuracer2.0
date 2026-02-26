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
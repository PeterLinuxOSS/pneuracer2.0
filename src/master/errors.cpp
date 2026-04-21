#include <Arduino.h>
#include "errors.h"
#include "globals.h"
#include "functions.h"

// --- ERROR SIGNALING WITH BUZZER AND LED ---

void signalError(ErrorCode code, bool critical) {
    Serial.printf("ERROR CODE: %d\n", code);
    errorActive = true;
    activeErrorCode = code;
    errorCritical = critical;

    if (code == ERR_BATTERY_LOW)
    {
        slavePower.disableMotor();
        haltIMU = true;
    }

    if (!beepRequestPending || beepRequestType != BEEP_ERROR)
    {
        requestBeepPattern(BEEP_ERROR, code > 0 ? code : 1, 1000, 200, 300);
    }
}

void signalSuccess() {
    Serial.println("SETUP OK!");
    pixels.fill(pixels.Color(0, 255, 0));
    pixels.show();

    if (!beepRequestPending)
    {
        requestBeepPattern(BEEP_SUCCESS, 3, 2500, 150, 200);
    }
}

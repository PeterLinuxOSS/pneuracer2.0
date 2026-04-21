#ifndef ERRORS_H
#define ERRORS_H

// --- ERROR CODES ---
enum ErrorCode {
    ERR_NONE = 0,           // No error
    ERR_I2C_INIT = 1,       // I2C initialization failed
    ERR_IMU_INIT = 2,       // IMU initialization failed
    ERR_CRSF_INIT = 3,      // CRSF initialization failed
    ERR_SERVO_INIT = 4,     // Servo initialization failed
    ERR_BATTERY_LOW = 5,    // Critical - low battery voltage
    ERR_TILT_WARNING = 6,   // Warning - abnormal tilt
    ERR_TILT_CRITICAL = 7,  // Critical - vehicle is tipping over!
};

// --- SIGNALING FUNCTIONS ---
void signalError(ErrorCode code, bool critical = true);
void signalSuccess();

#endif // ERRORS_H

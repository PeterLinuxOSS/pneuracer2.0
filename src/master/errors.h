#ifndef ERRORS_H
#define ERRORS_H

// --- ERROR KÓDY ---
enum ErrorCode {
    ERR_NONE = 0,           // Bez chyby
    ERR_I2C_INIT = 1,       // I2C inicializácia failed
    ERR_IMU_INIT = 2,       // IMU inicializácia failed
    ERR_CRSF_INIT = 3,      // CRSF inicializácia failed
    ERR_SERVO_INIT = 4,     // Servo inicializácia failed
    ERR_TILT_WARNING = 6,   // Varovanie - abnormálny tilt
    ERR_TILT_CRITICAL = 7,  // Kritické - vozidlo sa prevracuje!
};

// --- SIGNALIZAČNÉ FUNKCIE ---
void signalError(ErrorCode code, bool critical = true);
void signalSuccess();

#endif // ERRORS_H

#ifndef SLAVE_CONFIG_H
#define SLAVE_CONFIG_H

// ============================================================
//  SLAVE CONFIG — all tunable values in one place
// ============================================================

// --- MOTION / AS5600 ---
#define MOVING_SPEED_THRESHOLD   150.0f   // deg/s — below this the car is considered stationary

// --- VALVES / TIMING ---
#define DELAY_MAX_MS             2000     // max pause between valve switches (ms)
#define THROTTLE_DEADZONE        200      // throttle value below which motor is inactive

// --- SERVO (µs) ---
#define SERVO_GEAR1_US           1800     // gear 1 position,lubim ta
#define SERVO_NEUTRAL_US         1450     // neutral position
#define SERVO_GEAR3_US           1200     // gear 3 (highest) position

// --- AUTOMATIC MODE ---
#define AUTO_DEFAULT_SPEED       3000.0f  // fallback target speed when none received (deg/s)
#define AUTO_KP                  0.04f    // proportional gain for delay controller
#define AUTO_MAX_ADJUSTMENT      15       // max delay change per cycle (ms)
#define GEAR_UP_RATIO    0.85f   // upshift keď speed >= AS × 0.92
#define GEAR_DOWN_RATIO  0.55f   // downshift keď speed < AS × 0.77
#define GEAR_UP_SPEED            2200.0f  // upshift to gear 3 when speed >= this (deg/s)
#define GEAR_DOWN_SPEED          1600.0f   // downshift to gear 1 when speed < this (deg/s)
#define GEAR_SHIFT_DELAY_MS      500     // hysteresis time before gear change (ms)

// --- COMMUNICATION ---
#define CONNECTION_TIMEOUT_MS    500      // ms without packet before failsafe triggers

// --- LED ---
#define LED_BRIGHTNESS           50       // NeoPixel brightness (0–255)
#define BLINK_INTERVAL_MS        1000     // LED blink period in failsafe mode (ms)

#define MANUAL_MIN_SPEED         500
#define MANUAL_MAX_SPEED         6000


#endif // SLAVE_CONFIG_H

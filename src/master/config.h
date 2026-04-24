#ifndef MASTER_CONFIG_H
#define MASTER_CONFIG_H

// ============================================================
//  MASTER CONFIG — all tunable values in one place
// ============================================================

// --- BATTERY ---
#define BATTERY_VOLTAGE_MIN             9.0f    // minimum cell voltage for 3S (V)
#define BATTERY_VOLTAGE_MAX             12.6f   // full charge for 3S (V)
#define BATTERY_VOLTAGE_CRITICAL        9.0f    // trigger low-battery error below this (V)
#define BATTERY_DISCONNECTED_THRESHOLD  1.0f    // below this = battery unplugged (V)
#define BATTERY_DROP_THRESHOLD          2.0f    // rapid drop that indicates disconnection (V)
#define BATTERY_LOW_CONFIRM_COUNT       5       // consecutive low readings before error fires
#define BATTERY_LOW_IGNORE_MS           5000    // ignore low-battery for this long after rapid drop
#define BATTERY_READ_INTERVAL_MS        100     // ADC sampling interval in check_battery()
#define BATTERY_CHECK_TASK_INTERVAL_MS  500     // how often TaskSlaveComms calls check_battery()

// --- ADC ---
#define ADC_REF_VOLTAGE                 3.3f    // ESP32 ADC reference voltage
#define ADC_RESOLUTION                  4095    // 12-bit ADC range
#define IMON_CONVERSION_FACTOR          0.62f   // current sense: 10µA/A × 62kΩ

// --- STEERING SERVO ---
#define SERVO_ATTACH_MIN                600     // min PWM (µs)
#define SERVO_ATTACH_MAX                2400    // max PWM (µs)
#define SERVO_CENTER                    1400    // center position (µs)

// --- BRAKE SERVO ---
#define BRAKE_SERVO_CENTER_US           1500    // brake released (µs)
#define BRAKE_SERVO_PRESSED_US          1000    // brake fully engaged (µs)

// --- TILT DETECTION ---
#define TILT_WARNING_THRESHOLD          30.0f   // warning tilt angle (degrees)
#define TILT_CRITICAL_THRESHOLD         45.0f   // critical tilt angle (degrees)
#define TILT_ALERT_COOLDOWN_MS          1000    // min time between tilt alerts (ms)

// --- BRAKE LIGHTS ---
#define BRAKE_LIGHT_START               17      // first LED of the brake light group (same as KR_GROUP_START)

// --- KNIGHT RIDER BOOT / FAILSAFE ANIMATION ---
#define KR_GROUP_START                  17      // first physical LED of the grouped block
#define KR_VIRT_COUNT                   (KR_GROUP_START + 1)  // total virtual positions
#define KR_TAIL                         5       // number of fading tail LEDs
#define KR_STEP_MS                      45      // time per animation step (ms)
#define BOOT_ANIM_DURATION_MS           2000    // how long the boot animation plays (ms)

// --- LED EFFECT TIMING ---
#define ERROR_BLINK_INTERVAL_MS         500     // error state LED blink period (ms)
#define AUTO_PULSE_INTERVAL_MS          1000    // automatic mode pulse period (ms)
#define AUTO_PULSE_DURATION_MS          100     // automatic mode pulse on-time (ms)
#define BLUE_PULSE_INTERVAL_MS          1000    // IMU-active pulse period (ms)
#define BLUE_PULSE_DURATION_MS          100     // IMU-active pulse on-time (ms)
#define STROBE_PERIOD_MS                2000    // airplane strobe cycle period (ms)

// --- BEEPER ---
#define HALT_BEEP_FREQUENCY             1500    // IMU halt beep tone (Hz)
#define HALT_BEEP_DURATION_MS           150     // beep on-time (ms)
#define HALT_BEEP_INTERVAL_MS           1000    // time between beeps (ms)

// --- TASK TIMING ---
#define SLAVE_COMMS_LOOP_MS             20      // TaskSlaveComms loop period (~50 Hz)

#endif // MASTER_CONFIG_H

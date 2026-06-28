#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>
#include <CRSFforArduino.hpp>

// --- FORWARD FUNCTION DECLARATIONS ---

/// Functions for buzzer and audio signals
void beep(int duration_ms, int frequency = 2000);

/// IMU calibration and reading
void calibrate_imu();
void read_and_display_imu();

/// CRSF callback for RC channels
void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels);

/// Battery monitoring
void check_battery();

/// Request a beep sequence to be played by TaskSlaveComms
void requestBeepPattern(uint8_t type, uint8_t count, uint16_t frequency, uint16_t duration_ms, uint16_t gap_ms);

/// Task on background (CORE 0)
void TaskSlaveComms(void *pvParameters);

/// Knight Rider scanner animation (KITT effect)
bool knightRiderStep();
void setKnightRiderColor(uint8_t r, uint8_t g, uint8_t b);
void bootKnightRider(unsigned long durationMs, uint8_t r = 255, uint8_t g = 0, uint8_t b = 0);

#endif // FUNCTIONS_H

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>
#include <CRSFforArduino.hpp>

// --- FORWARD DEKLARÁCIE FUNKCIÍ ---

/// Funkcie pre buzzer a audio signály
void beep(int duration_ms, int frequency = 2000);

/// IMU kaliácia a čítanie
void calibrate_imu();
void read_and_display_imu();

/// Kontrola ventilu
void gear_change(bool value = false);

/// CRSF callback pre RC kanály
void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels);

/// Task na pozadí (CORE 0)
void TaskSlaveComms(void *pvParameters);

#endif // FUNCTIONS_H

#ifndef ELRS_H
#define ELRS_H

#include <Arduino.h>
#include "pins.h"

/**
 * @class ELRSManager
 * @brief Manages ExpressLRS receiver power and UART connection
 * 
 * Handles proper power and communication sequencing:
 * - Connect: Power ON -> Initialize UART
 * - Disconnect: Stop UART -> Power OFF (protects against phantom voltage)
 */
class ELRSManager {
private:
    bool isConnected;
    
    void setupPowerPin() {
        pinMode(ELRS_PWR_PIN, OUTPUT);
        digitalWrite(ELRS_PWR_PIN, LOW); // Start with power off
    }
    
public:
    ELRSManager() : isConnected(false) {}
    
    /**
     * Initialize ELRS manager (setup pins, but don't power on yet)
     */
    void init() {
        setupPowerPin();
    }
    
    /**
     * Connect: Power on -> Initialize UART
     * This ensures the receiver gets power before RX/TX signals
     */
    void connect() {
        if (isConnected) return;
        
        Serial.println("[ELRS] Powering up...");
        // Step 1: Enable power first
        digitalWrite(ELRS_PWR_PIN, HIGH);
        delay(500); // Wait for power stabilization
        
        Serial.println("[ELRS] Initializing UART...");
        // Step 2: Initialize UART (after power is stable)
        
        delay(100);
        
        isConnected = true;
        Serial.println("[ELRS] Connected");
    }
    
    /**
     * Disconnect: Stop UART -> Power off
     * This protects the receiver by removing TX/RX signals before power loss
     */
    void disconnect() {
        if (!isConnected) return;
        
        Serial.println("[ELRS] Stopping UART...");
        // Step 1: Stop UART first (before power loss)
        
        delay(100);
        
        Serial.println("[ELRS] Powering down...");
        pinMode(ELRS_RX_PIN, INPUT); // Set RX pin to input to avoid phantom voltage
        pinMode(ELRS_TX_PIN, INPUT); // Set RX pin to input to avoid phantom voltage
        // Step 2: Cut power (after UART is stopped)
        digitalWrite(ELRS_PWR_PIN, LOW);
        
        isConnected = false;
        Serial.println("[ELRS] Disconnected");
    }
    
    /**
     * Check if ELRS is currently connected
     */
    bool isActive() const {
        return isConnected;
    }
    
    /**
     * Get power pin state
     */
    bool isPowered() const {
        return digitalRead(ELRS_PWR_PIN);
    }
    
    /**
     * Soft reset: Disconnect then connect
     */
    void reset() {
        disconnect();
        delay(1000); // Wait for full discharge
        connect();
    }
};

#endif // ELRS_H

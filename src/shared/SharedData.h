#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <Arduino.h>


const uint16_t PACKET_HEADER = 0xBEEF;

struct __attribute__((packed)) ControlPacket {
    uint16_t header;       
    bool motorEnable;
    int16_t throttle; 
    bool elrsActive;
    int16_t button;
    bool haltIMU;
    bool brake;
    bool Automatic;
    int16_t AutomaticSpeed;
    int16_t Gear;
    bool launchControl;
    uint8_t checksum;

    
};

// Function to calculate checksum
inline uint8_t calculateChecksum(ControlPacket* pkt) {
    uint8_t sum = 0;
    uint8_t* data = (uint8_t*)pkt;

    for(size_t i = 0; i < sizeof(ControlPacket) - 1; i++) {
        sum += data[i];
    }
    return sum;
}

#endif
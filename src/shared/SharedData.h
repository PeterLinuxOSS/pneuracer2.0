#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <Arduino.h>


const uint16_t PACKET_HEADER = 0xBEEF;

struct __attribute__((packed)) ControlPacket {
    uint16_t header;       
    int16_t throttle; 
    bool elrsActive;
    int16_t button; 
    uint8_t checksum;   
    
};

// funkcia na výpočet checksumu
inline uint8_t calculateChecksum(ControlPacket* pkt) {
    uint8_t sum = 0;
    uint8_t* data = (uint8_t*)pkt;

    for(size_t i = 0; i < sizeof(ControlPacket) - 1; i++) {
        sum += data[i];
    }
    return sum;
}

#endif
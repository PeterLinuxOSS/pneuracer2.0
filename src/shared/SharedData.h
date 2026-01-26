#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <Arduino.h>

// Hlavička paketu (na synchronizáciu)
const uint16_t PACKET_HEADER = 0xBEEF;

// Štruktúra dát posielaná z Master -> Slave
// __attribute__((packed)) zabezpečí, že kompilátor nepridá medzery medzi byty
struct __attribute__((packed)) ControlPacket {
    uint16_t header;       // 0xBEEF
    int16_t throttle; // Uhly pre 4 servá (-90 až 90 alebo 0-180)
    uint8_t checksum;      // Kontrolný súčet
};

// Pomocná funkcia na výpočet checksumu
inline uint8_t calculateChecksum(ControlPacket* pkt) {
    uint8_t sum = 0;
    uint8_t* data = (uint8_t*)pkt;
    // Spočítame všetko okrem posledného bytu (kde je checksum)
    for(size_t i = 0; i < sizeof(ControlPacket) - 1; i++) {
        sum += data[i];
    }
    return sum;
}

#endif
🏎️ Pneuracer 2.0 — Firmware riadiaceho systému pneumatického vozidla
> **Projekt SOČ (Stredoškolská odborná činnosť)**  
> Tím **FLUJD** | SPŠ Poprad, Slovenská republika  
> Autor: Peter Rigo | 📧 rigopeter11@gmail.com 
---
📋 O projekte
Pneuracer 2.0 je firmware pre pneumaticky poháňaný model závodného vozidla, vyvinutý v rámci SOČ (Stredoškolská odborná činnosť). Systém využíva duálnu architektúru dvoch mikrokontrolérov ESP32-S3, ktoré komunikujú cez UART a spoločne riadia pohon, riadenie, detekciu naklonenia a diagnostiku vozidla.
Projekt je postavený na PlatformIO a kompiluje dva nezávislé firmvéry z jedného repozitára.
---
🏗️ Architektúra systému
```
[RC vysielač 2.4 GHz]
        │  ELRS/CRSF protokol
        ▼
┌──────────────────────────────┐              UART 921 600 baud               ┌─────────────────────────────────┐
│      ESP32-S3  MASTER        │ ◄───────────────────────────────────────────►│       ESP32-S3  SLAVE           │
│                              │          SharedData.h (ControlPacket)        │                                 │
│ • ELRS/CRSF prijímač         │          hlavička 0xBEEF + checksum          │ • Solenoidové ventily A/B       │
│ • IMU LSM6DS (tilt detect.)  │                                              │ • Vzduchové servo (prevodník)   │
│ • NeoPixel pás (29 LED)      │                                              │ • AS5600 magnetický enkodér     │
│ • Servá: riadenie, brzda,    │                                              │ • Hall senzory (start/end)      │
│ • IMU LSM6DSOX               │                                              │                                 │
│ • Bzučiak                    │                                              │ • NeoPixel LED (1 ks)           │
│ • ADC: batéria, prúd         │                                              │ • FreeRTOS TaskComms + Mutex    │
└──────────────────────────────┘                                              └─────────────────────────────────┘
```
---
📁 Štruktúra repozitára
```
pneuracer2.0/
├── platformio.ini              # Konfigurácia — 2 build environmenty
├── src/
│   ├── shared/
│   │   └── SharedData.h        # Zdieľaná štruktúra ControlPacket
│   ├── master/
│   │   ├── main.cpp            # Hlavná slučka (RC vstup, LED, servá, IMU)
│   │   ├── config.h            # Všetky laditeľné konštanty mastera
│   │   ├── pins.h              # Pin mapa mastera (SCH_MAIN_3)
│   │   ├── globals.h / .cpp    # Globálne premenné a objekty
│   │   ├── ELRS.h              # ELRS/CRSF manažér
│   │   ├── setups.h / .cpp     # Inicializácia periférií
│   │   ├── tilt_detection.h/.cpp  # Detekcia naklonenia cez LSM6DS
│   │   ├── errors.h / .cpp     # Správa chybových stavov
│   │   ├── functions.h         # Pomocné inline funkcie
│   │   └── class.h             # Definície tried
│   └── slave/
│       ├── main.cpp            # Hlavná slučka (ventily, enkodér, regulátor)
│       ├── config.h            # Všetky laditeľné konštanty slave
│       └── pins.h              # Pin mapa slave (SCH_MAIN_3)
└── .pio/                       # PlatformIO build cache (negitovať)
```
---
⚙️ Hardvérové komponenty
Komponent	Strona	Popis
ESP32-S3 (×2)	Master + Slave	`esp32-s3-devkitc-1`
LSM6DS	Master	IMU — detekcia naklonenia
LSM6DSOX	Slave	IMU — doplnkové meranie
AS5600	Slave	Magnetický enkodér (I2C) — uhlová rýchlosť °/s
Hall senzory	Slave	`HALL_START` pin 7, `HALL_END` pin 8, prah 2000
Solenoidové ventily	Slave	`VALVE_A` pin 6, `VALVE_B` pin 5
Vzduchové servo	Slave	`SERVO_AIR_REG` pin 4 — preraďovanie
ELRS prijímač	Master	CRSF, Serial2: RX=4, TX=5
NeoPixel pás	Master	29 LED, pin 3 — Knight Rider animácia pri boote
NeoPixel	Slave	1 LED, pin 34 — stavová signalizácia
Servo riadenie	Master	Pin 16
Servo brzda	Master	Pin 35
Bzučiak	Master	Pin 38, 1500 Hz
ADC batéria	Master	Pin 15, 3S LiPo (9–12.6 V)
ADC prúd	Master	Pin 12
---
🔌 Komunikačný protokol Master ↔ Slave
UART komunikácia beží na 921 600 baud. Každý paket má hlavičku `0xBEEF` a kontrolný súčet:
```cpp
// src/shared/SharedData.h
const uint16_t PACKET_HEADER = 0xBEEF;

struct __attribute__((packed)) ControlPacket {
    uint16_t header;         // 0xBEEF — identifikátor paketu
    bool     motorEnable;    // povolenie motora
    int16_t  throttle;       // plyn z RC
    bool     elrsActive;     // stav ELRS linky
    int16_t  button;         // stav tlačidla
    bool     haltIMU;        // pozastavenie IMU ochrany
    bool     brake;          // brzda
    bool     Automatic;      // automatický režim
    int16_t  AutomaticSpeed; // setpoint regulátora (°/s)
    int16_t  Gear;           // požadovaný prevodový stupeň
    uint8_t  checksum;       // kontrolný súčet (sumárny XOR)
};
```
Slave spracúva pakety vo FreeRTOS úlohe `TaskComms` s `dataMutex` semafórom — hlavná slučka nikdy nečíta dáta priamo bez zamknutia.
---
🤖 Režimy riadenia (Slave)
Manuálny režim
Hodnota `throttle` sa mapuje na dobu pauzy medzi prepínaním ventilov. Väčší plyn = kratšia pauza = vyšší výkon.
Automatický režim — P-regulátor
Slave meria uhlovú rýchlosť enkodérom AS5600 a porovnáva ju so setpointom `AutomaticSpeed`. Regulátor koriguje oneskorenie ventilov:
```cpp
float error      = automaticTargetSpeed - angular_speed;  // °/s
float adjustment = AUTO_KP * error;                       // KP = 0.04
currentDelayTarget -= (long)constrain(adjustment,
                                      -AUTO_MAX_ADJUSTMENT,
                                       AUTO_MAX_ADJUSTMENT);
```
Automatické preraďovanie
Podmienka	Akcia	Servo (µs)
rýchlosť ≥ 1600 °/s	3. stupeň	1200
rýchlosť < 1400 °/s	1. stupeň	1800
inak	Neutrál	1450
Hysterézna ochrana `GEAR_SHIFT_DELAY_MS = 250 ms` zabraňuje rýchlemu preraďovaniu.
---
🛠️ Zostavenie a nahranie firmvéru
Požiadavky
Visual Studio Code + PlatformIO rozšírenie
alebo PlatformIO CLI: `pip install platformio`
Postup
```bash
# 1. Klonuj repozitár
git clone https://github.com/PeterLinuxOSS/pneuracer2.0.git
cd pneuracer2.0

# 2. Nahraj firmvér do MASTER ESP32-S3
pio run -e master_esp --target upload

# 3. Nahraj firmvér do SLAVE ESP32-S3
pio run -e slave_esp --target upload

# 4. Monitorovanie (921600 baud)
pio device monitor
```
> **Porty:** Master hľadá `hwgrep://Standard COM Port`, Slave `hwgrep://Enhanced COM Port`.  
> Zmeň `upload_port` v `platformio.ini` podľa svojho systému ak je potrebné.
Knižnice (PlatformIO stiahne automaticky)
Knižnica	Verzia	Kde
`Adafruit NeoPixel`	^1.15.2	Master + Slave
`CRSFforArduino`	^2025.12.11	Master
`Adafruit LSM6DS`	^4.7.4	Master + Slave
`ESP32Servo`	^3.1.3	Master + Slave
`AS5600` (robtillaart)	^0.6.7	Master + Slave
`Arduino_LSM6DSOX`	^1.1.2	Slave
---
🔧 Ladenie parametrov
Všetky laditeľné hodnoty sú v `config.h` — žiadne "magic numbers" v kóde.
`src/slave/config.h` — regulátor a ventily:
```cpp
#define AUTO_KP               0.04f   // proporcionálny zosilňovač
#define AUTO_MAX_ADJUSTMENT   15      // max zmena oneskorenia za cyklus (ms)
#define AUTO_DEFAULT_SPEED    2000.0f // cieľová rýchlosť ak nepríde setpoint (°/s)
#define GEAR_UP_SPEED         1600.0f // preradenie nahor (°/s)
#define GEAR_DOWN_SPEED       1400.0f // preradenie nadol (°/s)
#define CONNECTION_TIMEOUT_MS 500     // failsafe pri výpadku komunikácie (ms)
```
`src/master/config.h` — batéria a bezpečnosť:
```cpp
#define TILT_WARNING_THRESHOLD   30.0f  // varovný uhol (°)
#define TILT_CRITICAL_THRESHOLD  45.0f  // kritický uhol (°)
#define BATTERY_VOLTAGE_MIN      9.0f   // min napätie 3S LiPo (V)
#define BATTERY_VOLTAGE_CRITICAL 9.0f   // spustí error stav (V)
```
---
⚡ Napájanie — PCB SCH_MAIN_3
Rail	Napätie	Účel
VBAT-RAW	9–12.6 V	3S LiPo vstup
+6V	6 V (LM22679TJ-ADJ)	Pohon, servá
+5V	5 V	Logika, senzory
+3.3V	3.3 V	ESP32-S3, I2C
> Transil **D9 (SMCJ15A)** na vetve VBAT-RAW chráni systém pred napäťovými špičkami.
---
👤 Autori
SPŠ Poprad, Slovensko
Peter Rigo — elektronika, firmware, riadiaci systém
📧 rigopeter11@gmail.com
---
📄 Licencia
Projekt bol vytvorený pre účely SOČ. Kód je zdieľaný pre vzdelávacie účely.

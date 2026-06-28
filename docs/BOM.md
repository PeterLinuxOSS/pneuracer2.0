# 📦 Bill of Materials (BOM) — Pneuracer 2.0

Parts list for the Pneuracer 2.0 vehicle. The build splits into two groups:

1. **Control electronics** — the custom PCB / control system (chosen by the team).
2. **Pneumatic & mechanical kit** — the SOČ competition *Starter KIT* (SMC pneumatics) plus mechanical parts.

> Order numbers are kept as-is for traceability. SMC parts can be looked up by their order code on the manufacturer's site. Always verify availability before ordering.

---

## ⚡ Control electronics

| Part | Order no. / type | Qty | Note |
|------|------------------|-----|------|
| MCU | **ESP32-S3** (`esp32-s3-devkitc-1`) | 2 | Master + Slave |
| LDO regulator | **LDL1117S33R** / LDI1117 (3.3 V) | — | logic supply |
| eFuse | **STEF12H60** (ST) | — | protected power switch |
| LED driver | **AL5809** (Diodes) | — | constant-current |
| N-MOSFET | **IAUCN04S7N019D** (Infineon) | — | power switching |
| USB ESD protection | **USBLC6-2SC6** | — | USB data-line protection |
| Electrolytic cap | **Panasonic EEE-FT1E102UP** (1000 µF) | — | servo / eFuse decoupling |
| Connector / part | **C109102** | — | see project notes |
| IMU | **LSM6DSO / LSM6DSOX** | 1–2 | tilt detection (Master), aux (Slave) |
| Magnetic encoder | **AS5600** | 1 | I²C, angular speed (Slave) |
| Addressable LEDs | **WS2812B** (NeoPixel) | 30 | strip 29 (Master) + 1 (Slave) |
| Hall sensors | — | 2 | start / end of stroke (Slave) |
| Buzzer | — | 1 | 1500 Hz (Master) |

See [README → References / Datasheets](../README.md#-references--datasheets) for the datasheets of these parts.

---

## 🛠️ Pneumatic & mechanical kit (SOČ Starter KIT — SMC)

| Part | Order no. | Note |
|------|-----------|------|
| Air tank — Tire booster 1 L | — | max fill pressure 0.7 MPa, fillable with a regular bike pump |
| Regulator | **ARJ210-M5G** | M5 thread |
| Pneumatic cylinder | **CD85N20-100-B** (+ variants CD85N16/20/25, strokes 100/110/125) | max 2 cylinders per team total |
| Valve 3/2 | **V114-VGU** | incl. seal + screws |
| Manifold base — 1 valve | **V100-74-1** | |
| Manifold base — 2 valves | **VV100-S41-02-M5** | incl. "P" port plug + push-in fittings |
| In-line throttle valve Ø6 | **AS1001FG-06** | |
| Tubing Ø6 | **TU0604BU-100** | |
| Push-in fitting, straight | **KQ2H06-M5A** / **KQ2L06-M5A** | |
| T-fitting Ø6 | **KQ2T06-00A** | |
| Elbow fitting Ø6 | **KQ2L06-00A** | |
| U-fitting Ø6 | **KQ2U06-00A** | |
| Quick coupling | **VHK3A-06F-06F** | |
| Servo | **MG996R** 180° | |
| Linear rail + carriage | **MGN7**, length 250 mm | dratek.cz |
| Filament | **AURAPOL PLA HT110** 1.75 mm (black) | max 1 kg per team |

---

## 🛒 Allowed e-shops (SOČ rules)

- Filament: <https://www.aurapol.com/cz/>
- Electronics: <https://dratek.cz/>
- Transmitters / controllers: <https://www.bighobby.cz/>
- Models / wheels / mechanical parts: <https://www.promodels.cz/>
- Other: <https://www.briol.cz>, <https://e-shop.exvalos.cz>

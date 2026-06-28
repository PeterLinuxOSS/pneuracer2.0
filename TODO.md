# 📝 TODO / Roadmap — Pneuracer 2.0

Improvements and known issues collected during development, intended for the **next PCB revision** and firmware iterations.

## 🔌 Hardware / PCB (next revision)

- [ ] Add the missing **electrolytic capacitors** for the servos — and review the eFuse that was damaged because they were missing.
- [ ] Hall sensor **decoupling capacitor**.
- [ ] **Transistor gain** stage for the solenoid drivers.
- [ ] **RC snubber** across the solenoids (flyback / spike suppression).
- [ ] Coil **pin 1** → route as input to the converter.
- [ ] Add **74HC125 buffer** for the servo signals.
- [ ] Let the ESP32 **detect whether it is powered from USB or battery**.
- [ ] Add **spare GPIO pins** broken out for future use.
- [ ] Add dedicated **debug pins**.

## 🧰 PCB layout guidelines (apply next time)

- Saturation **vias don't need to connect to the GND power plane** (currently a saturation via is tied to the supply/GND plane — fix).
- **Layer stack-up:** keep the signal layer as close to the GND plane as possible; mind dielectric thickness.
- **Vias:** don't place vias directly under a pad — offset them to the side where possible (the via sits slightly lower than the pad).
- **Crosstalk:** keep signal traces — especially analog — far apart, at least 3× the trace width.

## 🐞 Known issues

- Missing electrolytic capacitors for servos caused the **eFuse to fail**.
- Saturation vias are incorrectly connected to the GND power plane.

---

*Source: development notes (SOČ project, team FLUJD).*

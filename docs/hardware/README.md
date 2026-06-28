# 🔩 Hardware — PCB design files

EasyEDA Pro design files and manufacturing outputs for the Pneuracer 2.0 boards.

## Boards

| Folder | Board | Description |
|--------|-------|-------------|
| [`main-board/`](main-board/) | **Main board** (codename *Flujd*) | Carries the ESP32-S3 master/slave control electronics. |
| [`switching-regulator/`](switching-regulator/) | **Switching regulator** | Step-down power module — **2× placed on the main board**. |

## Files (per board)

| File | Description |
|------|-------------|
| `*.epro2` | EasyEDA Pro project (schematic + PCB) — open in [EasyEDA Pro](https://pro.easyeda.com) |
| `gerber.zip` | Gerber + drill files for manufacturing (e.g. JLCPCB) |
| `bom.xlsx` | Bill of materials for assembly |
| `pick-and-place.xlsx` | Component placement (CPL) for SMT assembly |
| `interactive-bom.html` | Interactive BOM — open in a browser for hand-assembly |

> System-level wiring schematics (draw.io) are in [`../schematics/`](../schematics/).
> Component-level BOM with order numbers is in [`../BOM.md`](../BOM.md).

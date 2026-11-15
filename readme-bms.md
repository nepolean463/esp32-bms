# ESP32 Smart BMS & Inverter Controller

[![ESP32](https://img.shields.io/badge/Hardware-ESP32--WROOM--32-blue.svg)](https://www.espressif.com/)
[![Battery](https://img.shields.io/badge/Battery-LiFePO4%2012.8V%204S%20100Ah-green.svg)](https://en.wikipedia.org/wiki/Lithium_iron_phosphate_battery)
[![BLE](https://img.shields.io/badge/BLE-NimBLE--Arduino-orange.svg)](https://github.com/h2zero/NimBLE-Arduino)
[![OTA](https://img.shields.io/badge/Updates-Web%20%26%20ArduinoOTA-brightgreen.svg)](#6-wireless-over-the-air-ota-firmware-updates)
[![License](https://img.shields.io/badge/License-MIT-purple.svg)](LICENSE)

An intelligent, self-contained **ESP32 controller and local Web Dashboard** for 12.8V 4S $\text{LiFePO}_4$ lithium battery packs equipped with the **SmarTEC / Power4S100A (SuperBMS)** Bluetooth Low Energy (BLE) Battery Management System.

The controller polls real-time battery pack diagnostics over BLE, serves a modern dark-mode Web Dashboard directly over local Wi-Fi, provides configurable relay protection with automatic hysteresis, and supports seamless **Over-The-Air (OTA) wireless updates** directly through your browser or the Arduino IDE.

---

## Table of Contents
- [1. Key Features](#1-key-features)
- [2. Hardware Pinout & Wiring](#2-hardware-pinout--wiring)
- [3. BLE Protocol & Frame Specification](#3-ble-protocol--frame-specification)
- [4. Automated Protection Logic & Hysteresis](#4-automated-protection-logic--hysteresis)
- [5. Getting Started (Arduino IDE)](#5-getting-started-arduino-ide)
- [6. Wireless Over-The-Air (OTA) Firmware Updates](#6-wireless-over-the-air-ota-firmware-updates)
- [7. REST API Documentation](#7-rest-api-documentation)
- [8. Repository File Structure](#8-repository-file-structure)

---

## 1. Key Features

* **Zero Cloud Dependency**: Operates 100% locally on your home Wi-Fi or as a standalone Wi-Fi Access Point (`ESP32-BMS-AP`). Accessible at `http://bms.local` via mDNS.
* **Low-Power BLE via NimBLE**: Uses the optimized `NimBLE-Arduino` library for fast connection, minimal SRAM footprint, and rock-solid reconnection resilience.
* **Rich Real-Time Web Telemetry**:
  * Pack Voltage, State of Charge (SOC %), and State of Health (SOH %).
  * Signed Current ($0.01\text{A}$ resolution) with directional badges: **Charging (+)**, **Discharging (-)**, and **Idle / Standby (0.0 A)** with deadband noise filtering.
  * Accurate Remaining Capacity ($99.9\text{ Ah}$) and Full Capacity ($100.0\text{ Ah}$).
  * Dynamic Cycle Life Counter and Pack Condition Status.
  * 4S Individual Cell Millivolt bars with min/max highlights and millivolt delta ($\Delta\text{mV}$) balance tracking.
  * 4-Channel Thermal Monitoring: Cell Temp Max, Cell Temp Min, Ambient Temp, and MOSFET Temp.
  * Charge and Discharge BMS MOSFET hardware switch states.
* **Smart Dual Relay Automation with Hysteresis**:
  * **Charger Relay (GPIO 18)**: Prevents overcharge by cutting AC/solar charging when cell or pack voltages exceed limits. Automatically resumes once voltages settle.
  * **Inverter Relay (GPIO 19)**: Prevents irreversible cell depletion by cutting inverter load when cells drop below cutoff limits. Automatically restores load when battery recharges.
* **Web UI Configurable Protection Thresholds**:
  * Modify cutoff and resume voltages directly from the dashboard form.
  * Stored permanently in ESP32 Flash memory (`Preferences.h` NVS) across power cuts and reboots.
* **Manual Override Respect**:
  * Manually switching a relay OFF keeps it OFF without the auto loop forcing it back on.
  * Automated resume activates strictly after genuine safety trips.
* **Dual OTA Updates**:
  * In-Browser firmware upload via drag-and-drop `.bin` file with live progress bar.
  * Arduino IDE Network Port flashing via `ArduinoOTA`.

---

## 2. Hardware Pinout & Wiring

| ESP32 Pin | Function | Peripheral Connected | Active Level | Default State |
| :--- | :--- | :--- | :--- | :--- |
| **GPIO 18** | Charger Relay | Inverter AC/Solar Charger control | Active LOW (`LOW` = ON) | `ON` (Closed) |
| **GPIO 19** | Inverter Relay | Inverter AC Load / Remote switch | Active LOW (`LOW` = ON) | `ON` (Closed) |
| **GPIO 0** | Factory Reset | Onboard ESP32 `BOOT` Button | Pull-up (Hold >5s to reset Wi-Fi) | Normal: HIGH |
| **VIN / GND** | Power Supply | 5V DC Supply or Step-down buck | 5V / GND | Power |

```
                +---------------------+
                |     ESP32 Board     |
                |                     |
                |  GPIO 18 ---------> | Relay 1 (IN1) --> AC Charger Line
                |  GPIO 19 ---------> | Relay 2 (IN2) --> Inverter Remote Switch
                |  GND -------------> | Relay GND
                |  VIN (5V) --------> | Relay VCC
                |                     |
                |  GPIO 0 (BOOT) <--- | Press >5s to reset saved Wi-Fi
                +---------------------+
```

---

## 3. BLE Protocol & Frame Specification

The SmarTEC / Power4S100A BMS communicates over standard Bluetooth Low Energy:
* **Advertised Name**: `Power4S100A`
* **Default Service UUID**: `0000ffe0-0000-1000-8000-00805f9b34fb` (Short alias `0xFFE0`)
* **Characteristic UUID**: `0000ffe1-0000-1000-8000-00805f9b34fb` (Short alias `0xFFE1`)
* **Properties**: `READ` | `WRITE_WITHOUT_RESPONSE` | `NOTIFY`

### 3.1 Polling Commands (ESP32 $\rightarrow$ BMS)
The ESP32 queries the BMS cyclically every second:

| Command | Frame Hex (8 Bytes) | Target |
| :--- | :--- | :--- |
| `CMD_CELLS` | `EA D1 01 04 FF 02 F9 F5` | Frame `0x02`: Individual cell voltages |
| `CMD_STATUS` | `EA D1 01 04 FF 03 F8 F5` | Frame `0x03`: Temperatures, Pack Current, Faults |
| `CMD_METRICS` | `EA D1 01 04 FF 04 FF F5` | Frame `0x04`: Cycle Count, SOC %, Capacities, Timers |

### 3.2 Frame Decoding (BMS $\rightarrow$ ESP32)

#### Frame `0x02` — Cell Voltages (19 Bytes)
* Bytes `[9..10]`: Cell 1 Voltage ($mV$, Big-Endian)
* Bytes `[11..12]`: Cell 2 Voltage ($mV$, Big-Endian)
* Bytes `[13..14]`: Cell 3 Voltage ($mV$, Big-Endian)
* Bytes `[15..16]`: Cell 4 Voltage ($mV$, Big-Endian)

#### Frame `0x03` — Temperatures & Pack Current (29 Bytes)
* Bytes `[7..8]`: Signed Pack Current ($0.01\text{A}$ resolution / centiamperes).
  * Positive values = Charging current ($+1000 = +10.00\text{A}$).
  * Two's complement negative values = Discharging current ($-1500 = -15.00\text{A}$).
* Byte `[13]`: Number of NTC thermal probes (typically 3).
* Byte `[14]`: Cell Temperature probe ($\text{Temp }^\circ\text{C} = \text{Raw} - 40$).
* Byte `[15]`: Ambient Temperature probe ($\text{Temp }^\circ\text{C} = \text{Raw} - 40$).
* Byte `[16]`: MOSFET Temperature probe ($\text{Temp }^\circ\text{C} = \text{Raw} - 40$).

#### Frame `0x04` — Pack Metrics & Capacities (57 Bytes)
* Byte `[7]` (Tag `0x01`): SOH % (State of Health).
* Bytes `[9..10]` (Tag `0x02`): Dynamic Cycle Life Counter.
* Byte `[13]` (Tag `0x03`): Discharge MOSFET state (`1` = Enabled, `0` = OFF).
* Byte `[19]` (Tag `0x05`): Charge MOSFET state (`1` = Enabled, `0` = OFF).
* Bytes `[12..13, 15..16]`: Full Capacity in $mAh$ ($\div 1000 = \text{Ah}$).
* Bytes `[18..19, 21..22]`: Remaining Capacity in $mAh$. Truncated to tenths digit matching SuperBMS app ($99960\text{ mAh} \rightarrow 99.9\text{ Ah}$).
* Bytes `[30..31]`: Remaining Discharge Time (minutes, `0xFFFF` = idle).
* Bytes `[33..34]`: Remaining Charge Time (minutes).
* Bytes `[36..37]`: Current Charge Interval (hours).
* Bytes `[38..39]`: Maximum Charge Interval (hours).

---

## 4. Automated Protection Logic & Hysteresis

The firmware employs asymmetric hysteresis protection to eliminate rapid relay cycling near voltage thresholds:

```
                  CHARGER OVERVOLTAGE HYSTERESIS
   Relay ON (Charging) ----[ Max Cell >= 3.65V OR Pack >= 14.40V ]----> Relay OFF (Cutoff)
          ^                                                                     |
          +----[ Max Cell <= 3.40V AND Pack <= 13.60V (Settled) ]---------------+

                  INVERTER UNDERVOLTAGE HYSTERESIS
   Relay ON (Powering) ----[ Min Cell <= 2.80V OR Pack <= 11.20V ]----> Relay OFF (Cutoff)
          ^                                                                     |
          +----[ Min Cell >= 3.10V AND Pack >= 12.50V (Recharged) ]-------------+
```

### Factory Default & Safe Boundary Ranges

| Parameter | Factory Default | Safe Bounds Window | Hysteresis Rule |
| :--- | :--- | :--- | :--- |
| **Cell Charge Cutoff** | $3650\text{ mV}$ ($3.65\text{V}$) | $3400 - 3700\text{ mV}$ | Must be $> \text{Resume } + 50\text{ mV}$ |
| **Cell Charge Resume** | $3400\text{ mV}$ ($3.40\text{V}$) | $3200 - 3650\text{ mV}$ | Must be $< \text{Cutoff } - 50\text{ mV}$ |
| **Pack Charge Cutoff** | $14.40\text{ V}$ | $13.60 - 14.80\text{ V}$ | Must be $> \text{Resume } + 0.20\text{ V}$ |
| **Pack Charge Resume** | $13.60\text{ V}$ | $13.00 - 14.60\text{ V}$ | Must be $< \text{Cutoff } - 0.20\text{ V}$ |
| **Cell Inverter Cutoff** | $2800\text{ mV}$ ($2.80\text{V}$) | $2400 - 3100\text{ mV}$ | Must be $< \text{Resume } - 50\text{ mV}$ |
| **Cell Inverter Resume** | $3100\text{ mV}$ ($3.10\text{V}$) | $2500 - 3400\text{ mV}$ | Must be $> \text{Cutoff } + 50\text{ mV}$ |
| **Pack Inverter Cutoff** | $11.20\text{ V}$ | $10.00 - 12.40\text{ V}$ | Must be $< \text{Resume } - 0.30\text{ V}$ |
| **Pack Inverter Resume** | $12.50\text{ V}$ | $10.50 - 13.40\text{ V}$ | Must be $> \text{Cutoff } + 0.30\text{ V}$ |

*All thresholds are dynamically configurable from the Web UI and saved to Flash NVS (`Preferences.h` under namespace `"bms_cfg"`).*

---

## 5. Getting Started (Arduino IDE)

### 5.1 Prerequisites
1. Install [Arduino IDE 2.x](https://www.arduino.cc/en/software).
2. Install the **esp32** board package (by Espressif) via **Tools → Board Manager**.
3. Install the **NimBLE-Arduino** library (by h2zero) via **Sketch → Include Library → Manage Libraries...**.

### 5.2 Board Configuration
In Arduino IDE, configure the following settings under the **Tools** menu:
* **Board**: `ESP32 Dev Module` (or `DOIT ESP32 DEVKIT V1`)
* **Partition Scheme**: `Minimal SPIFFS (Large APPS with OTA)` *(Required for wireless OTA updates)*
* **CPU Frequency**: `240MHz`
* **Flash Frequency**: `80MHz`
* **Upload Speed**: `921600`
* **Port**: Select the USB Serial Port (e.g., `COM4`)

### 5.3 First-Time USB Upload
1. Open [`Arduino_BMS_Web/Arduino_BMS_Web.ino`](Arduino_BMS_Web/Arduino_BMS_Web.ino).
2. If known, set your BMS Bluetooth MAC address in `#define BMS_MAC_ADDRESS "bc:f3:10:xx:xx:xx"`, or leave it blank to auto-scan by name (`Power4S100A`).
3. Click **Upload**.
4. Open the Serial Monitor at `115200 baud`. The ESP32 will output its assigned Wi-Fi IP address (e.g. `192.168.31.17`).
5. Open your browser to `http://bms.local` or the assigned IP address.

---

## 6. Wireless Over-The-Air (OTA) Firmware Updates

Once the initial sketch is flashed via USB, **you never need to physically reconnect the ESP32 to your computer again**.

### Method A: Web Browser Drag & Drop (Recommended)
1. In Arduino IDE, make your code modifications.
2. Click **Sketch → Export Compiled Binary** (or press `Ctrl + Alt + S`).
3. Arduino IDE will build and place a `.bin` file in your sketch directory.
4. Navigate to your Web Dashboard in Chrome/Edge at `http://bms.local` or your ESP32's IP address.
5. Scroll down to the **📡 Wireless Firmware Update (OTA)** card.
6. Select the exported `.bin` file and click **Flash Firmware**.
7. The progress bar displays live percentage. Upon completion, the ESP32 reboots and refreshes the dashboard automatically within 10 seconds.

### Method B: Arduino IDE Network Port
1. In Arduino IDE, open **Tools → Port**.
2. Under **Network Ports**, select `bms at <ESP32_IP>`.
3. Click the standard **Upload** arrow button to flash directly over Wi-Fi.

---

## 7. REST API Documentation

The ESP32 exposes a lightweight JSON REST API for third-party integrations (Home Assistant, Node-RED, scripts):

| Endpoint | Method | Description | Parameters / Payload |
| :--- | :--- | :--- | :--- |
| `/api/metrics` | `GET` | Returns full live BMS telemetry, cell voltages, temperatures, relay states, and active thresholds. | None |
| `/api/thresholds` | `GET` | Returns active protection cutoff and resume thresholds in JSON. | None |
| `/api/thresholds` | `POST` | Updates and persists protection thresholds to NVS memory. | `chg_cut_cell`, `chg_res_cell`, `chg_cut_pack`, `chg_res_pack`, `inv_cut_cell`, `inv_res_cell`, `inv_cut_pack`, `inv_res_pack` |
| `/api/relay` | `POST` | Toggles or sets relays and auto protection mode. | `target`: `charger` \| `inverter` \| `auto`<br>`action`: `toggle` \| `on` \| `off` |
| `/api/wifi` | `POST` | Saves new 2.4GHz Wi-Fi credentials and triggers reconnection. | `ssid`, `pass` |
| `/update` | `POST` | Accepts multi-part firmware binary upload for OTA flashing. | `update` (Binary stream) |

---

## 8. Repository File Structure

```text
.
├── Arduino_BMS_Web/
│   └── Arduino_BMS_Web.ino       # Main self-contained Arduino sketch (Firmware, Web UI & BLE)
├── BMS_SPECIFICATION_AND_GUIDE.md# Deep-dive binary frame decoding & protocol reference
├── BMS_COMMANDS_AND_METRICS.csv  # Complete byte-by-byte register map & formulas
├── partitions.csv                # Custom 4MB Dual-OTA Partition Table (1.9MB app0 / 1.9MB app1)
├── platformio.ini                # PlatformIO build configuration for VS Code users
├── include/                      # Modular C++ header files (PlatformIO build mode)
├── src/                          # Modular C++ source files (PlatformIO build mode)
├── .gitignore                    # Git exclusion rules for clean commits
└── README.md                     # Main documentation & user manual
```

---

## License

This project is licensed under the MIT License - see the LICENSE file for details.

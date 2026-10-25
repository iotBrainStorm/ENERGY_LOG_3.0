# ⚡ EnergyLog: Advanced Smart Power Monitor & Controller

**EnergyLog** is a robust, dual-ESP32 based smart energy monitoring and protection system. It provides real-time electrical telemetry (Voltage, Current, Power, Energy, Frequency, Power Factor), advanced grid protection mechanisms, Automatic Power Factor Correction (AFPC), and a comprehensive web-based dashboard.

The system utilizes a primary ESP32 for core logic, web hosting, and sensor data acquisition, paired with a secondary ESP32 dedicated to driving an external LCD and managing up to 16 capacitor banks for reactive power compensation.

---

## 📑 Table of Contents

1. [Key Features](#-key-features)
2. [System Architecture](#-system-architecture)
3. [Hardware Requirements](#-hardware-requirements)
4. [Pin Configuration](#-pin-configuration)
5. [Software Dependencies](#-software-dependencies)
6. [Installation & Setup](#-installation--setup)
7. [Usage & Interface](#-usage--interface)
8. [Automation & Protections](#-automation--protections)
9. [License](#-license)

---

## 🌟 Key Features

### 🔌 Real-Time Power Monitoring

- Integrates with the **PZEM-004Tv30** sensor for high-accuracy AC telemetry.
- Tracks Voltage (V), Current (A), Power (W), Energy (kWh), Frequency (Hz), and Power Factor (PF).
- Calculates exact billing days/uptime via accurate NTP synchronization.

### 🛡️ Advanced Safety Protections

- **Auto-Cutoff:** Safely cuts the main relay upon detecting High/Low Voltage, High Power Overload, or Frequency anomalies.
- **Recovery Delay:** Configurable countdown timer before restoring power after a grid fault clears.
- **kWh Saver:** Automatically cuts power when a user-defined energy quota (Max kWh) is reached.
- **Instant Alerts:** Triggers local buzzer and visual indicators immediately upon anomaly detection.

### ⚡ Automatic Power Factor Correction (AFPC)

- Intelligently calculates required Microfarads (MFD) to achieve a target Power Factor.
- Drives up to **16 sequential capacitor banks** via a 74HC595 shift register.
- Includes dead-band hysteresis and low-wattage cutoffs to prevent relay chattering.

### 🌐 Comprehensive Web Dashboard & Connectivity

- **Asynchronous Web Server:** Fast, non-blocking UI served from SPIFFS (supports GZIP).
- **Security:** Role-based authentication (Admin/User) with a 3-attempt brute-force lockout.
- **Automation:** Daily scheduling (by days of the week), Future date scheduling, and Quick Countdown timers.
- **Integrations:** Built-in **mDNS** (`energylog.local`) and direct HTTP POST streaming to **Node-RED/InfluxDB**.

### 🎛️ Dual Local Interfaces

- **Primary Unit:** ST7920 128x64 graphical OLED/LCD with a Rotary Encoder for deep menu navigation without a PC.
- **Secondary Unit:** 16x2 I2C LCD for glanceable telemetry, accompanied by proportional Watt-pulse and Protection LEDs.

---

## 🏗️ System Architecture

The project is split across two microcontrollers communicating via a dedicated UART serial link.

1. **Main ESP32 (Controller):** Handles WiFi, Web UI, NTP Time, PZEM sensor reading, primary relay control, OLED display, and the Rotary Encoder menu.
2. **Secondary ESP32 (Display/AFPC Driver):** Receives telemetry JSON via serial. It handles the 16x2 LCD UI, visual LED alerts, and translates PF data into shift-register outputs (74HC595) to toggle physical AFPC relays.

---

## 🛠️ Hardware Requirements

- 2x **ESP32** Development Boards (e.g., NodeMCU-32S)
- 1x **PZEM-004Tv30** AC Energy Measurement Module
- 1x **ST7920 128x64** Display (Primary UI)
- 1x **16x2 I2C LCD** (Secondary UI)
- 1x **Rotary Encoder** (with push-button)
- 2x **74HC595** Shift Register IC (For 16 chanel Relay Module)
- Relay Modules (2 chanel for main output and PZEM control, 16 chanel for AFPC)
- Active Buzzer, Push buttons, and LEDs

---

## 📌 Pin Configuration

### Primary ESP32 (Main Node)

| Component           | ESP32 Pin           | Description                                |
| :------------------ | :------------------ | :----------------------------------------- |
| **Main Relay**      | `GPIO 19`           | Active LOW (Controls main load)            |
| **PZEM Relay**      | `GPIO 21`           | Active LOW (Powers PZEM sensor)            |
| **Buzzer**          | `GPIO 13`           | Active HIGH                                |
| **Reset Button**    | `GPIO 35`           | External push button (Active HIGH)         |
| **Encoder CLK**     | `GPIO 32`           | Rotary Encoder Clock                       |
| **Encoder DT**      | `GPIO 33`           | Rotary Encoder Data                        |
| **Encoder SW**      | `GPIO 34`           | Rotary Encoder Button (Active LOW)         |
| **PZEM-004T**       | `TX: 16` / `RX: 17` | Connect to PZEM RX / TX                    |
| **ESP Serial Link** | `TX: 25` / `RX: 26` | Connect to Secondary ESP32 RX/TX           |
| **ST7920 Display**  | Multiple            | `E=18`, `RW=23`, `RS=5`, `RST=22` (SW SPI) |

### Secondary ESP32 (AFPC Node)

| Component           | ESP32 Pin           | Description                                 |
| :------------------ | :------------------ | :------------------------------------------ |
| **ESP Serial Link** | `TX: 25` / `RX: 26` | Connect to Primary ESP32 RX/TX              |
| **16x2 LCD**        | `I2C Pins`          | Standard SDA/SCL (Address `0x27`)           |
| **Watt LED**        | `GPIO 17`           | Pulses proportional to power usage          |
| **Process LED**     | `GPIO 18`           | Blinks on serial packet receipt             |
| **Protec LED**      | `GPIO 19`           | High when active protection tripped         |
| **74HC595 Data**    | `GPIO 14`           | Shift Register Serial Data (DS)             |
| **74HC595 Clock**   | `GPIO 27`           | Shift Register Clock (SHCP)                 |
| **74HC595 Latch**   | `GPIO 16`           | Shift Register Latch (STCP)                 |
| **74HC595 OE**      | `GPIO 4`            | Output Enable (Active LOW, use 10k pull-up) |

---

## 💻 Software Dependencies

To compile this project, install the following libraries in your Arduino IDE or PlatformIO environment:

- [WiFiManager](https://github.com/tzapu/WiFiManager) (Handles initial WiFi AP setup)
- [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer) (Web Dashboard)
- [ArduinoJson](https://arduinojson.org/) (Data formatting & Serial link)
- [RotaryEncoder](https://github.com/mathertel/RotaryEncoder) (Menu navigation)
- [U8g2](https://github.com/olikraus/u8g2_Arduino) (Primary OLED/LCD graphics)
- [PZEM004Tv30](https://github.com/mandulaj/PZEM-004T-v30) (Sensor communication)
- [LiquidCrystal_I2C](https://github.com/johnrickman/LiquidCrystal_I2C) (Secondary LCD)

---

## 🚀 Installation & Setup

1. **Flash the Primary ESP32:**
   - Open the Main `.ino` file.
   - **Crucial Step:** You _must_ upload the `index.html` and `config.html` files to the ESP32's SPIFFS storage. Use the "ESP32 Sketch Data Upload" tool in Arduino IDE, or PlatformIO's `Upload File System image` task.
   - Partition Scheme: Huge APP (3MB No OTA/1MB SPIFFS).
   - Compile and upload the sketch.

2. **Flash the Secondary ESP32:**
   - Open the Secondary `.ino` file.
   - Partition Scheme: Huge APP (3MB No OTA/1MB SPIFFS).
   - Compile and upload the sketch
3. **Initial Boot (Captive Portal):**
   - On first boot, the Main ESP32 will host a WiFi Access Point named **`EnergyLog`**.
   - Connect to this network. A captive portal will appear (or navigate to `192.168.4.1`).
   - Enter your local WiFi credentials. The ESP32 will reboot and connect to your network.

4. **Accessing the Dashboard:**
   - By default, navigate to **`http://energylog.local`** or **`http://energylog-XXXX.local`** in your browser (where XXXX is last 4 chars of MAC).
   - Default Admin Login:
     - **Username:** `esp`
     - **Password:** `456456`

---

## ⚙️ Usage & Interface

### LCD Menu (Rotary Encoder)

Click the rotary encoder to enter the menu. You can configure almost all parameters offline, including:

- WiFi Disconnect/Forget
- mDNS and Node-RED configuration
- Adjusting Protections thresholds (V, W, Hz)
- AFPC Target PF and Capacitor sizing
- Factory Resets and Clear Data

### Hardware Boot Button Sequences

The physical `BOOT` button on the primary ESP32 has built-in safety overrides:

- **Resetting Users:** Triggered via the Web UI, requires holding the BOOT button for 15 seconds.
- **Factory Reset:** Can be initiated from the UI or OLED menu, verified by a physical 15-second hold.

---

## ⚖️ License

**MIT License**

Copyright (c) 2024

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

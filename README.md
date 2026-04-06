<div align="center">

# ⚡ ENERGY LOG 2.0

### Advanced ESP32-Based Energy Monitoring System

[![ESP32](https://img.shields.io/badge/ESP32-Dual%20Core-blue.svg)](https://www.espressif.com/en/products/socs/esp32)
[![Arduino](https://img.shields.io/badge/Arduino-IDE-00979D.svg)](https://www.arduino.cc/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Version](https://img.shields.io/badge/Version-2.0-orange.svg)](https://github.com/yourusername/ENERGY_LOG_2.0)

_Real-time power monitoring • Firebase Integration • Web Dashboard • OLED Display_

[Features](#-features) • [Hardware](#%EF%B8%8F-hardware-requirements) • [Installation](#-installation) • [Usage](#-usage) • [Screenshots](#-screenshots) • [Contributing](#-contributing)

</div>

---

## 🌟 Overview

**ENERGY LOG 2.0** is a professional-grade energy monitoring solution built on the ESP32 platform. It combines precision measurement hardware with an intuitive web interface and real-time data visualization, making power consumption tracking accessible and actionable.

### Why ENERGY LOG 2.0?

- 📊 **Real-time Monitoring**: Track voltage, current, power, energy, frequency, and power factor
- 🌐 **Web Dashboard**: Beautiful, responsive interface accessible from any device
- 🔥 **Firebase Integration**: Cloud data logging with real-time synchronization
- 🖥️ **OLED Display**: Local monitoring with rotary encoder navigation
- 🔒 **Safety Features**: Overvoltage/undervoltage/overpower protection with configurable delays
- 📡 **Dual ESP Communication**: Main controller + secondary ESP for extended functionality
- 💾 **Data Persistence**: EEPROM storage for settings and cumulative energy tracking
- 🎯 **kWh Saver**: Automatic load control based on energy consumption limits

---

## ✨ Features

### 📈 Monitoring Capabilities

- ⚡ **Voltage** (V) - Real-time AC voltage measurement
- 🔌 **Current** (A) - Precise current sensing up to 100A
- 💡 **Power** (W) - Active power calculation
- 🔋 **Energy** (kWh) - Cumulative energy consumption tracking
- 📊 **Power Factor** (PF) - Reactive power analysis
- 〰️ **Frequency** (Hz) - Mains frequency monitoring
- ⏱️ **Uptime** - System runtime tracking
- 📅 **Total Days** - Long-term operation logging

### 🛡️ Protection & Safety

- ⚠️ **High/Low Voltage Protection** - Automatic relay cutoff
- ⚠️ **Overpower Protection** - Load limiting
- ⚠️ **Frequency Deviation Protection** - Mains stability monitoring
- 🔔 **Configurable Alerts** - Buzzer + visual notifications
- ⏲️ **Protection Delay Timer** - Prevents nuisance tripping
- 🔄 **Manual Override** - Emergency control via push button

### 🌐 Connectivity & Integration

- 📱 **WiFi Manager** - Easy network configuration via captive portal
- 🔥 **Firebase Realtime Database** - Cloud data logging and remote control
- 🌊 **Node-RED Integration** - IoT platform compatibility
- 🔗 **RESTful API** - JSON endpoints for external systems
- 📡 **ESP-to-ESP Communication** - UART data sharing

### 🖥️ User Interfaces

- **Web Dashboard**

  - Real-time data visualization with animated charts
  - Responsive design (mobile/tablet/desktop)
  - Firebase configuration panel
  - Main output control with confirmation dialogs
  - Live status indicators (WiFi, Node-RED, Firebase, Protection, Alerts)

- **OLED Display (128×64)**
  - Multi-screen layout with live readings
  - Rotary encoder navigation
  - Interactive menu system
  - Date/time display (NTP synchronized)
  - Status bar with connectivity icons

### ⚙️ Advanced Settings

- 🎛️ **AFPC (Automatic Factor Power Correction)** - Smart load management
- 🔋 **kWh Saver Mode** - Auto-shutoff at energy limit
- 🔊 **Buzzer Control** - Configurable audio alerts
- ⏰ **Startup Delay** - Safety check before relay activation
- 📊 **Adjustable Thresholds** - Custom voltage/power/frequency limits
- ⏱️ **Configurable Intervals** - PZEM reading, Node-RED, Firebase, alerts

---

## 🛠️ Hardware Requirements

### Core Components

| Component             | Specification             | Purpose             |
| --------------------- | ------------------------- | ------------------- |
| **ESP32 DevKit**      | Dual-core 240MHz, WiFi/BT | Main controller     |
| **PZEM-004T v3.0**    | AC 80-260V, 0-100A        | Energy meter sensor |
| **U8G2 OLED Display** | 128×64, I2C               | Local UI            |
| **Rotary Encoder**    | KY-040 or similar         | Menu navigation     |
| **Relay Module**      | 5V, 10A+ (active-low)     | Load control        |
| **Buzzer**            | 5V active                 | Audio alerts        |
| **Push Button**       | Normally open             | Emergency override  |

### Pin Configuration

```cpp
// GPIO Assignments
#define MAIN_RELAY      26  // Main output relay (active-low)
#define PZEM_RELAY      27  // PZEM sensor power control
#define RESET_BTN       14  // Protection reset button
#define ENCODER_SW      33  // Rotary encoder switch
#define BUZZER_PIN      25  // Alert buzzer
#define ESP_COMM_RX     13  // Secondary ESP RX
#define ESP_COMM_TX     12  // Secondary ESP TX

// I2C (OLED)
#define SDA_PIN         22
#define SCL_PIN         23

// PZEM-004T (UART)
#define PZEM_RX         16
#define PZEM_TX         17
```

### Power Supply

- **ESP32**: 5V/1A USB or regulated DC
- **PZEM-004T**: Powered from monitored AC line
- **Relay**: 5V from ESP32 (ensure adequate current rating)

---

## 📦 Installation

### 1️⃣ Arduino IDE Setup

```bash
# Install ESP32 board support
# File → Preferences → Additional Board Manager URLs:
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

# Tools → Board → Boards Manager → Search "ESP32" → Install
```

### 2️⃣ Library Dependencies

Install via Arduino Library Manager:

```
WiFiManager      (tzapu)
ESPAsyncWebServer (me-no-dev)
ArduinoJson      (Benoit Blanchon)
U8g2             (olikraus)
PZEM004Tv30      (mandulaj)
RotaryEncoder    (mathertel)
```

### 3️⃣ File System Setup

```bash
# Install SPIFFS plugin for Arduino IDE
# Upload web files to ESP32:
# Tools → ESP32 Sketch Data Upload

# Required files in /data folder:
├── index.html       # Main dashboard
├── style.css        # Styling
├── firebase.html    # Firebase configuration page
├── plug_icon.svg    # Favicon
└── table.svg        # UI assets
```

### 4️⃣ Upload Firmware

```bash
# 1. Connect ESP32 via USB
# 2. Select board: ESP32 Dev Module
# 3. Upload speed: 921600
# 4. Flash size: 4MB (3MB APP / 1MB SPIFFS)
# 5. Upload code
```

---

## 🚀 Usage

### Initial Setup

1. **Power On**: ESP32 will start WiFi Manager if no credentials saved
2. **Connect**: Join WiFi network `ESP32-ENERGY-MONITOR`
3. **Configure**: Browser opens automatically → Select your WiFi → Enter password
4. **Access**: Note IP address shown on OLED display

### Web Dashboard

```
http://<ESP32-IP-ADDRESS>/
```

- View real-time measurements
- Configure Firebase integration
- Control main output relay
- Monitor system status

### Menu Navigation (OLED)

```
Push Button → Enter Menu
Rotate Encoder → Navigate
Push Encoder → Select/Confirm
```

**Menu Structure**:

```
Main Menu
├── Settings
│   ├── WiFi Setup
│   ├── Node-RED Config
│   ├── Buzzer Control
│   ├── Startup Delay
│   └── Intervals
├── Protection
│   ├── Voltage Limits
│   ├── Power Limits
│   ├── Frequency Limits
│   └── Protection Delay
├── Alerts
│   ├── Voltage Alerts
│   ├── Power Alerts
│   └── Frequency Alerts
├── Main Output Control
│   ├── Enable/Disable
│   └── kWh Saver
└── AFPC Settings
```

### Firebase Integration

1. Navigate to `http://<ESP32-IP>/firebase`
2. Enter Firebase Realtime Database URL
3. Paste authentication token
4. Set data upload interval
5. Test connection → Enable

**Firebase Structure**:

```json
{
  "energy_data": {
    "voltage": 230.5,
    "current": 1.25,
    "power": 287.5,
    "energy": 12.34,
    "frequency": 50.02,
    "pf": 0.98,
    "uptime": 145.2,
    "totalDays": 3.5,
    "output": 1,
    "timestamp": 1234567890
  }
}
```

---

## 📸 Screenshots

### Web Dashboard

![Dashboard](docs/images/dashboard.png)
_Real-time monitoring with live graphs and status indicators_

### OLED Display

![OLED](docs/images/oled_display.jpg)
_Local interface with rotary encoder control_

### Firebase Console

![Firebase](docs/images/firebase_integration.png)
_Cloud data logging and remote monitoring_

---

## 🔧 Configuration

### EEPROM Memory Map

```cpp
// Address allocation (512 bytes total)
#define SETTINGS_FLAG_ADDR      0    // Validation flag
#define SETTINGS_VERSION_ADDR   1    // Version tracking
#define SETTINGS_DATA_ADDR      2    // Main settings struct
#define FB_EEPROM_ADDR          256  // Firebase settings struct
```

### Default Settings

```cpp
// Safety Thresholds
Voltage High:  250.0 V
Voltage Low:   150.0 V
Power High:    5000 W
Frequency High: 51.0 Hz
Frequency Low:  49.0 Hz

// Timers
Startup Delay:       60 seconds
Protection Delay:    1200 seconds (20 min)
PZEM Read Interval:  1 second
Node-RED Interval:   30 seconds
Firebase Interval:   10 seconds
Alert Interval:      180 seconds
```

---

## 🤝 Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit changes (`git commit -m 'Add AmazingFeature'`)
4. Push to branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

### Development Guidelines

- Follow existing code style
- Comment complex logic
- Test on hardware before submitting
- Update documentation for new features

---

## 📝 Changelog

### Version 2.0 (Current)

- ✅ Dual-core architecture (Core 0: Firebase, Core 1: Main tasks)
- ✅ Firebase bidirectional sync (read/write output state)
- ✅ Web-based output control with confirmation dialog
- ✅ Auto-reconnect WiFi after network dropout
- ✅ Display mutex protection (prevents garbage display)
- ✅ Yielding delays (eliminates watchdog resets)
- ✅ Optimized EEPROM operations (256 bytes, error handling)
- ✅ Enhanced status bar (WiFi, Node-RED, Firebase, Protection, Alerts)
- ✅ kWh Saver mode with configurable limit

### Version 1.0

- Initial release with basic monitoring
- OLED display with rotary encoder
- WiFi Manager integration
- PZEM-004T sensor support

---

## 🐛 Troubleshooting

### ESP32 Restarts After "Starting..."

- **Cause**: Watchdog timer timeout during display operations
- **Fix**: Firmware now uses yielding delays and display mutex

### "Loading Settings..." Hangs

- **Cause**: EEPROM corruption or size mismatch
- **Fix**: Flash with `EEPROM.begin(256)` and factory reset code

### Firebase Not Syncing

- **Cause**: Invalid URL or authentication token
- **Fix**: Verify Firebase URL format: `https://yourproject.firebaseio.com`

### OLED Shows Garbage Values

- **Cause**: Buffer conflicts from multiple threads
- **Fix**: All display functions now protected with `displayMux`

### Relay Not Responding

- **Cause**: Active-low relay wiring or incorrect pin
- **Fix**: Verify `LOW = ON`, `HIGH = OFF` logic and GPIO 26 connection

---

## 📄 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

---

## 👨‍💻 Author

**M. Maity**  
_Embedded Systems Developer_

- GitHub: [@yourusername](https://github.com/yourusername)
- Email: your.email@example.com

---

## 🙏 Acknowledgments

- [PZEM-004T Library](https://github.com/mandulaj/PZEM-004T-v30) by mandulaj
- [U8g2 Library](https://github.com/olikraus/u8g2) by olikraus
- [WiFiManager](https://github.com/tzapu/WiFiManager) by tzapu
- [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer) by me-no-dev
- ESP32 Community for extensive documentation

---

## ⭐ Star History

[![Star History Chart](https://api.star-history.com/svg?repos=yourusername/ENERGY_LOG_2.0&type=Date)](https://star-history.com/#yourusername/ENERGY_LOG_2.0&Date)

---

<div align="center">

### 📬 Questions or Issues?

Open an [Issue](https://github.com/yourusername/ENERGY_LOG_2.0/issues) or start a [Discussion](https://github.com/yourusername/ENERGY_LOG_2.0/discussions)

**Made with ❤️ and ESP32**

</div>

// Library Includes
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>  // Node-Red
#include <Wire.h>        // I2C communication
#include <SPI.h>
#include <EEPROM.h>          // Settings storage
#include <SPIFFS.h>          // File system
#include <HardwareSerial.h>  // Serial communication
#include <RotaryEncoder.h>   // Encoder handling
#include "time.h"            // Time management
#include <U8g2lib.h>         // OLED display
#include <PZEM004Tv30.h>     // Power meter
#include <ArduinoJson.h>     // make json format
#include <Preferences.h>
#include <FirebaseESP8266.h>  // For cloud share


// Pin Definitions
// -- Control Pins
#define MAIN_RELAY 19  // add 10k pull up resistance to avoid flickering issue at startup
#define PZEM_RELAY 21  // add 10k pull up resistance to avoid flickering issue at startup
#define BUZZER_PIN 13
#define RESET_BTN 35

// -- Web Server
AsyncWebServer server(80);
bool webServerStarted = false;

// -- Display Refresh time
#define DISPLAY_REFRESH_INTERVAL 1000  // ms

// -- Encoder Pins
#define ENCODER_CLK 32
#define ENCODER_DT 33
#define ENCODER_SW 34

// Settings struct
#define SETTINGS_VALID_FLAG 0xAB  // Validation flag value
#define SETTINGS_VERSION 1        // Increment this when you change Settings struct
#define SETTINGS_FLAG_ADDR 0
#define SETTINGS_VERSION_ADDR 1
#define SETTINGS_DATA_ADDR 4

// -- Communication Pins
#define ESP_COMM_TX 25   // Connected to RX (26) of secondary ESP32
#define ESP_COMM_RX 26   // Connected to TX (25) of secondary ESP32
#define PZEM_COMM_TX 16  // Connected to RX of PZEM
#define PZEM_COMM_RX 17  // Connected to TX of PZEM

// Hardware Initialization
HardwareSerial espSerial(1);  // Use Serial1 for ESP-to-ESP communication
PZEM004Tv30 pzem(Serial2, PZEM_COMM_TX, PZEM_COMM_RX);


// -- Second Core
TaskHandle_t firebaseTask;

// -- Display
// U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, 18, 23, 5, 22);  //E=18, RW=23, RS=5, RST=22, PSB=GND

// -- Welcome Message
const char MSG_WELCOME[] PROGMEM = "ESP";
const char MSG_SUBTITLE[] PROGMEM = "ENERGY - LOG";
const char MSG_DEVELOPER[] PROGMEM = "developed by M.Maity";

// -- Time Management
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 19800;
const int daylightOffset_sec = 0;

// -- Days Tracking
Preferences preferences;
float kwhDays = 0.0;
const unsigned long TOTAL_DAYS_INTERVAL = 10UL * 60UL * 1000UL;  // 10 minutes in ms
bool resetDaysRequested = false;

unsigned long lastTotalDaysUpdate = 0;
unsigned long deviceStartMillis = 0;

// -- Encoder
RotaryEncoder encoder(ENCODER_CLK, ENCODER_DT);
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile int encoderValue = 0;
long lastEncoderPos = 0;

// Global Variables
// -- Menu State
int menuLevel = 0;
int menuIndex[3] = { 0, 0, 0 };
bool editing = false;
bool menuActive = false;

// -- PZEM Measurements
struct Measurements {
  float voltage;
  float current;
  float power;
  float energy;
  float frequency;
  float pf;
  float uptime;
  float totalDays;

  // Constructor with default values
  Measurements()
    : voltage(000.0),
      current(0.00),
      power(0.0),
      energy(0.00),
      frequency(00.00),
      pf(0.00),
      uptime(0.0),
      totalDays(0.0) {}
};
portMUX_TYPE measureMux = portMUX_INITIALIZER_UNLOCKED;
Measurements readings;

// -- PZEM Read
bool resetKwhRequested = false;


// -- Node-Red
bool nodeRedConnected = false;
String lastNodeRedResponse = "";

// -- Safety Alert Check
const int ALERT_BEEP_DURATION = 100;  // Beep duration in ms
const int ALERT_BEEP_PAUSE = 100;     // Pause between beeps

// -- Safety protection
bool shouldTurnOff = false;
static bool protectionWasActive = false;
static unsigned long protectionClearedTime = 0;
unsigned long PROTECTION_DELAY = 12e5;  // 20 minutes in milliseconds

// -- ESP Data Share
unsigned long lastSendEspUpdate = 0;
const unsigned long SEND_ESP_INTERVAL = 1500;

// -- WiFi Status
uint8_t wifiRSSI = 0;
String ssid = "";

// Settings Structure
struct __attribute__((packed)) Settings {
  // Communication Settings
  bool nodeRedEnabled;
  char nodeRedIP[32];

  // Protection Settings
  bool highVoltageAlert, lowVoltageAlert;
  bool highPowerAlert, lowPowerAlert;
  bool highFreqAlert, lowFreqAlert;
  bool highVProtec, LowVProtec;
  bool highPProtec;
  bool highHzProtec, lowHzProtec;

  // Feature Settings
  bool afpcEnabled;
  bool mainOutput, kWhSaver;
  bool buzzerEnabled;

  // Threshold Values
  float voltHigh, voltLow;
  float powerHigh, powerLow;
  float freqHigh, freqLow;
  float targetFactor;

  // System Settings
  int startupTime;
  float mfdPerChannel;
  int maxkWh;
  float totalDays;

  // AFPC thresholds
  int afpcLowWatt;
  int afpcHighWatt;

  // Update intervals (in milliseconds)
  int espShareInterval;
  int nodeRedInterval;
  int pzemInterval;
  int alertInterval;
  int protectionDelay;
};

Settings settings;
const int EEPROM_ADDR = 0;

// Firebase Settings structure
struct __attribute__((packed)) FirebaseSettings {
  bool enabled;
  char host[128];     // Firebase project URL
  char auth[128];     // Database secret/auth token
  int dataInterval;   // Data share interval in seconds
  uint8_t validFlag;  // Validation flag
};

FirebaseSettings fbSettings;
const int FB_EEPROM_ADDR = sizeof(Settings) + 10;  // Offset after main settings

// Firebase status variables
FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;
String firebaseStatus = "";
bool firebaseConnected = false;
bool fbCredentialsSaved = false;
bool fbTestPending = false;
String lastFirebaseResponse = "";
unsigned long lastFirebaseSend = 0;



const char* mainMenu[] = {
  "Back",            // 0
  "WiFi Setup",      // 1
  "Node-Red",        // 2
  "Cloud Share",     // 3
  "AFPC",            // 4
  "Startup",         // 5
  "Interval",        // 6
  "Thresholds",      // 7
  "Safety Alert",    // 8
  "Safety Protec.",  // 9
  "Main Output",     // 10
  "Update Time",     // 11
  "Clear Data",      // 12
  "Factory Reset",   // 13
  "Restart",         // 14
  "About"            // 15
};
const int mainMenuCount = sizeof(mainMenu) / sizeof(mainMenu[0]);

// Sub Menu Items
const char* wifiSetupSubmenu[] = { "Back", "Connect WiFi", "WiFi Status", "Disconnect", "Forget Wifi" };
const char* nodeRedSubmenu[] = { "Back", "Enable/Disable", "IP Address", "Port", "Status" };
const char* firebaseSubmenu[] = { "Back", "Enable/Disable", "Credentials Setup", "Upload Interval", "Test Connection", "Forget Credentials", "Status" };
const char* afpcSubmenu[] = { "Back", "Enable/Disable", "Target PF", "MFD Per Channel" };
const char* startupSubmenu[] = { "Back", "Delay Time" };
const char* safetyAlertSubmenu[] = { "Back", "High Voltage", "Low Voltage", "High Power", "Low Power", "Under Freq.", "Over Freq." };
const char* safetyProtectionSubmenu[] = { "Back", "High V Protection", "Low V Protection", "High P Protection", "High Hz Protection", "Low Hz Protection" };
const char* thresholdSubmenu[] = { "Back", "Voltage High", "Voltage Low", "Power High", "Power Low", "Freq. High", "Freq. Low", "Max kWh", "AFPC Low Watt", "AFPC High Watt" };
const char* clearDataSubmenu[] = { "Back", "Clear kWh", "Clear Days" };
const char* mainOutputSubmenu[] = { "Back", "Enable/Disable", "kWh Saver" };
const char* factoryResetSubmenu[] = { "Back", "Reset Data", "Reset Settings" };
const char* updateIntervalSubmenu[] = { "Back", "ESP Sharing", "Node-Red Update", "Sensor Update", "Safety Alert", "Protection Delay" };


// 1. Basic Utility Functions
void playBeep(int duration) {
  //  if (!settings.buzzerEnabled) return;

  digitalWrite(BUZZER_PIN, HIGH);  // Turn buzzer ON
  delay(duration);                 // Keep ON for duration
  digitalWrite(BUZZER_PIN, LOW);   // Turn buzzer OFF
}
void showMessage(const char* msg, int delay_ms = 1000) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 24, msg);
  u8g2.sendBuffer();
  delay(delay_ms);
}
void welcomeMsg() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB18_tr);
  u8g2.drawStr(0, 22, MSG_WELCOME);
  u8g2.setFont(u8g2_font_ncenR12_tr);
  u8g2.drawStr(0, 40, MSG_SUBTITLE);
  u8g2.setFont(u8g2_font_t0_11_tr);
  u8g2.drawStr(2, 60, MSG_DEVELOPER);
  u8g2.sendBuffer();
}

// 2. Input/Encoder Functions
bool buttonPressed() {
  if (digitalRead(ENCODER_SW) == LOW) {
    delay(200);
    while (digitalRead(ENCODER_SW) == LOW)
      ;
    return true;
  }
  return false;
}
void IRAM_ATTR encoderISR() {
  portENTER_CRITICAL_ISR(&encoderMux);
  encoder.tick();
  encoderValue = encoder.getPosition();
  portEXIT_CRITICAL_ISR(&encoderMux);
}

// 9. Display functions
void clearLCD(const long x, uint8_t y, uint8_t wid, uint8_t hig) {
  /*  box wid is right x, box height is below y
      where font wid is right x, font height is upper y
  */
  u8g2.setDrawColor(0);
  u8g2.drawBox(x, y, wid, hig);
  u8g2.setDrawColor(1);
}

void displayLayout() {
  u8g2.clearBuffer();
  u8g2.drawFrame(0, 0, 128, 64);
  u8g2.drawLine(54, 1, 54, 62);
  u8g2.drawLine(1, 39, 126, 39);
  u8g2.sendBuffer();
}

void displayUnits() {
  u8g2.setFont(u8g2_font_t0_11_tr);
  u8g2.drawStr(46, 12, "V");
  u8g2.drawStr(113, 12, "PF");
  u8g2.drawStr(46, 24, "A");
  u8g2.drawStr(118, 24, "W");
  u8g2.drawStr(40, 36, "Hz");
  u8g2.drawStr(107, 36, "kWh");
  u8g2.drawStr(46, 50, "H");
  u8g2.drawStr(46, 61, "D");
  u8g2.sendBuffer();
}

void displayStatusbar() {
  u8g2.setFont(u8g2_font_t0_11_tr);

  // ✅ WiFi Status
  clearLCD(58, 52, 9, 9);
  if (WiFi.status() == WL_CONNECTED) {
    u8g2.drawStr(59, 61, "W");
  }

  // Node-Red Status
  clearLCD(70, 52, 9, 9);
  if (settings.nodeRedEnabled) {
    u8g2.drawStr(72, 61, "R");
  }

  // Firebase Status
  clearLCD(86, 52, 12, 9);
  if (fbSettings.enabled) {
    u8g2.drawStr(85, 61, "FB");
  }

  // Protection Status
  clearLCD(103, 52, 9, 9);
  if (settings.highVProtec || settings.LowVProtec || settings.highPProtec || settings.highHzProtec || settings.lowHzProtec) {
    u8g2.drawStr(105, 61, "P");
  }

  // Alert Status
  clearLCD(117, 52, 9, 9);
  if (settings.highVoltageAlert || settings.lowVoltageAlert || settings.highPowerAlert || settings.lowPowerAlert || settings.highFreqAlert || settings.lowFreqAlert) {
    u8g2.drawStr(119, 61, "A");
  }

  // ✅ IMPORTANT: Send buffer after drawing all status icons
  u8g2.sendBuffer();
}

bool connectToSavedWiFi() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "Connecting WiFi...");
  u8g2.sendBuffer();

  WiFiManager wm;
  bool success = false;

  // ✅ Always set WiFi mode and begin - even if no saved credentials
  WiFi.mode(WIFI_STA);
  WiFi.begin();  // This initializes WiFi stack

  int attempts = 0;
  const int MAX_ATTEMPTS = 5;

  while (attempts < MAX_ATTEMPTS) {
    char attemptStr[16];
    snprintf(attemptStr, 16, "Attempt: %d/5", attempts + 1);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 12, "Connecting WiFi...");
    u8g2.drawStr(0, 24, attemptStr);
    u8g2.sendBuffer();

    if (WiFi.status() == WL_CONNECTED) {
      // Success - show connection details
      u8g2.clearBuffer();
      u8g2.drawStr(0, 12, "WiFi Connected!");
      u8g2.drawStr(0, 24, "SSID:");
      u8g2.drawStr(0, 36, WiFi.SSID().c_str());
      u8g2.drawStr(0, 48, "IP:");
      u8g2.drawStr(0, 60, WiFi.localIP().toString().c_str());
      u8g2.sendBuffer();
      delay(2000);
      return true;
    }

    delay(2000);
    attempts++;
  }

  // ✅ Don't show AP portal during startup - just note WiFi not available
  u8g2.clearBuffer();
  u8g2.drawStr(0, 12, "No WiFi Found");
  u8g2.drawStr(0, 24, "Will auto-connect");
  u8g2.drawStr(0, 36, "when available");
  u8g2.drawStr(0, 48, "or use menu");
  u8g2.sendBuffer();
  delay(2000);

  return false;  // ✅ Return false but WiFi stack is initialized
}

void performSafetyCheck() {
  int remainingTime = settings.startupTime;
  int totalTime = settings.startupTime;
  bool safeToStart = true;
  bool manualOverride = false;

  // Bar parameters
  const int barMaxWidth = 120;  // Maximum width of progress bar
  const int barHeight = 6;      // Height of progress bar
  const int barX = 4;           // X position of bar
  const int barY = 40;          // Y position of bar

  while (remainingTime > 0 || !safeToStart) {
    // Get real-time measurements
    portENTER_CRITICAL(&measureMux);
    readings.voltage = pzem.voltage();      // Test value
    readings.frequency = pzem.frequency();  // Test value
    portEXIT_CRITICAL(&measureMux);
    delay(100);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);

    if (!isnan(readings.voltage)) {

      // Display voltage at top right with status
      char voltStr[10];
      dtostrf(readings.voltage, 6, 2, voltStr);
      u8g2.drawStr(62, 9, voltStr);
      u8g2.drawStr(100, 9, "V");

      if (readings.voltage > settings.voltHigh) {
        u8g2.drawStr(0, 9, "high V");
        safeToStart = false;
      } else if (readings.voltage < settings.voltLow) {
        u8g2.drawStr(0, 9, "low V");
        safeToStart = false;
      } else {
        u8g2.drawStr(0, 9, "normal");
      }

      // Display frequency at right with status
      char freqStr[10];
      dtostrf(readings.frequency, 5, 2, freqStr);
      u8g2.drawStr(62, 23, freqStr);
      u8g2.drawStr(100, 23, "Hz");

      if (readings.frequency > settings.freqHigh) {
        u8g2.drawStr(0, 23, "high fre.");
        safeToStart = false;
      } else if (readings.frequency < settings.freqLow) {
        u8g2.drawStr(0, 23, "undr fre.");
        safeToStart = false;
      } else {
        u8g2.drawStr(0, 23, "normal");
      }

      // Show countdown or override message at bottom
      if (!safeToStart) {
        u8g2.drawStr(0, 60, "press push to turn on!");
        if (digitalRead(RESET_BTN) == HIGH) {
          manualOverride = true;
          break;
        }
      } else {

        // Draw progress bar
        int barWidth = map(totalTime - remainingTime, 0, totalTime, 0, barMaxWidth);

        // Draw bar frame
        u8g2.drawFrame(barX, barY, barMaxWidth, barHeight);

        // Draw filled portion
        if (barWidth > 0) {
          u8g2.drawBox(barX, barY, barWidth, barHeight);
        }

        u8g2.drawStr(4, 60, "wait for...");
        char timeStr[8];
        sprintf(timeStr, "%d s", remainingTime);
        u8g2.drawStr(80, 60, timeStr);
        remainingTime--;
      }

      u8g2.sendBuffer();

      if (digitalRead(RESET_BTN) == HIGH) {
        manualOverride = true;
        break;
      }

      delay(1000);
    }

    // Reset safety flag if values return to normal
    if (readings.voltage >= settings.voltLow && readings.voltage <= settings.voltHigh && readings.frequency >= settings.freqLow && readings.frequency <= settings.freqHigh) {
      safeToStart = true;
    }
  }

  // If startup successful or manually overridden
  if (safeToStart || manualOverride) {
    u8g2.clearBuffer();
    u8g2.drawStr(31, 35, "Starting...");
    u8g2.sendBuffer();
    digitalWrite(BUZZER_PIN, HIGH);
    delay(1500);
    digitalWrite(BUZZER_PIN, LOW);
    delay(1000);
  }
}

// 9. Main Output
void controlMainOutput() {
  portENTER_CRITICAL(&measureMux);
  Measurements current = readings;
  portEXIT_CRITICAL(&measureMux);

  bool currentProtectionActive = false;
  static bool waitingForDelay = false;

  // ✅ FIRST: Check if manually turned OFF from menu
  if (!settings.mainOutput) {
    shouldTurnOff = true;
    digitalWrite(MAIN_RELAY, HIGH);  // ✅ Turn OFF relay immediately

    if (!menuActive) {
      clearLCD(57, 41, 69, 10);
      u8g2.setFont(u8g2_font_t0_11_tr);
      u8g2.drawStr(59, 50, "MAIN: OFF");
      u8g2.sendBuffer();
    }
    return;  // ✅ Exit early - don't check other conditions
  }

  // ✅ THEN: Check kWh Saver
  if (settings.kWhSaver && current.energy >= settings.maxkWh) {
    shouldTurnOff = true;
    digitalWrite(MAIN_RELAY, HIGH);  // Turn OFF

    if (!menuActive) {
      clearLCD(57, 41, 69, 10);
      u8g2.setFont(u8g2_font_t0_11_tr);
      u8g2.drawStr(59, 50, "kWh SAVER");
      u8g2.sendBuffer();
    }
    return;
  }

  // ✅ THEN: Check Protection conditions
  if ((settings.highVProtec && current.voltage > settings.voltHigh) || (settings.LowVProtec && current.voltage < settings.voltLow) || (settings.highPProtec && current.power > settings.powerHigh) || (settings.highHzProtec && current.frequency > settings.freqHigh) || (settings.lowHzProtec && current.frequency < settings.freqLow)) {

    currentProtectionActive = true;
    waitingForDelay = false;
    digitalWrite(MAIN_RELAY, HIGH);  // Turn OFF

    if (!menuActive) {
      clearLCD(57, 41, 69, 10);
      u8g2.setFont(u8g2_font_t0_11_tr);
      u8g2.drawStr(59, 50, "PRO. TRIG.");
      u8g2.sendBuffer();
    }
    shouldTurnOff = true;

  } else {
    shouldTurnOff = false;
  }

  // Handle protection delay timer
  if (currentProtectionActive) {
    protectionWasActive = true;
    protectionClearedTime = millis();
    shouldTurnOff = true;
    digitalWrite(MAIN_RELAY, HIGH);  // Keep OFF

  } else if (protectionWasActive) {
    // Protection cleared but waiting for delay
    if (!waitingForDelay) {
      protectionClearedTime = millis();
      waitingForDelay = true;
    }

    unsigned long elapsedTime = millis() - protectionClearedTime;
    unsigned long remainingTime = PROTECTION_DELAY - elapsedTime;
    int minutes = remainingTime / 60000;
    int seconds = (remainingTime % 60000) / 1000;

    // Check if wait time completed or reset pressed
    if (elapsedTime >= PROTECTION_DELAY || digitalRead(RESET_BTN) == HIGH) {
      protectionWasActive = false;
      waitingForDelay = false;
      shouldTurnOff = false;
      digitalWrite(MAIN_RELAY, LOW);  // ✅ Turn ON after delay
    } else {
      shouldTurnOff = true;
      digitalWrite(MAIN_RELAY, HIGH);  // Keep OFF during wait

      char timeStr[32];
      snprintf(timeStr, sizeof(timeStr), "Wait: %02d:%02d", minutes, seconds);
      if (!menuActive) {
        clearLCD(57, 41, 69, 10);
        u8g2.setFont(u8g2_font_t0_11_tr);
        u8g2.drawStr(59, 50, timeStr);
        u8g2.sendBuffer();
      }
    }

  } else {
    // ✅ All conditions OK - Turn relay ON
    shouldTurnOff = false;
    digitalWrite(MAIN_RELAY, LOW);
  }
}

void displayValues() {
  u8g2.setFont(u8g2_font_t0_11_tr);
  char buffer[16];

  portENTER_CRITICAL(&measureMux);
  Measurements current = readings;  // Take a snapshot
  portEXIT_CRITICAL(&measureMux);

  // Voltage
  dtostrf(current.voltage, 0, 1, buffer);  // Just convert to string
  clearLCD(2, 3, 43, 9);
  u8g2.drawStr(3, 12, buffer);
  u8g2.sendBuffer();

  // Current
  dtostrf(current.current, 0, 2, buffer);
  clearLCD(2, 15, 43, 9);
  u8g2.drawStr(3, 24, buffer);
  u8g2.sendBuffer();

  // Frequency
  dtostrf(current.frequency, 0, 2, buffer);
  clearLCD(2, 27, 37, 9);
  u8g2.drawStr(3, 36, buffer);
  u8g2.sendBuffer();

  // Uptime
  dtostrf(current.uptime, 0, 1, buffer);
  clearLCD(2, 41, 43, 9);
  u8g2.drawStr(3, 50, buffer);
  u8g2.sendBuffer();

  // Total days
  dtostrf(current.totalDays, 0, 1, buffer);
  clearLCD(2, 52, 43, 9);
  u8g2.drawStr(3, 61, buffer);
  u8g2.sendBuffer();

  // Power factor
  dtostrf(current.pf, 0, 2, buffer);
  clearLCD(57, 3, 43, 9);
  u8g2.drawStr(59, 12, buffer);
  u8g2.sendBuffer();

  // Power
  dtostrf(current.power, 0, 1, buffer);
  clearLCD(57, 15, 57, 9);
  u8g2.drawStr(59, 24, buffer);
  u8g2.sendBuffer();

  // Energy
  dtostrf(current.energy, 0, 2, buffer);
  clearLCD(57, 27, 49, 9);
  u8g2.drawStr(59, 36, buffer);
  u8g2.sendBuffer();
}

void displayDateAndTime() {
  if (shouldTurnOff) return;  // for showing protection message
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    char dateStr[8];
    char timeStr[8];

    // Format date as DD/MM
    strftime(dateStr, sizeof(dateStr), "%d/%m", &timeinfo);
    // Format time as HH:MM
    strftime(timeStr, sizeof(timeStr), "%H:%M", &timeinfo);

    u8g2.setFont(u8g2_font_t0_11_tr);
    clearLCD(57, 41, 69, 10);
    u8g2.drawStr(59, 50, dateStr);  // Show date
    u8g2.drawStr(95, 50, timeStr);  // Show time
    u8g2.sendBuffer();
  } else {
    // Show dashes if time not available
    if ((WiFi.status() == WL_CONNECTED)) {
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      struct tm timeinfo;
    }
    clearLCD(57, 41, 69, 10);
    u8g2.setFont(u8g2_font_t0_11_tr);
    u8g2.drawStr(59, 50, "--/--");
    u8g2.drawStr(95, 50, "--:--");
    u8g2.sendBuffer();
  }
}

void showMainDisplay() {
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate >= DISPLAY_REFRESH_INTERVAL) {
    displayLayout();
    displayValues();
    displayUnits();
    displayStatusbar();
    displayDateAndTime();
    lastDisplayUpdate = millis();
  }
}

// 10. PZEM Functions
void updatePzemReadings() {
  digitalWrite(PZEM_RELAY, LOW);
  unsigned long currentMillis = millis();
  float ut = currentMillis / 36e5;

  static unsigned long lastUpdate = 0;
  if (currentMillis - lastUpdate < (settings.pzemInterval * 1000)) return;
  float td = (currentMillis - lastUpdate) / 864e5;
  kwhDays += td;
  lastUpdate = currentMillis;

  //  portENTER_CRITICAL(&measureMux);
  // Generate realistic random values for testing
  //  readings.voltage = 220.0 + random(-500, 500) / 100.0;    // 215.00 to 225.00V
  //  readings.current = random(0, 1000) / 100.0;              // 0.00 to 10.00A
  //  readings.power = readings.voltage * readings.current;     // Calculated from V*A
  //  readings.energy += readings.power / 3600000.0;           // Add kWh per second
  //  readings.frequency = 50.0 + random(-20, 20) / 100.0;     // 49.80 to 50.20Hz
  //  readings.pf = 0.85 + random(0, 15) / 100.0;             // 0.85 to 1.00
  //  readings.uptime = millis() / 3600000.0;                  // Hours
  //  readings.totalDays = settings.totalDays;                 // From settings

  readings.voltage = pzem.voltage();
  readings.current = pzem.current();
  readings.power = pzem.power();
  // readings.power = random(0, 100000) / 100.0;
  readings.energy = pzem.energy();
  readings.frequency = pzem.frequency();
  readings.pf = pzem.pf();
  // readings.pf = 0.35 + random(0, 65) / 100.0;
  readings.uptime = ut;
  readings.totalDays = kwhDays;

  // Reset Energy
  if (resetKwhRequested) {
    if (pzem.resetEnergy()) {
      Serial.println("Energy Reset Done!");
      resetKwhRequested = false;
      showMessage("Restarting...");
      ESP.restart();
    }
  }
  // Reset Days
  if (resetDaysRequested) {
    Serial.println("Days Reset Done!");
    resetDaysValues();
    resetDaysRequested = false;
    showMessage("Restarting...");
    ESP.restart();
  }
}

// 11. Async Web Server
// void handleGetData(AsyncWebServerRequest* request) {
//   AsyncResponseStream* response = request->beginResponseStream("application/json");
//   StaticJsonDocument<200> jsonDoc;

//   // Take mutex before reading values
//   portENTER_CRITICAL(&measureMux);
//   Measurements current = readings;  // Take a snapshot
//   portEXIT_CRITICAL(&measureMux);

//   // Use snapshot values
//   jsonDoc["voltage"] = String(current.voltage, 1);
//   jsonDoc["current"] = String(current.current, 2);
//   jsonDoc["power"] = String(current.power, 1);
//   jsonDoc["energy"] = String(current.energy, 2);
//   jsonDoc["frequency"] = String(current.frequency, 2);
//   jsonDoc["pf"] = String(current.pf, 2);
//   jsonDoc["days"] = String(current.totalDays, 1);
//   jsonDoc["uptime"] = String(current.uptime, 1);

//   serializeJson(jsonDoc, *response);
//   request->send(response);
// }

// function to setup and start web server
void setupWebServer() {
  if (webServerStarted) return;  // Already started

  Serial.println("Setting up web server...");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(SPIFFS, "/style.css", "text/css");
  });

  server.on("/table.svg", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(SPIFFS, "/table.svg", "image/svg+xml");
  });

  server.on("/plug_icon.svg", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(SPIFFS, "/plug_icon.svg", "image/svg+xml");
  });

  server.on("/sensor.json", HTTP_GET, [](AsyncWebServerRequest* request) {
    portENTER_CRITICAL(&measureMux);
    Measurements current = readings;
    portEXIT_CRITICAL(&measureMux);

    StaticJsonDocument<512> doc;
    doc["voltage"] = String(current.voltage, 1);
    doc["current"] = String(current.current, 2);
    doc["power"] = String(current.power, 1);
    doc["energy"] = String(current.energy, 2);
    doc["frequency"] = String(current.frequency, 2);
    doc["pf"] = String(current.pf, 2);
    doc["uptime"] = String(current.uptime, 1);
    doc["days"] = String(current.totalDays, 1);
    doc["mainOutput"] = settings.mainOutput ? 1 : 0;

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // Main Output Control Route - SET new state
  server.on(
    "/output/set", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, (const char*)data);

      if (!error) {
        bool state = doc["state"];
        bool previousState = settings.mainOutput;

        settings.mainOutput = state;
        saveSettings();

        // Control relay pin
        digitalWrite(MAIN_RELAY, state ? LOW : HIGH);

        Serial.printf("Web Control: Output set to %s\n", state ? "ON" : "OFF");

        // Push to Firebase only on change
        if (fbSettings.enabled && WiFi.status() == WL_CONNECTED && previousState != state) {
          writeOutputToFirebase(state);
        }

        // Send success response
        StaticJsonDocument<100> response;
        response["success"] = true;
        response["mainOutput"] = state ? 1 : 0;

        String jsonResponse;
        serializeJson(response, jsonResponse);
        request->send(200, "application/json", jsonResponse);
      } else {
        request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
      }
    });

  server.begin();
  webServerStarted = true;
  Serial.println("Web server started successfully");
}

// Add this function to check WiFi and start server if needed
// Enhanced checkWiFiAndStartServer() with proper notification display
void checkWiFiAndStartServer() {
  static unsigned long lastCheck = 0;
  static bool wasConnected = false;
  static bool reconnecting = false;

  // Check every 5 seconds
  if (millis() - lastCheck < 5000) return;
  lastCheck = millis();

  bool isConnected = (WiFi.status() == WL_CONNECTED);

  // WiFi just connected (first time or reconnected)
  if (isConnected && !wasConnected) {
    Serial.println("WiFi connected! Starting services...");

    if (!webServerStarted) {
      setupWebServer();
    }

    // ✅ Show notification on OLED - PROPERLY refresh display
    // if (!menuActive) {
    //   u8g2.clearBuffer();
    //   u8g2.setFont(u8g2_font_6x12_tf);
    //   u8g2.drawStr(0, 12, "WiFi Connected!");
    //   u8g2.drawStr(0, 24, "Server Started");
    //   u8g2.drawStr(0, 36, "IP:");
    //   u8g2.drawStr(0, 48, WiFi.localIP().toString().c_str());
    //   u8g2.sendBuffer();
    //   delay(2000);

    //   // ✅ IMPORTANT: Force complete redraw
    //   u8g2.clearBuffer();  // Clear everything first
    //   displayLayout();     // Draw borders
    //   displayUnits();      // Draw unit labels
    //   u8g2.sendBuffer();   // ✅ Send buffer BEFORE statusbar

    //   // ✅ Now draw statusbar separately
    //   displayStatusbar();
    // }

    // Start Firebase task if enabled
    if (fbSettings.enabled && strlen(fbSettings.host) > 0 && firebaseTask == NULL) {
      Serial.println("Starting Firebase task...");
      xTaskCreatePinnedToCore(
        firebaseTaskFunction,
        "FirebaseTask",
        8192,
        NULL,
        1,
        &firebaseTask,
        0);
    }

    reconnecting = false;
  }

  // WiFi disconnected - try to reconnect
  if (!isConnected && wasConnected) {
    Serial.println("WiFi disconnected - will attempt reconnection");

    // Stop Firebase task
    if (firebaseTask != NULL) {
      vTaskDelete(firebaseTask);
      firebaseTask = NULL;
      Serial.println("Firebase task stopped");
    }

    reconnecting = true;
  }

  // ✅ Active reconnection attempts when disconnected
  if (!isConnected && reconnecting) {
    static unsigned long lastReconnectAttempt = 0;

    // Try reconnecting every 30 seconds
    if (millis() - lastReconnectAttempt > 30000) {
      lastReconnectAttempt = millis();

      Serial.println("Attempting WiFi reconnection...");

      // ✅ Show reconnecting status with proper display refresh
      if (!menuActive) {
        // Force reconnection attempt
        WiFi.disconnect();
        delay(100);
        WiFi.begin();  // ✅ Reconnect to saved credentials

        // Redraw layout after reconnection attempt
        u8g2.clearBuffer();
        displayLayout();
        displayUnits();
        u8g2.sendBuffer();

        // Update statusbar
        displayStatusbar();
      } else {
        // Just reconnect without display update if in menu
        WiFi.disconnect();
        delay(100);
        WiFi.begin();
      }
    }
  }

  wasConnected = isConnected;
}

// 12. Satety Features
void checkSafetyAlerts() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < (settings.alertInterval * 1000)) return;
  lastCheck = millis();

  // Get measurements once
  portENTER_CRITICAL(&measureMux);
  Measurements current = readings;
  portEXIT_CRITICAL(&measureMux);

  // Check alerts without repeated mutex locks
  bool alertsTriggered = ((settings.highVoltageAlert && current.voltage > settings.voltHigh) || (settings.lowVoltageAlert && current.voltage < settings.voltLow) || (settings.highPowerAlert && current.power > settings.powerHigh) || (settings.lowPowerAlert && current.power < settings.powerLow) || (settings.highFreqAlert && current.frequency > settings.freqHigh) || (settings.lowFreqAlert && current.frequency < settings.freqLow));

  if (alertsTriggered) {
    digitalWrite(BUZZER_PIN, HIGH);
    vTaskDelay(ALERT_BEEP_DURATION / portTICK_PERIOD_MS);
    digitalWrite(BUZZER_PIN, LOW);
    vTaskDelay(ALERT_BEEP_PAUSE / portTICK_PERIOD_MS);
    digitalWrite(BUZZER_PIN, HIGH);
    vTaskDelay(ALERT_BEEP_DURATION / portTICK_PERIOD_MS);
    digitalWrite(BUZZER_PIN, LOW);
    vTaskDelay(ALERT_BEEP_PAUSE / portTICK_PERIOD_MS);
    digitalWrite(BUZZER_PIN, HIGH);
    vTaskDelay(ALERT_BEEP_DURATION / portTICK_PERIOD_MS);
    digitalWrite(BUZZER_PIN, LOW);
    vTaskDelay(ALERT_BEEP_PAUSE / portTICK_PERIOD_MS);
  }
}

// Days Tracking
void saveTotalDays() {
  static unsigned long lastSave = 0;
  static float lastSavedValue = -1.0;  // Track last saved value

  unsigned long now = millis();

  // Save conditions:
  // 1. Regular interval (10 minutes)
  // 2. Significant change (>0.1 days difference)
  // 3. First time initialization

  bool shouldSave = false;

  // Regular interval save
  if (now - lastSave >= TOTAL_DAYS_INTERVAL) {
    shouldSave = true;
  }

  // Significant change save (prevents data loss on unexpected reboot)
  if (lastSavedValue < 0 || abs(kwhDays - lastSavedValue) >= 0.1) {
    shouldSave = true;
  }

  if (shouldSave) {
    preferences.putFloat("days", kwhDays);
    lastSave = now;
    lastSavedValue = kwhDays;

    Serial.printf("Days saved to flash: %.2f (auto-save)\n", kwhDays);
  }
}

// Enhanced reset with confirmation
void resetDaysValues() {
  kwhDays = 0.0;
  preferences.putFloat("days", kwhDays);
  // preferences.commit();  // ← Remove this line - not needed

  Serial.println("Days value reset to 0 and saved to flash");
}

// Add this function to force save on critical events (optional)
void forceSaveTotalDays() {
  preferences.putFloat("days", kwhDays);
  // preferences.commit();  // ← Remove this line - not needed
  Serial.printf("Days force-saved: %.2f\n", kwhDays);
}



void configDateTime() {
  if (WiFi.status() != WL_CONNECTED) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 24, "No WiFi!");
    u8g2.drawStr(0, 36, "Time not synced!");
    u8g2.drawStr(0, 48, "Starting offline...");
    u8g2.sendBuffer();
    // Yielding delay
    unsigned long start = millis();
    while (millis() - start < 2000) yield();

    // Set default time: 12:00, 01/01/2025
    struct tm tm;
    tm.tm_year = 2025 - 1900;
    tm.tm_mon = 0;
    tm.tm_mday = 1;
    tm.tm_hour = 12;
    tm.tm_min = 0;
    tm.tm_sec = 0;
    time_t t = mktime(&tm);
    struct timeval now = { .tv_sec = t };
    settimeofday(&now, nullptr);
    return;
  }

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "Syncing Time...");
  u8g2.sendBuffer();

  // Yielding delay
  unsigned long start = millis();
  while (millis() - start < 1000) yield();

  int attempts = 0;
  const int MAX_ATTEMPTS = 5;
  struct tm timeinfo;

  while (attempts < MAX_ATTEMPTS) {
    char attemptStr[16];
    snprintf(attemptStr, 16, "Attempt: %d/5", attempts + 1);
    u8g2.clearBuffer();
    u8g2.drawStr(0, 12, "Syncing Time...");
    u8g2.drawStr(0, 24, attemptStr);
    u8g2.sendBuffer();

    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    // Shorter yielding delay to avoid watchdog
    start = millis();
    while (millis() - start < 1000) yield();
    if (getLocalTime(&timeinfo)) {
      break;
    }
    attempts++;
  }

  if (getLocalTime(&timeinfo)) {
    char timeStr[16];
    char dateStr[18];
    char gmtStr[30];
    strftime(timeStr, sizeof(timeStr), "Time: %H:%M:%S", &timeinfo);
    strftime(dateStr, sizeof(dateStr), "Date: %d.%m.%Y", &timeinfo);
    strftime(gmtStr, sizeof(gmtStr), "GMT: %z %Z", &timeinfo);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_t0_14_tr);
    u8g2.drawStr(0, 16, timeStr);
    u8g2.drawStr(0, 32, dateStr);
    u8g2.drawStr(0, 62, gmtStr);
    u8g2.sendBuffer();
    // Yielding delay
    start = millis();
    while (millis() - start < 2000) yield();
  } else {
    // Apply default time if NTP fails
    struct tm tm;
    tm.tm_year = 2025 - 1900;
    tm.tm_mon = 0;
    tm.tm_mday = 1;
    tm.tm_hour = 12;
    tm.tm_min = 0;
    tm.tm_sec = 0;
    time_t t = mktime(&tm);
    struct timeval now = { .tv_sec = t };
    settimeofday(&now, nullptr);

    u8g2.clearBuffer();
    u8g2.drawStr(0, 12, "Time not synced!");
    u8g2.drawStr(0, 24, "Check internet!");
    u8g2.drawStr(0, 48, "Starting offline...");
    u8g2.sendBuffer();
    // Yielding delay
    start = millis();
    while (millis() - start < 2000) yield();
  }
}

void showAboutScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_7x14B_tr);
  u8g2.drawStr(0, 12, "ESP Energy Log");
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 28, "Version: 2.0");
  u8g2.drawStr(0, 40, "By: M.Maity");
  u8g2.drawStr(0, 60, "Press to exit");
  u8g2.sendBuffer();

  while (!buttonPressed()) {
    delay(100);
  }
  drawMenu(mainMenu, mainMenuCount, 15);
}

void stopFirebaseTask() {
  if (firebaseTask != NULL) {
    vTaskDelete(firebaseTask);
    firebaseTask = NULL;
    Serial.println("Firebase task stopped");
  }
}

void startFirebaseTask() {
  if (firebaseTask == NULL && fbSettings.enabled && strlen(fbSettings.host) > 0) {
    xTaskCreatePinnedToCore(
      firebaseTaskFunction,
      "FirebaseTask",
      8192,
      NULL,
      1,
      &firebaseTask,
      0);
    Serial.println("Firebase task restarted");
  }
}

int adjustThreshold(const char* title, int currentValue, int minValue, int maxValue, char unit = 'V', int step = 1) {
  bool editing = true;
  int tempValue = currentValue;
  encoder.setPosition(tempValue / step);  // Divide by step for encoder position
  long lastPos = tempValue / step;

  while (editing) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_7x14B_tr);
    u8g2.drawStr(0, 12, title);

    u8g2.setFont(u8g2_font_6x12_tf);
    char valueStr[8];
    encoder.tick();
    long pos = encoder.getPosition();

    if (pos != lastPos) {
      tempValue = constrain(pos * step, minValue, maxValue);  // Multiply by step
      lastPos = pos;
      playBeep(25);
    }

    snprintf(valueStr, 8, "%d%c", tempValue, unit);
    u8g2.drawStr(45, 35, valueStr);
    u8g2.drawStr(0, 55, "Click to save");
    u8g2.sendBuffer();

    if (buttonPressed()) {
      if (showSaveConfirmDialog()) {
        showMessage("Saved!");
      } else {
        showMessage("Cancelled!");
        tempValue = currentValue;
      }
      editing = false;
    }
  }
  return tempValue;
}
float adjustFrequency(const char* title, float currentValue, float minValue, float maxValue) {
  bool editing = true;
  float tempValue = currentValue;
  int scaledValue = tempValue * 100;  // Convert to integer (e.g. 50.00 -> 5000)
  encoder.setPosition(scaledValue);
  long lastPos = scaledValue;

  while (editing) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_7x14B_tr);
    u8g2.drawStr(0, 12, title);

    u8g2.setFont(u8g2_font_6x12_tf);
    char valueStr[8];
    encoder.tick();
    long pos = encoder.getPosition();

    if (pos != lastPos) {
      scaledValue = constrain(pos, minValue * 100, maxValue * 100);
      tempValue = scaledValue / 100.0;  // Convert back to float
      lastPos = pos;
      playBeep(25);
    }

    snprintf(valueStr, 8, "%.2fHz", tempValue);
    u8g2.drawStr(45, 35, valueStr);
    u8g2.drawStr(0, 55, "Click to save");
    u8g2.sendBuffer();

    if (buttonPressed()) {
      if (showSaveConfirmDialog()) {
        showMessage("Saved!");
        editing = false;  // Add this line
        return tempValue;
      } else {
        showMessage("Cancelled!");
        editing = false;  // Add this line
        return currentValue;
      }
    }
  }
  return currentValue;
}

float adjustMfdPerChannel() {
  bool editing = true;
  float tempMfd = settings.mfdPerChannel;
  int scaledValue = tempMfd * 10;  // Convert to integer (e.g., 4.0 -> 40)
  encoder.setPosition(scaledValue);
  long lastPos = scaledValue;

  while (editing) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_7x14B_tr);
    u8g2.drawStr(0, 12, "MFD Per Chan");

    u8g2.setFont(u8g2_font_6x12_tf);
    char valueStr[8];
    encoder.tick();
    long pos = encoder.getPosition();

    if (pos != lastPos) {
      scaledValue = constrain(pos, 1, 150);  // 0.1 to 15.0
      tempMfd = scaledValue / 10.0;          // Convert back to float
      lastPos = pos;
      playBeep(25);
    }

    snprintf(valueStr, 8, "%.1f", tempMfd);
    u8g2.drawStr(45, 35, valueStr);
    u8g2.drawStr(0, 55, "Click to save");
    u8g2.sendBuffer();

    if (buttonPressed()) {
      if (showSaveConfirmDialog()) {
        showMessage("Saved!");
        return tempMfd;
      } else {
        showMessage("Cancelled!");
        return settings.mfdPerChannel;
      }
    }
  }
  return settings.mfdPerChannel;
}

void adjustTargetPF() {
  bool editing = true;
  float tempPF = settings.targetFactor;
  encoder.setPosition(tempPF * 100);  // Scale up for integer steps

  while (editing) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_7x14B_tr);
    u8g2.drawStr(0, 12, "Target PF");

    u8g2.setFont(u8g2_font_6x12_tf);
    char pf[6];
    long pos = encoder.getPosition();
    pos = constrain(pos, 50, 99);  // Limit between 0.50 and 0.99
    tempPF = pos / 100.0;
    snprintf(pf, 6, "%.2f", tempPF);

    u8g2.drawStr(45, 35, pf);
    u8g2.drawStr(0, 55, "Click to Save!");
    u8g2.sendBuffer();

    encoder.tick();
    if (pos != lastEncoderPos) {
      lastEncoderPos = pos;
      playBeep(25);
    }

    if (buttonPressed()) {
      // Show confirmation dialog
      if (showSaveConfirmDialog()) {
        settings.targetFactor = tempPF;
        saveSettings();

        showMessage("PF Saved!");
      } else {
        showMessage("Cancelled!");
      }
      editing = false;
    }
  }
}

// 5. Alert & Protection Functions
void showAlertResult(const char* alertName, bool enabled) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 24, alertName);
  u8g2.drawStr(0, 36, enabled ? "Alert Enabled" : "Alert Disabled");
  u8g2.sendBuffer();
  delay(1000);
}
void showProtectionResult(const char* protectionName, bool enabled) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 24, protectionName);
  u8g2.drawStr(0, 36, enabled ? "Protection ON" : "Protection OFF");
  u8g2.sendBuffer();
  delay(1000);
}

// New helper function to draw values for selected items
void drawSubmenuValue(int mainIndex, int subIndex) {
  char val[16];

  switch (mainIndex) {
    case 1:  // WiFi Setup
      if (subIndex == 2) u8g2.drawStr(85, 12, WiFi.status() == WL_CONNECTED ? "conn" : "---");
      break;

    case 2:  // Node-Red
      if (subIndex == 1) {
        u8g2.drawStr(110, 12, settings.nodeRedEnabled ? "ON" : "OFF");
      } else if (subIndex == 3) {
        char* port = strchr(settings.nodeRedIP, ':');
        if (port) u8g2.drawStr(85, 12, port + 1);
      } else if (subIndex == 4) {
        u8g2.drawStr(85, 12, settings.nodeRedEnabled ? (nodeRedConnected ? "conn" : "---") : "OFF");
      }
      break;

    case 3:  // Cloud Share
      if (subIndex == 1) {
        u8g2.drawStr(110, 12, fbSettings.enabled ? "ON" : "OFF");
      } else if (subIndex == 3) {
        snprintf(val, 8, "%ds", fbSettings.dataInterval);
        u8g2.drawStr(100, 12, val);
      } else if (subIndex == 6) {
        if (fbSettings.enabled && strlen(fbSettings.host) > 0) {
          u8g2.drawStr(100, 12, firebaseConnected ? " " : " ");
        } else {
          u8g2.drawStr(100, 12, "---");
        }
      }
      break;

    case 4:  // AFPC
      if (subIndex == 1) {
        u8g2.drawStr(110, 12, settings.afpcEnabled ? "ON" : "OFF");
      } else if (subIndex == 2) {
        snprintf(val, 6, "%.2f", settings.targetFactor);
        u8g2.drawStr(85, 12, val);
      } else if (subIndex == 3) {
        snprintf(val, 6, "%.1f", settings.mfdPerChannel);
        u8g2.drawStr(85, 12, val);
      }
      break;

    case 5:  // Startup
      if (subIndex == 1) {
        snprintf(val, 6, "%ds", settings.startupTime);
        u8g2.drawStr(85, 12, val);
      }
      break;

    case 6:  // Update Interval
      {
        int* intervals[] = { nullptr, &settings.espShareInterval, &settings.nodeRedInterval,
                             &settings.pzemInterval, &settings.alertInterval, &settings.protectionDelay };
        if (subIndex >= 1 && subIndex <= 5) {
          snprintf(val, 8, "%ds", *intervals[subIndex]);
          u8g2.drawStr(85, 12, val);
        }
      }
      break;

    case 7:  // Thresholds
      if (subIndex == 1) snprintf(val, 6, "%dV", (int)settings.voltHigh);
      else if (subIndex == 2) snprintf(val, 6, "%dV", (int)settings.voltLow);
      else if (subIndex == 3) snprintf(val, 6, "%dW", (int)settings.powerHigh);
      else if (subIndex == 4) snprintf(val, 6, "%dW", (int)settings.powerLow);
      else if (subIndex == 5) snprintf(val, 7, "%.2fHz", settings.freqHigh);
      else if (subIndex == 6) snprintf(val, 7, "%.2fHz", settings.freqLow);
      else if (subIndex == 7) snprintf(val, 8, "%dkWh", settings.maxkWh);
      else if (subIndex == 8) snprintf(val, 8, "%dW", settings.afpcLowWatt);
      else if (subIndex == 9) snprintf(val, 8, "%dW", settings.afpcHighWatt);
      else return;
      u8g2.drawStr(85, 12, val);
      break;

    case 8:  // Safety Alert
      {
        bool* alerts[] = { nullptr, &settings.highVoltageAlert, &settings.lowVoltageAlert,
                           &settings.highPowerAlert, &settings.lowPowerAlert,
                           &settings.lowFreqAlert, &settings.highFreqAlert };
        if (subIndex >= 1 && subIndex <= 6) {
          u8g2.drawStr(110, 12, *alerts[subIndex] ? "ON" : "OFF");
        }
      }
      break;

    case 9:  // Safety Protection
      {
        bool* protections[] = { nullptr, &settings.highVProtec, &settings.LowVProtec,
                                &settings.highPProtec, &settings.highHzProtec, &settings.lowHzProtec };
        if (subIndex >= 1 && subIndex <= 5) {
          u8g2.drawStr(110, 12, *protections[subIndex] ? "ON" : "OFF");
        }
      }
      break;

    case 10:  // Main Output
      if (subIndex == 1) {
        u8g2.drawStr(110, 12, settings.mainOutput ? "ON" : "OFF");
      } else if (subIndex == 2) {
        u8g2.drawStr(110, 12, settings.kWhSaver ? "ON" : "OFF");
      }
      break;

    case 12:  // Clear Data
      if (subIndex == 1) {
        snprintf(val, 8, "%dkWh", (int)readings.energy);
        u8g2.drawStr(85, 12, val);
      } else if (subIndex == 2) {
        snprintf(val, 6, "%dd", (int)readings.totalDays);
        u8g2.drawStr(85, 12, val);
      }
      break;
  }
}

// New helper function to draw submenu with values
void drawSubmenu(int mainIndex, const char** items, int len, int selected) {
  u8g2.clearBuffer();

  // Header
  u8g2.setFont(u8g2_font_7x14B_tr);
  u8g2.drawStr(0, 12, mainMenu[mainIndex]);

  // Draw items (3 visible)
  u8g2.setFont(u8g2_font_6x12_tf);
  int start = (selected / 3) * 3;
  for (int i = 0; i < 3 && (start + i) < len; i++) {
    int itemIndex = start + i;
    int yPos = 28 + i * 12;

    if (itemIndex == selected) {
      u8g2.drawStr(0, yPos, ">");
      drawSubmenuValue(mainIndex, itemIndex);  // Show value for selected item
    }
    u8g2.drawStr(10, yPos, items[itemIndex]);
  }
  u8g2.sendBuffer();
}


// 6. WiFi & Communication Functions
void connectWiFi() {
  WiFiManager wm;
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);

  // Show initial message
  u8g2.drawStr(0, 12, "AP: ESP.EnergyLog");
  u8g2.drawStr(0, 24, "IP: 192.168.4.1");
  u8g2.drawStr(0, 48, "Timeout: 60s");
  u8g2.sendBuffer();

  wm.setConfigPortalTimeout(60);
  bool success = wm.autoConnect("ESP.EnergyLog");

  if (success) {
    u8g2.clearBuffer();
    u8g2.drawStr(0, 12, "Connected!");
    u8g2.drawStr(0, 24, "IP:");
    u8g2.drawStr(0, 36, WiFi.localIP().toString().c_str());
    u8g2.sendBuffer();
    delay(3000);
  } else {
    u8g2.clearBuffer();
    u8g2.drawStr(0, 24, "Connection Failed");
    u8g2.sendBuffer();
    delay(2000);
  }
}
void checkAndConnectWiFi() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);

  if (WiFi.status() == WL_CONNECTED) {
    // Already connected, show status
    u8g2.drawStr(0, 12, "Connected to:");
    u8g2.drawStr(0, 24, WiFi.SSID().c_str());
    u8g2.drawStr(0, 36, "IP:");
    u8g2.drawStr(0, 48, WiFi.localIP().toString().c_str());
    u8g2.sendBuffer();
    delay(3000);
  } else {
    // Try to reconnect
    u8g2.drawStr(0, 12, "Reconnecting to");
    u8g2.drawStr(0, 24, "saved network...");
    u8g2.sendBuffer();

    // Try to reconnect for 10 seconds
    WiFi.begin();
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 10) {
      delay(1000);
      attempts++;
      // Update progress
      u8g2.drawStr(0, 48, "Wait...");
      u8g2.drawStr(50, 48, String(10 - attempts).c_str());
      u8g2.drawStr(62, 48, "s");
      u8g2.sendBuffer();
    }

    if (WiFi.status() == WL_CONNECTED) {
      u8g2.clearBuffer();
      u8g2.drawStr(0, 12, "Connected!");
      u8g2.drawStr(0, 24, "SSID:");
      u8g2.drawStr(0, 36, WiFi.SSID().c_str());
      u8g2.drawStr(0, 48, "IP:");
      u8g2.drawStr(0, 60, WiFi.localIP().toString().c_str());
    } else {
      u8g2.clearBuffer();
      u8g2.drawStr(0, 24, "Connection Failed!");
      u8g2.drawStr(0, 36, "Goto Connect WiFi");
    }
    u8g2.sendBuffer();
    delay(3000);
  }
}


// WiFi Menu Handler
void handleWiFiSubmenu(int subIndex) {
  switch (subIndex) {
    case 1:  // Connect WiFi
      connectWiFi();
      break;

    case 2:  // WiFi Status
      checkAndConnectWiFi();
      break;

    case 3:  // Disconnect
      WiFi.disconnect();
      showMessage("WiFi Disconnected!");
      break;

    case 4:
      {  // Forget WiFi
        bool confirmed = false;
        showConfirmDialog("Forgot WiFi?", confirmed);
        if (confirmed) {
          WiFiManager wm;
          wm.resetSettings();
          showMessage("WiFi Reset Done!");
          ESP.restart();
        } else {
          showMessage("Cancelled!");
        }
        break;
      }
  }
}

// Node-Red Menu Handler
void handleNodeRedSubmenu(int subIndex) {
  switch (subIndex) {
    case 1:  // Enable/Disable
      showConfirmDialog("Enable Node-Red?", settings.nodeRedEnabled);
      saveSettings();
      showMessage(settings.nodeRedEnabled ? "Node-Red ON" : "Node-Red OFF");
      break;

    case 2:  // IP Address
      editNodeRedIP();
      break;

    case 3:
      {  // Port
        char* portStr = strchr(settings.nodeRedIP, ':');
        int currentPort = portStr ? atoi(portStr + 1) : 1880;

        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x12_tf);
        u8g2.drawStr(0, 24, "Current Port:");
        char portDisplay[6];
        snprintf(portDisplay, 6, "%d", currentPort);
        u8g2.drawStr(0, 36, portDisplay);
        u8g2.drawStr(0, 55, "Hold to edit >");
        u8g2.sendBuffer();
        delay(3000);

        if (buttonPressed()) {
          int newPort = adjustThreshold("Edit Port", currentPort, 1, 9999, ' ');
          if (showSaveConfirmDialog()) {
            if (portStr) snprintf(portStr + 1, 6, "%d", newPort);
            saveSettings();
            showMessage("Port Saved!");
          } else {
            showMessage("Cancelled!");
          }
        }
        break;
      }

    case 4:  // Status
      if (!settings.nodeRedEnabled) break;

      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_6x12_tf);
      u8g2.drawStr(0, 12, "Node-RED Status");
      u8g2.drawStr(0, 24, settings.nodeRedIP);
      u8g2.drawStr(0, 36, nodeRedConnected ? "Connected!" : "Disconnected!");
      u8g2.drawStr(0, 48, lastNodeRedResponse.c_str());
      u8g2.drawStr(0, 60, "Click to exit");
      u8g2.sendBuffer();

      while (!buttonPressed()) delay(100);
      break;
  }
}

// Cloud Share Menu Handler (Placeholder)
void handleCloudShareSubmenu(int subIndex) {
  bool confirmed = false;

  switch (subIndex) {
    case 1:  // Enable/Disable
      {
        showConfirmDialog("Enable Firebase?", confirmed);
        fbSettings.enabled = confirmed;
        saveFirebaseSettings();

        if (fbSettings.enabled) {
          startFirebaseTask();  // Start task
        } else {
          stopFirebaseTask();  // Stop task
        }

        showMessage(fbSettings.enabled ? "Firebase ON" : "Firebase OFF");
      }
      break;

    case 2:  // Credentials Setup
      if (WiFi.status() != WL_CONNECTED) {
        showMessage("WiFi Required!");
        break;
      }
      stopFirebaseTask();  // Stop during config
      startFirebaseConfigPortal();
      if (fbSettings.enabled) startFirebaseTask();  // Restart if enabled
      break;

    case 3:  // Data Share Interval
      {
        stopFirebaseTask();  // Stop to change interval
        int intervalSeconds = fbSettings.dataInterval;
        intervalSeconds = adjustThreshold("Upload Interval", intervalSeconds, 10, 300, 's', 1);
        fbSettings.dataInterval = intervalSeconds;
        saveFirebaseSettings();
        if (fbSettings.enabled) startFirebaseTask();  // Restart with new interval
      }
      break;

    case 4:  // Test Connection
      {
        if (strlen(fbSettings.host) == 0) {
          showMessage("Setup credentials!");
          break;
        }

        if (WiFi.status() != WL_CONNECTED) {
          showMessage("WiFi Required!");
          break;
        }

        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x12_tf);
        u8g2.drawStr(0, 12, "Testing...");
        u8g2.drawStr(0, 24, "Uploading data");
        u8g2.sendBuffer();
        delay(500);

        bool testSuccess = testFirebaseConnection();

        u8g2.clearBuffer();
        if (testSuccess) {
          u8g2.drawStr(0, 20, "Test Successful!");
          u8g2.drawStr(0, 32, "Data uploaded");
          u8g2.drawStr(0, 44, lastFirebaseResponse.c_str());
        } else {
          u8g2.drawStr(0, 20, "Test Failed!");
          u8g2.drawStr(0, 32, lastFirebaseResponse.c_str());
        }
        u8g2.drawStr(0, 60, "Click to exit");
        u8g2.sendBuffer();

        while (!buttonPressed()) delay(100);
      }
      break;

    case 5:  // Forget Credentials
      showConfirmDialog("Delete credentials?", confirmed);
      if (confirmed) {
        stopFirebaseTask();  // Stop task
        strcpy(fbSettings.host, "");
        strcpy(fbSettings.auth, "");
        fbSettings.enabled = false;
        saveFirebaseSettings();
        showMessage("Credentials Deleted!");
      } else {
        showMessage("Cancelled!");
      }
      break;

    case 6:  // Status
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_6x12_tf);
      u8g2.drawStr(0, 12, "Firebase Status");

      if (!fbSettings.enabled) {
        u8g2.drawStr(0, 28, "Status: Disabled");
      } else if (strlen(fbSettings.host) == 0) {
        u8g2.drawStr(0, 28, "No credentials");
        u8g2.drawStr(0, 40, "Setup first!");
      } else {
        char urlShort[22];
        strncpy(urlShort, fbSettings.host, 21);
        urlShort[21] = '\0';
        u8g2.drawStr(0, 24, urlShort);

        if (lastFirebaseResponse.length() > 0) {
          u8g2.drawStr(0, 36, lastFirebaseResponse.c_str());
        } else {
          u8g2.drawStr(0, 36, "Waiting for data...");
        }

        char intervalStr[20];
        snprintf(intervalStr, 20, "Interval: %ds", fbSettings.dataInterval);
        u8g2.drawStr(0, 48, intervalStr);
      }

      u8g2.drawStr(0, 60, "Click to exit");
      u8g2.sendBuffer();

      while (!buttonPressed()) delay(100);
      break;
  }
}

// AFPC Menu Handler
void handleAFPCSubmenu(int subIndex) {
  switch (subIndex) {
    case 1:  // Enable/Disable
      showConfirmDialog("Enable AFPC?", settings.afpcEnabled);
      saveSettings();
      showMessage(settings.afpcEnabled ? "AFPC Enabled" : "AFPC Disabled");
      break;

    case 2:  // Target PF
      adjustTargetPF();
      break;

    case 3:  // MFD Per Channel
      settings.mfdPerChannel = adjustMfdPerChannel();
      saveSettings();
      break;
  }
}

// Update Interval Menu Handler
void handleIntervalSubmenu(int subIndex) {
  const char* titles[] = { "", "ESP Share", "Node-Red", "PZEM Update", "Alert Check", "Protection Delay" };
  int* intervals[] = { nullptr, &settings.espShareInterval, &settings.nodeRedInterval,
                       &settings.pzemInterval, &settings.alertInterval, &settings.protectionDelay };
  int maxVals[] = { 0, 300, 300, 60, 600, 3600 };
  int steps[] = { 0, 1, 1, 1, 5, 60 };

  if (subIndex >= 1 && subIndex <= 5) {
    *intervals[subIndex] = adjustThreshold(titles[subIndex], *intervals[subIndex],
                                           subIndex == 5 ? 60 : (subIndex == 4 ? 15 : (subIndex == 3 ? 1 : (subIndex == 2 ? 5 : 1))),
                                           maxVals[subIndex], 's', steps[subIndex]);
    saveSettings();
    if (subIndex == 5) PROTECTION_DELAY = settings.protectionDelay * 1000UL;
  }
}

// Threshold Menu Handler
void handleThresholdSubmenu(int subIndex) {
  switch (subIndex) {
    case 1: settings.voltHigh = adjustThreshold("High Voltage", (int)settings.voltHigh, 150, 280); break;
    case 2: settings.voltLow = adjustThreshold("Low Voltage", (int)settings.voltLow, 120, 250); break;
    case 3: settings.powerHigh = adjustThreshold("High Power", (int)settings.powerHigh, 100, 9900, 'W', 100); break;
    case 4: settings.powerLow = adjustThreshold("Low Power", (int)settings.powerLow, 100, 9900, 'W', 100); break;
    case 5: settings.freqHigh = adjustFrequency("High Freq.", settings.freqHigh, 49.0, 51.0); break;
    case 6: settings.freqLow = adjustFrequency("Low Freq.", settings.freqLow, 49.0, 51.0); break;
    case 7: settings.maxkWh = adjustThreshold("Max kWh", settings.maxkWh, 1, 9000, 'U', 5); break;
    case 8: settings.afpcLowWatt = adjustThreshold("AFPC Low Watt", settings.afpcLowWatt, 1, 9900, 'W', 1); break;
    case 9: settings.afpcHighWatt = adjustThreshold("AFPC High Watt", settings.afpcHighWatt, 100, 9900, 'W', 50); break;
  }
  if (subIndex >= 1 && subIndex <= 9) saveSettings();
}

// Safety Alert Menu Handler
void handleSafetyAlertSubmenu(int subIndex) {
  bool* alerts[] = { nullptr, &settings.highVoltageAlert, &settings.lowVoltageAlert,
                     &settings.highPowerAlert, &settings.lowPowerAlert,
                     &settings.lowFreqAlert, &settings.highFreqAlert };
  const char* names[] = { "", "High Voltage", "Low Voltage", "High Power",
                          "Low Power", "Under Freq", "Over Freq" };

  if (subIndex >= 1 && subIndex <= 6) {
    showConfirmDialog("Enable Alert?", *alerts[subIndex]);
    saveSettings();
    showAlertResult(names[subIndex], *alerts[subIndex]);
  }
}

// Safety Protection Menu Handler
void handleSafetyProtectionSubmenu(int subIndex) {
  bool* protections[] = { nullptr, &settings.highVProtec, &settings.LowVProtec,
                          &settings.highPProtec, &settings.highHzProtec, &settings.lowHzProtec };
  const char* names[] = { "", "High Voltage", "Low Voltage", "High Power",
                          "High Freq", "Low Freq" };

  if (subIndex >= 1 && subIndex <= 5) {
    showConfirmDialog("Enable Protec?", *protections[subIndex]);
    saveSettings();
    showProtectionResult(names[subIndex], *protections[subIndex]);
  }
}

// Main Output Menu Handler
void handleMainOutputSubmenu(int subIndex) {
  switch (subIndex) {
    case 1:  // Enable/Disable
      {
        bool previousState = settings.mainOutput;
        showConfirmDialog("Enable Output?", settings.mainOutput);
        saveSettings();

        // ✅ Control relay immediately
        digitalWrite(MAIN_RELAY, settings.mainOutput ? LOW : HIGH);

        // ✅ Reset protection states when manually enabling
        if (settings.mainOutput) {
          protectionWasActive = false;
          shouldTurnOff = false;
        }

        // Push to Firebase only on change, only if enabled
        if (fbSettings.enabled && WiFi.status() == WL_CONNECTED && settings.mainOutput != previousState) {
          writeOutputToFirebase(settings.mainOutput);
        }

        showMessage(settings.mainOutput ? "Output Enabled" : "Output Disabled");
      }
      break;

    case 2:  // kWh Saver
      showConfirmDialog("Enable kWh Saver?", settings.kWhSaver);
      saveSettings();
      showMessage(settings.kWhSaver ? "kWh Saver ON" : "kWh Saver OFF");
      break;
  }
}

// Clear Data Menu Handler
void handleClearDataSubmenu(int subIndex) {
  bool confirmed = false;

  switch (subIndex) {
    case 1:  // Clear kWh
      showConfirmDialog("Clear kWh?", confirmed);
      if (confirmed) {
        resetKwhRequested = true;
        showMessage("Energy Reset!");
      } else {
        showMessage("Cancelled!");
      }
      break;

    case 2:  // Clear Days
      showConfirmDialog("Clear Days?", confirmed);
      if (confirmed) {
        resetDaysRequested = true;
        showMessage("Days Reset!");
      } else {
        showMessage("Cancelled!");
      }
      break;
  }
}

// Helper function to reset data to defaults
void resetDataDefaults() {
  settings.startupTime = 60;
  delay(10);
  settings.mfdPerChannel = 4.0;
  delay(10);
  settings.maxkWh = 1;
  delay(10);
  settings.targetFactor = 0.9;
  delay(10);
  settings.voltHigh = 245;
  delay(10);
  settings.voltLow = 120;
  delay(10);
  settings.powerHigh = 5000;
  delay(10);
  settings.powerLow = 100;
  delay(10);
  settings.freqHigh = 51;
  delay(10);
  settings.freqLow = 49;
  delay(10);
  settings.afpcLowWatt = 100;
  delay(10);
  settings.afpcHighWatt = 1000;
  delay(10);
  settings.espShareInterval = 2;
  delay(10);
  settings.nodeRedInterval = 30;
  delay(10);
  settings.pzemInterval = 1;
  delay(10);
  settings.alertInterval = 180;
  delay(10);
  saveSettings();
  delay(100);
  showMessage("Data Reset Done!");
  delay(1000);
}

// Helper function to reset settings to defaults
void resetSettingsDefaults() {
  settings.nodeRedEnabled = false;
  delay(10);
  settings.buzzerEnabled = true;
  delay(10);
  settings.highVoltageAlert = false;
  delay(10);
  settings.lowVoltageAlert = false;
  delay(10);
  settings.highPowerAlert = false;
  delay(10);
  settings.lowPowerAlert = false;
  delay(10);
  settings.highFreqAlert = false;
  delay(10);
  settings.lowFreqAlert = false;
  delay(10);
  settings.afpcEnabled = false;
  delay(10);
  settings.mainOutput = true;
  delay(10);
  settings.kWhSaver = false;
  delay(10);
  settings.highVProtec = false;
  delay(10);
  settings.LowVProtec = false;
  delay(10);
  settings.highPProtec = false;
  delay(10);
  settings.highHzProtec = false;
  delay(10);
  settings.lowHzProtec = false;
  delay(10);
  strcpy(settings.nodeRedIP, "192.168.0.1:1880");
  delay(50);

  WiFiManager wm;
  wm.resetSettings();
  delay(100);

  resetFirebaseSettings();
  delay(100);

  saveSettings();
  delay(100);
  showMessage("Settings Reset!");
  delay(1000);
}

// Factory Reset Menu Handler
void handleFactoryResetSubmenu(int subIndex) {
  bool confirmed = false;

  switch (subIndex) {
    case 1:  // Reset Data
      showConfirmDialog("Are you sure?", confirmed);
      if (confirmed) {
        showMessage("Resetting Data...");
        resetDataDefaults();
        showMessage("Restarting...");
        delay(500);
        ESP.restart();
      } else {
        showMessage("Cancelled!");
      }
      break;

    case 2:  // Reset Settings
      showConfirmDialog("Are you sure?", confirmed);
      if (confirmed) {
        showMessage("Resetting Settings...");
        resetSettingsDefaults();
        showMessage("Restarting...");
        delay(500);
        ESP.restart();
      } else {
        showMessage("Cancelled!");
      }
      break;
  }
}


void handleSubmenuSelection(int mainIndex, int subIndex) {
  bool confirmed = false;

  switch (mainIndex) {
    case 1:  // WiFi Setup
      handleWiFiSubmenu(subIndex);
      break;

    case 2:  // Node-Red
      handleNodeRedSubmenu(subIndex);
      break;

    case 3:  // Cloud Share
      handleCloudShareSubmenu(subIndex);
      break;

    case 4:  // AFPC
      handleAFPCSubmenu(subIndex);
      break;

    case 5:  // Startup
      if (subIndex == 1) {
        settings.startupTime = adjustThreshold("Startup Delay", settings.startupTime, 1, 600, 's', 5);
        saveSettings();
      }
      break;

    case 6:  // Update Interval
      handleIntervalSubmenu(subIndex);
      break;

    case 7:  // Thresholds
      handleThresholdSubmenu(subIndex);
      break;

    case 8:  // Safety Alert
      handleSafetyAlertSubmenu(subIndex);
      break;

    case 9:  // Safety Protection
      handleSafetyProtectionSubmenu(subIndex);
      break;

    case 10:  // Main Output
      handleMainOutputSubmenu(subIndex);
      break;

    case 12:  // Clear Data
      handleClearDataSubmenu(subIndex);
      break;

    case 13:  // Factory Reset
      handleFactoryResetSubmenu(subIndex);
      break;
  }
}

// 7. Menu Processing Functions
void processSelection(int index) {
  // Handle Back option
  if (index == 0) {
    menuActive = false;
    displayLayout();
    displayUnits();
    displayStatusbar();
    return;
  }

  // Handle direct actions (no submenu)
  switch (index) {
    case 11:  // Update Time
      configDateTime();
      drawMenu(mainMenu, mainMenuCount, index);
      return;

    case 14:  // Restart
      {
        bool confirmed = false;
        showConfirmDialog("Are you sure?", confirmed);
        if (confirmed) {
          showMessage("Saving data...");
          forceSaveTotalDays();  // ← Add this
          delay(500);
          showMessage("Restarting...");
          ESP.restart();
        }
      }
      break;

    case 15:  // About
      showAboutScreen();
      return;
  }

  // Get submenu details
  const char** submenu = nullptr;
  int submenuLength = 0;

  switch (index) {
    case 1:
      submenu = wifiSetupSubmenu;
      submenuLength = 5;
      break;
    case 2:
      submenu = nodeRedSubmenu;
      submenuLength = 5;
      break;
    case 3:
      submenu = firebaseSubmenu;
      submenuLength = 7;
      break;
    case 4:
      submenu = afpcSubmenu;
      submenuLength = 4;
      break;
    case 5:
      submenu = startupSubmenu;
      submenuLength = 2;
      break;
    case 6:
      submenu = updateIntervalSubmenu;
      submenuLength = 6;
      break;
    case 7:
      submenu = thresholdSubmenu;
      submenuLength = 10;
      break;
    case 8:
      submenu = safetyAlertSubmenu;
      submenuLength = 7;
      break;
    case 9:
      submenu = safetyProtectionSubmenu;
      submenuLength = 6;
      break;
    case 10:
      submenu = mainOutputSubmenu;
      submenuLength = 3;
      break;
    case 12:
      submenu = clearDataSubmenu;
      submenuLength = 3;
      break;
    case 13:
      submenu = factoryResetSubmenu;
      submenuLength = 3;
      break;
    default: return;
  }

  // Submenu navigation
  int subIndex = 0;
  bool inSubmenu = true;
  encoder.setPosition(0);

  while (inSubmenu) {
    drawSubmenu(index, submenu, submenuLength, subIndex);

    // Handle encoder rotation
    encoder.tick();
    long pos = encoder.getPosition();
    if (pos != lastEncoderPos) {
      subIndex = constrain(subIndex + (pos > lastEncoderPos ? 1 : -1), 0, submenuLength - 1);
      lastEncoderPos = pos;
      playBeep(25);
    }

    // Handle button press
    if (buttonPressed()) {
      playBeep(50);
      if (subIndex == 0) {  // Back
        inSubmenu = false;
        encoder.setPosition(index);
        drawMenu(mainMenu, mainMenuCount, index);
      } else {
        handleSubmenuSelection(index, subIndex);
      }
    }
    vTaskDelay(1);
  }
}


void handleMenuControl() {
  portENTER_CRITICAL(&encoderMux);
  long pos = encoderValue;
  portEXIT_CRITICAL(&encoderMux);

  // Handle encoder rotation
  if (pos != lastEncoderPos) {
    if (pos > lastEncoderPos) {
      menuIndex[0] = constrain(menuIndex[0] + 1, 0, mainMenuCount - 1);
    } else {
      menuIndex[0] = constrain(menuIndex[0] - 1, 0, mainMenuCount - 1);
    }
    lastEncoderPos = pos;
    playBeep(30);
    drawMenu(mainMenu, mainMenuCount, menuIndex[0]);
  }

  // Handle button press
  if (buttonPressed()) {
    playBeep(60);
    processSelection(menuIndex[0]);
  }
}

// 3. Display Functions
void drawMenu(const char* items[], int len, int index) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_t0_11_tr);

  // Header bar
  int16_t strWidth;
  int16_t x;
  strWidth = u8g2.getStrWidth("Menu Control");
  x = (128 - strWidth) / 2;

  u8g2.setDrawColor(1);
  u8g2.drawBox(0, 0, 128, 12);
  u8g2.setDrawColor(0);
  u8g2.drawStr(x, 10, "Menu Control");
  u8g2.setDrawColor(1);

  // Window calculation (3 visible rows)
  int start = index - 1;
  if (start < 0) start = 0;
  int end = start + 3;
  if (end > len) end = len;

  // Draw menu items
  for (int i = start; i < end; i++) {
    int row = i - start;
    int yBox = 13 + row * 14;   // box top position
    int yText = 24 + row * 14;  // text baseline

    if (i == index) {
      // Selected item: inverted colors with box
      u8g2.setDrawColor(1);
      u8g2.drawBox(0, yBox, 128, 14);
      u8g2.setDrawColor(0);
      u8g2.drawStr(2, yText, items[i]);
      u8g2.setDrawColor(1);
    } else {
      // Unselected item: normal text
      u8g2.drawStr(2, yText, items[i]);
    }
  }

  u8g2.sendBuffer();
}

void showConfirmDialog(const char* message, bool& value) {
  bool dialogActive = true;
  int selection = 0;
  encoder.setPosition(0);

  while (dialogActive) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_7x14B_tr);
    u8g2.drawStr(0, 12, "Confirm");
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 28, message);

    // Draw Yes/No options
    if (selection == 0) u8g2.drawStr(0, 45, ">");
    u8g2.drawStr(10, 45, "Yes");
    if (selection == 1) u8g2.drawStr(0, 57, ">");
    u8g2.drawStr(10, 57, "No");
    u8g2.sendBuffer();

    encoder.tick();
    long pos = encoder.getPosition();
    if (pos != lastEncoderPos) {
      selection = pos % 2;
      lastEncoderPos = pos;
      playBeep(25);
    }

    if (buttonPressed()) {
      playBeep(50);
      value = (selection == 0);
      dialogActive = false;
    }
  }
}
bool showSaveConfirmDialog() {
  bool dialogActive = true;
  int selection = 0;
  encoder.setPosition(0);

  while (dialogActive) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_7x14B_tr);
    u8g2.drawStr(0, 12, "Save Changes?");

    u8g2.setFont(u8g2_font_6x12_tf);
    if (selection == 0) u8g2.drawStr(0, 45, ">");
    u8g2.drawStr(10, 45, "Yes");
    if (selection == 1) u8g2.drawStr(0, 57, ">");
    u8g2.drawStr(10, 57, "No");
    u8g2.sendBuffer();

    encoder.tick();
    long pos = encoder.getPosition();
    if (pos != lastEncoderPos) {
      selection = pos % 2;
      lastEncoderPos = pos;
      playBeep(25);
    }

    if (buttonPressed()) {
      playBeep(50);
      return (selection == 0);
    }
  }
  return false;
}

// 4. Settings Management
void loadSettings() {
  EEPROM.begin(512);  // Initialize EEPROM first

  // Check if settings are valid using flag
  uint8_t validFlag = EEPROM.read(SETTINGS_FLAG_ADDR);
  uint8_t version = EEPROM.read(SETTINGS_VERSION_ADDR);

  if (validFlag != SETTINGS_VALID_FLAG || version != SETTINGS_VERSION) {
    Serial.println("First time setup or invalid settings - loading defaults");

    // First time or invalid settings - set defaults
    settings.nodeRedEnabled = false;
    strcpy(settings.nodeRedIP, "192.168.0.1:1880");
    settings.buzzerEnabled = true;
    settings.startupTime = 60;
    settings.mfdPerChannel = 4.0f;
    settings.targetFactor = 0.9;
    settings.voltHigh = 250.0f;
    settings.voltLow = 150.0f;
    settings.powerHigh = 5000.0f;
    settings.powerLow = 100.0f;
    settings.freqHigh = 51.0f;
    settings.freqLow = 49.0f;
    settings.maxkWh = 1;

    settings.highVoltageAlert = false;
    settings.lowVoltageAlert = false;
    settings.highPowerAlert = false;
    settings.lowPowerAlert = false;
    settings.highFreqAlert = false;
    settings.lowFreqAlert = false;

    settings.highVProtec = false;
    settings.LowVProtec = false;
    settings.highPProtec = false;
    settings.highHzProtec = false;
    settings.lowHzProtec = false;
    settings.afpcEnabled = false;

    settings.mainOutput = true;
    settings.kWhSaver = false;
    settings.totalDays = 0.0;

    settings.afpcLowWatt = 100;    // Default low power threshold
    settings.afpcHighWatt = 1000;  // Default high power threshold

    settings.espShareInterval = 2;  // 1.5 second
    settings.nodeRedInterval = 30;  // 30 seconds
    settings.pzemInterval = 1;      // 1 second
    settings.alertInterval = 180;   // 3 minutes
    settings.protectionDelay = 1200;

    // Save defaults and validation flag
    EEPROM.write(SETTINGS_FLAG_ADDR, SETTINGS_VALID_FLAG);
    EEPROM.write(SETTINGS_VERSION_ADDR, SETTINGS_VERSION);
    EEPROM.put(SETTINGS_DATA_ADDR, settings);
    EEPROM.commit();

    Serial.println("Default settings saved");
  } else {
    // Load existing settings
    EEPROM.get(SETTINGS_DATA_ADDR, settings);
    Serial.println("Existing settings loaded");
  }
}

// Also update saveSettings() function
void saveSettings() {
  EEPROM.write(SETTINGS_FLAG_ADDR, SETTINGS_VALID_FLAG);
  EEPROM.put(SETTINGS_DATA_ADDR, settings);
  if (EEPROM.commit()) {
    Serial.println("Settings saved successfully");
  } else {
    Serial.println("Error saving settings!");
  }
}

void editNodeRedIP() {
  bool editing = true;
  int segment = 0;                             // Current IP segment (0-3) and port (4)
  uint8_t ipSegments[4] = { 192, 168, 0, 1 };  // Default segments
  int port = 1880;                             // Default port

  // Parse current IP and port
  char currentIP[32];
  strcpy(currentIP, settings.nodeRedIP);
  char* portStr = strchr(currentIP, ':');
  if (portStr) {
    *portStr = '\0';  // Split IP and port
    port = atoi(portStr + 1);
    sscanf(currentIP, "%d.%d.%d.%d", &ipSegments[0], &ipSegments[1],
           &ipSegments[2], &ipSegments[3]);
  }

  // First show current IP
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 24, "Current IP:");
  u8g2.drawStr(0, 36, settings.nodeRedIP);
  u8g2.drawStr(0, 55, "Hold to edit >");
  u8g2.sendBuffer();
  delay(3000);


  if (!buttonPressed()) return;  // Exit if no confirmation to edit

  while (editing) {
    encoder.setPosition(segment < 4 ? ipSegments[segment] : port);

    while (!buttonPressed()) {  // Edit current segment until button press
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_7x14B_tr);
      u8g2.drawStr(0, 12, "Edit IP Address");

      // Show IP segments with cursor
      char ipStr[32];
      snprintf(ipStr, 32, "%d.%d.%d.%d:%d",
               ipSegments[0], ipSegments[1], ipSegments[2], ipSegments[3], port);
      u8g2.setFont(u8g2_font_6x12_tf);
      u8g2.drawStr(0, 35, ipStr);

      // Show cursor under current segment
      int cursorPos = 0;
      for (int i = 0; i < segment; i++) {
        cursorPos += (i < 4 ? (ipSegments[i] < 100 ? (ipSegments[i] < 10 ? 1 : 2) : 3) + 1
                            : 1);
      }
      u8g2.drawStr(cursorPos * 6, 45, "^");

      encoder.tick();
      long pos = encoder.getPosition();
      if (pos != lastEncoderPos) {
        if (segment < 4) {
          ipSegments[segment] = constrain(pos, 0, 255);
        } else {
          port = constrain(pos, 1, 9999);
        }
        lastEncoderPos = pos;
        playBeep(25);
      }

      u8g2.drawStr(0, 55, "Click for next");
      u8g2.sendBuffer();
    }

    segment++;
    if (segment > 4) {  // All segments edited
      if (showSaveConfirmDialog()) {
        snprintf(settings.nodeRedIP, 32, "%d.%d.%d.%d:%d",
                 ipSegments[0], ipSegments[1], ipSegments[2], ipSegments[3], port);
        saveSettings();
        showMessage("IP Saved!");
      } else {
        showMessage("Cancelled!");
      }
      editing = false;
    }
    delay(200);  // Debounce
  }
}


// Firebase EEPROM Functions
void loadFirebaseSettings() {
  EEPROM.get(FB_EEPROM_ADDR, fbSettings);

  // Check if valid, if not set defaults
  if (fbSettings.validFlag != SETTINGS_VALID_FLAG) {
    Serial.println("First time Firebase setup - loading defaults");
    fbSettings.enabled = false;
    strcpy(fbSettings.host, "");
    strcpy(fbSettings.auth, "");
    fbSettings.dataInterval = 10;  // Default 10 seconds
    fbSettings.validFlag = SETTINGS_VALID_FLAG;
    saveFirebaseSettings();
  }
}

void saveFirebaseSettings() {
  fbSettings.validFlag = SETTINGS_VALID_FLAG;
  EEPROM.put(FB_EEPROM_ADDR, fbSettings);
  if (EEPROM.commit()) {
    Serial.println("Firebase settings saved");
  } else {
    Serial.println("Error saving Firebase settings!");
  }
}

void resetFirebaseSettings() {
  fbSettings.enabled = false;
  strcpy(fbSettings.host, "");
  strcpy(fbSettings.auth, "");
  fbSettings.dataInterval = 300;
  saveFirebaseSettings();
}

// Add new function to test Firebase connection with one upload
// Modified testFirebaseConnection() to include output
bool testFirebaseConnection() {
  if (strlen(fbSettings.host) == 0 || strlen(fbSettings.auth) == 0) {
    lastFirebaseResponse = "No credentials";
    return false;
  }

  // Get current readings snapshot
  portENTER_CRITICAL(&measureMux);
  Measurements current = readings;
  portEXIT_CRITICAL(&measureMux);

  // Build Firebase URL
  String url = String(fbSettings.host);
  if (!url.endsWith("/")) url += "/";
  url += "energy_data.json?auth=" + String(fbSettings.auth);

  // Create test JSON payload
  StaticJsonDocument<512> doc;
  doc["test"] = "connection_test";
  doc["timestamp"] = millis();
  doc["voltage"] = String(current.voltage, 1);
  doc["current"] = String(current.current, 2);
  doc["power"] = String(current.power, 1);
  doc["energy"] = String(current.energy, 2);
  doc["frequency"] = String(current.frequency, 2);
  doc["pf"] = String(current.pf, 2);

  String jsonPayload;
  serializeJson(doc, jsonPayload);

  HTTPClient http;
  http.begin(url);
  http.setTimeout(10000);
  http.addHeader("Content-Type", "application/json");

  Serial.println("Testing Firebase connection...");
  int httpCode = http.sendRequest("PATCH", jsonPayload);

  bool success = false;
  if (httpCode > 0) {
    if (httpCode == HTTP_CODE_OK || httpCode == 200) {
      firebaseConnected = true;
      lastFirebaseResponse = "Success";
      Serial.println("Firebase test: SUCCESS");
      success = true;
    } else {
      firebaseConnected = false;
      lastFirebaseResponse = "HTTP " + String(httpCode);
      Serial.printf("Firebase test: HTTP error %d\n", httpCode);
    }
  } else {
    firebaseConnected = false;
    lastFirebaseResponse = http.errorToString(httpCode);
    Serial.printf("Firebase test: %s\n", lastFirebaseResponse.c_str());
  }

  http.end();
  return success;
}

// Firebase Web Portal
void startFirebaseConfigPortal() {
  if (WiFi.status() != WL_CONNECTED) {
    showMessage("WiFi Required!");
    return;
  }

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "Firebase Setup");
  u8g2.drawStr(0, 24, "Server Started");
  u8g2.drawStr(0, 36, "IP:");
  u8g2.drawStr(0, 48, WiFi.localIP().toString().c_str());
  u8g2.drawStr(0, 60, "Timeout: 180s");
  u8g2.sendBuffer();

  // Reset flags
  fbCredentialsSaved = false;

  // Setup Firebase config route (reuse existing server)
  AsyncWebHandler* firebaseHandler = &server.on("/firebase", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(SPIFFS, "/firebaseIndex.html", "text/html");
  });

  AsyncWebHandler* saveHandler = &server.on("/firebase/save", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (request->hasParam("url", true) && request->hasParam("token", true)) {
      String firebaseUrl = request->getParam("url", true)->value();
      String firebaseToken = request->getParam("token", true)->value();

      // Validate and save
      if (firebaseUrl.length() < 128 && firebaseToken.length() < 128) {
        strncpy(fbSettings.host, firebaseUrl.c_str(), 127);
        strncpy(fbSettings.auth, firebaseToken.c_str(), 127);
        fbSettings.host[127] = '\0';
        fbSettings.auth[127] = '\0';
        saveFirebaseSettings();

        fbCredentialsSaved = true;

        request->send(200, "text/plain", "SAVED");
        Serial.println("Firebase credentials saved - testing connection...");
      } else {
        request->send(400, "text/plain", "INVALID");
      }
    } else {
      request->send(400, "text/plain", "MISSING");
    }
  });

  // Wait for 180 seconds or button press
  unsigned long startTime = millis();
  const unsigned long timeout = 180000;

  while ((millis() - startTime) < timeout) {
    unsigned long remaining = (timeout - (millis() - startTime)) / 1000;

    char timeStr[16];
    snprintf(timeStr, 16, "Time: %lus", remaining);
    clearLCD(0, 52, 128, 10);
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 60, timeStr);
    u8g2.sendBuffer();

    // Test connection after credentials saved
    if (fbCredentialsSaved) {
      u8g2.clearBuffer();
      u8g2.drawStr(0, 12, "Testing Firebase...");
      u8g2.sendBuffer();
      delay(500);

      // Perform one-time upload test
      bool testSuccess = testFirebaseConnection();

      if (testSuccess) {
        u8g2.clearBuffer();
        u8g2.drawStr(0, 24, "Connection OK!");
        u8g2.drawStr(0, 36, "Data uploaded!");
        u8g2.sendBuffer();
        delay(2000);
      } else {
        u8g2.clearBuffer();
        u8g2.drawStr(0, 24, "Connection Failed!");
        u8g2.drawStr(0, 36, lastFirebaseResponse.c_str());
        u8g2.drawStr(0, 48, "Check credentials");
        u8g2.sendBuffer();
        delay(3000);
      }
      break;  // Exit loop after test
    }

    // Check for button press to exit early
    if (buttonPressed()) {
      break;
    }

    yield();
    delay(500);
  }

  // **IMPORTANT: Remove the Firebase config routes to prevent misuse**
  server.removeHandler(firebaseHandler);
  server.removeHandler(saveHandler);

  // Show cleanup message
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 24, "Config Portal");
  u8g2.drawStr(0, 36, "Closed!");
  u8g2.sendBuffer();
  delay(1000);

  if (!fbCredentialsSaved) {
    showMessage("Timeout/Cancelled", 2000);
  }
}

// Helper: write output state to Firebase (1/0)
bool writeOutputToFirebase(bool state) {
  if (!fbSettings.enabled || strlen(fbSettings.host) == 0) return false;
  if (WiFi.status() != WL_CONNECTED) return false;

  String url = String(fbSettings.host);
  if (!url.endsWith("/")) url += "/";
  url += "energy_data/output.json?auth=" + String(fbSettings.auth);

  HTTPClient http;
  http.begin(url);
  http.setTimeout(5000);
  http.addHeader("Content-Type", "application/json");

  String payload = state ? "1" : "0";  // number, not string
  int httpCode = http.PUT(payload);
  bool ok = (httpCode == 200 || httpCode == HTTP_CODE_OK);
  if (!ok) Serial.printf("[FB] write output failed: %d\n", httpCode);
  http.end();
  return ok;
}

// Helper: read output state from Firebase ONCE (used only in sendDataToFirebase)
void readOutputFromFirebase() {
  if (!fbSettings.enabled || strlen(fbSettings.host) == 0) return;
  if (WiFi.status() != WL_CONNECTED) return;

  String url = String(fbSettings.host);
  if (!url.endsWith("/")) url += "/";
  url += "energy_data/output.json?auth=" + String(fbSettings.auth);

  HTTPClient http;
  http.begin(url);
  http.setTimeout(4000);
  int httpCode = http.GET();

  if (httpCode == 200 || httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    payload.trim();
    int val = -1;
    if (payload == "1" || payload.equalsIgnoreCase("true")) val = 1;
    else if (payload == "0" || payload.equalsIgnoreCase("false")) val = 0;

    if (val == 0 || val == 1) {
      bool newState = (val == 1);
      if (settings.mainOutput != newState) {
        settings.mainOutput = newState;
        saveSettings();
        digitalWrite(MAIN_RELAY, settings.mainOutput ? LOW : HIGH);  // active-low
        Serial.printf("[FB] pulled output=%d and applied\n", val);
      }
    }
  } else {
    Serial.printf("[FB] read output failed: %d\n", httpCode);
  }
  http.end();
}

// Send data to Firebase
void sendDataToFirebase() {
  // Remove static timing check since task handles interval
  if (!fbSettings.enabled || strlen(fbSettings.host) == 0) return;
  if (WiFi.status() != WL_CONNECTED) return;

  // Read output from Firebase only at this moment (lazy pull)
  readOutputFromFirebase();

  // Get snapshot with mutex
  portENTER_CRITICAL(&measureMux);
  Measurements current = readings;
  portEXIT_CRITICAL(&measureMux);

  // Build Firebase URL
  String url = String(fbSettings.host);
  if (!url.endsWith("/")) url += "/";
  url += "energy_data.json?auth=" + String(fbSettings.auth);

  // Create JSON payload
  StaticJsonDocument<512> doc;
  doc["timestamp"] = millis();
  doc["voltage"] = String(current.voltage, 1);
  doc["current"] = String(current.current, 2);
  doc["power"] = String(current.power, 1);
  doc["energy"] = String(current.energy, 2);
  doc["frequency"] = String(current.frequency, 2);
  doc["pf"] = String(current.pf, 2);
  doc["uptime"] = String(current.uptime, 1);
  doc["totalDays"] = String(current.totalDays, 1);
  doc["output"] = settings.mainOutput ? 1 : 0;

  String jsonPayload;
  serializeJson(doc, jsonPayload);

  // Use HTTPClient (safe for Core 0)
  HTTPClient http;
  http.begin(url);
  http.setTimeout(5000);
  http.addHeader("Content-Type", "application/json");

  Serial.println("[Firebase Core 0] Sending data...");
  // ✅ Use PATCH to merge data
  int httpCode = http.sendRequest("PATCH", jsonPayload);

  if (httpCode > 0) {
    if (httpCode == HTTP_CODE_OK || httpCode == 200) {
      firebaseConnected = true;
      lastFirebaseResponse = "Success";
      Serial.println("[Firebase Core 0] Data sent successfully");
    } else {
      firebaseConnected = false;
      lastFirebaseResponse = "HTTP " + String(httpCode);
      Serial.printf("[Firebase Core 0] HTTP error %d\n", httpCode);
    }
  } else {
    firebaseConnected = false;
    lastFirebaseResponse = http.errorToString(httpCode);
    Serial.printf("[Firebase Core 0] Connection error - %s\n", lastFirebaseResponse.c_str());
  }

  http.end();
}

// 8. Others Functions
void sendDataToNodeRed() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < (settings.nodeRedInterval * 1000)) return;
  lastUpdate = millis();

  if (!settings.nodeRedEnabled || WiFi.status() != WL_CONNECTED) {
    return;  // Exit if Node-RED is disabled or WiFi not connected
  }

  HTTPClient http;

  // Get IP and port from settings
  char ip[32];
  strcpy(ip, settings.nodeRedIP);
  char* portStr = strchr(ip, ':');
  if (!portStr) return;  // Invalid format
  *portStr = '\0';
  String url = "http://" + String(ip) + ":" + String(portStr + 1) + "/pzem-data";

  http.begin(url);
  http.setTimeout(5000);
  http.addHeader("Content-Type", "application/json");

  // Get thread-safe copy of measurements
  portENTER_CRITICAL(&measureMux);
  Measurements current = readings;
  portEXIT_CRITICAL(&measureMux);

  // Create JSON payload manually
  String jsonPayload = "{";
  jsonPayload += "\"voltage\":" + String(current.voltage, 1) + ",";
  jsonPayload += "\"current\":" + String(current.current, 2) + ",";
  jsonPayload += "\"power\":" + String(current.power, 1) + ",";
  jsonPayload += "\"energy\":" + String(current.energy, 1) + ",";
  jsonPayload += "\"pf\":" + String(current.pf, 2) + ",";
  jsonPayload += "\"frequency\":" + String(current.frequency, 1) + ",";
  jsonPayload += "\"uptime\":" + String(current.uptime, 1) + ",";
  jsonPayload += "\"totalDays\":" + String(current.totalDays, 1);
  jsonPayload += "}";

  int httpResponseCode = http.POST(jsonPayload);

  if (httpResponseCode > 0) {
    nodeRedConnected = true;
    lastNodeRedResponse = String(httpResponseCode);
  } else {
    nodeRedConnected = false;
    lastNodeRedResponse = http.errorToString(httpResponseCode);
  }

  http.end();
}

void shareData() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < (settings.espShareInterval * 1000)) return;
  lastCheck = millis();

  StaticJsonDocument<1024> doc;

  // Take mutex before reading values
  portENTER_CRITICAL(&measureMux);
  Measurements current = readings;  // Take a snapshot
  portEXIT_CRITICAL(&measureMux);

  // Use the snapshot values
  doc["v"] = String(current.voltage, 1);
  doc["i"] = String(current.current, 2);
  doc["p"] = String(current.power, 1);
  doc["e"] = String(current.energy, 2);
  doc["f"] = String(current.frequency, 2);
  doc["pf"] = String(current.pf, 2);
  doc["up"] = String(current.uptime, 1);
  doc["d"] = String(kwhDays, 1);

  // All threshold values & settings
  doc["vh"] = String(settings.voltHigh, 1);
  doc["vl"] = String(settings.voltLow, 1);
  doc["ph"] = String(settings.powerHigh, 1);
  doc["pl"] = String(settings.powerLow, 1);
  doc["fh"] = String(settings.freqHigh, 2);
  doc["fl"] = String(settings.freqLow, 2);
  doc["tpf"] = String(settings.targetFactor, 2);
  doc["mfd"] = String(settings.mfdPerChannel, 2);
  doc["max"] = settings.maxkWh;
  doc["st"] = settings.startupTime;
  doc["pfl"] = settings.afpcLowWatt;
  doc["pfh"] = settings.afpcHighWatt;

  // Interval Values
  doc["esi"] = settings.espShareInterval;
  doc["nri"] = settings.nodeRedInterval;
  doc["pzi"] = settings.pzemInterval;
  doc["sai"] = settings.alertInterval;

  // Status flags
  doc["hva"] = settings.highVoltageAlert;
  doc["lva"] = settings.lowVoltageAlert;
  doc["hpa"] = settings.highPowerAlert;
  doc["lpa"] = settings.lowPowerAlert;
  doc["hfa"] = settings.highFreqAlert;
  doc["lfa"] = settings.lowFreqAlert;

  // Protection flags
  doc["hvp"] = settings.highVProtec;
  doc["lvp"] = settings.LowVProtec;
  doc["hpp"] = settings.highPProtec;
  doc["hhp"] = settings.highHzProtec;
  doc["lhp"] = settings.lowHzProtec;

  // Feature flags
  doc["afpc"] = settings.afpcEnabled;
  doc["out"] = settings.mainOutput;
  doc["kwhs"] = settings.kWhSaver;

  // Send with start/end markers
  espSerial.print('S');
  serializeJson(doc, espSerial);
  espSerial.print('E');
}


void firebaseTaskFunction(void* parameter) {
  Serial.println("Firebase Task started on Core 0");

  // Initial delay to let main core stabilize
  vTaskDelay(5000 / portTICK_PERIOD_MS);

  for (;;) {  // Infinite loop
    // Only run if Firebase is enabled and WiFi connected
    if (fbSettings.enabled && strlen(fbSettings.host) > 0 && WiFi.status() == WL_CONNECTED) {
      sendDataToFirebase();
    }

    // Wait for the configured interval (non-blocking)
    vTaskDelay((fbSettings.dataInterval * 1000) / portTICK_PERIOD_MS);
  }
}


// 13. Core Functions
void setup() {
  // 1. Initialize Serial Communications
  Serial.begin(115200);
  espSerial.begin(115200, SERIAL_8N1, ESP_COMM_RX, ESP_COMM_TX);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);  //for PZEM

  // 2. Initialize Hardware First!
  pinMode(MAIN_RELAY, OUTPUT);
  pinMode(PZEM_RELAY, OUTPUT);
  pinMode(RESET_BTN, INPUT);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(MAIN_RELAY, HIGH);
  digitalWrite(PZEM_RELAY, HIGH);

  // 3. Initialize I2C and Display
  Wire.begin(22, 23);
  u8g2.begin();

  // 4. Show Welcome Message
  welcomeMsg();
  delay(3000);

  // 5. Stability Wait Period
  // const int WAIT_TIME = 15;
  // for (int i = WAIT_TIME; i > 0; i--) {
  //   u8g2.clearBuffer();
  //   u8g2.setFont(u8g2_font_6x12_tf);
  //   u8g2.drawStr(0, 24, "Wait for stability...");
  //   char timeStr[16];
  //   snprintf(timeStr, sizeof(timeStr), "Time: %d sec", i);
  //   u8g2.drawStr(0, 48, timeStr);
  //   u8g2.sendBuffer();
  //   delay(1000);
  // }

  // Double beep after stability
  digitalWrite(BUZZER_PIN, HIGH);
  delay(300);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(300);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);

  // 6. Initialize Components
  // Hardware
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "Hardware Init...");
  u8g2.sendBuffer();
  delay(300);

  // Encoder
  u8g2.drawStr(0, 24, "Encoder Setup...");
  u8g2.sendBuffer();
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_DT), encoderISR, CHANGE);
  encoder.setPosition(0);
  delay(300);

  // Storage
  u8g2.drawStr(0, 36, "Storage Init...");
  u8g2.sendBuffer();
  EEPROM.begin(512);
  if (!SPIFFS.begin(true)) {
    u8g2.drawStr(0, 48, "SPIFFS Error!");
  } else {
    u8g2.drawStr(0, 48, "SPIFFS OK");
  }

  u8g2.sendBuffer();
  delay(300);

  preferences.begin("energy-meter", false);
  kwhDays = preferences.getFloat("days", 0.0);
  u8g2.drawStr(0, 60, "Days Fetched!");
  u8g2.sendBuffer();
  delay(300);


  // 7. Load Settings
  u8g2.clearBuffer();
  u8g2.drawStr(0, 12, "Loading Settings...");
  u8g2.sendBuffer();
  loadSettings();
  loadFirebaseSettings();
  PROTECTION_DELAY = settings.protectionDelay * 1000UL;
  delay(300);

  // 8. WiFi Connection
  delay(300);
  connectToSavedWiFi();

  // 9. Time Configuration
  configDateTime();
  delay(300);

  // 10. Start Web server (only if WiFi connected at startup)
  if (WiFi.status() == WL_CONNECTED) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 12, "Starting Server...");
    u8g2.sendBuffer();
    delay(300);

    setupWebServer();  // ← Use the new function

    u8g2.clearBuffer();
    u8g2.drawStr(0, 12, "Server Started!");
    u8g2.drawStr(0, 24, "IP Address:");
    u8g2.drawStr(0, 36, WiFi.localIP().toString().c_str());
    u8g2.sendBuffer();
    delay(2000);

  } else {
    u8g2.clearBuffer();
    u8g2.drawStr(0, 12, "WiFi Failed!");
    u8g2.drawStr(0, 24, "Will auto-start");
    u8g2.drawStr(0, 36, "when connected");
    u8g2.sendBuffer();
    delay(2000);
  }


  // 11. System Ready
  u8g2.clearBuffer();
  int16_t strWidth = u8g2.getStrWidth("System Ready!");
  int16_t x = (128 - strWidth) / 2;
  u8g2.drawStr(x, 36, "System Ready!");
  u8g2.sendBuffer();
  digitalWrite(PZEM_RELAY, LOW);
  delay(1000);

  // 12. Safety Check
  performSafetyCheck();

  // 13. Start Firebase Task on Core 0
  if (fbSettings.enabled && strlen(fbSettings.host) > 0) {
    // u8g2.clearBuffer();
    // u8g2.setFont(u8g2_font_6x12_tf);
    // u8g2.drawStr(0, 24, "Starting Firebase");
    // u8g2.drawStr(0, 36, "on Core 0...");
    // u8g2.sendBuffer();
    // delay(1000);

    xTaskCreatePinnedToCore(
      firebaseTaskFunction,  // Function to run
      "FirebaseTask",        // Task name
      8192,                  // Stack size (8KB - enough for HTTP)
      NULL,                  // Parameters
      1,                     // Priority (same as loop)
      &firebaseTask,         // Task handle
      0                      // Core 0 (separate from main loop on Core 1)
    );

    // u8g2.clearBuffer();
    // u8g2.drawStr(0, 24, "Firebase Task");
    // u8g2.drawStr(0, 36, "Started!");
    // u8g2.sendBuffer();
    // delay(500);
  }

  // 14. Show Main Display
  showMainDisplay();

  espSerial.println("HELLO");  // For Handshaking
  delay(50);
}

void loop() {
  updatePzemReadings();
  saveTotalDays();
  sendDataToNodeRed();
  shareData();
  checkWiFiAndStartServer();

  if (!menuActive && buttonPressed()) {
    menuActive = true;
    playBeep(50);
    encoder.setPosition(0);
    lastEncoderPos = 0;
    menuIndex[0] = 0;
    drawMenu(mainMenu, mainMenuCount, 0);
  }


  if (menuActive) {
    handleMenuControl();
  } else {
    displayValues();
    displayDateAndTime();
  }

  controlMainOutput();
  checkSafetyAlerts();
}

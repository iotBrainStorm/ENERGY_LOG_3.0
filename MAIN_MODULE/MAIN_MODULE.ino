#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <Preferences.h>
#include <HardwareSerial.h>
#include <RotaryEncoder.h>
#include <U8g2lib.h>
#include <PZEM004Tv30.h>
#include <ArduinoJson.h>
#include "time.h"

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define MAIN_RELAY 19  // Active LOW (LOW = Relay ON, HIGH = Relay OFF)
#define PZEM_RELAY 21  // Active LOW (LOW = Sensor Powered, HIGH = Power Cut)
#define BUZZER_PIN 13  // Active HIGH
#define RESET_BTN 35   // External push button (HIGH when pressed)
#define BOOT_BTN 0     // Built-in ESP32 BOOT button (LOW when pressed)

#define ENCODER_CLK 32
#define ENCODER_DT 33
#define ENCODER_SW 34  // Active LOW (external pull-up)

#define ESP_COMM_TX 25   // To RX (26) of Secondary ESP32
#define ESP_COMM_RX 26   // To TX (25) of Secondary ESP32
#define PZEM_COMM_TX 16  // To RX of PZEM-004T
#define PZEM_COMM_RX 17  // To TX of PZEM-004T

// ============================================================================
// HARDWARE INSTANCES
// ============================================================================
AsyncWebServer server(80);
HardwareSerial espSerial(1);
PZEM004Tv30 pzem(Serial2, PZEM_COMM_TX, PZEM_COMM_RX);
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, 18, 23, 5, 22);  // E=18, RW=23, RS=5, RST=22
RotaryEncoder encoder(ENCODER_CLK, ENCODER_DT);
Preferences preferences;

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE measureMux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================================
// DATA STRUCTURES
// ============================================================================
struct Measurements {
  float voltage = 0.0f;
  float current = 0.0f;
  float power = 0.0f;
  float energy = 0.0f;
  float frequency = 0.0f;
  float pf = 0.0f;
  float uptime = 0.0f;
  float totalDays = 0.0f;
  bool valid = false;
};

#define SETTINGS_MAGIC 0xE304
struct Settings {
  uint16_t magic;

  // Connectivity
  bool nodeRedEnabled;
  char nodeRedIP[32];
  bool mdnsEnabled;
  char mdnsHostname[32];
  long gmtOffset;

  // Alerts
  bool highVoltageAlert, lowVoltageAlert;
  bool highPowerAlert, lowPowerAlert;
  bool highFreqAlert, lowFreqAlert;

  // Protections
  bool highVProtec, LowVProtec;
  bool highPProtec;
  bool highHzProtec, lowHzProtec;

  // Features
  bool afpcEnabled;
  bool mainOutput;
  bool kWhSaver;
  bool buzzerEnabled;

  // Thresholds
  float voltHigh, voltLow;
  float powerHigh, powerLow;
  float freqHigh, freqLow;
  float targetFactor;
  float mfdPerChannel;
  int maxkWh;
  int afpcLowWatt;
  int afpcHighWatt;

  // Timing & Intervals (seconds)
  int startupTime;
  int espShareInterval;
  int nodeRedInterval;
  int pzemInterval;
  int alertInterval;
  int protectionDelay;
};

struct UserAccount {
  bool active;
  char username[20];
  char password[20];
  char role[10];  // "admin" or "user"
};

struct DailySchedule {
  bool active;
  uint8_t hour;
  uint8_t minute;
  bool turnOn;
  uint8_t daysMask;  // Bit 0 = Mon ... Bit 6 = Sun
  int lastTriggeredYDay;
};

struct FutureSchedule {
  bool active;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  bool turnOn;
};

#define MAX_USERS 6
#define MAX_DAILY_SCHEDULES 8
#define MAX_FUTURE_SCHEDULES 8

Measurements readings;
Settings settings;
UserAccount users[MAX_USERS];
DailySchedule dailySchedules[MAX_DAILY_SCHEDULES];
FutureSchedule futureSchedules[MAX_FUTURE_SCHEDULES];

// ============================================================================
// RUNTIME STATE VARIABLES
// ============================================================================
float kwhDays = 0.0f;
unsigned long lastResetEpoch = 0;         // ADD: Epoch timestamp (seconds) of last reset
char lastResetTimeStr[24] = "Not Reset";  // ADD: Formatted timestamp string for Web UI
const char* ntpServer = "pool.ntp.org";
bool ntpSynced = false;
unsigned long lastNtpSyncMillis = 0;

// Relay & Protection States
bool relayPhysicalOn = false;
bool shouldTurnOff = false;
bool protectionWasActive = false;
bool waitingForDelay = false;
unsigned long protectionClearedTime = 0;
char protectionStatusText[16] = "MAIN: ON";

// Quick Countdown Timer (Volatile)
bool quickTimerActive = false;
unsigned long quickTimerEndMillis = 0;

// Deferred Action Flags (Safe execution outside AsyncWebServer callbacks)
volatile bool pendingRestart = false;
unsigned long restartScheduledMillis = 0;
volatile bool pendingWiFiDisconnect = false;
volatile bool pendingWiFiForget = false;
volatile bool pendingWiFiPortal = false;
volatile bool pendingNtpSync = false;
volatile bool pendingMdnsRestart = false;
volatile bool resetKwhRequested = false;
volatile bool resetDaysRequested = false;
volatile bool pendingFactoryData = false;
volatile bool pendingFactoryAll = false;

// Node-RED Status
bool nodeRedConnected = false;
String lastNodeRedResponse = "---";

// Non-Blocking Buzzer State Machine
struct BuzzerState {
  bool active = false;
  bool pinHigh = false;
  int remainingBeeps = 0;
  unsigned long onDuration = 100;
  unsigned long offPause = 100;
  unsigned long lastTransition = 0;
} buzzer;

// PZEM Auto-Recovery State
uint8_t pzemFailCount = 0;
bool pzemPowerCycling = false;
unsigned long pzemRelayOffMillis = 0;

// Encoder & Button State
volatile long isrEncoderPos = 0;
long lastEncoderPos = 0;
unsigned long lastMenuActivityMillis = 0;
const unsigned long MENU_TIMEOUT_MS = 60000UL;

// Boot Button 5-Second Factory Reset State
bool bootBtnHolding = false;
unsigned long bootBtnPressStart = 0;
int lastBootCountdownSec = -1;

// Startup Safety Check State Machine
bool startupCheckCompleted = false;
int startupRemainingSec = 0;
unsigned long lastStartupTickMillis = 0;

// ============================================================================
// LCD MENU DEFINITIONS (Cleaned up, Firebase removed, mDNS added)
// ============================================================================
enum UIState {
  UI_STARTUP_SAFETY,
  UI_DASHBOARD,
  UI_MAIN_MENU,
  UI_SUB_MENU,
  UI_EDIT_INT,
  UI_EDIT_FLOAT,
  UI_CONFIRM,
  UI_INFO_VIEW,
  UI_TOAST
};

UIState uiState = UI_STARTUP_SAFETY;
UIState uiReturnState = UI_DASHBOARD;

const char* mainMenu[] = {
  "Back",            // 0
  "WiFi Setup",      // 1
  "mDNS Address",    // 2
  "Node-RED",        // 3
  "AFPC Control",    // 4
  "Startup Delay",   // 5
  "Intervals",       // 6
  "Thresholds",      // 7
  "Safety Alerts",   // 8
  "Safety Protec.",  // 9
  "Main Output",     // 10
  "Update Time",     // 11
  "Clear Data",      // 12
  "Factory Reset",   // 13
  "Restart Device",  // 14
  "About"            // 15
};
const int mainMenuCount = sizeof(mainMenu) / sizeof(mainMenu[0]);

const char* wifiSetupSubmenu[] = { "Back", "Connect Portal", "WiFi Status", "Disconnect", "Forget WiFi" };
const char* mdnsSubmenu[] = { "Back", "Enable/Disable", "Reset Default", "View Hostname" };
const char* nodeRedSubmenu[] = { "Back", "Enable/Disable", "Status" };
const char* afpcSubmenu[] = { "Back", "Enable/Disable", "Target PF", "MFD / Channel" };
const char* startupSubmenu[] = { "Back", "Delay Time" };
const char* updateIntervalSubmenu[] = { "Back", "ESP Sharing", "Node-RED", "Sensor Read", "Safety Alert", "Protec. Delay" };
const char* thresholdSubmenu[] = { "Back", "Voltage High", "Voltage Low", "Power High", "Power Low", "Freq. High", "Freq. Low", "Max kWh", "AFPC Low W", "AFPC High W" };
const char* safetyAlertSubmenu[] = { "Back", "High Voltage", "Low Voltage", "High Power", "Low Power", "Under Freq.", "Over Freq." };
const char* safetyProtectionSubmenu[] = { "Back", "High V Protec", "Low V Protec", "High P Protec", "High Hz Protec", "Low Hz Protec" };
const char* mainOutputSubmenu[] = { "Back", "Enable/Disable", "kWh Saver" };
const char* clearDataSubmenu[] = { "Back", "Clear kWh", "Last Billing" };
const char* factoryResetSubmenu[] = { "Back", "Reset Data", "Reset Settings" };

int currentMainMenuIdx = 0;
int currentSubMenuIdx = 0;
const char** activeSubmenuPtr = nullptr;
int activeSubmenuLen = 0;

// Non-blocking Editor Context
struct EditContext {
  const char* title;
  int* intTarget;
  float* floatTarget;
  int intTemp, intMin, intMax, intStep;
  float floatTemp, floatMin, floatMax, floatScale;
  char unit;
  bool confirmChoice;
  void (*onConfirm)();
} editCtx;

// Toast / Info View Context
char toastMsgLine1[24] = "";
char toastMsgLine2[24] = "";
char infoLine1[24] = "";
char infoLine2[24] = "";
char infoLine3[24] = "";
char infoLine4[24] = "";
unsigned long toastEndMillis = 0;

// ============================================================================
// UTILITY & NON-BLOCKING BUZZER FUNCTIONS
// ============================================================================
void triggerBeep(unsigned long durationMs, int count = 1, unsigned long pauseMs = 80) {
  if (!settings.buzzerEnabled && count == 1 && durationMs < 100) return;
  buzzer.onDuration = durationMs;
  buzzer.offPause = pauseMs;
  buzzer.remainingBeeps = count;
  buzzer.active = true;
  buzzer.pinHigh = true;
  buzzer.lastTransition = millis();
  digitalWrite(BUZZER_PIN, HIGH);
}

void updateBuzzer() {
  if (!buzzer.active) return;
  unsigned long now = millis();
  if (buzzer.pinHigh) {
    if (now - buzzer.lastTransition >= buzzer.onDuration) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzer.pinHigh = false;
      buzzer.lastTransition = now;
      buzzer.remainingBeeps--;
      if (buzzer.remainingBeeps <= 0) {
        buzzer.active = false;
      }
    }
  } else {
    if (now - buzzer.lastTransition >= buzzer.offPause) {
      digitalWrite(BUZZER_PIN, HIGH);
      buzzer.pinHigh = true;
      buzzer.lastTransition = now;
    }
  }
}

void showToastLCD(const char* l1, const char* l2 = "", unsigned long durationMs = 1200, UIState nextState = UI_MAIN_MENU) {
  strncpy(toastMsgLine1, l1, sizeof(toastMsgLine1) - 1);
  strncpy(toastMsgLine2, l2, sizeof(toastMsgLine2) - 1);
  toastEndMillis = millis() + durationMs;
  uiReturnState = nextState;
  uiState = UI_TOAST;
}

String getMacSuffix() {
  String mac = WiFi.macAddress();
  mac.replace(":", "");
  mac.toUpperCase();
  if (mac.length() >= 4) return mac.substring(mac.length() - 4);
  return "0000";
}

String getDefaultMdnsName() {
  return "energylog-" + getMacSuffix();
}

void setupMDNS() {
  MDNS.end();
  if (!settings.mdnsEnabled || WiFi.status() != WL_CONNECTED) return;
  String host = String(settings.mdnsHostname);
  host.trim();
  if (host.length() == 0) host = getDefaultMdnsName();
  if (MDNS.begin(host.c_str())) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("[mDNS] Responder started: http://%s.local\n", host.c_str());
  }
}

// ============================================================================
// ENCODER & BUTTON HANDLING (NON-BLOCKING)
// ============================================================================
void IRAM_ATTR encoderISR() {
  portENTER_CRITICAL_ISR(&encoderMux);
  encoder.tick();
  isrEncoderPos = encoder.getPosition();
  portEXIT_CRITICAL_ISR(&encoderMux);
}

long getEncoderPos() {
  portENTER_CRITICAL(&encoderMux);
  encoder.tick();
  long p = encoder.getPosition();
  portEXIT_CRITICAL(&encoderMux);
  return p;
}

void setEncoderPos(long p) {
  portENTER_CRITICAL(&encoderMux);
  encoder.setPosition(p);
  isrEncoderPos = p;
  lastEncoderPos = p;
  portEXIT_CRITICAL(&encoderMux);
}

bool checkEncoderButtonClick() {
  static bool lastState = HIGH;
  static unsigned long pressedTime = 0;
  static bool clickConsumed = false;

  bool current = digitalRead(ENCODER_SW);
  unsigned long now = millis();

  if (current == LOW && lastState == HIGH) {
    pressedTime = now;
    clickConsumed = false;
  } else if (current == LOW && !clickConsumed && (now - pressedTime >= 45)) {
    clickConsumed = true;
    lastState = current;
    lastMenuActivityMillis = now;
    return true;
  }
  lastState = current;
  return false;
}

// ============================================================================
// STORAGE: SETTINGS, USERS, SCHEDULES (PREFERENCES NVS)
// ============================================================================
void applyDefaultSettings() {
  settings.magic = SETTINGS_MAGIC;
  settings.nodeRedEnabled = false;
  strcpy(settings.nodeRedIP, "192.168.1.100:1880");
  settings.mdnsEnabled = true;
  strncpy(settings.mdnsHostname, getDefaultMdnsName().c_str(), 31);
  settings.mdnsHostname[31] = '\0';
  settings.gmtOffset = 19800;  // IST UTC+5:30

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
  settings.buzzerEnabled = true;

  settings.voltHigh = 250.0f;
  settings.voltLow = 180.0f;
  settings.powerHigh = 3000.0f;
  settings.powerLow = 100.0f;
  settings.freqHigh = 50.50f;
  settings.freqLow = 49.50f;
  settings.targetFactor = 0.90f;
  settings.mfdPerChannel = 4.0f;
  settings.maxkWh = 100;
  settings.afpcLowWatt = 50;
  settings.afpcHighWatt = 3000;

  settings.startupTime = 15;
  settings.espShareInterval = 2;
  settings.nodeRedInterval = 5;
  settings.pzemInterval = 1;
  settings.alertInterval = 15;
  settings.protectionDelay = 1200;
}

void saveSettings() {
  settings.magic = SETTINGS_MAGIC;
  preferences.putBytes("cfg", &settings, sizeof(Settings));
}

void loadSettings() {
  if (preferences.getBytesLength("cfg") == sizeof(Settings)) {
    preferences.getBytes("cfg", &settings, sizeof(Settings));
    if (settings.magic != SETTINGS_MAGIC) {
      applyDefaultSettings();
      saveSettings();
    }
  } else {
    applyDefaultSettings();
    saveSettings();
  }
  if (strlen(settings.mdnsHostname) == 0) {
    strncpy(settings.mdnsHostname, getDefaultMdnsName().c_str(), 31);
  }
  if (settings.startupTime < 5) {
    settings.startupTime = 15;  // Ensure startup safety check always runs
  }
}

void saveUsers() {
  preferences.putBytes("users", users, sizeof(users));
}

void loadUsers() {
  if (preferences.getBytesLength("users") == sizeof(users)) {
    preferences.getBytes("users", users, sizeof(users));
  } else {
    memset(users, 0, sizeof(users));
    users[0].active = true;
    strcpy(users[0].username, "esp");
    strcpy(users[0].password, "456456");
    strcpy(users[0].role, "admin");
    saveUsers();
  }
}

bool verifyAdminCreds(const String& u, const String& p) {
  for (int i = 0; i < MAX_USERS; i++) {
    if (users[i].active && u == users[i].username && p == users[i].password && String(users[i].role) == "admin") {
      return true;
    }
  }
  return false;
}

void saveSchedulesToNVS() {
  preferences.putBytes("dsched", dailySchedules, sizeof(dailySchedules));
  preferences.putBytes("fsched", futureSchedules, sizeof(futureSchedules));
}

void loadSchedulesFromNVS() {
  if (preferences.getBytesLength("dsched") == sizeof(dailySchedules)) {
    preferences.getBytes("dsched", dailySchedules, sizeof(dailySchedules));
  } else {
    memset(dailySchedules, 0, sizeof(dailySchedules));
  }
  if (preferences.getBytesLength("fsched") == sizeof(futureSchedules)) {
    preferences.getBytes("fsched", futureSchedules, sizeof(futureSchedules));
  } else {
    memset(futureSchedules, 0, sizeof(futureSchedules));
  }
}

// Record current NTP timestamp to NVS when Energy/Billing is reset
bool recordEnergyResetTimestamp() {
  struct tm ti;
  if (!ntpSynced || !getLocalTime(&ti, 0) || (ti.tm_year + 1900 < 2024)) {
    return false;
  }
  time_t nowEpoch;
  time(&nowEpoch);
  lastResetEpoch = (unsigned long)nowEpoch;
  strftime(lastResetTimeStr, sizeof(lastResetTimeStr), "%d/%m/%y %H:%M", &ti);

  preferences.putULong("rstEpoch", lastResetEpoch);
  preferences.putString("rstTime", lastResetTimeStr);
  kwhDays = 0.0f;
  return true;
}

// ============================================================================
// 5-SECOND BOOT BUTTON FACTORY RESET
// ============================================================================
void executeFullFactoryReset() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_7x14B_tr);
  u8g2.drawStr(10, 28, "FACTORY RESET!");
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(14, 46, "Formatting NVS...");
  u8g2.sendBuffer();

  digitalWrite(MAIN_RELAY, HIGH);  // Safe state: Relay OFF
  digitalWrite(BUZZER_PIN, HIGH);
  delay(800);
  digitalWrite(BUZZER_PIN, LOW);

  preferences.clear();
  applyDefaultSettings();
  saveSettings();
  kwhDays = 0.0f;
  lastResetEpoch = 0;
  strcpy(lastResetTimeStr, "Not Reset");

  WiFi.disconnect(true, true);
  ESP.restart();
}

void checkBootFactoryReset() {
  if (digitalRead(BOOT_BTN) == LOW) {
    unsigned long now = millis();
    if (!bootBtnHolding) {
      bootBtnHolding = true;
      bootBtnPressStart = now;
      lastBootCountdownSec = -1;
    } else {
      unsigned long elapsed = now - bootBtnPressStart;
      if (elapsed >= 5000UL) {
        executeFullFactoryReset();
      } else if (elapsed >= 500UL) {
        int remSec = 5 - (elapsed / 1000UL);
        if (remSec != lastBootCountdownSec) {
          lastBootCountdownSec = remSec;
          triggerBeep(60, 1, 0);
        }
      }
    }
  } else {
    bootBtnHolding = false;
    lastBootCountdownSec = -1;
  }
}

// ============================================================================
// SILENT BACKGROUND NTP & AUTOMATION SCHEDULER
// ============================================================================
void updateNtpSilently() {
  if (WiFi.status() != WL_CONNECTED) return;

  unsigned long now = millis();
  if (pendingNtpSync || !ntpSynced || (now - lastNtpSyncMillis >= 3600000UL)) {
    if (pendingNtpSync || lastNtpSyncMillis == 0 || (now - lastNtpSyncMillis >= 30000UL)) {
      configTime(settings.gmtOffset, 0, ntpServer, "time.nist.gov");
      lastNtpSyncMillis = now;
      pendingNtpSync = false;
    }
  }

  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 0)) {
    if (timeinfo.tm_year + 1900 >= 2024) {
      ntpSynced = true;
    }
  }
}

void checkTimersAndSchedules() {
  // 1. Volatile Quick Countdown Timer
  if (quickTimerActive) {
    if ((long)(millis() - quickTimerEndMillis) >= 0) {
      quickTimerActive = false;
      settings.mainOutput = false;
      saveSettings();
      triggerBeep(200, 2, 100);
    }
  }

  // 2. Persistent Daily & Future Schedules (checked once per second when NTP time is valid)
  static unsigned long lastSchedCheck = 0;
  if (millis() - lastSchedCheck < 1000UL) return;
  lastSchedCheck = millis();

  struct tm ti;
  if (!getLocalTime(&ti, 0) || (ti.tm_year + 1900 < 2024)) return;

  // Map tm_wday (0=Sun..6=Sat) to UI index (0=Mon..6=Sun)
  int uiDay = (ti.tm_wday == 0) ? 6 : (ti.tm_wday - 1);

  for (int i = 0; i < MAX_DAILY_SCHEDULES; i++) {
    if (dailySchedules[i].active && (dailySchedules[i].daysMask & (1 << uiDay)) && dailySchedules[i].hour == ti.tm_hour && dailySchedules[i].minute == ti.tm_min && dailySchedules[i].lastTriggeredYDay != ti.tm_yday) {
      dailySchedules[i].lastTriggeredYDay = ti.tm_yday;
      settings.mainOutput = dailySchedules[i].turnOn;
      if (settings.mainOutput) {
        protectionWasActive = false;
        waitingForDelay = false;
      }
      saveSettings();
      saveSchedulesToNVS();
      triggerBeep(120, 2, 80);
    }
  }

  bool futureChanged = false;
  for (int i = 0; i < MAX_FUTURE_SCHEDULES; i++) {
    if (futureSchedules[i].active && futureSchedules[i].year == (ti.tm_year + 1900) && futureSchedules[i].month == (ti.tm_mon + 1) && futureSchedules[i].day == ti.tm_mday && futureSchedules[i].hour == ti.tm_hour && futureSchedules[i].minute == ti.tm_min) {
      settings.mainOutput = futureSchedules[i].turnOn;
      if (settings.mainOutput) {
        protectionWasActive = false;
        waitingForDelay = false;
      }
      futureSchedules[i].active = false;
      futureChanged = true;
      saveSettings();
      triggerBeep(120, 2, 80);
    }
  }
  if (futureChanged) saveSchedulesToNVS();
}

// ============================================================================
// PZEM-004T READING
// ============================================================================
void updatePzemReadings() {
  // Do not turn on or poll PZEM until setup() finishes and enables PZEM_RELAY
  digitalWrite(PZEM_RELAY, LOW);

  unsigned long now = millis();
  static unsigned long lastPzemRead = 0;
  unsigned long intervalMs = max(1, settings.pzemInterval) * 1000UL;
  if (now - lastPzemRead < intervalMs) return;
  lastPzemRead = now;

  // Calculate exact elapsed days from last reset timestamp to present
  if (ntpSynced && lastResetEpoch > 0) {
    time_t nowEpoch;
    time(&nowEpoch);
    if (nowEpoch >= (time_t)lastResetEpoch) {
      kwhDays = (float)(nowEpoch - (time_t)lastResetEpoch) / 86400.0f;
    } else {
      kwhDays = 0.0f;
    }
  }

  float v = pzem.voltage();
  float c = pzem.current();
  float p = pzem.power();
  float e = pzem.energy();
  float hz = pzem.frequency();
  float pf = pzem.pf();
  float ut = now / 3600000.0f;

  bool sensorOk = !isnan(v) && !isnan(hz);

  portENTER_CRITICAL(&measureMux);
  readings.valid = sensorOk;
  readings.voltage = isnan(v) ? 0.0f : v;
  readings.current = isnan(c) ? 0.0f : c;
  readings.power = isnan(p) ? 0.0f : p;
  if (!isnan(e)) readings.energy = e;
  readings.frequency = isnan(hz) ? 0.0f : hz;
  readings.pf = isnan(pf) ? 0.0f : pf;
  readings.uptime = ut;
  readings.totalDays = kwhDays;
  portEXIT_CRITICAL(&measureMux);
}

// ============================================================================
// MAIN POWER OUTPUT RELAY & PROTECTION ENGINE (HIGH PRIORITY)
// ============================================================================
void controlMainOutput() {
  // Keep relay OFF during initial boot safety check
  if (!startupCheckCompleted) {
    digitalWrite(MAIN_RELAY, HIGH);
    relayPhysicalOn = false;
    return;
  }

  portENTER_CRITICAL(&measureMux);
  Measurements cur = readings;
  portEXIT_CRITICAL(&measureMux);

  unsigned long protDelayMs = (unsigned long)max(10, settings.protectionDelay) * 1000UL;

  // 1. Manual Output OFF
  if (!settings.mainOutput) {
    shouldTurnOff = true;
    digitalWrite(MAIN_RELAY, HIGH);
    relayPhysicalOn = false;
    strcpy(protectionStatusText, "MAIN: OFF");
    return;
  }

  // 2. kWh Energy Saver Cutoff
  if (settings.kWhSaver && cur.energy >= (float)settings.maxkWh) {
    shouldTurnOff = true;
    digitalWrite(MAIN_RELAY, HIGH);
    relayPhysicalOn = false;
    strcpy(protectionStatusText, "kWh SAVER");
    return;
  }

  // 3. Evaluate Protection Faults (Only trip on low V / low Hz if sensor is valid)
  bool faultActive = false;
  if (cur.valid) {
    if (settings.highVProtec && cur.voltage > settings.voltHigh) faultActive = true;
    if (settings.LowVProtec && cur.voltage < settings.voltLow) faultActive = true;
    if (settings.highPProtec && cur.power > settings.powerHigh) faultActive = true;
    if (settings.highHzProtec && cur.frequency > settings.freqHigh) faultActive = true;
    if (settings.lowHzProtec && cur.frequency < settings.freqLow) faultActive = true;
  }

  if (faultActive) {
    if (!protectionWasActive) {
      triggerBeep(250, 3, 100);  // Instant trip alarm
    }
    protectionWasActive = true;
    waitingForDelay = false;
    shouldTurnOff = true;
    digitalWrite(MAIN_RELAY, HIGH);  // Immediately cut power
    relayPhysicalOn = false;
    strcpy(protectionStatusText, "PRO. TRIG.");
    return;
  }

  // 4. Handle Protection Recovery Wait Timer
  if (protectionWasActive) {
    if (!waitingForDelay) {
      protectionClearedTime = millis();
      waitingForDelay = true;
    }

    unsigned long elapsed = millis() - protectionClearedTime;
    if (elapsed >= protDelayMs || digitalRead(RESET_BTN) == HIGH) {
      protectionWasActive = false;
      waitingForDelay = false;
      shouldTurnOff = false;
      digitalWrite(MAIN_RELAY, LOW);  // Engage Relay
      relayPhysicalOn = true;
      strcpy(protectionStatusText, "MAIN: ON");
      triggerBeep(150, 1, 0);
    } else {
      shouldTurnOff = true;
      digitalWrite(MAIN_RELAY, HIGH);
      relayPhysicalOn = false;
      unsigned long rem = protDelayMs - elapsed;
      int mins = rem / 60000UL;
      int secs = (rem % 60000UL) / 1000UL;
      snprintf(protectionStatusText, sizeof(protectionStatusText), "Wait:%02d:%02d", mins, secs);
    }
  } else {
    shouldTurnOff = false;
    digitalWrite(MAIN_RELAY, LOW);  // Engage Relay
    relayPhysicalOn = true;
    strcpy(protectionStatusText, "MAIN: ON");
  }
}

// ============================================================================
// INSTANT SAFETY ALERTS (0ms Edge Trigger + Periodic Repeat)
// ============================================================================
void checkSafetyAlerts() {
  if (!startupCheckCompleted) return;

  portENTER_CRITICAL(&measureMux);
  Measurements cur = readings;
  portEXIT_CRITICAL(&measureMux);

  if (!cur.valid) return;

  uint8_t currentFaultMask = 0;
  if (settings.highVoltageAlert && cur.voltage > settings.voltHigh) currentFaultMask |= (1 << 0);
  if (settings.lowVoltageAlert && cur.voltage < settings.voltLow) currentFaultMask |= (1 << 1);
  if (settings.highPowerAlert && cur.power > settings.powerHigh) currentFaultMask |= (1 << 2);
  if (settings.lowPowerAlert && cur.power < settings.powerLow) currentFaultMask |= (1 << 3);
  if (settings.highFreqAlert && cur.frequency > settings.freqHigh) currentFaultMask |= (1 << 4);
  if (settings.lowFreqAlert && cur.frequency < settings.freqLow) currentFaultMask |= (1 << 5);

  static uint8_t prevFaultMask = 0;
  static unsigned long lastAlertRepeat = 0;
  unsigned long now = millis();

  // Instant trigger if ANY new alert bit turns ON
  bool newFaultAppeared = (currentFaultMask & ~prevFaultMask) != 0;
  prevFaultMask = currentFaultMask;

  if (currentFaultMask != 0) {
    unsigned long repeatMs = (unsigned long)max(5, settings.alertInterval) * 1000UL;
    if (newFaultAppeared || (now - lastAlertRepeat >= repeatMs)) {
      lastAlertRepeat = now;
      triggerBeep(100, 3, 100);
    }
  }
}

// ============================================================================
// SECONDARY ESP32 SERIAL TELEMETRY & HANDSHAKE
// ============================================================================
void shareDataWithSecondary() {
  // Listen for handshake request from secondary if it rebooted
  while (espSerial.available()) {
    String line = espSerial.readStringUntil('\n');
    line.trim();
    if (line == "PING" || line == "HELLO") {
      espSerial.println("READY");
    }
  }

  static unsigned long lastShare = 0;
  unsigned long intervalMs = (unsigned long)max(1, settings.espShareInterval) * 1000UL;
  if (millis() - lastShare < intervalMs) return;
  lastShare = millis();

  portENTER_CRITICAL(&measureMux);
  Measurements cur = readings;
  portEXIT_CRITICAL(&measureMux);

  StaticJsonDocument<768> doc;
  doc["v"] = serialized(String(cur.voltage, 1));
  doc["i"] = serialized(String(cur.current, 2));
  doc["p"] = serialized(String(cur.power, 1));
  doc["e"] = serialized(String(cur.energy, 2));
  doc["f"] = serialized(String(cur.frequency, 2));
  doc["pf"] = serialized(String(cur.pf, 2));
  doc["up"] = serialized(String(cur.uptime, 1));
  doc["d"] = serialized(String(kwhDays, 1));
  doc["ok"] = cur.valid ? 1 : 0;

  doc["vh"] = serialized(String(settings.voltHigh, 1));
  doc["vl"] = serialized(String(settings.voltLow, 1));
  doc["ph"] = serialized(String(settings.powerHigh, 1));
  doc["pl"] = serialized(String(settings.powerLow, 1));
  doc["fh"] = serialized(String(settings.freqHigh, 2));
  doc["fl"] = serialized(String(settings.freqLow, 2));
  doc["tpf"] = serialized(String(settings.targetFactor, 2));
  doc["mfd"] = serialized(String(settings.mfdPerChannel, 2));
  doc["max"] = settings.maxkWh;
  doc["st"] = settings.startupTime;
  doc["pfl"] = settings.afpcLowWatt;
  doc["pfh"] = settings.afpcHighWatt;

  doc["esi"] = settings.espShareInterval;
  doc["nri"] = settings.nodeRedInterval;
  doc["pzi"] = settings.pzemInterval;
  doc["sai"] = settings.alertInterval;

  doc["hva"] = settings.highVoltageAlert;
  doc["lva"] = settings.lowVoltageAlert;
  doc["hpa"] = settings.highPowerAlert;
  doc["lpa"] = settings.lowPowerAlert;
  doc["hfa"] = settings.highFreqAlert;
  doc["lfa"] = settings.lowFreqAlert;

  doc["hvp"] = settings.highVProtec;
  doc["lvp"] = settings.LowVProtec;
  doc["hpp"] = settings.highPProtec;
  doc["hhp"] = settings.highHzProtec;
  doc["lhp"] = settings.lowHzProtec;

  doc["afpc"] = settings.afpcEnabled;
  doc["out"] = settings.mainOutput;
  doc["kwhs"] = settings.kWhSaver;

  espSerial.print('S');
  serializeJson(doc, espSerial);
  espSerial.print("E\n");
}

// ============================================================================
// NODE-RED STREAMING
// ============================================================================
void sendDataToNodeRed() {
  if (!settings.nodeRedEnabled || WiFi.status() != WL_CONNECTED) return;

  static unsigned long lastSend = 0;
  unsigned long intervalMs = (unsigned long)max(5, settings.nodeRedInterval) * 1000UL;
  if (millis() - lastSend < intervalMs) return;
  lastSend = millis();

  char addr[32];
  strncpy(addr, settings.nodeRedIP, sizeof(addr) - 1);
  addr[31] = '\0';
  char* colon = strchr(addr, ':');
  String url;
  if (colon) {
    *colon = '\0';
    url = "http://" + String(addr) + ":" + String(colon + 1) + "/pzem-data";
  } else {
    url = "http://" + String(addr) + ":1880/pzem-data";
  }

  portENTER_CRITICAL(&measureMux);
  Measurements cur = readings;
  portEXIT_CRITICAL(&measureMux);

  StaticJsonDocument<384> doc;
  doc["voltage"] = cur.voltage;
  doc["current"] = cur.current;
  doc["power"] = cur.power;
  doc["energy"] = cur.energy;
  doc["pf"] = cur.pf;
  doc["frequency"] = cur.frequency;
  doc["uptime"] = cur.uptime;
  doc["totalDays"] = cur.totalDays;  // Keep key name for InfluxDB compatibility!
  doc["mainOutput"] = settings.mainOutput ? 1 : 0;
  doc["resetTime"] = lastResetTimeStr;  // Optional metadata

  String payload;
  serializeJson(doc, payload);

  HTTPClient http;
  http.begin(url);
  http.setTimeout(1500);  // Short timeout to prevent loop lag
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(payload);
  if (code > 0 && code < 400) {
    nodeRedConnected = true;
    lastNodeRedResponse = "HTTP " + String(code);
  } else {
    nodeRedConnected = false;
    lastNodeRedResponse = "Err";
  }
  http.end();
}

// ============================================================================
// WIFI & WEBSERVER MANAGEMENT
// ============================================================================
void serveGzipOrRaw(AsyncWebServerRequest* request, const char* path, const char* mimeType) {
  String gzPath = String(path) + ".gz";
  if (SPIFFS.exists(gzPath)) {
    AsyncWebServerResponse* response = request->beginResponse(SPIFFS, gzPath, mimeType);
    response->addHeader("Content-Encoding", "gzip");
    response->addHeader("Cache-Control", "no-cache");
    request->send(response);
  } else if (SPIFFS.exists(path)) {
    AsyncWebServerResponse* response = request->beginResponse(SPIFFS, path, mimeType);
    response->addHeader("Cache-Control", "no-cache");
    request->send(response);
  } else {
    request->send(404, "text/plain", "File Not Found in SPIFFS");
  }
}

void setupWebServerRoutes() {
  // Static Pages (Supports both .gz and uncompressed .html)
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    serveGzipOrRaw(request, "/index.html", "text/html");
  });
  server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest* request) {
    serveGzipOrRaw(request, "/index.html", "text/html");
  });
  server.on("/config.html", HTTP_GET, [](AsyncWebServerRequest* request) {
    serveGzipOrRaw(request, "/config.html", "text/html");
  });
  server.on("/plug_icon.svg", HTTP_GET, [](AsyncWebServerRequest* request) {
    serveGzipOrRaw(request, "/plug_icon.svg", "image/svg+xml");
  });

  // Live Sensor Telemetry
  server.on("/sensor.json", HTTP_GET, [](AsyncWebServerRequest* request) {
    portENTER_CRITICAL(&measureMux);
    Measurements cur = readings;
    portEXIT_CRITICAL(&measureMux);

    StaticJsonDocument<512> doc;
    doc["voltage"] = String(cur.voltage, 1);
    doc["current"] = String(cur.current, 2);
    doc["power"] = String(cur.power, 1);
    doc["energy"] = String(cur.energy, 2);
    doc["frequency"] = String(cur.frequency, 2);
    doc["pf"] = String(cur.pf, 2);
    doc["uptime"] = String(cur.uptime, 1);
    doc["days"] = String(cur.totalDays, 1);
    doc["mainOutput"] = settings.mainOutput ? 1 : 0;
    doc["resetTime"] = lastResetTimeStr;   // ADD: Send last reset timestamp
    doc["ntpSynced"] = ntpSynced ? 1 : 0;  // ADD: Send NTP sync status

    String res;
    serializeJson(doc, res);
    request->send(200, "application/json", res);
  });

  // WiFi Status
  server.on("/wifi-status.json", HTTP_GET, [](AsyncWebServerRequest* request) {
    StaticJsonDocument<256> doc;
    bool conn = (WiFi.status() == WL_CONNECTED);
    doc["connected"] = conn;
    doc["ssid"] = conn ? WiFi.SSID() : "";
    doc["ip"] = conn ? WiFi.localIP().toString() : "---";
    doc["rssi"] = conn ? WiFi.RSSI() : 0;
    doc["mac"] = WiFi.macAddress();

    String res;
    serializeJson(doc, res);
    request->send(200, "application/json", res);
  });

  // All Settings JSON
  server.on("/settings.json", HTTP_GET, [](AsyncWebServerRequest* request) {
    StaticJsonDocument<1024> doc;
    doc["mac"] = WiFi.macAddress();
    doc["macSuffix"] = getMacSuffix();
    doc["mdnsEnabled"] = settings.mdnsEnabled;
    doc["mdnsHostname"] = settings.mdnsHostname;
    doc["gmtOffset"] = settings.gmtOffset;
    doc["mainOutput"] = settings.mainOutput;
    doc["kWhSaver"] = settings.kWhSaver;
    doc["afpcEnabled"] = settings.afpcEnabled;
    doc["targetFactor"] = settings.targetFactor;
    doc["mfdPerChannel"] = settings.mfdPerChannel;
    doc["nodeRedEnabled"] = settings.nodeRedEnabled;
    doc["nodeRedIP"] = settings.nodeRedIP;
    doc["voltHigh"] = (int)settings.voltHigh;
    doc["voltLow"] = (int)settings.voltLow;
    doc["powerHigh"] = (int)settings.powerHigh;
    doc["powerLow"] = (int)settings.powerLow;
    doc["freqHigh"] = settings.freqHigh;
    doc["freqLow"] = settings.freqLow;
    doc["maxkWh"] = settings.maxkWh;
    doc["afpcLowWatt"] = settings.afpcLowWatt;
    doc["afpcHighWatt"] = settings.afpcHighWatt;
    doc["highVoltageAlert"] = settings.highVoltageAlert;
    doc["lowVoltageAlert"] = settings.lowVoltageAlert;
    doc["highPowerAlert"] = settings.highPowerAlert;
    doc["lowPowerAlert"] = settings.lowPowerAlert;
    doc["lowFreqAlert"] = settings.lowFreqAlert;
    doc["highFreqAlert"] = settings.highFreqAlert;
    doc["highVProtec"] = settings.highVProtec;
    doc["LowVProtec"] = settings.LowVProtec;
    doc["highPProtec"] = settings.highPProtec;
    doc["highHzProtec"] = settings.highHzProtec;
    doc["lowHzProtec"] = settings.lowHzProtec;
    doc["espShareInterval"] = settings.espShareInterval;
    doc["nodeRedInterval"] = settings.nodeRedInterval;
    doc["pzemInterval"] = settings.pzemInterval;
    doc["alertInterval"] = settings.alertInterval;
    doc["protectionDelay"] = settings.protectionDelay;
    doc["startupTime"] = settings.startupTime;

    String res;
    serializeJson(doc, res);
    request->send(200, "application/json", res);
  });

  // Main Output Toggle from Home Page
  server.on(
    "/output/set", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<128> doc;
      if (!deserializeJson(doc, data, len)) {
        bool state = doc["state"];
        settings.mainOutput = state;
        if (state) {
          protectionWasActive = false;
          waitingForDelay = false;
          shouldTurnOff = false;
        }
        saveSettings();
        triggerBeep(60, 1, 0);
        request->send(200, "application/json", state ? "{\"success\":true,\"mainOutput\":1}" : "{\"success\":true,\"mainOutput\":0}");
      } else {
        request->send(400, "application/json", "{\"success\":false}");
      }
    });

  // Individual Setting Update
  server.on(
    "/setting/set", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<256> doc;
      if (deserializeJson(doc, data, len)) {
        request->send(400, "application/json", "{\"success\":false}");
        return;
      }
      String key = doc["key"] | "";
      if (key == "mainOutput") {
        settings.mainOutput = doc["value"].as<bool>();
        if (settings.mainOutput) {
          protectionWasActive = false;
          waitingForDelay = false;
        }
      } else if (key == "kWhSaver") settings.kWhSaver = doc["value"].as<bool>();
      else if (key == "afpcEnabled") settings.afpcEnabled = doc["value"].as<bool>();
      else if (key == "targetFactor") settings.targetFactor = constrain(doc["value"].as<float>(), 0.50f, 0.99f);
      else if (key == "mfdPerChannel") settings.mfdPerChannel = constrain(doc["value"].as<float>(), 0.1f, 25.0f);
      else if (key == "nodeRedEnabled") settings.nodeRedEnabled = doc["value"].as<bool>();
      else if (key == "mdnsEnabled") {
        settings.mdnsEnabled = doc["value"].as<bool>();
        pendingMdnsRestart = true;
      } else if (key == "mdnsHostname") {
        strncpy(settings.mdnsHostname, doc["value"] | "", 31);
        pendingMdnsRestart = true;
      } else if (key == "voltHigh") settings.voltHigh = doc["value"].as<float>();
      else if (key == "voltLow") settings.voltLow = doc["value"].as<float>();
      else if (key == "powerHigh") settings.powerHigh = doc["value"].as<float>();
      else if (key == "powerLow") settings.powerLow = doc["value"].as<float>();
      else if (key == "freqHigh") settings.freqHigh = doc["value"].as<float>();
      else if (key == "freqLow") settings.freqLow = doc["value"].as<float>();
      else if (key == "maxkWh") settings.maxkWh = doc["value"].as<int>();
      else if (key == "afpcLowWatt") settings.afpcLowWatt = doc["value"].as<int>();
      else if (key == "afpcHighWatt") settings.afpcHighWatt = doc["value"].as<int>();
      else if (key == "highVoltageAlert") settings.highVoltageAlert = doc["value"].as<bool>();
      else if (key == "lowVoltageAlert") settings.lowVoltageAlert = doc["value"].as<bool>();
      else if (key == "highPowerAlert") settings.highPowerAlert = doc["value"].as<bool>();
      else if (key == "lowPowerAlert") settings.lowPowerAlert = doc["value"].as<bool>();
      else if (key == "lowFreqAlert") settings.lowFreqAlert = doc["value"].as<bool>();
      else if (key == "highFreqAlert") settings.highFreqAlert = doc["value"].as<bool>();
      else if (key == "highVProtec") settings.highVProtec = doc["value"].as<bool>();
      else if (key == "LowVProtec") settings.LowVProtec = doc["value"].as<bool>();
      else if (key == "highPProtec") settings.highPProtec = doc["value"].as<bool>();
      else if (key == "highHzProtec") settings.highHzProtec = doc["value"].as<bool>();
      else if (key == "lowHzProtec") settings.lowHzProtec = doc["value"].as<bool>();
      else if (key == "espShareInterval") settings.espShareInterval = max(1, doc["value"].as<int>());
      else if (key == "nodeRedInterval") settings.nodeRedInterval = max(5, doc["value"].as<int>());
      else if (key == "pzemInterval") settings.pzemInterval = max(1, doc["value"].as<int>());
      else if (key == "alertInterval") settings.alertInterval = max(5, doc["value"].as<int>());
      else if (key == "protectionDelay") settings.protectionDelay = max(10, doc["value"].as<int>());
      else if (key == "startupTime") settings.startupTime = constrain(doc["value"].as<int>(), 0, 600);

      saveSettings();
      request->send(200, "application/json", "{\"success\":true}");
    });

  // Batch Section Save Endpoints
  server.on(
    "/output/save", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<128> doc;
      if (!deserializeJson(doc, data, len)) {
        settings.mainOutput = doc["mainOutput"] | settings.mainOutput;
        settings.kWhSaver = doc["kWhSaver"] | settings.kWhSaver;
        if (settings.mainOutput) {
          protectionWasActive = false;
          waitingForDelay = false;
        }
        saveSettings();
      }
      request->send(200, "application/json", "{\"success\":true}");
    });

  server.on(
    "/afpc/save", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<128> doc;
      if (!deserializeJson(doc, data, len)) {
        settings.afpcEnabled = doc["afpcEnabled"] | settings.afpcEnabled;
        settings.targetFactor = doc["targetFactor"] | settings.targetFactor;
        settings.mfdPerChannel = doc["mfdPerChannel"] | settings.mfdPerChannel;
        saveSettings();
      }
      request->send(200, "application/json", "{\"success\":true}");
    });

  server.on(
    "/thresholds/save", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<256> doc;
      if (!deserializeJson(doc, data, len)) {
        settings.voltHigh = doc["voltHigh"] | settings.voltHigh;
        settings.voltLow = doc["voltLow"] | settings.voltLow;
        settings.powerHigh = doc["powerHigh"] | settings.powerHigh;
        settings.powerLow = doc["powerLow"] | settings.powerLow;
        settings.maxkWh = doc["maxkWh"] | settings.maxkWh;
        settings.freqHigh = doc["freqHigh"] | settings.freqHigh;
        settings.freqLow = doc["freqLow"] | settings.freqLow;
        settings.afpcLowWatt = doc["afpcLowWatt"] | settings.afpcLowWatt;
        settings.afpcHighWatt = doc["afpcHighWatt"] | settings.afpcHighWatt;
        saveSettings();
      }
      request->send(200, "application/json", "{\"success\":true}");
    });

  server.on(
    "/alerts/save", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<256> doc;
      if (!deserializeJson(doc, data, len)) {
        settings.highVoltageAlert = doc["highVoltageAlert"] | false;
        settings.lowVoltageAlert = doc["lowVoltageAlert"] | false;
        settings.highPowerAlert = doc["highPowerAlert"] | false;
        settings.lowPowerAlert = doc["lowPowerAlert"] | false;
        settings.lowFreqAlert = doc["lowFreqAlert"] | false;
        settings.highFreqAlert = doc["highFreqAlert"] | false;
        saveSettings();
      }
      request->send(200, "application/json", "{\"success\":true}");
    });

  server.on(
    "/protection/save", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<256> doc;
      if (!deserializeJson(doc, data, len)) {
        settings.highVProtec = doc["highVProtec"] | false;
        settings.LowVProtec = doc["LowVProtec"] | false;
        settings.highPProtec = doc["highPProtec"] | false;
        settings.highHzProtec = doc["highHzProtec"] | false;
        settings.lowHzProtec = doc["lowHzProtec"] | false;
        saveSettings();
      }
      request->send(200, "application/json", "{\"success\":true}");
    });

  server.on(
    "/intervals/save", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<256> doc;
      if (!deserializeJson(doc, data, len)) {
        settings.espShareInterval = max(1, doc["espShareInterval"] | settings.espShareInterval);
        settings.nodeRedInterval = max(5, doc["nodeRedInterval"] | settings.nodeRedInterval);
        settings.pzemInterval = max(1, doc["pzemInterval"] | settings.pzemInterval);
        settings.alertInterval = max(5, doc["alertInterval"] | settings.alertInterval);
        settings.protectionDelay = max(10, doc["protectionDelay"] | settings.protectionDelay);
        saveSettings();
      }
      request->send(200, "application/json", "{\"success\":true}");
    });

  server.on(
    "/startup/save", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<128> doc;
      if (!deserializeJson(doc, data, len)) {
        settings.startupTime = constrain(doc["startupTime"] | settings.startupTime, 0, 600);
        saveSettings();
      }
      request->send(200, "application/json", "{\"success\":true}");
    });

  // mDNS Endpoints
  server.on(
    "/mdns/save", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<128> doc;
      if (!deserializeJson(doc, data, len)) {
        settings.mdnsEnabled = doc["enabled"] | true;
        String host = doc["hostname"] | "";
        host.trim();
        if (host.length() > 0 && host.length() < 31) {
          strncpy(settings.mdnsHostname, host.c_str(), 31);
          settings.mdnsHostname[31] = '\0';
        }
        saveSettings();
        pendingMdnsRestart = true;
        request->send(200, "application/json", "{\"success\":true}");
      } else {
        request->send(400, "application/json", "{\"success\":false}");
      }
    });

  server.on(
    "/mdns/reset", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      settings.mdnsEnabled = true;
      strncpy(settings.mdnsHostname, getDefaultMdnsName().c_str(), 31);
      settings.mdnsHostname[31] = '\0';
      saveSettings();
      pendingMdnsRestart = true;
      request->send(200, "application/json", "{\"success\":true}");
    });

  server.on("/mdns/delete", HTTP_POST, [](AsyncWebServerRequest* request) {
    settings.mdnsEnabled = false;
    strncpy(settings.mdnsHostname, getDefaultMdnsName().c_str(), 31);
    saveSettings();
    pendingMdnsRestart = true;
    request->send(200, "application/json", "{\"success\":true}");
  });

  // Node-RED Endpoints
  server.on(
    "/nodered/save", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<128> doc;
      if (!deserializeJson(doc, data, len)) {
        String addr = doc["address"] | "";
        if (addr.length() > 0 && addr.length() < 31) {
          strncpy(settings.nodeRedIP, addr.c_str(), 31);
          settings.nodeRedIP[31] = '\0';
          saveSettings();
        }
      }
      request->send(200, "application/json", "{\"success\":true}");
    });

  server.on("/nodered/test", HTTP_GET, [](AsyncWebServerRequest* request) {
    StaticJsonDocument<128> doc;
    doc["connected"] = nodeRedConnected;
    String res;
    serializeJson(doc, res);
    request->send(200, "application/json", res);
  });

  // WiFi Action Endpoints (Deferred to loop)
  server.on("/wifi/connect", HTTP_POST, [](AsyncWebServerRequest* request) {
    pendingWiFiPortal = true;
    request->send(200, "application/json", "{\"success\":true}");
  });
  server.on("/wifi/disconnect", HTTP_POST, [](AsyncWebServerRequest* request) {
    pendingWiFiDisconnect = true;
    request->send(200, "application/json", "{\"success\":true}");
  });
  server.on("/wifi/forget", HTTP_POST, [](AsyncWebServerRequest* request) {
    pendingWiFiForget = true;
    request->send(200, "application/json", "{\"success\":true}");
  });

  // Data, Time & System Maintenance Endpoints
  server.on("/data/clear-kwh", HTTP_POST, [](AsyncWebServerRequest* request) {
    struct tm ti;
    if (!ntpSynced || !getLocalTime(&ti, 0) || (ti.tm_year + 1900 < 2024)) {
      request->send(400, "application/json", "{\"success\":false,\"error\":\"NTP not synced! Sync time before resetting kWh.\"}");
      return;
    }
    resetKwhRequested = true;
    request->send(200, "application/json", "{\"success\":true}");
  });
  server.on("/data/clear-days", HTTP_POST, [](AsyncWebServerRequest* request) {
    struct tm ti;
    if (!ntpSynced || !getLocalTime(&ti, 0) || (ti.tm_year + 1900 < 2024)) {
      request->send(400, "application/json", "{\"success\":false,\"error\":\"NTP not synced! Sync time before resetting billing.\"}");
      return;
    }
    resetDaysRequested = true;
    request->send(200, "application/json", "{\"success\":true}");
  });
  server.on("/factory/reset-data", HTTP_POST, [](AsyncWebServerRequest* request) {
    pendingFactoryData = true;
    request->send(200, "application/json", "{\"success\":true}");
  });
  server.on("/factory/reset-settings", HTTP_POST, [](AsyncWebServerRequest* request) {
    pendingFactoryAll = true;
    request->send(200, "application/json", "{\"success\":true}");
  });
  server.on(
    "/time/timezone", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<128> doc;
      if (!deserializeJson(doc, data, len)) {
        settings.gmtOffset = doc["offset"] | 19800;
        saveSettings();
        pendingNtpSync = true;
      }
      request->send(200, "application/json", "{\"success\":true}");
    });
  server.on("/time/sync", HTTP_POST, [](AsyncWebServerRequest* request) {
    pendingNtpSync = true;
    request->send(200, "application/json", "{\"success\":true}");
  });
  server.on("/restart", HTTP_POST, [](AsyncWebServerRequest* request) {
    pendingRestart = true;
    restartScheduledMillis = millis() + 400;
    request->send(200, "application/json", "{\"success\":true}");
  });

  // Automation: Quick Timer
  server.on("/automation/timer/status", HTTP_GET, [](AsyncWebServerRequest* request) {
    StaticJsonDocument<128> doc;
    doc["active"] = quickTimerActive;
    long rem = quickTimerActive ? max(0L, (long)(quickTimerEndMillis - millis()) / 1000L) : 0;
    doc["remaining"] = rem;
    String res;
    serializeJson(doc, res);
    request->send(200, "application/json", res);
  });

  server.on(
    "/automation/timer/start", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<128> doc;
      if (!deserializeJson(doc, data, len)) {
        unsigned long durSec = doc["duration"] | 0;
        if (durSec > 0) {
          quickTimerActive = true;
          quickTimerEndMillis = millis() + (durSec * 1000UL);
          request->send(200, "application/json", "{\"success\":true}");
          return;
        }
      }
      request->send(400, "application/json", "{\"success\":false}");
    });

  server.on("/automation/timer/cancel", HTTP_POST, [](AsyncWebServerRequest* request) {
    quickTimerActive = false;
    request->send(200, "application/json", "{\"success\":true}");
  });

  // Automation: Daily Schedules
  server.on("/automation/schedule/list", HTTP_GET, [](AsyncWebServerRequest* request) {
    StaticJsonDocument<768> doc;
    JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < MAX_DAILY_SCHEDULES; i++) {
      if (dailySchedules[i].active) {
        JsonObject o = arr.createNestedObject();
        char tBuf[8];
        snprintf(tBuf, sizeof(tBuf), "%02d:%02d", dailySchedules[i].hour, dailySchedules[i].minute);
        o["time"] = tBuf;
        o["action"] = dailySchedules[i].turnOn ? "on" : "off";
        JsonArray dArr = o.createNestedArray("days");
        for (int d = 0; d < 7; d++) {
          if (dailySchedules[i].daysMask & (1 << d)) dArr.add(d);
        }
      }
    }
    String res;
    serializeJson(arr, res);
    request->send(200, "application/json", res);
  });

  server.on(
    "/automation/schedule/add", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<256> doc;
      if (!deserializeJson(doc, data, len)) {
        for (int i = 0; i < MAX_DAILY_SCHEDULES; i++) {
          if (!dailySchedules[i].active) {
            String t = doc["time"] | "08:00";
            dailySchedules[i].hour = t.substring(0, 2).toInt();
            dailySchedules[i].minute = t.substring(3, 5).toInt();
            dailySchedules[i].turnOn = (String(doc["action"] | "on") == "on");
            uint8_t mask = 0;
            JsonArray dArr = doc["days"].as<JsonArray>();
            for (JsonVariant v : dArr) mask |= (1 << v.as<int>());
            dailySchedules[i].daysMask = mask;
            dailySchedules[i].lastTriggeredYDay = -1;
            dailySchedules[i].active = true;
            saveSchedulesToNVS();
            request->send(200, "application/json", "{\"success\":true}");
            return;
          }
        }
      }
      request->send(400, "application/json", "{\"success\":false,\"error\":\"Schedule list full\"}");
    });

  server.on(
    "/automation/schedule/delete", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<128> doc;
      if (!deserializeJson(doc, data, len)) {
        int targetIdx = doc["index"] | -1;
        int activeCounter = 0;
        for (int i = 0; i < MAX_DAILY_SCHEDULES; i++) {
          if (dailySchedules[i].active) {
            if (activeCounter == targetIdx) {
              dailySchedules[i].active = false;
              saveSchedulesToNVS();
              break;
            }
            activeCounter++;
          }
        }
      }
      request->send(200, "application/json", "{\"success\":true}");
    });

  // Automation: Future Schedules
  server.on("/automation/future/list", HTTP_GET, [](AsyncWebServerRequest* request) {
    StaticJsonDocument<768> doc;
    JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < MAX_FUTURE_SCHEDULES; i++) {
      if (futureSchedules[i].active) {
        JsonObject o = arr.createNestedObject();
        char dBuf[12], tBuf[8];
        snprintf(dBuf, sizeof(dBuf), "%04d-%02d-%02d", futureSchedules[i].year, futureSchedules[i].month, futureSchedules[i].day);
        snprintf(tBuf, sizeof(tBuf), "%02d:%02d", futureSchedules[i].hour, futureSchedules[i].minute);
        o["date"] = dBuf;
        o["time"] = tBuf;
        o["action"] = futureSchedules[i].turnOn ? "on" : "off";
      }
    }
    String res;
    serializeJson(arr, res);
    request->send(200, "application/json", res);
  });

  server.on(
    "/automation/future/add", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<256> doc;
      if (!deserializeJson(doc, data, len)) {
        for (int i = 0; i < MAX_FUTURE_SCHEDULES; i++) {
          if (!futureSchedules[i].active) {
            String d = doc["date"] | "";
            String t = doc["time"] | "";
            if (d.length() >= 10 && t.length() >= 5) {
              futureSchedules[i].year = d.substring(0, 4).toInt();
              futureSchedules[i].month = d.substring(5, 7).toInt();
              futureSchedules[i].day = d.substring(8, 10).toInt();
              futureSchedules[i].hour = t.substring(0, 2).toInt();
              futureSchedules[i].minute = t.substring(3, 5).toInt();
              futureSchedules[i].turnOn = (String(doc["action"] | "on") == "on");
              futureSchedules[i].active = true;
              saveSchedulesToNVS();
              request->send(200, "application/json", "{\"success\":true}");
              return;
            }
          }
        }
      }
      request->send(400, "application/json", "{\"success\":false,\"error\":\"Schedule list full\"}");
    });

  server.on(
    "/automation/future/delete", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<128> doc;
      if (!deserializeJson(doc, data, len)) {
        int targetIdx = doc["index"] | -1;
        int activeCounter = 0;
        for (int i = 0; i < MAX_FUTURE_SCHEDULES; i++) {
          if (futureSchedules[i].active) {
            if (activeCounter == targetIdx) {
              futureSchedules[i].active = false;
              saveSchedulesToNVS();
              break;
            }
            activeCounter++;
          }
        }
      }
      request->send(200, "application/json", "{\"success\":true}");
    });

  // User Accounts Management
  server.on("/users/list", HTTP_GET, [](AsyncWebServerRequest* request) {
    StaticJsonDocument<512> doc;
    JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < MAX_USERS; i++) {
      if (users[i].active) {
        JsonObject o = arr.createNestedObject();
        o["username"] = users[i].username;
        o["password"] = users[i].password;
        o["role"] = users[i].role;
      }
    }
    String res;
    serializeJson(arr, res);
    request->send(200, "application/json", res);
  });

  server.on(
    "/users/add", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<256> doc;
      if (deserializeJson(doc, data, len)) {
        request->send(400, "application/json", "{\"success\":false}");
        return;
      }
      String adminU = doc["adminUser"] | "";
      String adminP = doc["adminPass"] | "";
      String newU = doc["username"] | "";
      String newP = doc["password"] | "";
      String role = doc["role"] | "user";

      if (!verifyAdminCreds(adminU, adminP)) {
        request->send(403, "application/json", "{\"success\":false,\"error\":\"Invalid admin credentials\"}");
        return;
      }
      for (int i = 0; i < MAX_USERS; i++) {
        if (users[i].active && newU == users[i].username) {
          request->send(400, "application/json", "{\"success\":false,\"error\":\"User already exists\"}");
          return;
        }
      }
      for (int i = 0; i < MAX_USERS; i++) {
        if (!users[i].active) {
          users[i].active = true;
          strncpy(users[i].username, newU.c_str(), 19);
          strncpy(users[i].password, newP.c_str(), 19);
          strncpy(users[i].role, role.c_str(), 9);
          saveUsers();
          request->send(200, "application/json", "{\"success\":true}");
          return;
        }
      }
      request->send(400, "application/json", "{\"success\":false,\"error\":\"Maximum user limit reached\"}");
    });

  server.on(
    "/users/delete", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<256> doc;
      if (deserializeJson(doc, data, len)) {
        request->send(400, "application/json", "{\"success\":false}");
        return;
      }
      String adminU = doc["adminUser"] | "";
      String adminP = doc["adminPass"] | "";
      String target = doc["username"] | "";

      if (!verifyAdminCreds(adminU, adminP)) {
        request->send(403, "application/json", "{\"success\":false,\"error\":\"Invalid admin credentials\"}");
        return;
      }

      int adminCount = 0;
      int targetIdx = -1;
      for (int i = 0; i < MAX_USERS; i++) {
        if (users[i].active) {
          if (String(users[i].role) == "admin") adminCount++;
          if (target == users[i].username) targetIdx = i;
        }
      }
      if (targetIdx == -1) {
        request->send(404, "application/json", "{\"success\":false,\"error\":\"User not found\"}");
        return;
      }
      if (String(users[targetIdx].role) == "admin" && adminCount <= 1) {
        request->send(400, "application/json", "{\"success\":false,\"error\":\"Cannot remove the last admin\"}");
        return;
      }
      users[targetIdx].active = false;
      saveUsers();
      request->send(200, "application/json", "{\"success\":true}");
    });

  server.begin();
  Serial.println("[WebServer] AsyncWebServer started on port 80");
}

void maintainWiFiAndServices() {
  static bool wasConnected = false;
  static unsigned long lastReconnectTry = 0;

  bool isConnected = (WiFi.status() == WL_CONNECTED);

  if (isConnected && !wasConnected) {
    wasConnected = true;
    Serial.printf("[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    setupMDNS();
    pendingNtpSync = true;
  } else if (!isConnected && wasConnected) {
    wasConnected = false;
    Serial.println("[WiFi] Connection lost. Will retry in background...");
  }

  // Non-blocking background reconnect every 20 seconds
  if (!isConnected && (millis() - lastReconnectTry >= 20000UL)) {
    lastReconnectTry = millis();
    WiFi.reconnect();
  }

  // Process deferred tasks safely in loop()
  if (pendingMdnsRestart) {
    pendingMdnsRestart = false;
    setupMDNS();
  }

  if (pendingWiFiDisconnect) {
    pendingWiFiDisconnect = false;
    WiFi.disconnect(false, false);
  }

  if (pendingWiFiForget) {
    pendingWiFiForget = false;
    WiFiManager wm;
    wm.resetSettings();
    WiFi.disconnect(true, true);
    pendingRestart = true;
    restartScheduledMillis = millis() + 500;
  }

  if (pendingWiFiPortal) {
    pendingWiFiPortal = false;
    server.end();  // Temporarily free port 80 for WiFiManager portal
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 14, "AP: ESP.EnergyLog");
    u8g2.drawStr(0, 28, "IP: 192.168.4.1");
    u8g2.drawStr(0, 46, "Portal Open (60s)");
    u8g2.sendBuffer();

    WiFiManager wm;
    wm.setConfigPortalTimeout(60);
    wm.startConfigPortal("ESP.EnergyLog");

    WiFi.mode(WIFI_STA);
    WiFi.begin();
    server.begin();  // Relaunch AsyncWebServer cleanly
    setupMDNS();
    showToastLCD("WiFi Portal", "Closed", 1200, UI_DASHBOARD);
  }

  if (resetKwhRequested) {
    resetKwhRequested = false;
    if (recordEnergyResetTimestamp()) {
      pzem.resetEnergy();
      portENTER_CRITICAL(&measureMux);
      readings.energy = 0.0f;
      readings.totalDays = 0.0f;
      portEXIT_CRITICAL(&measureMux);
      showToastLCD("Energy Reset!", lastResetTimeStr, 1400, UI_DASHBOARD);
    } else {
      triggerBeep(150, 2, 80);
      showToastLCD("Reset Failed!", "NTP Not Synced", 1500, UI_DASHBOARD);
    }
  }

  if (resetDaysRequested) {
    resetDaysRequested = false;
    if (recordEnergyResetTimestamp()) {
      portENTER_CRITICAL(&measureMux);
      readings.totalDays = 0.0f;
      portEXIT_CRITICAL(&measureMux);
      showToastLCD("Last Billing", "Reset Done!", 1200, UI_DASHBOARD);
    } else {
      triggerBeep(150, 2, 80);
      showToastLCD("Reset Failed!", "NTP Not Synced", 1500, UI_DASHBOARD);
    }
  }

  if (pendingFactoryData) {
    pendingFactoryData = false;
    if (ntpSynced) {
      recordEnergyResetTimestamp();
    } else {
      lastResetEpoch = 0;
      strcpy(lastResetTimeStr, "Not Reset");
      preferences.remove("rstEpoch");
      preferences.remove("rstTime");
    }
    pzem.resetEnergy();
    kwhDays = 0.0f;
    memset(dailySchedules, 0, sizeof(dailySchedules));
    memset(futureSchedules, 0, sizeof(futureSchedules));
    saveSchedulesToNVS();
    showToastLCD("All Data", "Reset Done!", 1200, UI_DASHBOARD);
  }

  if (pendingFactoryAll) {
    pendingFactoryAll = false;
    executeFullFactoryReset();
  }

  if (pendingRestart && (long)(millis() - restartScheduledMillis) >= 0) {
    pendingRestart = false;
    ESP.restart();
  }
}

// ============================================================================
// SINGLE-PASS OLED DISPLAY & NON-BLOCKING MENU ENGINE
// ============================================================================
void renderDashboardFrame() {
  portENTER_CRITICAL(&measureMux);
  Measurements cur = readings;
  portEXIT_CRITICAL(&measureMux);

  u8g2.clearBuffer();

  // 1. Layout Grid Lines
  u8g2.drawFrame(0, 0, 128, 64);
  u8g2.drawLine(54, 1, 54, 62);
  u8g2.drawLine(1, 39, 126, 39);

  // 2. Unit Labels
  u8g2.setFont(u8g2_font_t0_11_tr);
  u8g2.drawStr(46, 12, "V");
  u8g2.drawStr(113, 12, "PF");
  u8g2.drawStr(46, 24, "A");
  u8g2.drawStr(118, 24, "W");
  u8g2.drawStr(40, 36, "Hz");
  u8g2.drawStr(107, 36, "kWh");
  u8g2.drawStr(46, 50, "H");
  u8g2.drawStr(46, 61, "D");

  // 3. Telemetry Values
  char buf[16];
  dtostrf(cur.voltage, 0, 1, buf);
  u8g2.drawStr(3, 12, buf);
  dtostrf(cur.current, 0, 2, buf);
  u8g2.drawStr(3, 24, buf);
  dtostrf(cur.frequency, 0, 2, buf);
  u8g2.drawStr(3, 36, buf);
  dtostrf(cur.uptime, 0, 1, buf);
  u8g2.drawStr(3, 50, buf);
  dtostrf(cur.totalDays, 0, 1, buf);
  u8g2.drawStr(3, 61, buf);
  dtostrf(cur.pf, 0, 2, buf);
  u8g2.drawStr(59, 12, buf);
  dtostrf(cur.power, 0, 1, buf);
  u8g2.drawStr(59, 24, buf);
  dtostrf(cur.energy, 0, 2, buf);
  u8g2.drawStr(59, 36, buf);

  // 4. Date/Time or Protection Status (Top right of bottom panel)
  if (shouldTurnOff) {
    u8g2.drawStr(58, 50, protectionStatusText);
  } else {
    struct tm ti;
    if (ntpSynced && getLocalTime(&ti, 0)) {
      char dStr[8], tStr[8];
      strftime(dStr, sizeof(dStr), "%d/%m", &ti);
      strftime(tStr, sizeof(tStr), "%H:%M", &ti);
      u8g2.drawStr(58, 50, dStr);
      u8g2.drawStr(94, 50, tStr);
    } else {
      u8g2.drawStr(58, 50, "--/--");
      u8g2.drawStr(94, 50, "--:--");
    }
  }

  // 5. Status Bar Icons (Bottom right)
  if (WiFi.status() == WL_CONNECTED) u8g2.drawStr(59, 61, "W");
  if (settings.mdnsEnabled) u8g2.drawStr(71, 61, "M");
  if (settings.nodeRedEnabled) u8g2.drawStr(83, 61, "R");
  if (settings.afpcEnabled) u8g2.drawStr(94, 61, "C");
  if (settings.highVProtec || settings.LowVProtec || settings.highPProtec || settings.highHzProtec || settings.lowHzProtec) {
    u8g2.drawStr(106, 61, "P");
  }
  if (settings.highVoltageAlert || settings.lowVoltageAlert || settings.highPowerAlert || settings.lowPowerAlert || settings.highFreqAlert || settings.lowFreqAlert) {
    u8g2.drawStr(118, 61, "A");
  }

  // Single SPI transfer for the entire screen!
  u8g2.sendBuffer();
}

void drawSubmenuRightValue(int mainIdx, int subIdx) {
  char val[16] = "";
  switch (mainIdx) {
    case 1:  // WiFi
      if (subIdx == 2) strcpy(val, WiFi.status() == WL_CONNECTED ? "ON" : "---");
      break;
    case 2:  // mDNS
      if (subIdx == 1) strcpy(val, settings.mdnsEnabled ? "ON" : "OFF");
      break;
    case 3:  // Node-RED
      if (subIdx == 1) strcpy(val, settings.nodeRedEnabled ? "ON" : "OFF");
      else if (subIdx == 2) strcpy(val, nodeRedConnected ? "OK" : "---");
      break;
    case 4:  // AFPC
      if (subIdx == 1) strcpy(val, settings.afpcEnabled ? "ON" : "OFF");
      else if (subIdx == 2) snprintf(val, sizeof(val), "%.2f", settings.targetFactor);
      else if (subIdx == 3) snprintf(val, sizeof(val), "%.1f", settings.mfdPerChannel);
      break;
    case 5:  // Startup
      if (subIdx == 1) snprintf(val, sizeof(val), "%ds", settings.startupTime);
      break;
    case 6:  // Intervals
      if (subIdx == 1) snprintf(val, sizeof(val), "%ds", settings.espShareInterval);
      else if (subIdx == 2) snprintf(val, sizeof(val), "%ds", settings.nodeRedInterval);
      else if (subIdx == 3) snprintf(val, sizeof(val), "%ds", settings.pzemInterval);
      else if (subIdx == 4) snprintf(val, sizeof(val), "%ds", settings.alertInterval);
      else if (subIdx == 5) snprintf(val, sizeof(val), "%ds", settings.protectionDelay);
      break;
    case 7:  // Thresholds
      if (subIdx == 1) snprintf(val, sizeof(val), "%dV", (int)settings.voltHigh);
      else if (subIdx == 2) snprintf(val, sizeof(val), "%dV", (int)settings.voltLow);
      else if (subIdx == 3) snprintf(val, sizeof(val), "%dW", (int)settings.powerHigh);
      else if (subIdx == 4) snprintf(val, sizeof(val), "%dW", (int)settings.powerLow);
      else if (subIdx == 5) snprintf(val, sizeof(val), "%.2f", settings.freqHigh);
      else if (subIdx == 6) snprintf(val, sizeof(val), "%.2f", settings.freqLow);
      else if (subIdx == 7) snprintf(val, sizeof(val), "%dkWh", settings.maxkWh);
      else if (subIdx == 8) snprintf(val, sizeof(val), "%dW", settings.afpcLowWatt);
      else if (subIdx == 9) snprintf(val, sizeof(val), "%dW", settings.afpcHighWatt);
      break;
    case 8:  // Alerts
      {
        bool* arr[] = { nullptr, &settings.highVoltageAlert, &settings.lowVoltageAlert, &settings.highPowerAlert, &settings.lowPowerAlert, &settings.lowFreqAlert, &settings.highFreqAlert };
        if (subIdx >= 1 && subIdx <= 6) strcpy(val, *arr[subIdx] ? "ON" : "OFF");
      }
      break;
    case 9:  // Protections
      {
        bool* arr[] = { nullptr, &settings.highVProtec, &settings.LowVProtec, &settings.highPProtec, &settings.highHzProtec, &settings.lowHzProtec };
        if (subIdx >= 1 && subIdx <= 5) strcpy(val, *arr[subIdx] ? "ON" : "OFF");
      }
      break;
    case 10:  // Main Output
      if (subIdx == 1) strcpy(val, settings.mainOutput ? "ON" : "OFF");
      else if (subIdx == 2) strcpy(val, settings.kWhSaver ? "ON" : "OFF");
      break;
  }
  if (strlen(val) > 0) {
    int w = u8g2.getStrWidth(val);
    u8g2.drawStr(126 - w, 11, val);
  }
}

void startIntEditor(const char* title, int* target, int minV, int maxV, int stepV, char unit) {
  editCtx.title = title;
  editCtx.intTarget = target;
  editCtx.intTemp = *target;
  editCtx.intMin = minV;
  editCtx.intMax = maxV;
  editCtx.intStep = stepV;
  editCtx.unit = unit;
  setEncoderPos((*target) / stepV);
  uiState = UI_EDIT_INT;
}

void startFloatEditor(const char* title, float* target, float minV, float maxV, float scale, char unit) {
  editCtx.title = title;
  editCtx.floatTarget = target;
  editCtx.floatTemp = *target;
  editCtx.floatMin = minV;
  editCtx.floatMax = maxV;
  editCtx.floatScale = scale;
  editCtx.unit = unit;
  setEncoderPos(round((*target) * scale));
  uiState = UI_EDIT_FLOAT;
}

void handleSubmenuAction(int mainIdx, int subIdx) {
  switch (mainIdx) {
    case 1:  // WiFi Setup
      if (subIdx == 1) pendingWiFiPortal = true;
      else if (subIdx == 2) {
        strcpy(infoLine1, WiFi.status() == WL_CONNECTED ? "WiFi: Connected" : "WiFi: Offline");
        strncpy(infoLine2, WiFi.SSID().c_str(), 23);
        strncpy(infoLine3, WiFi.localIP().toString().c_str(), 23);
        snprintf(infoLine4, 23, "RSSI: %d dBm", WiFi.RSSI());
        uiReturnState = UI_SUB_MENU;
        uiState = UI_INFO_VIEW;
      } else if (subIdx == 3) {
        pendingWiFiDisconnect = true;
        showToastLCD("WiFi", "Disconnected!", 1200, UI_SUB_MENU);
      } else if (subIdx == 4) {
        pendingWiFiForget = true;
        showToastLCD("WiFi Cleared", "Restarting...", 1200, UI_DASHBOARD);
      }
      break;

    case 2:  // mDNS
      if (subIdx == 1) {
        settings.mdnsEnabled = !settings.mdnsEnabled;
        saveSettings();
        pendingMdnsRestart = true;
        showToastLCD("mDNS Responder", settings.mdnsEnabled ? "Enabled" : "Disabled", 1000, UI_SUB_MENU);
      } else if (subIdx == 2) {
        settings.mdnsEnabled = true;
        strncpy(settings.mdnsHostname, getDefaultMdnsName().c_str(), 31);
        saveSettings();
        pendingMdnsRestart = true;
        showToastLCD("mDNS Reset:", settings.mdnsHostname, 1400, UI_SUB_MENU);
      } else if (subIdx == 3) {
        strcpy(infoLine1, "mDNS Hostname:");
        snprintf(infoLine2, 23, "%s.local", settings.mdnsHostname);
        snprintf(infoLine3, 23, "Status: %s", settings.mdnsEnabled ? "Active" : "OFF");
        strcpy(infoLine4, "Click to exit");
        uiReturnState = UI_SUB_MENU;
        uiState = UI_INFO_VIEW;
      }
      break;

    case 3:  // Node-RED
      if (subIdx == 1) {
        settings.nodeRedEnabled = !settings.nodeRedEnabled;
        saveSettings();
        showToastLCD("Node-RED", settings.nodeRedEnabled ? "Enabled" : "Disabled", 1000, UI_SUB_MENU);
      } else if (subIdx == 2) {
        strcpy(infoLine1, "Node-RED Status");
        strncpy(infoLine2, settings.nodeRedIP, 23);
        strcpy(infoLine3, nodeRedConnected ? "Connected" : "Unreachable");
        strncpy(infoLine4, lastNodeRedResponse.c_str(), 23);
        uiReturnState = UI_SUB_MENU;
        uiState = UI_INFO_VIEW;
      }
      break;

    case 4:  // AFPC
      if (subIdx == 1) {
        settings.afpcEnabled = !settings.afpcEnabled;
        saveSettings();
        showToastLCD("AFPC Control", settings.afpcEnabled ? "Enabled" : "Disabled", 1000, UI_SUB_MENU);
      } else if (subIdx == 2) {
        startFloatEditor("Target PF", &settings.targetFactor, 0.50f, 0.99f, 100.0f, ' ');
      } else if (subIdx == 3) {
        startFloatEditor("MFD / Channel", &settings.mfdPerChannel, 0.1f, 15.0f, 10.0f, 'u');
      }
      break;

    case 5:  // Startup
      if (subIdx == 1) startIntEditor("Startup Delay", &settings.startupTime, 0, 300, 5, 's');
      break;

    case 6:  // Intervals
      if (subIdx == 1) startIntEditor("ESP Sharing", &settings.espShareInterval, 1, 300, 1, 's');
      else if (subIdx == 2) startIntEditor("Node-RED Intv", &settings.nodeRedInterval, 5, 300, 5, 's');
      else if (subIdx == 3) startIntEditor("Sensor Read", &settings.pzemInterval, 1, 60, 1, 's');
      else if (subIdx == 4) startIntEditor("Alert Repeat", &settings.alertInterval, 15, 600, 5, 's');
      else if (subIdx == 5) startIntEditor("Protec. Delay", &settings.protectionDelay, 60, 3600, 60, 's');
      break;

    case 7:  // Thresholds
      {
        static int tempHolder = 0;
        if (subIdx == 1) {
          tempHolder = settings.voltHigh;
          startIntEditor("Voltage High", &tempHolder, 150, 280, 1, 'V');
          editCtx.onConfirm = []() {
            settings.voltHigh = *editCtx.intTarget;
          };
        } else if (subIdx == 2) {
          tempHolder = settings.voltLow;
          startIntEditor("Voltage Low", &tempHolder, 120, 250, 1, 'V');
          editCtx.onConfirm = []() {
            settings.voltLow = *editCtx.intTarget;
          };
        } else if (subIdx == 3) {
          tempHolder = settings.powerHigh;
          startIntEditor("Power High", &tempHolder, 100, 9900, 100, 'W');
          editCtx.onConfirm = []() {
            settings.powerHigh = *editCtx.intTarget;
          };
        } else if (subIdx == 4) {
          tempHolder = settings.powerLow;
          startIntEditor("Power Low", &tempHolder, 100, 9900, 100, 'W');
          editCtx.onConfirm = []() {
            settings.powerLow = *editCtx.intTarget;
          };
        } else if (subIdx == 5) startFloatEditor("Freq. High", &settings.freqHigh, 49.00f, 51.00f, 100.0f, 'H');
        else if (subIdx == 6) startFloatEditor("Freq. Low", &settings.freqLow, 49.00f, 51.00f, 100.0f, 'H');
        else if (subIdx == 7) startIntEditor("Max kWh", &settings.maxkWh, 1, 9000, 5, 'U');
        else if (subIdx == 8) startIntEditor("AFPC Low W", &settings.afpcLowWatt, 1, 9900, 10, 'W');
        else if (subIdx == 9) startIntEditor("AFPC High W", &settings.afpcHighWatt, 100, 9900, 50, 'W');
      }
      break;

    case 8:  // Alerts Toggle
      {
        bool* arr[] = { nullptr, &settings.highVoltageAlert, &settings.lowVoltageAlert, &settings.highPowerAlert, &settings.lowPowerAlert, &settings.lowFreqAlert, &settings.highFreqAlert };
        if (subIdx >= 1 && subIdx <= 6) {
          *arr[subIdx] = !(*arr[subIdx]);
          saveSettings();
          showToastLCD(safetyAlertSubmenu[subIdx], *arr[subIdx] ? "Alert ON" : "Alert OFF", 900, UI_SUB_MENU);
        }
      }
      break;

    case 9:  // Protections Toggle
      {
        bool* arr[] = { nullptr, &settings.highVProtec, &settings.LowVProtec, &settings.highPProtec, &settings.highHzProtec, &settings.lowHzProtec };
        if (subIdx >= 1 && subIdx <= 5) {
          *arr[subIdx] = !(*arr[subIdx]);
          saveSettings();
          showToastLCD(safetyProtectionSubmenu[subIdx], *arr[subIdx] ? "Protec. ON" : "Protec. OFF", 900, UI_SUB_MENU);
        }
      }
      break;

    case 10:  // Main Output
      if (subIdx == 1) {
        settings.mainOutput = !settings.mainOutput;
        if (settings.mainOutput) {
          protectionWasActive = false;
          waitingForDelay = false;
        }
        saveSettings();
        showToastLCD("Main Output", settings.mainOutput ? "Turned ON" : "Turned OFF", 1000, UI_SUB_MENU);
      } else if (subIdx == 2) {
        settings.kWhSaver = !settings.kWhSaver;
        saveSettings();
        showToastLCD("kWh Saver", settings.kWhSaver ? "Enabled" : "Disabled", 1000, UI_SUB_MENU);
      }
      break;

    case 12:  // Clear Data
      if (subIdx == 1) {
        if (!ntpSynced) {
          triggerBeep(150, 2, 80);
          showToastLCD("NTP Not Synced!", "Sync Time First", 1500, UI_SUB_MENU);
        } else {
          resetKwhRequested = true;
        }
      } else if (subIdx == 2) {
        // Show Last Billing Reset Timestamp & Elapsed Days on LCD
        strcpy(infoLine1, "Last Billing:");
        strncpy(infoLine2, lastResetTimeStr, 23);
        snprintf(infoLine3, 23, "Elapsed: %.1f Days", kwhDays);
        snprintf(infoLine4, 23, "Energy: %.2f kWh", readings.energy);
        uiReturnState = UI_SUB_MENU;
        uiState = UI_INFO_VIEW;
      }
      break;

    case 13:  // Factory Reset
      if (subIdx == 1) pendingFactoryData = true;
      else if (subIdx == 2) pendingFactoryAll = true;
      break;
  }
}

void updateDisplayAndUI() {
  unsigned long now = millis();

  // If BOOT button is held for > 500ms, override display with Factory Reset Countdown
  if (bootBtnHolding && (now - bootBtnPressStart >= 500UL)) {
    int rem = 5 - ((now - bootBtnPressStart) / 1000UL);
    u8g2.clearBuffer();
    u8g2.drawFrame(0, 0, 128, 64);
    u8g2.setFont(u8g2_font_7x14B_tr);
    u8g2.drawStr(14, 20, "FACTORY RESET");
    u8g2.setFont(u8g2_font_6x12_tf);
    char cStr[24];
    snprintf(cStr, sizeof(cStr), "Hold BOOT: %d sec", max(0, rem));
    u8g2.drawStr(18, 40, cStr);
    int barW = map(now - bootBtnPressStart, 500, 5000, 0, 108);
    u8g2.drawFrame(10, 48, 108, 8);
    u8g2.drawBox(10, 48, constrain(barW, 0, 108), 8);
    u8g2.sendBuffer();
    return;
  }

  // Auto-exit menu on inactivity
  if (uiState != UI_DASHBOARD && uiState != UI_STARTUP_SAFETY && (now - lastMenuActivityMillis >= MENU_TIMEOUT_MS)) {
    uiState = UI_DASHBOARD;
  }

  // 1. Startup Safety Check State (Strictly requires valid PZEM reading!)
  if (uiState == UI_STARTUP_SAFETY) {
    portENTER_CRITICAL(&measureMux);
    Measurements cur = readings;
    portEXIT_CRITICAL(&measureMux);

    // Sensor MUST be valid AND within thresholds to be safe
    bool vOk = cur.valid && (cur.voltage >= settings.voltLow && cur.voltage <= settings.voltHigh);
    bool fOk = cur.valid && (cur.frequency >= settings.freqLow && cur.frequency <= settings.freqHigh);
    bool safe = cur.valid && vOk && fOk;

    if (now - lastStartupTickMillis >= 1000UL) {
      lastStartupTickMillis = now;
      if (safe && startupRemainingSec > 0) {
        startupRemainingSec--;
      }
    }

    // Debounce RESET_BTN (must hold for 300ms) so boot transients never skip safety check
    static unsigned long resetHighStart = 0;
    bool manualResetOverride = false;
    if (digitalRead(RESET_BTN) == HIGH) {
      if (resetHighStart == 0) resetHighStart = now;
      else if (now - resetHighStart >= 300UL) manualResetOverride = true;
    } else {
      resetHighStart = 0;
    }

    if ((safe && startupRemainingSec <= 0) || manualResetOverride || checkEncoderButtonClick()) {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_6x12_tf);
      u8g2.drawStr(31, 35, "Starting...");
      u8g2.sendBuffer();

      digitalWrite(BUZZER_PIN, HIGH);
      delay(1500);
      digitalWrite(BUZZER_PIN, LOW);
      delay(500);

      startupCheckCompleted = true;
      uiState = UI_DASHBOARD;
      return;
    }

    static unsigned long lastStartupDraw = 0;
    if (now - lastStartupDraw >= 200UL) {
      lastStartupDraw = now;
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_6x12_tf);

      if (!cur.valid) {
        u8g2.drawStr(2, 14, "PZEM SENSOR WAIT...");
        u8g2.drawStr(2, 28, "Checking AC Input");
        u8g2.drawStr(2, 58, "Push/Click Override");
      } else {
        char vBuf[24], fBuf[24];
        const char* vStat = (cur.voltage > settings.voltHigh) ? "high V" : ((cur.voltage < settings.voltLow) ? "low V" : "normal");
        const char* fStat = (cur.frequency > settings.freqHigh) ? "high fre." : ((cur.frequency < settings.freqLow) ? "undr fre." : "normal");

        snprintf(vBuf, sizeof(vBuf), "%-9s %6.2f V", vStat, cur.voltage);
        snprintf(fBuf, sizeof(fBuf), "%-9s %5.2f Hz", fStat, cur.frequency);
        u8g2.drawStr(0, 10, vBuf);
        u8g2.drawStr(0, 23, fBuf);

        if (safe) {
          int totalT = max(1, settings.startupTime);
          int barW = map(totalT - startupRemainingSec, 0, totalT, 0, 120);
          u8g2.drawFrame(4, 38, 120, 6);
          if (barW > 0) {
            u8g2.drawBox(4, 38, constrain(barW, 0, 120), 6);
          }
          char tBuf[24];
          snprintf(tBuf, sizeof(tBuf), "wait for...   %d s", startupRemainingSec);
          u8g2.drawStr(4, 58, tBuf);
        } else {
          u8g2.drawStr(0, 58, "press push to turn on!");
        }
      }
      u8g2.sendBuffer();
    }
    return;
  }

  // 2. Main Dashboard State
  if (uiState == UI_DASHBOARD) {
    if (checkEncoderButtonClick()) {
      triggerBeep(45, 1, 0);
      currentMainMenuIdx = 0;
      setEncoderPos(0);
      uiState = UI_MAIN_MENU;
      return;
    }
    static unsigned long lastDashDraw = 0;
    if (now - lastDashDraw >= 500UL) {
      lastDashDraw = now;
      renderDashboardFrame();
    }
    return;
  }

  // 3. Toast Notification State
  if (uiState == UI_TOAST) {
    if (now >= toastEndMillis || checkEncoderButtonClick()) {
      uiState = uiReturnState;
      return;
    }
    u8g2.clearBuffer();
    u8g2.drawFrame(0, 0, 128, 64);
    u8g2.setFont(u8g2_font_7x14B_tr);
    u8g2.drawStr(8, 28, toastMsgLine1);
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(8, 46, toastMsgLine2);
    u8g2.sendBuffer();
    return;
  }

  // 4. Info Screen State
  if (uiState == UI_INFO_VIEW) {
    if (checkEncoderButtonClick()) {
      triggerBeep(40, 1, 0);
      uiState = uiReturnState;
      return;
    }
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_7x14B_tr);
    u8g2.drawStr(0, 13, infoLine1);
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 29, infoLine2);
    u8g2.drawStr(0, 43, infoLine3);
    u8g2.drawStr(0, 59, infoLine4);
    u8g2.sendBuffer();
    return;
  }

  // 5. Main Menu Navigation
  if (uiState == UI_MAIN_MENU) {
    long pos = getEncoderPos();
    if (pos != lastEncoderPos) {
      currentMainMenuIdx = constrain(currentMainMenuIdx + (pos > lastEncoderPos ? 1 : -1), 0, mainMenuCount - 1);
      lastEncoderPos = pos;
      lastMenuActivityMillis = now;
      triggerBeep(20, 1, 0);
    }

    if (checkEncoderButtonClick()) {
      triggerBeep(45, 1, 0);
      if (currentMainMenuIdx == 0) {
        uiState = UI_DASHBOARD;
        return;
      } else if (currentMainMenuIdx == 11) {
        pendingNtpSync = true;
        showToastLCD("NTP Clock", "Sync Requested", 1200, UI_MAIN_MENU);
        return;
      } else if (currentMainMenuIdx == 14) {
        pendingRestart = true;
        restartScheduledMillis = millis() + 600;
        showToastLCD("System", "Restarting...", 1000, UI_DASHBOARD);
        return;
      } else if (currentMainMenuIdx == 15) {
        strcpy(infoLine1, "ESP Energy Log");
        strcpy(infoLine2, "Firmware: v3.0");
        snprintf(infoLine3, 23, "%s.local", settings.mdnsHostname);
        strcpy(infoLine4, "By: M.Maity");
        uiReturnState = UI_MAIN_MENU;
        uiState = UI_INFO_VIEW;
        return;
      }

      // Enter Submenu
      switch (currentMainMenuIdx) {
        case 1:
          activeSubmenuPtr = wifiSetupSubmenu;
          activeSubmenuLen = 5;
          break;
        case 2:
          activeSubmenuPtr = mdnsSubmenu;
          activeSubmenuLen = 4;
          break;
        case 3:
          activeSubmenuPtr = nodeRedSubmenu;
          activeSubmenuLen = 3;
          break;
        case 4:
          activeSubmenuPtr = afpcSubmenu;
          activeSubmenuLen = 4;
          break;
        case 5:
          activeSubmenuPtr = startupSubmenu;
          activeSubmenuLen = 2;
          break;
        case 6:
          activeSubmenuPtr = updateIntervalSubmenu;
          activeSubmenuLen = 6;
          break;
        case 7:
          activeSubmenuPtr = thresholdSubmenu;
          activeSubmenuLen = 10;
          break;
        case 8:
          activeSubmenuPtr = safetyAlertSubmenu;
          activeSubmenuLen = 7;
          break;
        case 9:
          activeSubmenuPtr = safetyProtectionSubmenu;
          activeSubmenuLen = 6;
          break;
        case 10:
          activeSubmenuPtr = mainOutputSubmenu;
          activeSubmenuLen = 3;
          break;
        case 12:
          activeSubmenuPtr = clearDataSubmenu;
          activeSubmenuLen = 3;
          break;
        case 13:
          activeSubmenuPtr = factoryResetSubmenu;
          activeSubmenuLen = 3;
          break;
      }
      currentSubMenuIdx = 0;
      setEncoderPos(0);
      uiState = UI_SUB_MENU;
      return;
    }

    // Draw Main Menu
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_t0_11_tr);
    u8g2.setDrawColor(1);
    u8g2.drawBox(0, 0, 128, 12);
    u8g2.setDrawColor(0);
    u8g2.drawStr(32, 10, "Menu Control");
    u8g2.setDrawColor(1);

    int start = max(0, min(currentMainMenuIdx - 1, mainMenuCount - 3));
    for (int i = 0; i < 3 && (start + i) < mainMenuCount; i++) {
      int idx = start + i;
      int yBox = 14 + i * 16;
      int yTxt = 25 + i * 16;
      if (idx == currentMainMenuIdx) {
        u8g2.drawBox(0, yBox, 128, 15);
        u8g2.setDrawColor(0);
        u8g2.drawStr(4, yTxt, mainMenu[idx]);
        u8g2.setDrawColor(1);
      } else {
        u8g2.drawStr(4, yTxt, mainMenu[idx]);
      }
    }
    u8g2.sendBuffer();
    return;
  }

  // 6. Submenu Navigation
  if (uiState == UI_SUB_MENU) {
    long pos = getEncoderPos();
    if (pos != lastEncoderPos) {
      currentSubMenuIdx = constrain(currentSubMenuIdx + (pos > lastEncoderPos ? 1 : -1), 0, activeSubmenuLen - 1);
      lastEncoderPos = pos;
      lastMenuActivityMillis = now;
      triggerBeep(20, 1, 0);
    }

    if (checkEncoderButtonClick()) {
      triggerBeep(45, 1, 0);
      if (currentSubMenuIdx == 0) {
        setEncoderPos(currentMainMenuIdx);
        uiState = UI_MAIN_MENU;
      } else {
        editCtx.onConfirm = nullptr;
        handleSubmenuAction(currentMainMenuIdx, currentSubMenuIdx);
      }
      return;
    }

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_7x14B_tr);
    u8g2.drawStr(0, 12, mainMenu[currentMainMenuIdx]);
    u8g2.setFont(u8g2_font_6x12_tf);
    drawSubmenuRightValue(currentMainMenuIdx, currentSubMenuIdx);
    u8g2.drawLine(0, 15, 127, 15);

    int start = (currentSubMenuIdx / 3) * 3;
    for (int i = 0; i < 3 && (start + i) < activeSubmenuLen; i++) {
      int itemIdx = start + i;
      int yPos = 29 + i * 13;
      if (itemIdx == currentSubMenuIdx) {
        u8g2.drawStr(0, yPos, ">");
      }
      u8g2.drawStr(10, yPos, activeSubmenuPtr[itemIdx]);
    }
    u8g2.sendBuffer();
    return;
  }

  // 7. Integer Value Editor
  if (uiState == UI_EDIT_INT) {
    long pos = getEncoderPos();
    if (pos != lastEncoderPos) {
      editCtx.intTemp = constrain(pos * editCtx.intStep, editCtx.intMin, editCtx.intMax);
      setEncoderPos(editCtx.intTemp / editCtx.intStep);
      lastMenuActivityMillis = now;
      triggerBeep(15, 1, 0);
    }
    if (checkEncoderButtonClick()) {
      *editCtx.intTarget = editCtx.intTemp;
      if (editCtx.onConfirm) editCtx.onConfirm();
      saveSettings();
      triggerBeep(60, 1, 0);
      showToastLCD(editCtx.title, "Saved!", 900, UI_SUB_MENU);
      return;
    }

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_7x14B_tr);
    u8g2.drawStr(0, 13, editCtx.title);
    u8g2.setFont(u8g2_font_ncenB18_tr);
    char vStr[16];
    snprintf(vStr, sizeof(vStr), "%d %c", editCtx.intTemp, editCtx.unit);
    u8g2.drawStr(20, 42, vStr);
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 60, "Click knob to save");
    u8g2.sendBuffer();
    return;
  }

  // 8. Float Value Editor
  if (uiState == UI_EDIT_FLOAT) {
    long pos = getEncoderPos();
    if (pos != lastEncoderPos) {
      long minS = round(editCtx.floatMin * editCtx.floatScale);
      long maxS = round(editCtx.floatMax * editCtx.floatScale);
      long clamped = constrain(pos, minS, maxS);
      editCtx.floatTemp = clamped / editCtx.floatScale;
      setEncoderPos(clamped);
      lastMenuActivityMillis = now;
      triggerBeep(15, 1, 0);
    }
    if (checkEncoderButtonClick()) {
      *editCtx.floatTarget = editCtx.floatTemp;
      saveSettings();
      triggerBeep(60, 1, 0);
      showToastLCD(editCtx.title, "Saved!", 900, UI_SUB_MENU);
      return;
    }

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_7x14B_tr);
    u8g2.drawStr(0, 13, editCtx.title);
    u8g2.setFont(u8g2_font_ncenB18_tr);
    char vStr[16];
    snprintf(vStr, sizeof(vStr), "%.2f %c", editCtx.floatTemp, editCtx.unit);
    u8g2.drawStr(20, 42, vStr);
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 60, "Click knob to save");
    u8g2.sendBuffer();
    return;
  }
}

// ============================================================================
// BOOT SEQUENCE HELPERS & SETUP
// ============================================================================
void bootConnectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  WiFi.begin();

  const int MAX_ATTEMPTS = 5;
  for (int attempts = 0; attempts < MAX_ATTEMPTS; attempts++) {
    char attemptStr[20];
    snprintf(attemptStr, sizeof(attemptStr), "Attempt: %d/%d", attempts + 1, MAX_ATTEMPTS);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 12, "Connecting WiFi...");
    u8g2.drawStr(0, 24, attemptStr);
    u8g2.sendBuffer();

    if (WiFi.status() == WL_CONNECTED) {
      u8g2.clearBuffer();
      u8g2.drawStr(0, 12, "WiFi Connected!");
      u8g2.drawStr(0, 24, "SSID:");
      u8g2.drawStr(0, 36, WiFi.SSID().c_str());
      u8g2.drawStr(0, 48, "IP:");
      u8g2.drawStr(0, 60, WiFi.localIP().toString().c_str());
      u8g2.sendBuffer();
      delay(2000);
      return;
    }
    delay(2000);
  }

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "No WiFi Found");
  u8g2.drawStr(0, 24, "Will auto-connect");
  u8g2.drawStr(0, 36, "when available");
  u8g2.drawStr(0, 48, "or use menu");
  u8g2.sendBuffer();
  delay(2000);
}

void bootSyncTime() {
  if (WiFi.status() != WL_CONNECTED) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 24, "No WiFi!");
    u8g2.drawStr(0, 36, "Time not synced!");
    u8g2.drawStr(0, 48, "Starting offline...");
    u8g2.sendBuffer();
    delay(2000);
    return;
  }

  struct tm timeinfo;
  const int MAX_ATTEMPTS = 5;
  for (int attempts = 0; attempts < MAX_ATTEMPTS; attempts++) {
    char attemptStr[20];
    snprintf(attemptStr, sizeof(attemptStr), "Attempt: %d/%d", attempts + 1, MAX_ATTEMPTS);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 12, "Syncing Time...");
    u8g2.drawStr(0, 24, attemptStr);
    u8g2.sendBuffer();

    configTime(settings.gmtOffset, 0, ntpServer, "time.nist.gov");
    delay(1000);
    if (getLocalTime(&timeinfo, 500) && (timeinfo.tm_year + 1900 >= 2024)) {
      ntpSynced = true;
      lastNtpSyncMillis = millis();
      break;
    }
  }

  if (ntpSynced) {
    char timeStr[20], dateStr[20], gmtStr[30];
    strftime(timeStr, sizeof(timeStr), "Time: %H:%M:%S", &timeinfo);
    strftime(dateStr, sizeof(dateStr), "Date: %d.%m.%Y", &timeinfo);
    strftime(gmtStr, sizeof(gmtStr), "GMT: %z %Z", &timeinfo);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_t0_14_tr);
    u8g2.drawStr(0, 16, timeStr);
    u8g2.drawStr(0, 32, dateStr);
    u8g2.drawStr(0, 62, gmtStr);
    u8g2.sendBuffer();
    delay(2000);
  } else {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 12, "Time not synced!");
    u8g2.drawStr(0, 24, "Check internet!");
    u8g2.drawStr(0, 48, "Starting offline...");
    u8g2.sendBuffer();
    delay(2000);
  }
}

void setup() {
  // 1. Configure Relay & Control Pins FIRST (Both Relays OFF at startup!)
  pinMode(MAIN_RELAY, OUTPUT);
  digitalWrite(MAIN_RELAY, HIGH);  // Main Output Relay OFF
  pinMode(PZEM_RELAY, OUTPUT);
  digitalWrite(PZEM_RELAY, HIGH);  // PZEM AC Input Relay OFF (Delayed Start!)

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(RESET_BTN, INPUT);
  pinMode(BOOT_BTN, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);

  // 2. Initialize Serial Buses (CRITICAL FIX: RX=16, TX=17 for PZEM!)
  Serial.begin(115200);
  espSerial.begin(115200, SERIAL_8N1, ESP_COMM_RX, ESP_COMM_TX);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);  // Pin 16 = RX2, Pin 17 = TX2

  // 3. Initialize Display & Show Welcome Message (3 seconds)
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB18_tr);
  u8g2.drawStr(0, 22, "ESP");
  u8g2.setFont(u8g2_font_ncenR12_tr);
  u8g2.drawStr(0, 40, "ENERGY - LOG");
  u8g2.setFont(u8g2_font_t0_11_tr);
  u8g2.drawStr(2, 60, "developed by M.Maity");
  u8g2.sendBuffer();
  delay(3000);

  // Double beep after welcome screen
  digitalWrite(BUZZER_PIN, HIGH);
  delay(300);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(300);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);

  // 4. Step-by-Step Hardware, Encoder, and Storage Init Screen
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "Hardware Init...");
  u8g2.sendBuffer();
  delay(300);

  u8g2.drawStr(0, 24, "Encoder Setup...");
  u8g2.sendBuffer();
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_DT), encoderISR, CHANGE);
  setEncoderPos(0);
  delay(300);

  u8g2.drawStr(0, 36, "Storage Init...");
  u8g2.sendBuffer();
  if (!SPIFFS.begin(true)) {
    u8g2.drawStr(0, 48, "SPIFFS Error!");
  } else {
    u8g2.drawStr(0, 48, "SPIFFS OK");
  }
  u8g2.sendBuffer();
  delay(300);

  preferences.begin("energy-meter", false);
  lastResetEpoch = preferences.getULong("rstEpoch", 0);
  String savedRstStr = preferences.getString("rstTime", "Not Reset");
  strncpy(lastResetTimeStr, savedRstStr.c_str(), sizeof(lastResetTimeStr) - 1);
  lastResetTimeStr[sizeof(lastResetTimeStr) - 1] = '\0';
  u8g2.drawStr(0, 60, "Days Fetched!");
  u8g2.sendBuffer();
  delay(300);

  // 5. Load Settings
  u8g2.clearBuffer();
  u8g2.drawStr(0, 12, "Loading Settings...");
  u8g2.sendBuffer();
  loadSettings();
  loadUsers();
  loadSchedulesFromNVS();
  delay(400);

  // 6. Connect to Saved WiFi (with status screen)
  bootConnectWiFi();

  // 7. Sync NTP Date & Time (with status screen)
  bootSyncTime();

  // 8. Start Web Server & mDNS
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "Starting Server...");
  u8g2.sendBuffer();
  setupWebServerRoutes();
  if (WiFi.status() == WL_CONNECTED) {
    setupMDNS();
    u8g2.clearBuffer();
    u8g2.drawStr(0, 12, "Server Started!");
    u8g2.drawStr(0, 24, "IP Address:");
    u8g2.drawStr(0, 36, WiFi.localIP().toString().c_str());
    u8g2.sendBuffer();
    delay(2000);
  } else {
    u8g2.clearBuffer();
    u8g2.drawStr(0, 12, "Offline Mode");
    u8g2.drawStr(0, 24, "Server Ready");
    u8g2.sendBuffer();
    delay(1500);
  }

  // 9. System Ready -> NOW Turn ON PZEM AC Input Relay (Delayed Start!)
  u8g2.clearBuffer();
  int16_t strWidth = u8g2.getStrWidth("System Ready!");
  int16_t x = (128 - strWidth) / 2;
  u8g2.drawStr(x, 36, "System Ready!");
  u8g2.sendBuffer();

  digitalWrite(PZEM_RELAY, LOW);  // Power ON PZEM sensor now!
  delay(1500);                    // Wait 1.5s for PZEM-004T to boot & stabilize

  // 10. Take initial PZEM reading & send handshake to Secondary ESP32
  updatePzemReadings();
  espSerial.println("HELLO");
  shareDataWithSecondary();

  // 11. Flush any accidental button press before entering Startup Safety Check
  checkEncoderButtonClick();
  startupRemainingSec = max(5, settings.startupTime);
  startupCheckCompleted = false;
  uiState = UI_STARTUP_SAFETY;
  lastStartupTickMillis = millis();
}

void loop() {
  checkBootFactoryReset();
  updateBuzzer();
  updatePzemReadings();
  checkTimersAndSchedules();
  controlMainOutput();
  checkSafetyAlerts();
  shareDataWithSecondary();
  maintainWiFiAndServices();
  updateNtpSilently();
  sendDataToNodeRed();
  updateDisplayAndUI();
}
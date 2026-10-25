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
#include <esp_wifi.h>
#include <esp_sntp.h>

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

// Manual NTP Sync Status Tracking (Web UI & LCD)
enum NtpSyncReqState {
  NTP_REQ_IDLE = 0,
  NTP_REQ_SYNCING,
  NTP_REQ_SUCCESS,
  NTP_REQ_FAILED
};
volatile NtpSyncReqState ntpReqState = NTP_REQ_IDLE;
volatile bool ntpReqFromLCD = false;
unsigned long ntpReqStartMillis = 0;

// Relay & Protection States
bool relayPhysicalOn = false;
bool shouldTurnOff = false;
bool protectionWasActive = false;
bool waitingForDelay = false;
unsigned long protectionClearedTime = 0;
char protectionStatusText[16] = "MAIN: ON";

// ADD (Feature 1): Real-time Protection Notice Tracking
char protPhase[16] = "none";       // "none", "fault", "wait", "kwh_saver", "sensor_err"
char protName[36] = "None";        // Active or last triggered protection name
char protTriggeredAt[28] = "---";  // Timestamp when protection triggered
int protRemainingSec = 0;          // Remaining seconds before Main Output turns ON

// ADD (Feature 2): Max 3 Login / Verification Attempts Lockout (Volatile until reboot)
#define MAX_AUTH_ATTEMPTS 3
volatile int failedAuthAttempts = 0;
volatile bool authLockedOut = false;

// ADD (Feature 4): Volatile Last Manual NTP Sync Timestamp String
char volatileNtpSyncStr[28] = "";  // Resets on reboot (not saved in NVS)

// Quick Countdown Timer (Volatile)
bool quickTimerActive = false;
bool quickTimerTurnOn = false;
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

// Physical BOOT Button Guard (5s window to press BOOT + 15s hold)
// Shared by: Forget User, Reset Data, and Factory Reset
enum ForgetUserState {
  FORGET_IDLE,
  FORGET_WAIT_PRESS,
  FORGET_HOLDING,
  FORGET_DONE,
  FORGET_TIMEOUT,
  FORGET_ABORTED
};

enum HoldResetTarget {
  HOLD_TARGET_NONE = 0,
  HOLD_TARGET_USERS,
  HOLD_TARGET_DATA,
  HOLD_TARGET_FACTORY
};

volatile ForgetUserState forgetUserState = FORGET_IDLE;
volatile HoldResetTarget holdResetTarget = HOLD_TARGET_NONE;
unsigned long forgetUserTimerMillis = 0;
int lastForgetBeepSec = -1;
bool manualWiFiDisconnected = false;

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

bool mdnsRunning = false;

String getMacSuffix() {
  String mac = WiFi.macAddress();
  mac.replace(":", "");
  mac.toUpperCase();
  if (mac.length() >= 4 && mac != "000000000000") {
    return mac.substring(mac.length() - 4);
  }
  return "0000";
}

String getDefaultMdnsName() {
  String suffix = getMacSuffix();
  suffix.toLowerCase();
  return "energylog-" + suffix;
}

void setupMDNS() {
  if (mdnsRunning) {
    MDNS.end();
    mdnsRunning = false;
  }
  if (!settings.mdnsEnabled || WiFi.status() != WL_CONNECTED || WiFi.localIP() == IPAddress(0, 0, 0, 0)) {
    return;
  }

  String host = String(settings.mdnsHostname);
  host.trim();
  if (host.endsWith(".local") || host.endsWith(".LOCAL")) {
    host = host.substring(0, host.length() - 6);
  }
  if (host.length() == 0 || host.equalsIgnoreCase("energylog-0000")) {
    host = getDefaultMdnsName();
    strncpy(settings.mdnsHostname, host.c_str(), 31);
    settings.mdnsHostname[31] = '\0';
  }

  String lowerHost = host;
  lowerHost.toLowerCase();
  WiFi.setHostname(lowerHost.c_str());

  if (MDNS.begin(lowerHost.c_str())) {
    MDNS.addService("http", "tcp", 80);
    mdnsRunning = true;
    Serial.printf("[mDNS] Responder started: http://%s.local\n", lowerHost.c_str());
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
  long p = isrEncoderPos;  // Read ISR position directly without double-ticking
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

// 45ms settling filter: absorbs mechanical quadrature bounce so 1 click = 1 clean step
int readEncoderStep() {
  static unsigned long lastStepTime = 0;
  unsigned long now = millis();
  if (now - lastStepTime < 45UL) return 0;

  long pos = getEncoderPos();
  if (pos == lastEncoderPos) return 0;

  int step = (pos > lastEncoderPos) ? 1 : -1;
  lastEncoderPos = pos;
  lastStepTime = now;
  lastMenuActivityMillis = now;
  return step;
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
  settings.mdnsEnabled = false;  // Default: mDNS OFF
  strncpy(settings.mdnsHostname, getDefaultMdnsName().c_str(), 31);
  settings.mdnsHostname[31] = '\0';
  settings.gmtOffset = 19800;  // IST UTC+5:30

  // All Safety Alerts OFF
  settings.highVoltageAlert = false;
  settings.lowVoltageAlert = false;
  settings.highPowerAlert = false;
  settings.lowPowerAlert = false;
  settings.highFreqAlert = false;
  settings.lowFreqAlert = false;

  // All Safety Protections OFF
  settings.highVProtec = false;
  settings.LowVProtec = false;
  settings.highPProtec = false;
  settings.highHzProtec = false;
  settings.lowHzProtec = false;

  // Features: Main Output ON, kWh Saver OFF, AFPC OFF
  settings.afpcEnabled = false;
  settings.mainOutput = true;
  settings.kWhSaver = false;
  settings.buzzerEnabled = true;

  // Default Thresholds
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

  // Default Intervals & Startup Delay (60s)
  settings.startupTime = 60;
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
      preferences.putBool("needAP", true);  // First-time start -> launch AP
    }
  } else {
    applyDefaultSettings();
    saveSettings();
    preferences.putBool("needAP", true);  // First-time start -> launch AP
  }
  if (strlen(settings.mdnsHostname) == 0 || strcasecmp(settings.mdnsHostname, "energylog-0000") == 0) {
    strncpy(settings.mdnsHostname, getDefaultMdnsName().c_str(), 31);
    settings.mdnsHostname[31] = '\0';
    saveSettings();
  }
  if (settings.startupTime < 5) {
    settings.startupTime = 60;  // Ensure startup safety check always runs
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

void recordAuthFailure() {
  failedAuthAttempts++;
  if (failedAuthAttempts >= MAX_AUTH_ATTEMPTS) {
    authLockedOut = true;
    triggerBeep(300, 3, 100);
  }
}

bool verifyUserCreds(const String& u, const String& p, bool requireAdmin, String& outRole) {
  if (authLockedOut) return false;
  for (int i = 0; i < MAX_USERS; i++) {
    if (users[i].active && u == users[i].username && p == users[i].password) {
      outRole = String(users[i].role);
      if (requireAdmin && outRole != "admin") {
        recordAuthFailure();
        return false;
      }
      failedAuthAttempts = 0;  // Reset counter on valid login before lockout
      return true;
    }
  }
  recordAuthFailure();
  return false;
}

bool verifyAdminCreds(const String& u, const String& p) {
  String role;
  return verifyUserCreds(u, p, true, role);
}

// Helper to record exact timestamp when a protection triggers
void recordProtectionTrigger(const char* reason) {
  strncpy(protName, reason, sizeof(protName) - 1);
  protName[sizeof(protName) - 1] = '\0';

  struct tm ti;
  if (ntpSynced && getLocalTime(&ti, 0) && (ti.tm_year + 1900 >= 2024)) {
    strftime(protTriggeredAt, sizeof(protTriggeredAt), "%d.%m.%Y %H:%M:%S", &ti);
  } else {
    unsigned long sec = millis() / 1000UL;
    snprintf(protTriggeredAt, sizeof(protTriggeredAt), "Uptime %02lu:%02lu:%02lu",
             sec / 3600UL, (sec % 3600UL) / 60UL, sec % 60UL);
  }
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
// PHYSICAL BOOT BUTTON GUARD & RESET ENGINE (5s PRESS + 15s HOLD)
// ============================================================================
const char* getHoldTargetTitle() {
  if (holdResetTarget == HOLD_TARGET_DATA) return "RESET ALL DATA";
  if (holdResetTarget == HOLD_TARGET_FACTORY) return "FACTORY RESET";
  return "FORGET USER";
}

void startHoldResetFlow(HoldResetTarget target) {
  holdResetTarget = target;
  forgetUserState = FORGET_WAIT_PRESS;
  forgetUserTimerMillis = millis();
  lastForgetBeepSec = -1;
  triggerBeep(80, 2, 60);
}

void performSafeRestart() {
  digitalWrite(MAIN_RELAY, HIGH);  // Safe state: disengage load relay
  relayPhysicalOn = false;
  digitalWrite(PZEM_RELAY, HIGH);  // Power down PZEM sensor cleanly
  digitalWrite(BUZZER_PIN, LOW);   // Silence buzzer
  server.end();                    // Close open HTTP connections
  if (mdnsRunning) {
    MDNS.end();
    mdnsRunning = false;
  }
  preferences.end();
  delay(150);
  ESP.restart();
}

void eraseWiFiCredentialsNVS() {
  if (mdnsRunning) {
    MDNS.end();
    mdnsRunning = false;
  }
  WiFi.mode(WIFI_STA);  // Keep WiFi driver initialized so esp_wifi_restore() succeeds
  WiFi.persistent(true);
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false, true);  // Disconnect & wipe STA SSID/Pass without deinitializing driver
  esp_wifi_restore();            // Completely clear ESP-IDF WiFi NVS
  preferences.putBool("needAP", true);
}

void resetUsersToDefault() {
  memset(users, 0, sizeof(users));
  users[0].active = true;
  strcpy(users[0].username, "esp");
  strcpy(users[0].password, "456456");
  strcpy(users[0].role, "admin");
  saveUsers();
}

// Helper to reliably clear PZEM energy counter
bool clearPzemEnergyReliable() {
  digitalWrite(PZEM_RELAY, LOW);
  for (int attempt = 0; attempt < 5; attempt++) {
    while (Serial2.available()) Serial2.read();
    delay(150);
    if (pzem.resetEnergy()) {
      return true;
    }
  }
  return false;
}

void executeDataReset() {
  u8g2.clearBuffer();
  u8g2.drawFrame(0, 0, 128, 64);
  u8g2.drawBox(0, 0, 128, 14);
  u8g2.setDrawColor(0);
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(22, 11, "RESET ALL DATA");
  u8g2.setDrawColor(1);
  u8g2.drawStr(10, 34, "Restoring Defaults");
  u8g2.drawStr(18, 50, "Rebooting System...");
  u8g2.sendBuffer();

  digitalWrite(MAIN_RELAY, HIGH);  // Safe state: Relay OFF immediately
  relayPhysicalOn = false;
  digitalWrite(BUZZER_PIN, HIGH);
  delay(600);
  digitalWrite(BUZZER_PIN, LOW);

  applyDefaultSettings();
  saveSettings();

  // Clear all Daily & Future Schedules
  memset(dailySchedules, 0, sizeof(dailySchedules));
  memset(futureSchedules, 0, sizeof(futureSchedules));
  saveSchedulesToNVS();

  // Clear PZEM energy & billing counters reliably
  clearPzemEnergyReliable();
  if (ntpSynced) {
    recordEnergyResetTimestamp();
  } else {
    lastResetEpoch = 0;
    strcpy(lastResetTimeStr, "Not Reset");
    preferences.putULong("rstEpoch", 0);
    preferences.putString("rstTime", "Not Reset");
    kwhDays = 0.0f;
  }

  portENTER_CRITICAL(&measureMux);
  readings.energy = 0.0f;
  readings.totalDays = 0.0f;
  portEXIT_CRITICAL(&measureMux);

  delay(200);
  performSafeRestart();
}

void executeFullFactoryReset() {
  u8g2.clearBuffer();
  u8g2.drawFrame(0, 0, 128, 64);
  u8g2.drawBox(0, 0, 128, 14);
  u8g2.setDrawColor(0);
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(22, 11, "FACTORY RESET");
  u8g2.setDrawColor(1);
  u8g2.drawStr(10, 34, "Restoring Defaults");
  u8g2.drawStr(18, 50, "Rebooting System...");
  u8g2.sendBuffer();

  digitalWrite(MAIN_RELAY, HIGH);  // Safe state: Relay OFF immediately
  relayPhysicalOn = false;
  digitalWrite(BUZZER_PIN, HIGH);
  delay(800);
  digitalWrite(BUZZER_PIN, LOW);

  // 1. Reset all "Reset Data" parameters (Settings, Schedules, PZEM Energy & Billing)
  applyDefaultSettings();
  saveSettings();

  memset(dailySchedules, 0, sizeof(dailySchedules));
  memset(futureSchedules, 0, sizeof(futureSchedules));
  saveSchedulesToNVS();

  clearPzemEnergyReliable();
  if (ntpSynced) {
    recordEnergyResetTimestamp();
  } else {
    lastResetEpoch = 0;
    strcpy(lastResetTimeStr, "Not Reset");
    preferences.putULong("rstEpoch", 0);
    preferences.putString("rstTime", "Not Reset");
    kwhDays = 0.0f;
  }

  portENTER_CRITICAL(&measureMux);
  readings.energy = 0.0f;
  readings.totalDays = 0.0f;
  portEXIT_CRITICAL(&measureMux);

  // 2. Restore User Accounts to default (esp / 456456)
  resetUsersToDefault();

  // 3. Erase WiFi credentials & flag 'EnergyLog' AP on boot
  eraseWiFiCredentialsNVS();

  // 4. Mandatory safe restart
  delay(200);
  performSafeRestart();
}

void checkBootFactoryReset() {
  if (forgetUserState != FORGET_WAIT_PRESS && forgetUserState != FORGET_HOLDING) {
    return;
  }

  unsigned long now = millis();

  // --- Phase 1: 5s Window to Press BOOT ---
  if (forgetUserState == FORGET_WAIT_PRESS) {
    if (digitalRead(BOOT_BTN) == LOW) {
      forgetUserState = FORGET_HOLDING;
      forgetUserTimerMillis = now;
      lastForgetBeepSec = -1;
      triggerBeep(80, 1, 0);
    } else if (now - forgetUserTimerMillis >= 5000UL) {
      forgetUserState = FORGET_TIMEOUT;
      triggerBeep(150, 2, 80);
      showToastLCD(getHoldTargetTitle(), "5s Timeout!", 1500, UI_DASHBOARD);
    }
    return;
  }

  // --- Phase 2: 15s Continuous Hold ---
  if (forgetUserState == FORGET_HOLDING) {
    if (digitalRead(BOOT_BTN) == LOW) {
      unsigned long elapsed = now - forgetUserTimerMillis;
      if (elapsed >= 15000UL) {
        forgetUserState = FORGET_DONE;
        triggerBeep(200, 3, 100);

        if (holdResetTarget == HOLD_TARGET_USERS) {
          resetUsersToDefault();
          showToastLCD("Users Reset", "esp / 456456", 2000, UI_DASHBOARD);
        } else if (holdResetTarget == HOLD_TARGET_DATA) {
          pendingFactoryData = true;
          restartScheduledMillis = now + 1000;  // Allow Web UI 500ms poll to catch "done"
          showToastLCD("Data Reset", "Rebooting...", 1500, UI_DASHBOARD);
        } else if (holdResetTarget == HOLD_TARGET_FACTORY) {
          pendingFactoryAll = true;
          restartScheduledMillis = now + 1000;  // Allow Web UI 500ms poll to catch "done"
          showToastLCD("Factory Reset", "Rebooting...", 1500, UI_DASHBOARD);
        }
      } else {
        int remSec = 15 - (elapsed / 1000UL);
        if (remSec != lastForgetBeepSec) {
          lastForgetBeepSec = remSec;
          triggerBeep(40, 1, 0);
        }
      }
    } else {
      forgetUserState = FORGET_ABORTED;
      // triggerBeep(150, 2, 80);
      showToastLCD(getHoldTargetTitle(), "Hold Cancelled", 1500, UI_DASHBOARD);
    }
  }
}

// ============================================================================
// SILENT BACKGROUND NTP & AUTOMATION SCHEDULER
// ============================================================================
void updateNtpSilently() {
  unsigned long now = millis();

  if (WiFi.status() == WL_CONNECTED) {
    bool needRetry = !ntpSynced && (lastNtpSyncMillis == 0 || (now - lastNtpSyncMillis >= 30000UL));
    bool needHourly = ntpSynced && (now - lastNtpSyncMillis >= 3600000UL);

    if (pendingNtpSync || needRetry || needHourly) {
      sntp_set_sync_status(SNTP_SYNC_STATUS_RESET);
      configTime(settings.gmtOffset, 0, ntpServer, "time.nist.gov", "time.google.com");
      lastNtpSyncMillis = now;
      if (pendingNtpSync) {
        ntpReqState = NTP_REQ_SYNCING;
        ntpReqStartMillis = now;
      }
      pendingNtpSync = false;
    }
  } else if (pendingNtpSync) {
    pendingNtpSync = false;
    ntpReqState = NTP_REQ_FAILED;
    if (ntpReqFromLCD) {
      ntpReqFromLCD = false;
      // triggerBeep(150, 2, 80);
      showToastLCD("NTP Sync Failed", "No WiFi Network", 1500, UI_MAIN_MENU);
    }
  }

  // Evaluate active manual sync request (6-second timeout)
  if (ntpReqState == NTP_REQ_SYNCING) {
    struct tm ti;
    bool packetArrived = (sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED);
    bool timeValid = (now - ntpReqStartMillis >= 600UL) && getLocalTime(&ti, 0) && (ti.tm_year + 1900 >= 2024);

    if ((packetArrived || timeValid) && getLocalTime(&ti, 0) && (ti.tm_year + 1900 >= 2024)) {
      ntpSynced = true;
      ntpReqState = NTP_REQ_SUCCESS;
      strftime(volatileNtpSyncStr, sizeof(volatileNtpSyncStr), "%d.%m.%Y %H:%M:%S", &ti);
      if (ntpReqFromLCD) {
        ntpReqFromLCD = false;
        char tStr[20];
        strftime(tStr, sizeof(tStr), "Time: %H:%M:%S", &ti);
        // triggerBeep(80, 1, 0);
        showToastLCD("Time Updated!", tStr, 1500, UI_MAIN_MENU);
      }
    } else if (now - ntpReqStartMillis >= 6000UL) {
      ntpReqState = NTP_REQ_FAILED;
      if (ntpReqFromLCD) {
        ntpReqFromLCD = false;
        // triggerBeep(150, 2, 80);
        showToastLCD("NTP Sync Failed", "Server Timeout", 1500, UI_MAIN_MENU);
      }
    }
  }

  static unsigned long lastTimeCheck = 0;
  if (now - lastTimeCheck >= 1000UL) {
    lastTimeCheck = now;
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 0) && (timeinfo.tm_year + 1900 >= 2024)) {
      ntpSynced = true;
    }
  }
}

bool isProtectionRunning() {
  portENTER_CRITICAL(&measureMux);
  Measurements cur = readings;
  portEXIT_CRITICAL(&measureMux);

  if (settings.kWhSaver && cur.energy >= (float)settings.maxkWh) return true;

  bool anyProtEnabled = settings.highVProtec || settings.LowVProtec || settings.highPProtec || settings.highHzProtec || settings.lowHzProtec;

  // If protections are enabled but PZEM sensor is not valid, do not allow blind turn-on
  if (anyProtEnabled && !cur.valid) return true;

  if (cur.valid) {
    if (settings.highVProtec && cur.voltage > settings.voltHigh) return true;
    if (settings.LowVProtec && cur.voltage < settings.voltLow) return true;
    if (settings.highPProtec && cur.power > settings.powerHigh) return true;
    if (settings.highHzProtec && cur.frequency > settings.freqHigh) return true;
    if (settings.lowHzProtec && cur.frequency < settings.freqLow) return true;
  }
  return false;
}

void checkTimersAndSchedules() {
  // 1. Volatile Quick Countdown Timer (Supports ON or OFF + Protection Check)
  if (quickTimerActive) {
    if ((long)(millis() - quickTimerEndMillis) >= 0) {
      quickTimerActive = false;
      if (quickTimerTurnOn) {
        // Only allow ON if no live fault is active; if in countdown, wait for countdown to finish
        if (!isProtectionRunning()) {
          settings.mainOutput = true;
          saveSettings();
          if (!protectionWasActive) {
            shouldTurnOff = false;
            triggerBeep(200, 2, 100);
          }
        } else {
          Serial.println("[Timer] Skipped ON: Protection is currently active!");
          triggerBeep(120, 3, 80);
        }
      } else {
        settings.mainOutput = false;
        saveSettings();
        triggerBeep(200, 2, 100);
      }
    }
  }

  // 2. Persistent Daily & Future Schedules (checked once per second when NTP time is valid)
  static unsigned long lastSchedCheck = 0;
  if (millis() - lastSchedCheck < 1000UL) return;
  lastSchedCheck = millis();

  struct tm ti;
  if (!getLocalTime(&ti, 0) || (ti.tm_year + 1900 < 2024)) return;

  int uiDay = (ti.tm_wday == 0) ? 6 : (ti.tm_wday - 1);

  for (int i = 0; i < MAX_DAILY_SCHEDULES; i++) {
    if (dailySchedules[i].active && (dailySchedules[i].daysMask & (1 << uiDay)) && dailySchedules[i].hour == ti.tm_hour && dailySchedules[i].minute == ti.tm_min && dailySchedules[i].lastTriggeredYDay != ti.tm_yday) {
      dailySchedules[i].lastTriggeredYDay = ti.tm_yday;
      if (dailySchedules[i].turnOn) {
        if (!isProtectionRunning()) {
          settings.mainOutput = true;
          saveSettings();
          // Only beep immediately if not waiting on protection countdown
          // (controlMainOutput will beep when the countdown finishes)
          if (!protectionWasActive) {
            shouldTurnOff = false;
            triggerBeep(120, 2, 80);
          }
        }
      } else {
        settings.mainOutput = false;
        saveSettings();
        triggerBeep(120, 2, 80);
      }
      saveSchedulesToNVS();
    }
  }

  bool futureChanged = false;
  for (int i = 0; i < MAX_FUTURE_SCHEDULES; i++) {
    if (futureSchedules[i].active && futureSchedules[i].year == (ti.tm_year + 1900) && futureSchedules[i].month == (ti.tm_mon + 1) && futureSchedules[i].day == ti.tm_mday && futureSchedules[i].hour == ti.tm_hour && futureSchedules[i].minute == ti.tm_min) {
      if (futureSchedules[i].turnOn) {
        if (!isProtectionRunning()) {
          settings.mainOutput = true;
          saveSettings();
          if (!protectionWasActive) {
            shouldTurnOff = false;
            triggerBeep(120, 2, 80);
          }
        }
      } else {
        settings.mainOutput = false;
        saveSettings();
        triggerBeep(120, 2, 80);
      }
      futureSchedules[i].active = false;
      futureChanged = true;
    }
  }
  if (futureChanged) saveSchedulesToNVS();
}

// ============================================================================
// PZEM-004T READING
// ============================================================================
void updatePzemReadings() {
  if (resetKwhRequested || pendingFactoryData || pendingFactoryAll || pendingRestart) return;
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

  // Reject NaN AND reject physically impossible transient register values
  // (PZEM-004T v3.0 hardware range is 80-260V / 45-65Hz; allow 40-400V & 35-75Hz margin)
  bool vPlausible = !isnan(v) && (v >= 40.0f && v <= 400.0f);
  bool hzPlausible = !isnan(hz) && (hz >= 35.0f && hz <= 75.0f);
  bool sensorOk = vPlausible && hzPlausible;

  if (!sensorOk) {
    // Clear any misaligned bytes in UART2 RX buffer on bad/transient read
    while (Serial2.available()) {
      Serial2.read();
    }
  }

  portENTER_CRITICAL(&measureMux);
  readings.valid = sensorOk;
  readings.voltage = sensorOk ? v : 0.0f;
  readings.current = (sensorOk && !isnan(c) && c >= 0.0f && c <= 120.0f) ? c : 0.0f;
  readings.power = (sensorOk && !isnan(p) && p >= 0.0f && p <= 35000.0f) ? p : 0.0f;
  if (sensorOk && !isnan(e) && e >= 0.0f) readings.energy = e;
  readings.frequency = sensorOk ? hz : 0.0f;
  readings.pf = (sensorOk && !isnan(pf) && pf >= 0.0f && pf <= 1.0f) ? pf : 0.0f;
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

  // 1. Evaluate Protection Faults FIRST
  bool anyProtEnabled = settings.highVProtec || settings.LowVProtec || settings.highPProtec || settings.highHzProtec || settings.lowHzProtec;
  bool faultActive = false;
  const char* currentFaultReason = "";

  if (cur.valid) {
    if (settings.highVProtec && cur.voltage > settings.voltHigh) {
      faultActive = true;
      currentFaultReason = "High Voltage Protection";
    } else if (settings.LowVProtec && cur.voltage < settings.voltLow) {
      faultActive = true;
      currentFaultReason = "Low Voltage Protection";
    } else if (settings.highPProtec && cur.power > settings.powerHigh) {
      faultActive = true;
      currentFaultReason = "High Power Overload";
    } else if (settings.highHzProtec && cur.frequency > settings.freqHigh) {
      faultActive = true;
      currentFaultReason = "High Frequency Protection";
    } else if (settings.lowHzProtec && cur.frequency < settings.freqLow) {
      faultActive = true;
      currentFaultReason = "Low Frequency Protection";
    }
  } else if (anyProtEnabled) {
    // Sensor communication lost while active protection is required
    if (strcmp(protPhase, "sensor_err") != 0) {
      recordProtectionTrigger("PZEM Sensor Comm Error");
    }
    strcpy(protPhase, "sensor_err");
    protRemainingSec = max(10, settings.protectionDelay);
    shouldTurnOff = true;
    digitalWrite(MAIN_RELAY, HIGH);
    relayPhysicalOn = false;
    strcpy(protectionStatusText, "SENSOR ERR");
    return;
  }

  // If a grid fault is actively present right now:
  if (faultActive) {
    if (!protectionWasActive || waitingForDelay || strcmp(protName, currentFaultReason) != 0) {
      recordProtectionTrigger(currentFaultReason);
    }
    protectionWasActive = true;
    waitingForDelay = false;  // Reset countdown until fault releases
    shouldTurnOff = true;
    digitalWrite(MAIN_RELAY, HIGH);
    relayPhysicalOn = false;
    strcpy(protPhase, "fault");
    protRemainingSec = max(10, settings.protectionDelay);
    strcpy(protectionStatusText, "PROT TRIP");
    return;
  }

  // 2. Handle Protection Recovery Wait Timer (After fault is released)
  if (protectionWasActive) {
    if (!waitingForDelay) {
      protectionClearedTime = millis();
      waitingForDelay = true;
    }

    unsigned long elapsed = millis() - protectionClearedTime;
    if (elapsed >= protDelayMs || digitalRead(RESET_BTN) == HIGH) {
      protectionWasActive = false;
      waitingForDelay = false;
      strcpy(protPhase, "none");
      protRemainingSec = 0;
      if (settings.mainOutput) triggerBeep(150, 1, 0);
    } else {
      shouldTurnOff = true;
      digitalWrite(MAIN_RELAY, HIGH);
      relayPhysicalOn = false;
      unsigned long rem = protDelayMs - elapsed;
      protRemainingSec = (int)((rem + 999UL) / 1000UL);
      strcpy(protPhase, "wait");
      int mins = rem / 60000UL;
      int secs = (rem % 60000UL) / 1000UL;
      snprintf(protectionStatusText, sizeof(protectionStatusText), "Wait:%02d:%02d", mins, secs);
      return;
    }
  }

  // 3. kWh Energy Saver Cutoff
  if (settings.kWhSaver && cur.energy >= (float)settings.maxkWh) {
    if (strcmp(protPhase, "kwh_saver") != 0) {
      recordProtectionTrigger("kWh Saver Quota Reached");
    }
    strcpy(protPhase, "kwh_saver");
    protRemainingSec = 0;
    shouldTurnOff = true;
    digitalWrite(MAIN_RELAY, HIGH);
    relayPhysicalOn = false;
    strcpy(protectionStatusText, "kWh SAVER");
    return;
  }

  // Clear protection notice phase when no protection is holding output off
  strcpy(protPhase, "none");
  protRemainingSec = 0;

  // 4. Manual Output OFF
  if (!settings.mainOutput) {
    shouldTurnOff = true;
    digitalWrite(MAIN_RELAY, HIGH);
    relayPhysicalOn = false;
    strcpy(protectionStatusText, "MAIN: OFF");
    return;
  }

  // 5. All conditions normal -> Engage Relay
  shouldTurnOff = false;
  digitalWrite(MAIN_RELAY, LOW);
  relayPhysicalOn = true;
  strcpy(protectionStatusText, "MAIN: ON");
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
  doc["out"] = relayPhysicalOn;
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
  doc["mainOutput"] = relayPhysicalOn ? 1 : 0;
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
    if (authLockedOut) {
      request->send(403, "text/html",
                    "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>Locked</title></head>"
                    "<body style='background:#020617;color:#f87171;font-family:sans-serif;display:flex;align-items:center;justify-content:center;height:100vh;margin:0;text-align:center;padding:20px;'>"
                    "<div style='background:#0f172a;border:1px solid #7f1d1d;padding:28px;border-radius:16px;max-width:400px;'>"
                    "<h2 style='margin-top:0;color:#ef4444;'>Security Lockout Active</h2>"
                    "<p style='color:#94a3b8;font-size:13px;line-height:1.6;'>Maximum (3) login/verification attempts exceeded. <b>config page</b> is strictly stopped until you restart the device.</p>"
                    "<a href='/index.html' style='display:inline-block;margin-top:14px;padding:10px 18px;background:#1e293b;color:#f8fafc;text-decoration:none;border-radius:10px;font-size:12px;'>Back to Home Page</a>"
                    "</div></body></html>");
      return;
    }
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

    StaticJsonDocument<768> doc;
    doc["voltage"] = String(cur.voltage, 1);
    doc["current"] = String(cur.current, 2);
    doc["power"] = String(cur.power, 1);
    doc["energy"] = String(cur.energy, 2);
    doc["frequency"] = String(cur.frequency, 2);
    doc["pf"] = String(cur.pf, 2);
    doc["uptime"] = String(cur.uptime, 1);
    doc["days"] = String(cur.totalDays, 1);
    doc["mainOutput"] = relayPhysicalOn ? 1 : 0;
    doc["mainOutputSetting"] = settings.mainOutput ? 1 : 0;
    doc["statusText"] = protectionStatusText;
    doc["resetTime"] = lastResetTimeStr;
    doc["ntpSynced"] = ntpSynced ? 1 : 0;

    // ADD (Feature 1): Protection real-time telemetry
    doc["protPhase"] = protPhase;
    doc["protName"] = protName;
    doc["protTime"] = protTriggeredAt;
    doc["protRem"] = protRemainingSec;

    // ADD (Feature 2): Lockout state for Home page relay button
    doc["authLocked"] = authLockedOut ? 1 : 0;

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
    doc["mainOutput"] = relayPhysicalOn;
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
    doc["lastNtpSync"] = volatileNtpSyncStr;

    String res;
    serializeJson(doc, res);
    request->send(200, "application/json", res);
  });

  // Main Output Toggle from Home Page
  server.on(
    "/output/set", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      if (authLockedOut) {
        request->send(403, "application/json", "{\"success\":false,\"locked\":true,\"error\":\"Relay locked! Restart device.\"}");
        return;
      }
      StaticJsonDocument<128> doc;
      if (!deserializeJson(doc, data, len)) {
        bool state = doc["state"];
        if (state) {
          if (isProtectionRunning()) {
            triggerBeep(150, 2, 80);
            request->send(400, "application/json", "{\"success\":false,\"error\":\"Not possible! Protection Running\"}");
            return;
          }
          // Fault is already released: cancel countdown and engage output immediately
          protectionWasActive = false;
          waitingForDelay = false;
          strcpy(protPhase, "none");
          protRemainingSec = 0;
        }
        settings.mainOutput = state;
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
        bool reqState = doc["value"].as<bool>();
        if (reqState) {
          if (isProtectionRunning()) {
            triggerBeep(150, 2, 80);
            request->send(400, "application/json", "{\"success\":false,\"error\":\"Not possible! Protection Running\"}");
            return;
          }
          protectionWasActive = false;
          waitingForDelay = false;
          strcpy(protPhase, "none");
          protRemainingSec = 0;
        }
        settings.mainOutput = reqState;
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
      else if (key == "startupTime") settings.startupTime = constrain(doc["value"].as<int>(), 5, 600);

      saveSettings();
      request->send(200, "application/json", "{\"success\":true}");
    });

  // Batch Section Save Endpoints
  server.on(
    "/output/save", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<128> doc;
      if (!deserializeJson(doc, data, len)) {
        bool reqMain = doc["mainOutput"] | settings.mainOutput;
        settings.kWhSaver = doc["kWhSaver"] | settings.kWhSaver;
        if (reqMain) {
          if (isProtectionRunning()) {
            saveSettings();
            triggerBeep(150, 2, 80);
            request->send(400, "application/json", "{\"success\":false,\"error\":\"Not possible! Protection Running\"}");
            return;
          }
          protectionWasActive = false;
          waitingForDelay = false;
          strcpy(protPhase, "none");
          protRemainingSec = 0;
        }
        settings.mainOutput = reqMain;
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
        float vh = doc["voltHigh"] | settings.voltHigh;
        float vl = doc["voltLow"] | settings.voltLow;
        float ph = doc["powerHigh"] | settings.powerHigh;
        float pl = doc["powerLow"] | settings.powerLow;
        float fh = doc["freqHigh"] | settings.freqHigh;
        float fl = doc["freqLow"] | settings.freqLow;
        int al = doc["afpcLowWatt"] | settings.afpcLowWatt;
        int ah = doc["afpcHighWatt"] | settings.afpcHighWatt;

        if (vl >= vh || pl >= ph || fl >= fh || al >= ah) {
          request->send(400, "application/json", "{\"success\":false,\"error\":\"Low limit must be less than High limit\"}");
          return;
        }
        settings.voltHigh = vh;
        settings.voltLow = vl;
        settings.powerHigh = ph;
        settings.powerLow = pl;
        settings.maxkWh = doc["maxkWh"] | settings.maxkWh;
        settings.freqHigh = fh;
        settings.freqLow = fl;
        settings.afpcLowWatt = al;
        settings.afpcHighWatt = ah;
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
        settings.startupTime = constrain(doc["startupTime"] | settings.startupTime, 5, 600);
        saveSettings();
      }
      request->send(200, "application/json", "{\"success\":true}");
    });

  // mDNS Endpoints (Save, Reset, Delete)
  server.on(
    "/mdns/save", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<192> doc;
      if (!deserializeJson(doc, data, len)) {
        if (doc.containsKey("enabled")) {
          settings.mdnsEnabled = doc["enabled"].as<bool>();
        }
        String rawHost = doc["hostname"] | "";
        rawHost.trim();
        rawHost.toLowerCase();
        if (rawHost.endsWith(".local")) {
          rawHost = rawHost.substring(0, rawHost.length() - 6);
        }
        String cleanHost = "";
        for (size_t i = 0; i < rawHost.length() && cleanHost.length() < 30; i++) {
          char c = rawHost.charAt(i);
          if (isalnum(c) || c == '-') cleanHost += c;
        }
        while (cleanHost.startsWith("-")) cleanHost.remove(0, 1);
        while (cleanHost.endsWith("-")) cleanHost.remove(cleanHost.length() - 1);

        if (cleanHost.length() == 0) {
          cleanHost = getDefaultMdnsName();
        }
        strncpy(settings.mdnsHostname, cleanHost.c_str(), 31);
        settings.mdnsHostname[31] = '\0';

        saveSettings();
        pendingMdnsRestart = true;

        StaticJsonDocument<192> resDoc;
        resDoc["success"] = true;
        resDoc["enabled"] = settings.mdnsEnabled;
        resDoc["hostname"] = settings.mdnsHostname;
        String res;
        serializeJson(resDoc, res);
        request->send(200, "application/json", res);
      } else {
        request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
      }
    });

  server.on("/mdns/reset", HTTP_POST, [](AsyncWebServerRequest* request) {
    settings.mdnsEnabled = true;
    strncpy(settings.mdnsHostname, getDefaultMdnsName().c_str(), 31);
    settings.mdnsHostname[31] = '\0';
    saveSettings();
    pendingMdnsRestart = true;

    StaticJsonDocument<192> resDoc;
    resDoc["success"] = true;
    resDoc["enabled"] = settings.mdnsEnabled;
    resDoc["hostname"] = settings.mdnsHostname;
    String res;
    serializeJson(resDoc, res);
    request->send(200, "application/json", res);
  });

  server.on("/mdns/delete", HTTP_POST, [](AsyncWebServerRequest* request) {
    settings.mdnsEnabled = false;
    strncpy(settings.mdnsHostname, getDefaultMdnsName().c_str(), 31);
    settings.mdnsHostname[31] = '\0';
    saveSettings();
    pendingMdnsRestart = true;

    StaticJsonDocument<192> resDoc;
    resDoc["success"] = true;
    resDoc["enabled"] = false;
    resDoc["hostname"] = settings.mdnsHostname;
    String res;
    serializeJson(resDoc, res);
    request->send(200, "application/json", res);
  });

  // Node-RED Endpoints
  server.on(
    "/nodered/save", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<128> doc;
      if (!deserializeJson(doc, data, len)) {
        if (doc.containsKey("enabled")) {
          settings.nodeRedEnabled = doc["enabled"].as<bool>();
        }
        String addr = doc["address"] | "";
        if (addr.length() > 0 && addr.length() < 31) {
          strncpy(settings.nodeRedIP, addr.c_str(), 31);
          settings.nodeRedIP[31] = '\0';
        }
        saveSettings();
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
      request->send(400, "application/json", "{\"success\":false,\"error\":\"NTP not synced! Sync time first.\"}");
      return;
    }
    resetKwhRequested = true;
    request->send(200, "application/json", "{\"success\":true}");
  });

  server.on("/factory/reset-data", HTTP_POST, [](AsyncWebServerRequest* request) {
    startHoldResetFlow(HOLD_TARGET_DATA);
    request->send(200, "application/json", "{\"success\":true,\"state\":\"wait_press\"}");
  });

  server.on("/factory/reset-settings", HTTP_POST, [](AsyncWebServerRequest* request) {
    startHoldResetFlow(HOLD_TARGET_FACTORY);
    request->send(200, "application/json", "{\"success\":true,\"state\":\"wait_press\"}");
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
    if (WiFi.status() != WL_CONNECTED) {
      request->send(400, "application/json", "{\"success\":false,\"error\":\"WiFi disconnected! Connect to WiFi first.\"}");
      return;
    }
    ntpReqFromLCD = false;
    ntpReqState = NTP_REQ_SYNCING;
    ntpReqStartMillis = millis();
    pendingNtpSync = true;
    request->send(200, "application/json", "{\"success\":true,\"state\":\"syncing\"}");
  });

  server.on("/time/sync-status", HTTP_GET, [](AsyncWebServerRequest* request) {
    StaticJsonDocument<192> doc;
    if (ntpReqState == NTP_REQ_SYNCING) {
      doc["state"] = "syncing";
    } else if (ntpReqState == NTP_REQ_SUCCESS) {
      doc["state"] = "success";
      struct tm ti;
      if (getLocalTime(&ti, 0)) {
        char tBuf[16], dBuf[16];
        strftime(tBuf, sizeof(tBuf), "%H:%M:%S", &ti);
        strftime(dBuf, sizeof(dBuf), "%d/%m/%Y", &ti);
        doc["time"] = tBuf;
        doc["date"] = dBuf;
      }
      ntpReqState = NTP_REQ_IDLE;
      doc["lastSync"] = volatileNtpSyncStr;
    } else if (ntpReqState == NTP_REQ_FAILED) {
      doc["state"] = "failed";
      doc["error"] = (WiFi.status() != WL_CONNECTED) ? "WiFi disconnected" : "NTP server unreachable";
      ntpReqState = NTP_REQ_IDLE;
    } else {
      doc["state"] = "idle";
    }
    doc["ntpSynced"] = ntpSynced;
    String res;
    serializeJson(doc, res);
    request->send(200, "application/json", res);
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
    doc["action"] = quickTimerTurnOn ? "on" : "off";
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
        String action = doc["action"] | "off";
        if (durSec > 0) {
          quickTimerActive = true;
          quickTimerTurnOn = (action == "on");
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
        int rem = max(0, MAX_AUTH_ATTEMPTS - (int)failedAuthAttempts);
        StaticJsonDocument<192> errDoc;
        errDoc["success"] = false;
        errDoc["locked"] = authLockedOut;
        errDoc["remaining"] = rem;
        errDoc["error"] = authLockedOut
                            ? "Device locked (3 failed attempts)! Restart ESP32."
                            : ("Invalid admin credentials (" + String(rem) + " attempt(s) left)");
        String res;
        serializeJson(errDoc, res);
        request->send(403, "application/json", res);
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
          memset(&users[i], 0, sizeof(UserAccount));
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
        int rem = max(0, MAX_AUTH_ATTEMPTS - (int)failedAuthAttempts);
        StaticJsonDocument<192> errDoc;
        errDoc["success"] = false;
        errDoc["locked"] = authLockedOut;
        errDoc["remaining"] = rem;
        errDoc["error"] = authLockedOut
                            ? "Device locked (3 failed attempts)! Restart ESP32."
                            : ("Invalid admin credentials (" + String(rem) + " attempt(s) left)");
        String res;
        serializeJson(errDoc, res);
        request->send(403, "application/json", res);
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

  server.on("/users/forget-start", HTTP_POST, [](AsyncWebServerRequest* request) {
    startHoldResetFlow(HOLD_TARGET_USERS);
    request->send(200, "application/json", "{\"success\":true,\"state\":\"wait_press\"}");
  });

  auto sendHoldStatus = [](AsyncWebServerRequest* request) {
    StaticJsonDocument<256> doc;
    unsigned long now = millis();
    const char* targetStr = (holdResetTarget == HOLD_TARGET_DATA) ? "data" : (holdResetTarget == HOLD_TARGET_FACTORY) ? "factory"
                                                                                                                      : "users";
    doc["target"] = targetStr;

    if (forgetUserState == FORGET_WAIT_PRESS) {
      long rem = max(0L, 5L - (long)((now - forgetUserTimerMillis) / 1000UL));
      doc["state"] = "wait_press";
      doc["remaining"] = rem;
    } else if (forgetUserState == FORGET_HOLDING) {
      long rem = max(0L, 15L - (long)((now - forgetUserTimerMillis) / 1000UL));
      doc["state"] = "holding";
      doc["remaining"] = rem;
    } else if (forgetUserState == FORGET_DONE) {
      doc["state"] = "done";
      forgetUserState = FORGET_IDLE;
      holdResetTarget = HOLD_TARGET_NONE;
    } else if (forgetUserState == FORGET_TIMEOUT) {
      doc["state"] = "timeout";
      forgetUserState = FORGET_IDLE;
      holdResetTarget = HOLD_TARGET_NONE;
    } else if (forgetUserState == FORGET_ABORTED) {
      doc["state"] = "aborted";
      forgetUserState = FORGET_IDLE;
      holdResetTarget = HOLD_TARGET_NONE;
    } else {
      doc["state"] = "idle";
    }
    String res;
    serializeJson(doc, res);
    request->send(200, "application/json", res);
  };

  server.on("/users/forget-status", HTTP_GET, sendHoldStatus);
  server.on("/factory/reset-status", HTTP_GET, sendHoldStatus);

  // ADD (Feature 2): Server-side Login & Admin Verification with 3-Attempt Lockout
  server.on(
    "/auth/verify", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<256> doc;
      if (deserializeJson(doc, data, len)) {
        request->send(400, "application/json", "{\"success\":false}");
        return;
      }
      if (authLockedOut) {
        request->send(403, "application/json",
                      "{\"success\":false,\"locked\":true,\"remaining\":0,\"error\":\"Maximum 3 attempts exceeded! Restart device.\"}");
        return;
      }

      String u = doc["username"] | "";
      String p = doc["password"] | "";
      bool reqAdmin = doc["requireAdmin"] | false;
      String role = "";

      if (verifyUserCreds(u, p, reqAdmin, role)) {
        StaticJsonDocument<192> resDoc;
        resDoc["success"] = true;
        resDoc["username"] = u;
        resDoc["role"] = role;
        resDoc["locked"] = false;
        resDoc["remaining"] = MAX_AUTH_ATTEMPTS;
        String res;
        serializeJson(resDoc, res);
        request->send(200, "application/json", res);
      } else {
        int rem = max(0, MAX_AUTH_ATTEMPTS - (int)failedAuthAttempts);
        StaticJsonDocument<256> resDoc;
        resDoc["success"] = false;
        resDoc["locked"] = authLockedOut;
        resDoc["remaining"] = rem;
        resDoc["error"] = authLockedOut
                            ? "Device locked (3 failed attempts)! Restart ESP32."
                            : ("Invalid credentials (" + String(rem) + " attempt(s) left)");
        String res;
        serializeJson(resDoc, res);
        request->send(403, "application/json", res);
      }
    });

  // ADD (Feature 3): Real-time ESP32 CPU Temp, RAM & SPIFFS Stats Endpoint
  server.on("/system-stats.json", HTTP_GET, [](AsyncWebServerRequest* request) {
    StaticJsonDocument<256> doc;
    float cpuTemp = temperatureRead();
    uint32_t ramTotal = ESP.getHeapSize();
    uint32_t ramFree = ESP.getFreeHeap();
    uint32_t ramUsed = (ramTotal > ramFree) ? (ramTotal - ramFree) : 0;
    float ramPct = (ramTotal > 0) ? ((float)ramUsed * 100.0f / (float)ramTotal) : 0.0f;

    uint32_t spiffsTotal = SPIFFS.totalBytes();
    uint32_t spiffsUsed = SPIFFS.usedBytes();
    float spiffsPct = (spiffsTotal > 0) ? ((float)spiffsUsed * 100.0f / (float)spiffsTotal) : 0.0f;

    doc["cpuTemp"] = serialized(String(cpuTemp, 1));
    doc["ramUsed"] = ramUsed;
    doc["ramTotal"] = ramTotal;
    doc["ramPct"] = serialized(String(ramPct, 1));
    doc["spiffsUsed"] = spiffsUsed;
    doc["spiffsTotal"] = spiffsTotal;
    doc["spiffsPct"] = serialized(String(spiffsPct, 1));

    String res;
    serializeJson(doc, res);
    request->send(200, "application/json", res);
  });

  server.begin();
  Serial.println("[WebServer] AsyncWebServer started on port 80");
}

bool startAPConfigPortal(unsigned long timeoutSec = 180) {
  WiFiManager wm;
  wm.setAPStaticIPConfig(
    IPAddress(192, 168, 4, 1),
    IPAddress(192, 168, 4, 1),
    IPAddress(255, 255, 255, 0));
  wm.setConfigPortalTimeout(timeoutSec);

  u8g2.clearBuffer();
  u8g2.drawFrame(0, 0, 128, 64);
  u8g2.drawBox(0, 0, 128, 14);
  u8g2.setDrawColor(0);
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(16, 11, "WIFI SETUP PORTAL");
  u8g2.setDrawColor(1);
  u8g2.drawStr(6, 29, "AP : EnergyLog");
  u8g2.drawStr(6, 43, "IP : 192.168.4.1");
  u8g2.drawStr(6, 57, "Timeout: 180s");
  u8g2.sendBuffer();

  bool res = wm.startConfigPortal("EnergyLog");
  bool connected = (res || WiFi.status() == WL_CONNECTED);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  return connected;
}

void drawBootScreen(const char* header, const char* l1, const char* l2 = "", const char* l3 = "") {
  u8g2.clearBuffer();
  u8g2.drawFrame(0, 0, 128, 64);
  u8g2.drawBox(0, 0, 128, 14);
  u8g2.setDrawColor(0);
  u8g2.setFont(u8g2_font_6x12_tf);
  int hw = u8g2.getStrWidth(header);
  u8g2.drawStr((128 - hw) / 2, 11, header);
  u8g2.setDrawColor(1);

  if (strlen(l1) > 0) u8g2.drawStr(6, 29, l1);
  if (strlen(l2) > 0) u8g2.drawStr(6, 43, l2);
  if (strlen(l3) > 0) u8g2.drawStr(6, 57, l3);
  u8g2.sendBuffer();
}

void maintainWiFiAndServices() {
  static bool wasConnected = false;
  static unsigned long lastReconnectTry = 0;

  bool isConnected = (WiFi.status() == WL_CONNECTED && WiFi.localIP() != IPAddress(0, 0, 0, 0));

  if (isConnected && !wasConnected) {
    wasConnected = true;
    manualWiFiDisconnected = false;
    pendingMdnsRestart = false;  // Prevent duplicate mDNS restart in the same loop
    preferences.putBool("needAP", false);
    Serial.printf("[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    setupMDNS();
    pendingNtpSync = true;
  } else if (!isConnected && wasConnected) {
    wasConnected = false;
    if (mdnsRunning) {
      MDNS.end();
      mdnsRunning = false;
    }
    nodeRedConnected = false;
    Serial.println("[WiFi] Connection lost. Switching to offline fallback...");
  }

  // Ensure mDNS responder is active whenever WiFi is connected and mDNS is enabled
  if (isConnected && settings.mdnsEnabled && !mdnsRunning) {
    setupMDNS();
  }

  // Non-blocking background reconnect every 30s if not manually disconnected
  if (!isConnected && !manualWiFiDisconnected && (millis() - lastReconnectTry >= 30000UL)) {
    lastReconnectTry = millis();
    if (WiFi.getMode() != WIFI_STA) {
      WiFi.mode(WIFI_STA);
    }
    wifi_config_t conf;
    if (esp_wifi_get_config(WIFI_IF_STA, &conf) == ESP_OK && strlen(reinterpret_cast<const char*>(conf.sta.ssid)) > 0) {
      Serial.println("[WiFi] Retrying saved network in background...");
      WiFi.disconnect(false, false);
      WiFi.begin();
    }
  }

  // Process deferred tasks safely in loop()
  if (pendingMdnsRestart) {
    pendingMdnsRestart = false;
    setupMDNS();
  }

  if (pendingWiFiDisconnect) {
    pendingWiFiDisconnect = false;
    manualWiFiDisconnected = true;
    wasConnected = false;
    if (mdnsRunning) {
      MDNS.end();
      mdnsRunning = false;
    }
    nodeRedConnected = false;
    WiFi.setAutoReconnect(false);
    WiFi.disconnect(false, false);
  }

  if (pendingWiFiForget) {
    pendingWiFiForget = false;
    manualWiFiDisconnected = true;
    wasConnected = false;
    nodeRedConnected = false;
    eraseWiFiCredentialsNVS();  // Erases saved WiFi credentials; AP starts on next device restart
    showToastLCD("WiFi Forgotten", "Creds Erased", 1500, UI_DASHBOARD);
  }

  if (pendingWiFiPortal) {
    pendingWiFiPortal = false;
    manualWiFiDisconnected = false;
    if (mdnsRunning) {
      MDNS.end();
      mdnsRunning = false;
    }
    server.end();  // Free port 80 for WiFiManager
    delay(100);    // Allow LWIP stack to release port 80

    bool connected = startAPConfigPortal(180);  // 180 seconds AP timeout

    if (connected) {
      preferences.putBool("needAP", false);
      char ipStr[24];
      snprintf(ipStr, sizeof(ipStr), "IP: %s", WiFi.localIP().toString().c_str());
      drawBootScreen("WIFI CONNECTED", WiFi.SSID().c_str(), ipStr, "Restarting Device...");
      triggerBeep(100, 2, 80);
      delay(1500);
      performSafeRestart();
    } else {
      drawBootScreen("PORTAL CLOSED", "Offline Mode", "Relaunching Web...", "Restarting...");
      delay(1200);
      performSafeRestart();
    }
  }

  if (resetKwhRequested) {
    resetKwhRequested = false;
    digitalWrite(PZEM_RELAY, LOW);

    bool pzemCleared = false;
    for (int attempt = 0; attempt < 5; attempt++) {
      while (Serial2.available()) Serial2.read();
      delay(200);
      if (pzem.resetEnergy()) {
        pzemCleared = true;
        break;
      }
    }

    if (!recordEnergyResetTimestamp()) {
      lastResetEpoch = 0;
      strcpy(lastResetTimeStr, "Not Reset");
      preferences.putULong("rstEpoch", 0);
      preferences.putString("rstTime", "Not Reset");
      kwhDays = 0.0f;
    }

    portENTER_CRITICAL(&measureMux);
    readings.energy = 0.0f;
    readings.totalDays = 0.0f;
    portEXIT_CRITICAL(&measureMux);

    if (pzemCleared) {
      triggerBeep(100, 1, 0);
      showToastLCD("Energy Cleared", "Rebooting...", 1500, UI_DASHBOARD);
    } else {
      triggerBeep(150, 2, 80);
      showToastLCD("Sensor Error", "Check AC Input", 1500, UI_DASHBOARD);
    }
    pendingRestart = true;
    restartScheduledMillis = millis() + 1500;
  }

  if (pendingFactoryData && (long)(millis() - restartScheduledMillis) >= 0) {
    pendingFactoryData = false;
    executeDataReset();
  }

  if (pendingFactoryAll && (long)(millis() - restartScheduledMillis) >= 0) {
    pendingFactoryAll = false;
    executeFullFactoryReset();
  }

  if (pendingRestart && (long)(millis() - restartScheduledMillis) >= 0) {
    pendingRestart = false;
    performSafeRestart();
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
      if (subIdx == 1) strcpy(val, relayPhysicalOn ? "ON" : "OFF");
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

void startConfirmDialog(const char* title, void (*onYes)(), UIState returnState = UI_SUB_MENU) {
  editCtx.title = title;
  editCtx.confirmChoice = false;  // Default to "No" for safety
  editCtx.onConfirm = onYes;
  uiReturnState = returnState;
  setEncoderPos(0);
  uiState = UI_CONFIRM;
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
      if (subIdx == 1) startIntEditor("Startup Delay", &settings.startupTime, 5, 300, 5, 's');
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
        // 1. If an active fault is still present, block turning ON
        if (!relayPhysicalOn && isProtectionRunning()) {
          triggerBeep(150, 2, 80);
          showToastLCD("Not Possible!", "Protec. Running", 1400, UI_SUB_MENU);
        }
        // 2. If fault is released and countdown is running, cancel countdown & turn ON
        else if (protectionWasActive || waitingForDelay) {
          protectionWasActive = false;
          waitingForDelay = false;
          strcpy(protPhase, "none");
          protRemainingSec = 0;
          settings.mainOutput = true;
          saveSettings();
          showToastLCD("Main Output", "Turned ON", 1000, UI_SUB_MENU);
        }
        // 3. Normal operation: toggle ON / OFF
        else {
          settings.mainOutput = !settings.mainOutput;
          saveSettings();
          showToastLCD("Main Output", settings.mainOutput ? "Turned ON" : "Turned OFF", 1000, UI_SUB_MENU);
        }
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
          startConfirmDialog(
            "Clear kWh?", []() {
              resetKwhRequested = true;
            },
            UI_SUB_MENU);
        }
      } else if (subIdx == 2) {
        strcpy(infoLine1, "Last Billing:");
        strncpy(infoLine2, lastResetTimeStr, 23);
        snprintf(infoLine3, 23, "Elapsed: %.1f Days", kwhDays);
        snprintf(infoLine4, 23, "Energy: %.2f kWh", readings.energy);
        uiReturnState = UI_SUB_MENU;
        uiState = UI_INFO_VIEW;
      }
      break;

    case 13:  // Factory Reset (Requires 5s press + 15s hold on BOOT button)
      if (subIdx == 1) {
        startHoldResetFlow(HOLD_TARGET_DATA);
      } else if (subIdx == 2) {
        startHoldResetFlow(HOLD_TARGET_FACTORY);
      }
      break;
  }
}

void updateDisplayAndUI() {
  unsigned long now = millis();

  // Physical BOOT Button Hold Guard Screen (Forget User / Reset Data / Factory Reset)
  if (forgetUserState == FORGET_WAIT_PRESS || forgetUserState == FORGET_HOLDING) {
    u8g2.clearBuffer();
    u8g2.drawFrame(0, 0, 128, 64);
    u8g2.drawBox(0, 0, 128, 14);
    u8g2.setDrawColor(0);
    u8g2.setFont(u8g2_font_6x12_tf);
    const char* title = getHoldTargetTitle();
    int tw = u8g2.getStrWidth(title);
    u8g2.drawStr((128 - tw) / 2, 11, title);
    u8g2.setDrawColor(1);

    char cStr[24];
    if (forgetUserState == FORGET_WAIT_PRESS) {
      int rem = 5 - ((now - forgetUserTimerMillis) / 1000UL);
      snprintf(cStr, sizeof(cStr), "Press BOOT in %ds", max(0, rem));
      u8g2.drawStr(14, 36, cStr);
      u8g2.drawStr(14, 52, "to verify action");
    } else {
      unsigned long elapsed = now - forgetUserTimerMillis;
      int rem = 15 - (elapsed / 1000UL);
      snprintf(cStr, sizeof(cStr), "Hold BOOT: %ds", max(0, rem));
      u8g2.drawStr(20, 36, cStr);
      int barW = map(elapsed, 0, 15000, 0, 108);
      u8g2.drawFrame(10, 46, 108, 8);
      u8g2.drawBox(10, 46, constrain(barW, 0, 108), 8);
    }
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

    bool vOk = cur.valid && (cur.voltage >= settings.voltLow && cur.voltage <= settings.voltHigh);
    bool fOk = cur.valid && (cur.frequency >= settings.freqLow && cur.frequency <= settings.freqHigh);
    bool safe = cur.valid && vOk && fOk;

    if (now - lastStartupTickMillis >= 1000UL) {
      lastStartupTickMillis = now;
      if (safe && startupRemainingSec > 0) {
        startupRemainingSec--;
      }
    }

    // Debounce RESET_BTN (must hold for 300ms)
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
      u8g2.drawFrame(0, 0, 128, 64);
      u8g2.setFont(u8g2_font_7x14B_tr);
      u8g2.drawStr(16, 39, "SYSTEM ACTIVE");
      u8g2.sendBuffer();

      digitalWrite(BUZZER_PIN, HIGH);
      delay(1000);
      digitalWrite(BUZZER_PIN, LOW);
      delay(300);

      startupCheckCompleted = true;
      uiState = UI_DASHBOARD;
      return;
    }

    static unsigned long lastStartupDraw = 0;
    if (now - lastStartupDraw >= 200UL) {
      lastStartupDraw = now;
      u8g2.clearBuffer();
      u8g2.drawFrame(0, 0, 128, 64);
      u8g2.drawBox(0, 0, 128, 13);
      u8g2.setDrawColor(0);
      u8g2.setFont(u8g2_font_6x12_tf);
      u8g2.drawStr(16, 10, "LINE SAFETY CHECK");
      u8g2.setDrawColor(1);

      if (!cur.valid) {
        u8g2.drawStr(8, 43, "Waiting for Sensor");
      } else {
        char vBuf[24], fBuf[24];
        const char* vStat = (cur.voltage > settings.voltHigh) ? "HIGH" : ((cur.voltage < settings.voltLow) ? "LOW " : "OK  ");
        const char* fStat = (cur.frequency > settings.freqHigh) ? "HIGH" : ((cur.frequency < settings.freqLow) ? "LOW " : "OK  ");

        snprintf(vBuf, sizeof(vBuf), "V: %5.1fV  [%s]", cur.voltage, vStat);
        snprintf(fBuf, sizeof(fBuf), "F: %5.2fHz [%s]", cur.frequency, fStat);
        u8g2.drawStr(6, 26, vBuf);
        u8g2.drawStr(6, 38, fBuf);

        if (safe) {
          int totalT = max(1, settings.startupTime);
          int barW = map(totalT - startupRemainingSec, 0, totalT, 0, 116);
          u8g2.drawFrame(6, 43, 116, 6);
          if (barW > 0) {
            u8g2.drawBox(6, 43, constrain(barW, 0, 116), 6);
          }
          char tBuf[24];
          snprintf(tBuf, sizeof(tBuf), "Main ON in: %ds", startupRemainingSec);
          u8g2.drawStr(14, 59, tBuf);
        } else {
          u8g2.drawStr(6, 55, "FAULT: Output OFF");
        }
      }
      u8g2.sendBuffer();
    }
    return;
  }

  // 2. Main Dashboard State (RESTORED)
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
      lastEncoderPos = getEncoderPos();  // Prevent phantom step on return
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
      lastEncoderPos = getEncoderPos();  // Prevent phantom step on return
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

  // 5. Yes/No Confirmation Dialog State
  if (uiState == UI_CONFIRM) {
    int step = readEncoderStep();
    if (step != 0) {
      bool newChoice = !editCtx.confirmChoice;
      editCtx.confirmChoice = newChoice;
      // triggerBeep(12, 1, 0);
    }

    if (checkEncoderButtonClick()) {
      triggerBeep(45, 1, 0);
      if (editCtx.confirmChoice && editCtx.onConfirm) {
        editCtx.onConfirm();
      } else {
        showToastLCD("Action", "Cancelled", 800, uiReturnState);
      }
      return;
    }

    u8g2.clearBuffer();
    u8g2.drawFrame(0, 0, 128, 64);
    u8g2.setFont(u8g2_font_7x14B_tr);
    u8g2.drawStr(8, 16, editCtx.title);
    u8g2.drawLine(0, 20, 127, 20);

    u8g2.setFont(u8g2_font_6x12_tf);
    if (editCtx.confirmChoice) {
      u8g2.drawBox(12, 26, 44, 16);
      u8g2.setDrawColor(0);
      u8g2.drawStr(24, 38, "YES");
      u8g2.setDrawColor(1);
      u8g2.drawFrame(72, 26, 44, 16);
      u8g2.drawStr(88, 38, "NO");
    } else {
      u8g2.drawFrame(12, 26, 44, 16);
      u8g2.drawStr(24, 38, "YES");
      u8g2.drawBox(72, 26, 44, 16);
      u8g2.setDrawColor(0);
      u8g2.drawStr(88, 38, "NO");
      u8g2.setDrawColor(1);
    }
    u8g2.drawStr(14, 57, "Click to confirm");
    u8g2.sendBuffer();
    return;
  }

  // 6. Main Menu Navigation
  if (uiState == UI_MAIN_MENU) {
    int step = readEncoderStep();
    if (step != 0) {
      int newIdx = constrain(currentMainMenuIdx + step, 0, mainMenuCount - 1);
      if (newIdx != currentMainMenuIdx) {
        currentMainMenuIdx = newIdx;
        // triggerBeep(12, 1, 0);  // Only beep if selection actually moved
      }
    }

    if (checkEncoderButtonClick()) {
      triggerBeep(45, 1, 0);
      if (currentMainMenuIdx == 0) {
        uiState = UI_DASHBOARD;
        return;
      } else if (currentMainMenuIdx == 11) {
        ntpReqFromLCD = true;
        pendingNtpSync = true;
        showToastLCD("NTP Clock", "Syncing...", 2000, UI_MAIN_MENU);
        return;
      } else if (currentMainMenuIdx == 14) {
        startConfirmDialog(
          "Restart ESP32?", []() {
            pendingRestart = true;
            restartScheduledMillis = millis() + 600;
            showToastLCD("System", "Restarting...", 1000, UI_DASHBOARD);
          },
          UI_MAIN_MENU);
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

  // 7. Submenu Navigation
  if (uiState == UI_SUB_MENU) {
    int step = readEncoderStep();
    if (step != 0) {
      int newIdx = constrain(currentSubMenuIdx + step, 0, activeSubmenuLen - 1);
      if (newIdx != currentSubMenuIdx) {
        currentSubMenuIdx = newIdx;
        // triggerBeep(12, 1, 0);  // Only beep if selection actually moved
      }
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

  // 8. Integer Value Editor (No mid-rotation setEncoderPos glitch!)
  if (uiState == UI_EDIT_INT) {
    int step = readEncoderStep();
    if (step != 0) {
      int newVal = constrain(editCtx.intTemp + (step * editCtx.intStep), editCtx.intMin, editCtx.intMax);
      if (newVal != editCtx.intTemp) {
        editCtx.intTemp = newVal;
        // triggerBeep(10, 1, 0);
      }
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

  // 9. Float Value Editor (No mid-rotation setEncoderPos glitch!)
  if (uiState == UI_EDIT_FLOAT) {
    int step = readEncoderStep();
    if (step != 0) {
      long curS = round(editCtx.floatTemp * editCtx.floatScale);
      long minS = round(editCtx.floatMin * editCtx.floatScale);
      long maxS = round(editCtx.floatMax * editCtx.floatScale);
      long newS = constrain(curS + step, minS, maxS);
      if (newS != curS) {
        editCtx.floatTemp = newS / editCtx.floatScale;
        // triggerBeep(10, 1, 0);
      }
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
// PROFESSIONAL BOOT SEQUENCE & SETUP
// ============================================================================
void bootConnectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  wifi_config_t conf;
  bool hasSavedCreds = false;
  if (esp_wifi_get_config(WIFI_IF_STA, &conf) == ESP_OK) {
    hasSavedCreds = (strlen(reinterpret_cast<const char*>(conf.sta.ssid)) > 0);
  }
  bool needAP = preferences.getBool("needAP", false);

  // 1. After Factory Reset, Forget WiFi, or First-Time Start (no saved credentials) -> Launch AP 'EnergyLog'
  if (needAP || !hasSavedCreds) {
    bool connected = startAPConfigPortal(180);

    if (connected) {
      preferences.putBool("needAP", false);
      char ipStr[24];
      snprintf(ipStr, sizeof(ipStr), "IP: %s", WiFi.localIP().toString().c_str());
      drawBootScreen("WIFI CONNECTED", WiFi.SSID().c_str(), ipStr, "Restarting Device...");
      triggerBeep(100, 2, 80);
      delay(1500);
      performSafeRestart();  // Mandatory safe restart to cleanly launch AsyncWebServer on port 80
    } else {
      drawBootScreen("OFFLINE MODE", "WiFi Not Configured", "Local Control Ready", "AP Closed");
      delay(1500);
    }
    return;
  }

  // 2. Normal Boot with Saved Credentials
  String host = String(settings.mdnsHostname);
  host.trim();
  host.toLowerCase();
  if (host.length() == 0) host = getDefaultMdnsName();
  WiFi.setHostname(host.c_str());

  WiFi.begin();
  String targetSsid = String(reinterpret_cast<const char*>(conf.sta.ssid));
  const int MAX_ATTEMPTS = 6;
  for (int attempts = 0; attempts < MAX_ATTEMPTS; attempts++) {
    char stepStr[24];
    snprintf(stepStr, sizeof(stepStr), "Attempt %d of %d", attempts + 1, MAX_ATTEMPTS);
    drawBootScreen("NETWORK INIT", "Connecting WiFi...", targetSsid.c_str(), stepStr);

    if (WiFi.status() == WL_CONNECTED && WiFi.localIP() != IPAddress(0, 0, 0, 0)) {
      break;
    }
    delay(1500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    preferences.putBool("needAP", false);
    char ipStr[24];
    snprintf(ipStr, sizeof(ipStr), "IP: %s", WiFi.localIP().toString().c_str());
    drawBootScreen("NETWORK ONLINE", "Connected to:", WiFi.SSID().c_str(), ipStr);
    delay(1200);
  } else {
    drawBootScreen("OFFLINE MODE", "WiFi Unavailable", "Auto-Retry Active", "Local Control Ready");
    delay(1200);
  }
}

void bootSyncTime() {
  // Skip NTP screen completely if offline
  if (WiFi.status() != WL_CONNECTED || WiFi.localIP() == IPAddress(0, 0, 0, 0)) return;

  drawBootScreen("CLOCK SYNC", "Connecting NTP", ntpServer, "Please wait...");

  // Give router ARP & DNS forwarder a brief moment after DHCP IP assignment
  delay(400);

  // Reset SNTP state and include an IP-literal fallback (162.159.200.1 = Cloudflare NTP)
  // so slow router DNS on boot never blocks time sync
  if (esp_sntp_enabled()) {
    esp_sntp_stop();
  }
  sntp_set_sync_status(SNTP_SYNC_STATUS_RESET);
  configTime(settings.gmtOffset, 0, ntpServer, "time.google.com", "162.159.200.1");

  struct tm timeinfo;
  const int MAX_NTP_TRIES = 8;
  for (int attempts = 0; attempts < MAX_NTP_TRIES; attempts++) {
    // Re-trigger SNTP halfway through if the first UDP packet was dropped during ARP warmup
    if (attempts == 4) {
      configTime(settings.gmtOffset, 0, "162.159.200.1", ntpServer, "time.nist.gov");
    }

    if (getLocalTime(&timeinfo, 1000) && (timeinfo.tm_year + 1900 >= 2024)) {
      ntpSynced = true;
      lastNtpSyncMillis = millis();
      strftime(volatileNtpSyncStr, sizeof(volatileNtpSyncStr), "%d.%m.%Y %H:%M:%S", &timeinfo);
      break;
    }
  }

  if (ntpSynced) {
    char timeStr[24], dateStr[24];
    strftime(timeStr, sizeof(timeStr), "Time: %H:%M:%S", &timeinfo);
    strftime(dateStr, sizeof(dateStr), "Date: %d/%m/%Y", &timeinfo);
    drawBootScreen("CLOCK SYNCED", timeStr, dateStr, "Status: Done");
    delay(1200);
  } else {
    drawBootScreen("CLOCK SYNC", "NTP Unreachable", "Continuing boot...", "");
    delay(1000);
  }
}

void setup() {
  // 1. Configure Relay & Control Pins FIRST (Both Relays OFF at startup)
  pinMode(MAIN_RELAY, OUTPUT);
  digitalWrite(MAIN_RELAY, HIGH);
  pinMode(PZEM_RELAY, OUTPUT);
  digitalWrite(PZEM_RELAY, HIGH);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(RESET_BTN, INPUT);
  pinMode(BOOT_BTN, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);

  // 2. Initialize Serial Buses
  Serial.begin(115200);
  espSerial.begin(115200, SERIAL_8N1, ESP_COMM_RX, ESP_COMM_TX);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  // 3. Professional Splash Screen
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.drawFrame(0, 0, 128, 64);
  u8g2.setFont(u8g2_font_7x14B_tr);
  u8g2.drawStr(12, 20, "ENERGY MONITOR");
  // u8g2.setFont(u8g2_font_6x12_tf);
  // u8g2.drawStr(14, 36, "Smart Power System");
  u8g2.drawLine(8, 43, 120, 43);
  u8g2.setFont(u8g2_font_t0_11_tr);
  u8g2.drawStr(10, 57, "v3.0  |  M. Maity");
  u8g2.sendBuffer();

  // Short double confirmation beep
  digitalWrite(BUZZER_PIN, HIGH);
  delay(150);
  digitalWrite(BUZZER_PIN, LOW);
  delay(80);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(150);
  digitalWrite(BUZZER_PIN, LOW);
  delay(1600);

  // 4. Consolidated System Initialization (Encoder, SPIFFS, NVS, Config)
  drawBootScreen("SYSTEM INIT", "Mounting Storage...", "Loading Config...", "");
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_DT), encoderISR, CHANGE);
  setEncoderPos(0);

  // Initialize WiFi STA early so WiFi.macAddress() is valid before loadSettings()
  WiFi.mode(WIFI_STA);
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
      pendingMdnsRestart = true;
    } else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      mdnsRunning = false;
    }
  });

  bool spiffsOk = SPIFFS.begin(true);
  preferences.begin("energy-meter", false);
  lastResetEpoch = preferences.getULong("rstEpoch", 0);
  String savedRstStr = preferences.getString("rstTime", "Not Reset");
  strncpy(lastResetTimeStr, savedRstStr.c_str(), sizeof(lastResetTimeStr) - 1);
  lastResetTimeStr[sizeof(lastResetTimeStr) - 1] = '\0';

  loadSettings();
  loadUsers();
  loadSchedulesFromNVS();

  if (!spiffsOk) {
    drawBootScreen("STORAGE WARNING", "SPIFFS Mount Error", "Web UI not found", "");
    delay(1500);
  }

  // 5. Connect to Saved WiFi or Launch 'EnergyLog' AP Portal
  bootConnectWiFi();

  // 6. Sync NTP Clock (only if WiFi is connected)
  bootSyncTime();

  // 7. Start Web Server & Power On PZEM Sensor
  drawBootScreen("STARTING SERVICES", "Server : Port 80", "AC Sensor  : ON", "Initializing...");
  setupWebServerRoutes();

  digitalWrite(PZEM_RELAY, LOW);  // Power ON PZEM sensor
  delay(2200);                    // Allow PZEM-004T MCU & zero-crossing ADC to stabilize

  // Flush any optocoupler power-on glitch bytes from Serial2 RX FIFO
  while (Serial2.available()) {
    Serial2.read();
  }
  // Discard one warmup read so the PZEM cache clears any power-on transient
  (void)pzem.voltage();
  delay(200);
  while (Serial2.available()) {
    Serial2.read();
  }

  // 8. Initial Clean PZEM Poll & Secondary ESP32 Handshake
  updatePzemReadings();
  espSerial.println("HELLO");
  shareDataWithSecondary();

  // 9. Enter Startup Safety Check
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
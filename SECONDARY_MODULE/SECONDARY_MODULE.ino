#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define ESP_COMM_TX 25  // Connected to RX (26) of Main ESP32
#define ESP_COMM_RX 26  // Connected to TX (25) of Main ESP32

#define WATT_LED 17
#define PROCESS_LED 18
#define PROTEC_LED 19

#define DATA_PIN 14   // 74HC595 Serial Data
#define CLOCK_PIN 27  // 74HC595 Clock
#define LATCH_PIN 16  // 74HC595 Latch
#define OE_PIN 4      // 74HC595 Output Enable (Active LOW, 10k pull-up)

#define MAX_RELAYS 16

// ============================================================================
// HARDWARE INSTANCES
// ============================================================================
HardwareSerial espSerial(1);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ============================================================================
// TELEMETRY & SETTINGS STATE
// ============================================================================
struct Measurements {
  float voltage = 220.0f;
  float current = 0.0f;
  float power = 0.0f;
  float energy = 0.0f;
  float frequency = 50.0f;
  float pf = 1.0f;
  float totalDays = 0.0f;
  float uptime = 0.0f;
  bool valid = false;
} measurements;

struct Settings {
  bool highVoltageAlert = false, lowVoltageAlert = false;
  bool highPowerAlert = false, lowPowerAlert = false;
  bool highFreqAlert = false, lowFreqAlert = false;

  bool highVProtec = false, LowVProtec = false;
  bool highPProtec = false;
  bool highHzProtec = false, lowHzProtec = false;

  bool afpcEnabled = false;
  bool mainOutput = true;
  bool kWhSaver = false;

  float voltHigh = 250.0f, voltLow = 180.0f;
  float powerHigh = 3000.0f, powerLow = 100.0f;
  float freqHigh = 50.50f, freqLow = 49.50f;
  float targetFactor = 0.90f;
  float mfdPerChannel = 4.0f;
  int maxkWh = 100;
  int startupTime = 15;
  int afpcLowWatt = 50;
  int afpcHighWatt = 3000;
  int espShareInterval = 2;
} settings;

// ============================================================================
// RUNTIME VARIABLES
// ============================================================================
uint16_t currentRelayState = 0xFFFF;  // All 16 relays OFF (Active LOW)
uint16_t afpcRelayState = 0xFFFF;
char afpcStatusText[17] = "AFPC: STANDBY   ";

bool linkOnline = false;
unsigned long lastPacketMillis = 0;
unsigned long lastPingSentMillis = 0;

// LED Timers
unsigned long lastWattBlink = 0;
unsigned long wattLedOnTime = 0;
bool wattLedState = false;
unsigned long processLedOnTime = 0;
bool processLedState = false;

// Anti-flicker LCD shadow buffers
char lcdLine0Cache[17] = "";
char lcdLine1Cache[17] = "";

// ============================================================================
// SHIFT REGISTER & LCD HELPERS
// ============================================================================
void send16ToShiftRegister(uint16_t data) {
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, highByte(data));
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, lowByte(data));
  digitalWrite(LATCH_PIN, HIGH);
  currentRelayState = data;
}

void updateLcdLine(uint8_t row, const char* text) {
  char formatted[17];
  snprintf(formatted, sizeof(formatted), "%-16.16s", text);
  char* cache = (row == 0) ? lcdLine0Cache : lcdLine1Cache;
  if (strncmp(cache, formatted, 16) != 0) {
    strncpy(cache, formatted, 17);
    lcd.setCursor(0, row);
    lcd.print(formatted);
  }
}

void initializeOutputPins() {
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(OE_PIN, OUTPUT);
  pinMode(WATT_LED, OUTPUT);
  pinMode(PROCESS_LED, OUTPUT);
  pinMode(PROTEC_LED, OUTPUT);

  digitalWrite(OE_PIN, HIGH);     // Disable shift register outputs during boot
  send16ToShiftRegister(0xFFFF);  // All relays OFF
  digitalWrite(OE_PIN, LOW);      // Enable shift register outputs
  digitalWrite(WATT_LED, LOW);
  digitalWrite(PROCESS_LED, LOW);
  digitalWrite(PROTEC_LED, LOW);
}

// ============================================================================
// SERIAL JSON PARSER & LINK WATCHDOG
// ============================================================================
void parseIncomingJson(const char* jsonStr) {
  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, jsonStr);
  if (err) return;

  lastPacketMillis = millis();
  linkOnline = true;

  measurements.voltage = doc["v"].as<float>();
  measurements.current = doc["i"].as<float>();
  measurements.power = doc["p"].as<float>();
  measurements.energy = doc["e"].as<float>();
  measurements.frequency = doc["f"].as<float>();
  measurements.pf = doc["pf"].as<float>();
  measurements.uptime = doc["up"].as<float>();
  measurements.totalDays = doc["d"].as<float>();
  measurements.valid = (doc["ok"] | 1) == 1;

  settings.voltHigh = doc["vh"].as<float>();
  settings.voltLow = doc["vl"].as<float>();
  settings.powerHigh = doc["ph"].as<float>();
  settings.powerLow = doc["pl"].as<float>();
  settings.freqHigh = doc["fh"].as<float>();
  settings.freqLow = doc["fl"].as<float>();
  settings.targetFactor = doc["tpf"].as<float>();
  settings.mfdPerChannel = doc["mfd"].as<float>();
  settings.maxkWh = doc["max"] | 100;
  settings.startupTime = doc["st"] | 15;
  settings.afpcLowWatt = doc["pfl"] | 50;
  settings.afpcHighWatt = doc["pfh"] | 3000;
  settings.espShareInterval = max(1, doc["esi"] | 2);

  settings.highVoltageAlert = doc["hva"] | false;
  settings.lowVoltageAlert = doc["lva"] | false;
  settings.highPowerAlert = doc["hpa"] | false;
  settings.lowPowerAlert = doc["lpa"] | false;
  settings.highFreqAlert = doc["hfa"] | false;
  settings.lowFreqAlert = doc["lfa"] | false;

  settings.highVProtec = doc["hvp"] | false;
  settings.LowVProtec = doc["lvp"] | false;
  settings.highPProtec = doc["hpp"] | false;
  settings.highHzProtec = doc["hhp"] | false;
  settings.lowHzProtec = doc["lhp"] | false;

  settings.afpcEnabled = doc["afpc"] | false;
  settings.mainOutput = doc["out"] | false;
  settings.kWhSaver = doc["kwhs"] | false;
}

void processSerialLink() {
  static char rxBuffer[1024];
  static size_t rxIndex = 0;
  static bool receivingJson = false;

  while (espSerial.available()) {
    char c = (char)espSerial.read();

    digitalWrite(PROCESS_LED, HIGH);
    processLedOnTime = millis();
    processLedState = true;

    if (c == 'S') {
      receivingJson = true;
      rxIndex = 0;
      continue;
    }

    if (receivingJson) {
      if (c == 'E') {
        rxBuffer[rxIndex] = '\0';
        receivingJson = false;
        parseIncomingJson(rxBuffer);
      } else if (rxIndex < sizeof(rxBuffer) - 1) {
        rxBuffer[rxIndex++] = c;
      } else {
        // Buffer overflow protection
        receivingJson = false;
        rxIndex = 0;
      }
    }
  }

  // Turn off PROCESS_LED after 50ms
  if (processLedState && (millis() - processLedOnTime >= 50UL)) {
    digitalWrite(PROCESS_LED, LOW);
    processLedState = false;
  }

  // Link Watchdog: If no valid packet within max(6000ms, 3 * espShareInterval)
  unsigned long timeoutMs = max(6000UL, (unsigned long)settings.espShareInterval * 3000UL);
  if (linkOnline && (millis() - lastPacketMillis > timeoutMs)) {
    linkOnline = false;
    afpcRelayState = 0xFFFF;
    send16ToShiftRegister(0xFFFF);  // Fail-safe: turn OFF all capacitor banks
  }

  // Periodically send non-blocking PING when offline
  if (!linkOnline && (millis() - lastPingSentMillis >= 2000UL)) {
    lastPingSentMillis = millis();
    espSerial.println("PING");
  }
}

// ============================================================================
// SAFE AFPC CAPACITOR BANK CONTROLLER
// ============================================================================
void updateAFPC() {
  if (!linkOnline || !settings.afpcEnabled || !settings.mainOutput || !measurements.valid) {
    if (currentRelayState != 0xFFFF) {
      afpcRelayState = 0xFFFF;
      send16ToShiftRegister(0xFFFF);
    }
    strcpy(afpcStatusText, "AFPC: OFF       ");
    return;
  }

  static unsigned long lastAfpcCalc = 0;
  if (millis() - lastAfpcCalc < 3000UL) return;
  lastAfpcCalc = millis();

  if (measurements.power < settings.afpcLowWatt || measurements.power > settings.afpcHighWatt) {
    afpcRelayState = 0xFFFF;
    send16ToShiftRegister(0xFFFF);
    strcpy(afpcStatusText, "OUT OF RANGE    ");
    return;
  }

  // Guard against invalid voltage, frequency, or PF math
  if (measurements.voltage < 100.0f || measurements.frequency < 40.0f || measurements.pf <= 0.05f) {
    afpcRelayState = 0xFFFF;
    send16ToShiftRegister(0xFFFF);
    strcpy(afpcStatusText, "SENSOR WAIT...  ");
    return;
  }

  float currentPF = constrain(measurements.pf, 0.10f, 1.00f);
  float targetPF = constrain(settings.targetFactor, 0.50f, 0.99f);
  float mfdPerCap = max(0.1f, settings.mfdPerChannel);
  const float hysteresis = 0.01f;

  if (currentPF >= targetPF - hysteresis) {
    // PF is already at or above target
    snprintf(afpcStatusText, sizeof(afpcStatusText), "PF OK: %.2f     ", currentPF);
    return;
  }

  float reactivePowerVar = measurements.power * (tanf(acosf(currentPF)) - tanf(acosf(targetPF)));
  float denom = (measurements.voltage * measurements.voltage) * 2.0f * PI * measurements.frequency;
  float totalMfdNeeded = (denom > 1.0f) ? ((reactivePowerVar * 1e6f) / denom) : 0.0f;

  int capsNeeded = (int)roundf(totalMfdNeeded / mfdPerCap);
  if (capsNeeded > MAX_RELAYS) {
    int extraMfd = (capsNeeded - MAX_RELAYS) * (int)roundf(mfdPerCap);
    snprintf(afpcStatusText, sizeof(afpcStatusText), "NEED +%d MFD    ", extraMfd);
    capsNeeded = MAX_RELAYS;
  }

  capsNeeded = constrain(capsNeeded, 0, MAX_RELAYS);
  uint16_t newState = 0xFFFF;
  for (int i = 0; i < capsNeeded; i++) {
    bitClear(newState, i);  // Active LOW relay bank
  }

  if (newState != afpcRelayState) {
    afpcRelayState = newState;
    send16ToShiftRegister(newState);
  }

  int totalMfdActive = (int)roundf(capsNeeded * mfdPerCap);
  snprintf(afpcStatusText, sizeof(afpcStatusText), "AFPC:%2d=%3dMFD  ", capsNeeded, totalMfdActive);
}

// ============================================================================
// INSTANT ALERT & TELEMETRY LCD MANAGER
// ============================================================================
void updateDisplayAndLeds() {
  unsigned long now = millis();

  // 1. Protection Status LED
  bool anyProtec = settings.highVProtec || settings.LowVProtec || settings.highPProtec || settings.highHzProtec || settings.lowHzProtec;
  digitalWrite(PROTEC_LED, anyProtec ? HIGH : LOW);

  // 2. Proportional Watt Pulse LED
  if (linkOnline && measurements.power >= 5.0f) {
    float maxP = settings.powerHigh > 100.0f ? settings.powerHigh : 5000.0f;
    float p = constrain(measurements.power, 0.0f, maxP);
    float ratio = (maxP - p) / maxP;
    unsigned long offInterval = 100UL + (unsigned long)(1400.0f * ratio);

    if (!wattLedState && (now - lastWattBlink >= offInterval)) {
      wattLedState = true;
      digitalWrite(WATT_LED, HIGH);
      wattLedOnTime = now;
    } else if (wattLedState && (now - wattLedOnTime >= 50UL)) {
      wattLedState = false;
      digitalWrite(WATT_LED, LOW);
      lastWattBlink = now;
    }
  } else {
    digitalWrite(WATT_LED, LOW);
    wattLedState = false;
  }

  // 3. LCD Line 0 & Line 1: Hold cleanly while waiting for Primary ESP32 boot
  if (!linkOnline) {
    updateLcdLine(0, "PLEASE WAIT...  ");
    updateLcdLine(1, "                ");
    return;
  }

  if (!measurements.valid) {
    updateLcdLine(0, "SENSOR ERROR!   ");
    updateLcdLine(1, "CHECKING PZEM...");
    return;
  }

  // Active Fault/Alert Status on Line 0
  const char* activeAlerts[6];
  uint8_t alertCount = 0;

  if (measurements.voltage > settings.voltHigh) activeAlerts[alertCount++] = "HIGH VOLTAGE!   ";
  if (measurements.voltage < settings.voltLow) activeAlerts[alertCount++] = "LOW VOLTAGE!    ";
  if (measurements.power > settings.powerHigh) activeAlerts[alertCount++] = "HIGH POWER!     ";
  if (measurements.power < settings.powerLow) activeAlerts[alertCount++] = "LOW POWER!      ";
  if (measurements.frequency > settings.freqHigh) activeAlerts[alertCount++] = "HIGH FREQUENCY! ";
  if (measurements.frequency < settings.freqLow) activeAlerts[alertCount++] = "UNDER FREQUENCY!";

  static uint8_t alertCycleIdx = 0;
  static unsigned long lastAlertCycle = 0;
  static uint8_t prevAlertCount = 0;

  if (alertCount == 0) {
    updateLcdLine(0, settings.mainOutput ? "ALL : NORMAL    " : "OUTPUT : OFF    ");
    prevAlertCount = 0;
  } else {
    if (prevAlertCount == 0 || (now - lastAlertCycle >= 1500UL)) {
      lastAlertCycle = now;
      alertCycleIdx = (alertCycleIdx + 1) % alertCount;
      updateLcdLine(0, activeAlerts[alertCycleIdx]);
    }
    prevAlertCount = alertCount;
  }

  // 4. LCD Line 1: Smooth Telemetry & AFPC Cycling (Every 2 seconds)
  static unsigned long lastLine1Cycle = 0;
  static uint8_t line1Idx = 0;
  if (now - lastLine1Cycle >= 2000UL) {
    lastLine1Cycle = now;
    char buf[17];

    int totalSteps = settings.afpcEnabled ? 7 : 6;
    line1Idx = (line1Idx + 1) % totalSteps;

    if (line1Idx == 0) {
      snprintf(buf, sizeof(buf), "VOLT: %.1f V", measurements.voltage);
    } else if (line1Idx == 1) {
      snprintf(buf, sizeof(buf), "CURR: %.2f A", measurements.current);
    } else if (line1Idx == 2) {
      snprintf(buf, sizeof(buf), "POWR: %.1f W", measurements.power);
    } else if (line1Idx == 3) {
      snprintf(buf, sizeof(buf), "PF:%.2f F:%.1fHz", measurements.pf, measurements.frequency);
    } else if (line1Idx == 4) {
      snprintf(buf, sizeof(buf), "ENRG: %.2f kWh", measurements.energy);
    } else if (line1Idx == 5) {
      snprintf(buf, sizeof(buf), "BILL: %.1f Days", measurements.totalDays);
    } else {
      strncpy(buf, afpcStatusText, sizeof(buf));
    }
    updateLcdLine(1, buf);
  }
}

// ============================================================================
// SETUP & MAIN LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  espSerial.begin(115200, SERIAL_8N1, ESP_COMM_RX, ESP_COMM_TX);

  initializeOutputPins();

  Wire.begin();
  lcd.init();
  lcd.backlight();

  updateLcdLine(0, "ESP.ENERGY LOG  ");
  updateLcdLine(1, "SYSTEM READY    ");
  espSerial.println("PING");
}

void loop() {
  processSerialLink();
  updateAFPC();
  updateDisplayAndLeds();
}
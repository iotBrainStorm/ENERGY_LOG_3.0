// Library Includes
#include <Wire.h>               // I2C communication
#include <HardwareSerial.h>     // Serial communication
#include <LiquidCrystal_I2C.h>  // LCD display
#include <ArduinoJson.h>        // make json format

// Pin Definitions
// -- Communication Pins
#define ESP_COMM_TX 25  // Connected to RX (26) of main ESP32
#define ESP_COMM_RX 26  // Connected to TX (25) of main ESP32

// -- Status LED Pins
#define WATT_LED 17
#define PROCESS_LED 18
#define PROTEC_LED 19

// -- Shift Register Pins
#define DATA_PIN 14   // 74HC595 Serial Data
#define CLOCK_PIN 27  // 74HC595 Clock
#define LATCH_PIN 16  // 74HC595 Latch
#define OE_PIN 4      // 74HC595, connect OE to D4 through 10k pull up resistance to avoid flickering issue at starting

// Hardware Initialization
// -- Serial Communication
HardwareSerial espSerial(1);  // Use Serial1 for ESP-to-ESP communication

// -- Display
LiquidCrystal_I2C lcd(0x27, 16, 2);  // LCD address 0x27, 16 chars, 2 lines

// Global Variables


// -- PZEM Measurements
float voltage = 0.0;
float current = 0.0;
float power = 0.0;
float energy = 0.0;
float frequency = 0.0;
float pf = 0.0;
float totalDays = 0.0;
float uptime = 0.0;

// -- AFPC Control
#define MAX_RELAYS 16              // Total number of relay channels
uint16_t afpcRelayState = 0xFFFF;  // All relays OFF initially
unsigned long lastAfpcUpdate = 0;
const unsigned long AFPC_UPDATE_INTERVAL = 3000;  // Check every 3 seconds

// -- Sensor Data
static bool receiving = false;
//unsigned long DATA_TIMEOUT = 2000;

// -- LED Status Variables
unsigned long lastWattBlink = 0;
bool wattLedState = false;
unsigned long wattLedOnTime = 0;
unsigned long processLedOnTime = 0;
bool processLedState = false;

// -- Safety Alert Message
const unsigned long ALERT_DISPLAY_TIME = 2000;  // Each alert shows for 2 seconds
unsigned long lastAlertCheck = 0;
const unsigned long ALERT_CHECK_INTERVAL = 1000;  // Check alerts every second


uint16_t currentRelayState = 0xFFFF;  // All relays OFF initially
bool newSettingsReceived = false;

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
};
Settings settings;
struct Measurements {
  float voltage;
  float current;
  float power;
  float energy;
  float frequency;
  float pf;
  float totalDays;
  float uptime;
};
Measurements measurements;

void initializeDefaultValues() {
  // Default settings
  settings.voltHigh = 250.0f;
  settings.voltLow = 150.0f;
  settings.powerHigh = 5000.0f;
  settings.powerLow = 100.0f;
  settings.freqHigh = 51.0f;
  settings.freqLow = 49.0f;
  settings.targetFactor = 0.90f;
  settings.mfdPerChannel = 4.0f;
  settings.maxkWh = 1;
  settings.startupTime = 30;
  settings.afpcLowWatt = 100;
  settings.afpcHighWatt = 1000;

  settings.espShareInterval = 2;
  settings.nodeRedInterval = 30;
  settings.pzemInterval = 1;
  settings.alertInterval = 180;

  // Default alert states
  settings.highVoltageAlert = false;
  settings.lowVoltageAlert = false;
  settings.highPowerAlert = false;
  settings.lowPowerAlert = false;
  settings.highFreqAlert = false;
  settings.lowFreqAlert = false;

  // Default protection states
  settings.highVProtec = false;
  settings.LowVProtec = false;
  settings.highPProtec = false;
  settings.highHzProtec = false;
  settings.lowHzProtec = false;

  // Default feature states
  settings.afpcEnabled = false;
  settings.mainOutput = true;
  settings.kWhSaver = false;
  settings.buzzerEnabled = true;

  // Default measurements
  measurements.voltage = 220.0f;
  measurements.current = 0.0f;
  measurements.power = 0.0f;
  measurements.energy = 0.0f;
  measurements.frequency = 50.0f;
  measurements.pf = 1.0f;
  measurements.uptime = 0.0f;
  measurements.totalDays = 0.0f;
}


// Function Declarations
void initializeOutputPins();
void displayWelcomeMessage();
void send16ToShiftRegister(uint16_t data);
void updateRelayStates(uint16_t newState);

//=============================================================
// Helper Functions
//=============================================================
void initializeOutputPins() {
  // Setup shift register pins
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(OE_PIN, OUTPUT);
  pinMode(WATT_LED, OUTPUT);
  pinMode(PROCESS_LED, OUTPUT);
  pinMode(PROTEC_LED, OUTPUT);

  // Initialize outputs
  digitalWrite(OE_PIN, HIGH);      // Disable outputs during initialization
  send16ToShiftRegister(0xFFFF);   // All relays OFF
  digitalWrite(OE_PIN, LOW);       // Enable outputs
  digitalWrite(WATT_LED, LOW);     // Watt LED OFF
  digitalWrite(PROCESS_LED, LOW);  // Process LED
  digitalWrite(PROTEC_LED, LOW);   // Protection LED OFF
}

void displayWelcomeMessage() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ESP.ENERGY LOG  ");
  lcd.setCursor(0, 1);
  lcd.print("                ");
  //  lcd.setCursor(0, 1);
  //  lcd.print("System Ready");
}

void send16ToShiftRegister(uint16_t data) {
  digitalWrite(LATCH_PIN, LOW);
  // Send high byte first (Q8-Q15)
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, highByte(data));
  // Send low byte second (Q0-Q7)
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, lowByte(data));
  digitalWrite(LATCH_PIN, HIGH);
  currentRelayState = data;  // Always update
}

void updateRelayStates(uint16_t newState) {
  if (newState != currentRelayState) {
    send16ToShiftRegister(newState);
    // Optional: Display relay status on LCD
    lcd.setCursor(0, 1);
    lcd.print("Relay Update: ");
    lcd.print(currentRelayState, HEX);
  }
}

void processData() {
  static String jsonString;
  //  static bool receiving = false;
  const size_t CAPACITY = 1024;
  unsigned long lastDataTime = 0;
  while (espSerial.available()) {
    lastDataTime = millis();  // Reset timeout when data received
    char c = espSerial.read();

    // Turn PROCESS_LED ON for 60ms
    digitalWrite(PROCESS_LED, HIGH);
    processLedOnTime = millis();
    processLedState = true;

    if (c == 'S') {
      receiving = true;
      jsonString = "";
      continue;
    }

    if (receiving) {
      if (c == 'E') {
        receiving = false;
        processJsonData(jsonString);
      } else if (jsonString.length() < CAPACITY) {
        jsonString += c;
      }
    }
  }
}

void serialOutput() {
  Serial.println("\n=== Measurements ===");
  Serial.printf("Voltage: %.1fV\n", measurements.voltage);
  Serial.printf("Current: %.2fA\n", measurements.current);
  Serial.printf("Power: %.1fW\n", measurements.power);
  Serial.printf("Energy: %.2fkWh\n", measurements.energy);
  Serial.printf("Frequency: %.2fHz\n", measurements.frequency);
  Serial.printf("Power Factor: %.2f\n", measurements.pf);
  Serial.printf("Uptime: %.1fh\n", measurements.uptime);
  Serial.printf("Total Days: %.1f\n", measurements.totalDays);

  Serial.println("\n=== Threshold & Interval ===");
  Serial.printf("Voltage High: %.1fV\n", settings.voltHigh);
  Serial.printf("Voltage Low: %.1fV\n", settings.voltLow);
  Serial.printf("Power High: %.1fW\n", settings.powerHigh);
  Serial.printf("Power Low: %.1fW\n", settings.powerLow);
  Serial.printf("Frequency High: %.2fHz\n", settings.freqHigh);
  Serial.printf("Frequency Low: %.2fHz\n", settings.freqLow);
  Serial.printf("Target PF: %.2f\n", settings.targetFactor);
  Serial.printf("MFD per Channel: %.2f\n", settings.mfdPerChannel);
  Serial.printf("Max kWh: %d\n", settings.maxkWh);
  Serial.printf("Startup Time: %ds\n", settings.startupTime);
  Serial.printf("AFPC LOW WATT: %dW\n", settings.afpcLowWatt);
  Serial.printf("AFPC HIGH WATT: %dW\n", settings.afpcHighWatt);
  Serial.printf("ESP Share interval: %ds\n", settings.espShareInterval);
  Serial.printf("Node Red interval: %ds\n", settings.nodeRedInterval);
  Serial.printf("PZEM interval: %ds\n", settings.pzemInterval);
  Serial.printf("Alert Interval: %ds\n", settings.alertInterval);

  Serial.println("\n=== Alert States ===");
  Serial.printf("High Voltage Alert: %s\n", settings.highVoltageAlert ? "ON" : "OFF");
  Serial.printf("Low Voltage Alert: %s\n", settings.lowVoltageAlert ? "ON" : "OFF");
  Serial.printf("High Power Alert: %s\n", settings.highPowerAlert ? "ON" : "OFF");
  Serial.printf("Low Power Alert: %s\n", settings.lowPowerAlert ? "ON" : "OFF");
  Serial.printf("High Freq Alert: %s\n", settings.highFreqAlert ? "ON" : "OFF");
  Serial.printf("Low Freq Alert: %s\n", settings.lowFreqAlert ? "ON" : "OFF");

  Serial.println("\n=== Protection States ===");
  Serial.printf("High V Protection: %s\n", settings.highVProtec ? "ON" : "OFF");
  Serial.printf("Low V Protection: %s\n", settings.LowVProtec ? "ON" : "OFF");
  Serial.printf("High P Protection: %s\n", settings.highPProtec ? "ON" : "OFF");
  Serial.printf("High Hz Protection: %s\n", settings.highHzProtec ? "ON" : "OFF");
  Serial.printf("Low Hz Protection: %s\n", settings.lowHzProtec ? "ON" : "OFF");

  Serial.println("\n=== Feature States ===");
  Serial.printf("AFPC: %s\n", settings.afpcEnabled ? "ON" : "OFF");
  Serial.printf("Main Output: %s\n", settings.mainOutput ? "ON" : "OFF");
  Serial.printf("kWh Saver: %s\n", settings.kWhSaver ? "ON" : "OFF");
  Serial.println("============================\n");
}
void processJsonData(const String& jsonString) {
  Serial.println("\n=== Raw Data ===");
  Serial.println(jsonString);

  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, jsonString);

  if (!error) {
    // Convert string values to float for measurements
    measurements.voltage = doc["v"].as<String>().toFloat();
    measurements.current = doc["i"].as<String>().toFloat();
    measurements.power = doc["p"].as<String>().toFloat();
    measurements.energy = doc["e"].as<String>().toFloat();
    measurements.frequency = doc["f"].as<String>().toFloat();
    measurements.pf = doc["pf"].as<String>().toFloat();
    measurements.uptime = doc["up"].as<String>().toFloat();
    measurements.totalDays = doc["d"].as<String>().toFloat();

    // Update threshold values
    settings.voltHigh = doc["vh"].as<String>().toFloat();
    settings.voltLow = doc["vl"].as<String>().toFloat();
    settings.powerHigh = doc["ph"].as<String>().toFloat();
    settings.powerLow = doc["pl"].as<String>().toFloat();
    settings.freqHigh = doc["fh"].as<String>().toFloat();
    settings.freqLow = doc["fl"].as<String>().toFloat();
    settings.targetFactor = doc["tpf"].as<String>().toFloat();
    settings.mfdPerChannel = doc["mfd"].as<String>().toFloat();
    settings.maxkWh = doc["max"].as<int>();
    settings.startupTime = doc["st"].as<int>();

    settings.afpcLowWatt = doc["pfl"].as<int>();
    settings.afpcHighWatt = doc["pfh"].as<int>();
    settings.espShareInterval = doc["esi"].as<int>();
    settings.nodeRedInterval = doc["nri"].as<int>();
    settings.pzemInterval = doc["pzi"].as<int>();
    settings.alertInterval = doc["sai"].as<int>();

    // Update alert states
    settings.highVoltageAlert = doc["hva"] | false;
    settings.lowVoltageAlert = doc["lva"] | false;
    settings.highPowerAlert = doc["hpa"] | false;
    settings.lowPowerAlert = doc["lpa"] | false;
    settings.highFreqAlert = doc["hfa"] | false;
    settings.lowFreqAlert = doc["lfa"] | false;

    // Update protection states
    settings.highVProtec = doc["hvp"] | false;
    settings.LowVProtec = doc["lvp"] | false;
    settings.highPProtec = doc["hpp"] | false;
    settings.highHzProtec = doc["hhp"] | false;
    settings.lowHzProtec = doc["lhp"] | false;

    // Update feature states
    settings.afpcEnabled = doc["afpc"] | false;
    settings.mainOutput = doc["out"] | false;
    settings.kWhSaver = doc["kwhs"] | false;

    // Print detailed debug information
    serialOutput();
    newSettingsReceived = true;
  } else {
    Serial.printf("Parse error: %s\n", error.c_str());
    Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
  }
}
// AFPC Display Status
void displayAfpcStatus(const char* message, int activeRelays) {
  //  lcd.setCursor(0, 1);
  //  lcd.print("                "); // Clear any remaining chars
  lcd.setCursor(0, 1);
  lcd.print(message);


  // Optional: Log to serial for debugging
  Serial.println("\n=== AFPC Status ===");
  Serial.printf("Message: %s\n", message);
  Serial.printf("Active Relays: %d\n", activeRelays);
  Serial.printf("Current PF: %.2f\n", measurements.pf);
  Serial.printf("Target PF: %.2f\n", settings.targetFactor);
  Serial.printf("Power: %.1fW\n", measurements.power);
  Serial.println("==================\n");
}

// AFPC Controlling
void controlAFPC() {
  static bool wasEnabled = false;

  const float hysteresis = 0.01;  // To avoid relay flicker

  // Skip updates until interval has passed
  if (millis() - lastAfpcUpdate < AFPC_UPDATE_INTERVAL) {
    return;
  }
  lastAfpcUpdate = millis();

  wasEnabled = true;  // AFPC is enabled now


  // Check power range
  if (measurements.power < settings.afpcLowWatt || measurements.power > settings.afpcHighWatt) {
    afpcRelayState = 0xFFFF;  // All relays OFF
    send16ToShiftRegister(afpcRelayState);
    displayAfpcStatus("OUT OF RANGE    ", 0);
    return;
  }

  // Get target PF from settings
  float currentPF = measurements.pf;
  float targetPF = settings.targetFactor;
  float MFD_PER_CAP = settings.mfdPerChannel;

  // If PF is already acceptable
  if (currentPF >= targetPF - hysteresis) {
    afpcRelayState = 0xFFFF;  // All relays OFF
    send16ToShiftRegister(afpcRelayState);
    displayAfpcStatus("PF = OK         ", 0);
    return;
  }

  // Calculate required capacitance
  float kVAR = measurements.power * (tan(acos(currentPF)) - tan(acos(targetPF)));
  float totalMfdNeeded = (kVAR * 1e6) / (pow(measurements.voltage, 2) * 2 * PI * measurements.frequency);

  int capsNeeded = round(totalMfdNeeded / MFD_PER_CAP);

  // Check if we need more capacitors than available
  if (capsNeeded > MAX_RELAYS) {
    // DO NOT turn all relays ON at once!
    // Just keep all currently ON, show warning, but do not re-trigger all ON
    char statusMsg[16];
    snprintf(statusMsg, 16, "NEED %dMFD MORE",
             (capsNeeded - MAX_RELAYS) * (int)MFD_PER_CAP);
    displayAfpcStatus(statusMsg, MAX_RELAYS);
    return;
  }

  // Normal operation
  capsNeeded = constrain(capsNeeded, 0, MAX_RELAYS);
  uint16_t newState = 0xFFFF;  // Start with all OFF

  for (int i = 0; i < capsNeeded; i++) {
    bitClear(newState, i);  // Turn ON required number of relays
  }

  // Update relay state if changed
  if (newState != afpcRelayState) {
    afpcRelayState = newState;
    send16ToShiftRegister(newState);

    char statusMsg[16];
    int totalMfd = capsNeeded * MFD_PER_CAP;
    snprintf(statusMsg, 16, "AFPC:%2d=%3dMFD  ", capsNeeded, totalMfd);
    displayAfpcStatus(statusMsg, capsNeeded);
  }
}

// Watt Led Status
void updateWattLedBlink() {
  float minPower = 0.0f;
  float maxPower = settings.powerHigh > 0 ? settings.powerHigh : 5000.0f;  // Avoid divide by zero
  unsigned long minInterval = 100;                                         // Fastest blink (ms)
  unsigned long maxInterval = 1500;                                        // Slowest blink (ms)
  const unsigned long ledOnDuration = 50;                                  // LED ON time fixed (ms)

  float p = constrain(measurements.power, minPower, maxPower);

  // Calculate interval: higher power = shorter interval (faster blink)
  float ratio = (maxPower - p) / (maxPower - minPower);
  unsigned long offInterval = minInterval + (unsigned long)((maxInterval - minInterval) * ratio);

  // LED logic: ON for ledOnDuration, then OFF for offInterval
  if (!wattLedState) {
    // LED is OFF, check if it's time to turn ON
    if (millis() - lastWattBlink >= offInterval) {
      wattLedState = true;
      digitalWrite(WATT_LED, HIGH);
      wattLedOnTime = millis();
    }
  } else {
    // LED is ON, keep it ON for ledOnDuration
    if (millis() - wattLedOnTime >= ledOnDuration) {
      wattLedState = false;
      digitalWrite(WATT_LED, LOW);
      lastWattBlink = millis();
    }
  }
}

// Protection Led Status
void protectionLed() {
  if (settings.highVProtec || settings.LowVProtec || settings.highPProtec || settings.highHzProtec || settings.lowHzProtec) {
    digitalWrite(PROTEC_LED, HIGH);
  } else {
    digitalWrite(PROTEC_LED, LOW);
  }
}

// Safety alert message
void displayAlerts() {
  static unsigned long lastDisplayUpdate = 0;
  const unsigned long DISPLAY_UPDATE_INTERVAL = 2000;  // 2 seconds per alert

  if (millis() - lastDisplayUpdate < DISPLAY_UPDATE_INTERVAL) return;
  lastDisplayUpdate = millis();

  static uint8_t currentAlert = 0;
  bool anyAlertActive = false;

  // Create array of active alerts
  struct AlertMessage {
    bool active;
    const char* message;
  } alerts[6] = {
    { measurements.voltage > settings.voltHigh, "HIGH VOLTAGE!   " },
    { measurements.voltage < settings.voltLow, "LOW VOLTAGE!    " },
    { measurements.power > settings.powerHigh, "HIGH POWER!     " },
    { measurements.power < settings.powerLow, "LOW POWER!      " },
    { measurements.frequency > settings.freqHigh, "HIGH FREQUENCY! " },
    { measurements.frequency < settings.freqLow, "UNDER FREQUENCY!" }
  };

  // Find next active alert
  uint8_t alertCount = 0;
  for (int i = 0; i < 6; i++) {
    if (alerts[i].active) {
      anyAlertActive = true;
      alertCount++;
    }
  }

  if (anyAlertActive) {
    // Find next active alert to display
    do {
      currentAlert = (currentAlert + 1) % 6;
    } while (!alerts[currentAlert].active);

    // Display alert

    lcd.setCursor(0, 0);
    lcd.print(alerts[currentAlert].message);
  } else {
    lcd.setCursor(0, 0);
    lcd.print("ALL : NORMAL    ");
  }
}

// Handshake
void waitForHandshake() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("PLEASE WAIT...  ");
  lcd.setCursor(0, 1);
  lcd.print("                ");

  unsigned long startTime = millis();
  const unsigned long handshakeTimeout = 6e5;  // 10 minutes timeout (optional)

  while (true) {
    if (espSerial.available()) {
      String msg = espSerial.readStringUntil('\n');
      msg.trim();
      if (msg == "HELLO" || msg == "READY") {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("CONNECTED...    ");
        lcd.setCursor(0, 1);
        lcd.print("                ");
        delay(1000);
        break;
      }
    }
    // Optional: Timeout to avoid infinite wait
    if (millis() - startTime > handshakeTimeout) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("FAILED...       ");
      lcd.setCursor(0, 1);
      lcd.print("                ");
      delay(1000);
      break;
    }
    delay(10);
  }
}

//=============================================================
// Setup Function
//=============================================================

void setup() {
  Serial.begin(115200);  // Debug serial
  espSerial.begin(115200, SERIAL_8N1, ESP_COMM_RX, ESP_COMM_TX);
  initializeOutputPins();
  lcd.init();
  lcd.backlight();
  initializeDefaultValues();
  displayWelcomeMessage();
  delay(1000);
  waitForHandshake();
}

//=============================================================
// Main Loop
//=============================================================
void loop() {
  unsigned long currentMillis = millis();
  static unsigned long lastRefresh = 0;
  static unsigned long lastCycle = 0;
  const unsigned long CYCLE_INTERVAL = 2000;  // Cycle every 2 seconds
  static int cycleIndex = 0;                  // 0=V, 1=I, 2=P

  // Process serial data
  processData();
  updateWattLedBlink();
  protectionLed();

  // Cycle through V/I/P on line 1 (unless AFPC is overriding)
  if (currentMillis - lastCycle >= CYCLE_INTERVAL) {
    lastCycle = currentMillis;
    char displayBuf[17];  // 16 chars + null terminator

    if (cycleIndex == 0) {
      snprintf(displayBuf, 17, "VOLT: %.1f V           ", measurements.voltage);
    } else if (cycleIndex == 1) {
      snprintf(displayBuf, 17, "CURR: %.2f A           ", measurements.current);
    } else {
      snprintf(displayBuf, 17, "POWR: %.1f W           ", measurements.power);
    }

    cycleIndex = (cycleIndex + 1) % 3;  // Cycle: V -> I -> P -> V...

    // Only update if AFPC isn't actively displaying its status
    // (AFPC updates every 3s, so this won't conflict much)
    if (!settings.afpcEnabled || measurements.power < settings.afpcLowWatt || measurements.power > settings.afpcHighWatt) {
      lcd.setCursor(0, 1);
      lcd.print(displayBuf);
    }
  }

  // Update display and AFPC
  if (currentMillis - lastRefresh >= 100) {
    displayAlerts();

    if (!settings.afpcEnabled) {
      send16ToShiftRegister(0xFFFF);  // All relays OFF
      // Let cycling handle line 1
    }

    if (settings.afpcEnabled && measurements.power >= settings.afpcLowWatt && measurements.power <= settings.afpcHighWatt) {
      controlAFPC();  // Handle AFPC if enabled and within power range (this will override line 1 temporarily)
    }

    lastRefresh = currentMillis;
  }

  // Turn PROCESS_LED off after 60ms
  if (processLedState && (millis() - processLedOnTime >= 60)) {
    digitalWrite(PROCESS_LED, LOW);
    processLedState = false;
  }
}

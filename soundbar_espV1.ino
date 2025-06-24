#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP4725.h>
#include <HomeSpan.h>
#include <WiFi.h>

// I2C-Adressen
Adafruit_ADS1115 ads; // ADS1115 (Adresse wird in begin gesetzt)
Adafruit_MCP4725 dac; // MCP4725 (Adresse wird in begin gesetzt)

// GPIO für Schaltimpuls
#define SWITCH_PIN 17 // GPIO 17 für Schaltimpuls

// Globale Variablen
int lastLevel = 50; // Aktueller Lautstärkepegel (0–100%)
int lastNonZeroLevel = 50; // Letzter nicht-0-Lautstärkepegel
int potLevel = 50; // Aktueller Potentiometer-Wert
int lastPotLevel = 50; // Letzter Potentiometer-Wert (für Änderungsprüfung)
float inputVoltage = 0.0; // Eingangsspannung vom Potentiometer (A0)
float statusVoltage = 0.0; // Spannung von Status-LEDs (A1)
String lastSource = "Init"; // Letzte Änderungsquelle (Poti oder App)
bool switchState = true; // Schalter-Status (Ein=true, Aus=false)
bool lastSwitchState = true; // Letzter Schalter-Status (für Impulssteuerung)
unsigned long pulseStartTime = 0; // Zeitpunkt des Impulsstarts
bool pulseActive = false; // Ob ein Impuls aktiv ist

// Wi-Fi-Zugangsdaten
const char* ssid = "XXXXX";
const char* password = "XXXXXX";

// Globale Funktionen
void updateDAC(int value) {
  int dacValue = map(value, 0, 100, 0, 4095); // Skaliere auf 12-Bit (0–5 V)
  dac.setVoltage(dacValue, false);
}

void readPotentiometer() {
  int16_t adc = ads.readADC_SingleEnded(0); // A0 auslesen
  inputVoltage = (adc * 0.000125);          // 0.125 mV pro Schritt (bei Gain 1)
  potLevel = constrain(map(inputVoltage * 1000, 0, 4050, 0, 100), 0, 100);
}

void readStatusVoltage() {
  int16_t adc = ads.readADC_SingleEnded(1); // A1 auslesen
  statusVoltage = (adc * 0.000125);         // 0.125 mV pro Schritt (bei Gain 1)
}

void logChange() {
  float voltageOut = (lastLevel * 5.0) / 100.0; // Berechne Ausgangsspannung
  Serial.print("Änderung von: ");
  Serial.print(lastSource);
  Serial.print(" | Poti-Spannung (A0): ");
  Serial.print(inputVoltage);
  Serial.print(" V | Status-LEDs (A1): ");
  Serial.print(statusVoltage);
  Serial.print(" V | Level: ");
  Serial.print(lastLevel);
  Serial.print("% | Ausgangsspannung: ");
  Serial.print(voltageOut);
  Serial.print(" V | Schalter: ");
  Serial.println(switchState ? "Ein" : "Aus");
}

void triggerPulse() {
  digitalWrite(SWITCH_PIN, HIGH); // Starte Impuls
  pulseStartTime = millis();
  pulseActive = true;
}

// HomeSpan Dimmer Service (Lautstärke)
struct Dimmer : Service::LightBulb {
  SpanCharacteristic *level;

  Dimmer() : Service::LightBulb() {
    level = new Characteristic::Brightness(50); // Start mit 50%
  }

  boolean update() {
    if (level->updated() && level->getNewVal() != lastLevel) {
      lastLevel = level->getNewVal();
      lastSource = "Apple Home (Level)";
      if (lastLevel > 0) {
        lastNonZeroLevel = lastLevel;
      }
      updateDAC(lastLevel);
      logChange();
    }
    return true;
  }

  void loop() {
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck > 100) { // Prüfe alle 100 ms
      readPotentiometer();

      // Potentiometer-Steuerung für Lautstärke
      if (abs(potLevel - lastPotLevel) > 5) {
        lastLevel = potLevel;
        lastPotLevel = potLevel;
        lastSource = "Potentiometer (Level)";
        if (lastLevel > 0) {
          lastNonZeroLevel = lastLevel;
        }
        level->setVal(lastLevel); // Aktualisiere Apple Home
        updateDAC(lastLevel);
        logChange();
      }

      lastCheck = millis();
    }
  }
};

// HomeSpan Switch Service (Ein/Aus)
struct PowerSwitch : Service::Switch {
  SpanCharacteristic *power;

  PowerSwitch() : Service::Switch() {
    power = new Characteristic::On(true); // Start mit Ein
  }

  boolean update() {
    if (power->updated()) {
      switchState = power->getNewVal();
      lastSource = "Apple Home (Switch)";
      triggerPulse();
      logChange();
    }
    return true;
  }

  void loop() {
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck > 100) { // Prüfe alle 100 ms
      readStatusVoltage();

      // Schalter-Status basierend auf Status-LEDs (A1)
      bool newSwitchState = (statusVoltage > 0.01); // Schwellwert (Dioden-Spannungsabfall berücksichtigt)
      if (newSwitchState != switchState) {
        switchState = newSwitchState;
        power->setVal(switchState);
        lastSource = "Status-LEDs (A1)";
        triggerPulse();
        logChange();
      }

      // Impulssteuerung
      if (switchState != lastSwitchState) {
        triggerPulse();
        lastSwitchState = switchState;
      }

      lastCheck = millis();
    }

    // Impulssteuerung
    if (pulseActive && millis() - pulseStartTime >= 500) {
      digitalWrite(SWITCH_PIN, LOW); // Impuls beenden
      pulseActive = false;
    }
  }
};

Dimmer *dimmer;
PowerSwitch *powerSwitch;

// Wi-Fi reconnect Funktion
void connectToWiFi() {
  Serial.print("Verbinde mit Wi-Fi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi-Fi verbunden, IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWi-Fi Verbindung fehlgeschlagen!");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Warte auf Serielle Verbindung
  Wire.begin(); // I2C initialisieren

  // GPIO 17 Setup
  pinMode(SWITCH_PIN, OUTPUT);
  digitalWrite(SWITCH_PIN, LOW); // Standardmäßig aus

  // Wi-Fi Setup
  WiFi.mode(WIFI_STA);
  connectToWiFi();

  // HomeSpan Setup
  Serial.println("Initialisiere HomeSpan...");
  homeSpan.deleteStoredValues(); // Lösche HomeKit-Daten bei jedem Start
  homeSpan.setWifiCredentials(ssid, password);
  homeSpan.begin();
  homeSpan.setPairingCode("45165015"); // Pairing-Code: 451-65-015

  // ADS1115 Setup
  ads.setGain(GAIN_ONE); // ±4.096 V Bereich
  if (!ads.begin(0x48)) {
    Serial.println("Fehler: ADS1115 nicht gefunden!");
    while (1);
  }

  // MCP4725 Setup
  if (!dac.begin(0x60)) {
    Serial.println("Fehler: MCP4725 nicht gefunden!");
    while (1);
  }
  dac.setVoltage(2048, false); // Start mit 50% (2,5 V)

  // HomeSpan Accessory Setup
  new SpanAccessory();
  new Service::AccessoryInformation();
  new Characteristic::Name("Soundanlage Dimmer");
  new Characteristic::Manufacturer("DIY");
  new Characteristic::SerialNumber("0001");
  new Characteristic::Model("Dimmer");
  new Characteristic::FirmwareRevision("1.0");
  new Characteristic::Identify();
  dimmer = new Dimmer();
  powerSwitch = new PowerSwitch();

  Serial.println("Setup abgeschlossen. HomeKit Pairing-Code: 451-65-015");
  Serial.println("Warte auf Änderungen...");
}

void loop() {
  homeSpan.poll(); // HomeSpan aktualisieren

  // Wi-Fi-Verbindung überwachen
  static unsigned long lastWifiCheck = 0;
  if (millis() - lastWifiCheck > 5000) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Wi-Fi nicht verbunden! Versuche erneut...");
      connectToWiFi();
    } else {
      Serial.println("Wi-Fi verbunden, IP: " + WiFi.localIP().toString());
    }
    lastWifiCheck = millis();
  }
}
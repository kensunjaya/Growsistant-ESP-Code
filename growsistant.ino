#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <BH1750.h>
#include <DHT.h>
#include <ArduinoJson.h>

#include "secrets.h"

// ---------- Pins & sensors ----------
#define DHTPIN   D4
#define DHTTYPE  DHT11
#define PUMP_PIN D5
#define LED_PIN  D6
#define SOIL_PIN A0

BH1750 lightMeter;
DHT dht(DHTPIN, DHTTYPE);

// ---------- Control thresholds ----------
int soilDryThreshold = 600;
int lightThreshold   = 200;

// ---------- MQTT ----------
WiFiClientSecure espClient;
PubSubClient client(espClient);

String clientId;
String topicSensors;
String topicCmd;

// ---------- Schedulers ----------
const unsigned long PUBLISH_EVERY_MS = 6000;
unsigned long lastPublishMs = 0;

// ---------- Manual overrides / timers ----------
bool          pumpManualActive = false;
unsigned long pumpOffAtMs      = 0;

bool          lightManualActive = false;
bool          isLightON         = false;
int           lightManualPwm    = 0;    // 0..1023
unsigned long lightOffAtMs      = 0;    // 0 if indefinite

// ---------- Auto enable flags ----------
bool autoPumpEnabled  = true;
bool autoLightEnabled = true;

// ---------- Forward decl ----------
void ensureMqtt();
void handleCommandJson(const String& json);
void setLampPwm(int pwm);
void startPumpFor(unsigned long ms);

// ---------- Sensor snapshot helpers ----------
struct SensorSnapshot {
  int   soil;
  float lux;
  float temp;
  float hum;
};

SensorSnapshot readSensors() {
  SensorSnapshot s;
  s.soil = analogRead(SOIL_PIN);
  s.lux  = lightMeter.readLightLevel();
  s.hum  = dht.readHumidity();
  s.temp = dht.readTemperature();
  return s;
}

void publishSensors(const SensorSnapshot& s) {
  StaticJsonDocument<240> doc;
  doc["soil"]   = s.soil;
  doc["lux"]    = s.lux;
  doc["temp"]   = s.temp;
  doc["hum"]    = s.hum;
  doc["lamp"]   = isLightON;
  doc["auto_lamp"] = autoLightEnabled;
  doc["auto_pump"] = autoPumpEnabled;
  
  char json[256];
  size_t n = serializeJson(doc, json, sizeof(json));
  bool ok = client.publish(topicSensors.c_str(), json, n); // QoS0
  Serial.print("Published -> ");
  Serial.print(topicSensors);
  Serial.print(" : ");
  Serial.println(json);
  if (!ok) Serial.println("MQTT publish failed");
}

void setupWifi() {
  WiFiManager wifiManager;
  wifiManager.autoConnect("Growsistant-Setup");
  Serial.println("WiFi connected.");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  String msg; msg.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  Serial.print("MQTT IN  ["); Serial.print(topic); Serial.print("] ");
  Serial.println(msg);

  if (String(topic) != topicCmd) return;

  // --- Raw text "UPD" support ---
  String trimmed = msg; trimmed.trim();
  if (trimmed.equalsIgnoreCase("UPD")) {
    SensorSnapshot s = readSensors();
    publishSensors(s);
    lastPublishMs = millis(); // reset periodic timer
    return;
  }

  // --- Otherwise JSON commands ---
  handleCommandJson(msg);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection... ");
    if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
      Serial.println("connected");
      client.subscribe(topicCmd.c_str(), 1); // PubSubClient will use QoS0 internally
      Serial.print("Subscribed to: "); Serial.println(topicCmd);
    } else {
      Serial.print("failed, rc="); Serial.print(client.state());
      Serial.println(" -> retry in 5s");
      delay(5000);
    }
  }
}

void ensureMqtt() {
  if (!client.connected()) reconnect();
  client.loop();
}


// 1) {"upd":true}  or {"cmd":"UPD"}  -> publish sensors now (manual)
// 2) {"light":false}                 -> LED ON  (PWM 800), indefinite
// 3) {"light":true}                  -> LED OFF (release manual)
// 4) {"light_pwm": 900}              -> LED PWM, indefinite
// 5) {"light_pwm": 900, "ms": 30000} -> LED PWM for 30s
// 6) {"pump_ms": 2000}               -> Run pump 2s
// 7) {"auto_light":true/false}, {"auto_pump":true/false}

void handleCommandJson(const String& json) {
  StaticJsonDocument<320> doc;
  DeserializationError err = deserializeJson(doc, json);
  if (err) {
    Serial.print("JSON parse error: "); Serial.println(err.c_str());
    return;
  }

  // Manual update triggers first
  if ((doc.containsKey("upd") && doc["upd"] == true) ||
      (doc.containsKey("cmd") && String(doc["cmd"].as<const char*>()).equalsIgnoreCase("UPD"))) {
    SensorSnapshot s = readSensors();
    publishSensors(s);
    lastPublishMs = millis();
    // Don't return; allow combining with other commands in one payload if desired
  }

  // Auto toggles
  if (doc.containsKey("auto_light")) {
    autoLightEnabled = doc["auto_light"];
    Serial.print("Auto light: "); Serial.println(autoLightEnabled ? "ENABLED" : "DISABLED");
    if (autoLightEnabled && !lightManualActive) {
      setLampPwm(0);
      isLightON = false;
    }
  }
  if (doc.containsKey("auto_pump")) {
    autoPumpEnabled = doc["auto_pump"];
    Serial.print("Auto pump: "); Serial.println(autoPumpEnabled ? "ENABLED" : "DISABLED");
  }

  // Lamp control
  if (doc.containsKey("light")) {
    bool v = doc["light"];
    if (v) {
      lightManualActive = true;
      isLightON = true;
      setLampPwm(800);
      lightOffAtMs = 0; // indefinite
      Serial.println("Lamp -> ON (manual, PWM=800)");
    } else {
      lightManualActive = false;
      isLightON = false;
      setLampPwm(0);
      lightOffAtMs = 0;
      Serial.println("Lamp -> OFF (manual released)");
    }
  }

  if (doc.containsKey("light_pwm")) {
    int pwm = constrain((int)doc["light_pwm"], 0, 1023);
    lightManualActive = true;
    setLampPwm(pwm);
    isLightON = (pwm > 0);

    if (doc.containsKey("ms")) {
      unsigned long ms = (unsigned long)doc["ms"];
      lightOffAtMs = millis() + ms;
      Serial.printf("Lamp -> PWM=%d for %lu ms\n", pwm, ms);
    } else {
      lightOffAtMs = 0; // indefinite
      Serial.printf("Lamp -> PWM=%d (manual, indefinite)\n", pwm);
    }
  }

  // Pump control
  if (doc.containsKey("pump_ms")) {
    unsigned long ms = (unsigned long)doc["pump_ms"];
    startPumpFor(ms);
    Serial.printf("Pump -> RUN for %lu ms\n", ms);
  }
}

void startPumpFor(unsigned long ms) {
  pumpManualActive = true;
  pumpOffAtMs      = millis() + ms;
  digitalWrite(PUMP_PIN, HIGH);
}

void setLampPwm(int pwm) {
  pwm = constrain(pwm, 0, 1023);
  lightManualPwm = pwm;
  analogWrite(LED_PIN, pwm);
}


void setup() {
  Serial.begin(9600);
  delay(100);

  Wire.begin();
  lightMeter.begin();
  dht.begin();

  pinMode(PUMP_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  analogWrite(LED_PIN, 0);

  setupWifi();

  espClient.setInsecure();

  client.setServer(MQTT_SERVER, MQTT_PORT); // 8883 for TLS
  client.setCallback(onMqttMessage);
  client.setKeepAlive(20);

  // Build clientId and topics
  clientId     = "GS-" + String(ESP.getChipId(), HEX); // e.g. GS-f6a3ab
  topicSensors = clientId + "/sensors";
  topicCmd     = clientId + "/cmd";

  reconnect();
}

void loop() {
  ensureMqtt();

  const unsigned long now = millis();

  // --- Manual timers ---
  if (pumpManualActive && now >= pumpOffAtMs) {
    pumpManualActive = false;
    digitalWrite(PUMP_PIN, LOW);
    Serial.println("Pump -> STOP (timer elapsed)");
  }

  if (lightManualActive && lightOffAtMs != 0 && now >= lightOffAtMs) {
    lightManualActive = false;
    lightOffAtMs = 0;
    setLampPwm(0);
    isLightON = false;
    Serial.println("Lamp -> OFF (timer elapsed)");
  }

  // --- Periodic sample & publish ---
  if (now - lastPublishMs >= PUBLISH_EVERY_MS) {
    lastPublishMs = now;

    SensorSnapshot s = readSensors();

    // Local controllers (auto)
    if (autoPumpEnabled && !pumpManualActive) {
      if (s.soil > soilDryThreshold) {
        startPumpFor(2000);
        Serial.println("Auto Pump -> DRY detected, pulse 2s");
      }
    }
    if (autoLightEnabled && !lightManualActive) {
      if (s.lux < lightThreshold) {
        isLightON = true;
        setLampPwm(800);
      } else {
        isLightON = false;
        setLampPwm(0);
      }
    }

    publishSensors(s);
  }

  // 1ms yield keeps WiFi happy without blocking loop responsiveness
  delay(1);
}

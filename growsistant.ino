#include <WiFiClientSecure.h>
#include <ESP8266WiFi.h>
#include <WiFiManager.h> 
#include <PubSubClient.h>
#include <Wire.h>
#include <BH1750.h>
#include <DHT.h>

#include "secrets.h"

#define DHTPIN D4
#define DHTTYPE DHT11
#define PUMP_PIN D5
#define LED_PIN D6
#define SOIL_PIN A0

BH1750 lightMeter;
DHT dht(DHTPIN, DHTTYPE);

int soilDryThreshold = 600;
int lightThreshold = 200;

WiFiClientSecure espClient;
PubSubClient client(espClient);

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.gatewayIP());
  Serial.println(WiFi.dnsIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    String clientId = "ESP8266Client-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(8000);
    }
  }
}

void setup() {
  Serial.begin(9600);
  WiFiManager wifiManager;
  Wire.begin();
  lightMeter.begin();
  dht.begin();
  
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
//  setup_wifi();
  wifiManager.autoConnect("Growsistant-Setup");
  Serial.println("WiFi connected.");
  Serial.println(WiFi.localIP());
  espClient.setInsecure();
  
  client.setServer(MQTT_SERVER, MQTT_PORT);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  // --- Read Sensors ---
  int soilValue = analogRead(SOIL_PIN);
  float lux = lightMeter.readLightLevel();
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // Local controller
  if (soilValue > soilDryThreshold) {
    digitalWrite(PUMP_PIN, HIGH);
  } else {
    digitalWrite(PUMP_PIN, LOW);
  }
  
  if (lux < lightThreshold) {
    analogWrite(LED_PIN, 800);
  } else {
    analogWrite(LED_PIN, 0);
  }

  // Publish to MQTT
  String payload = "{";
  payload += "\"soil\": " + String(soilValue);
  payload += ", \"lux\": " + String(lux);
  payload += ", \"temp\": " + String(t);
  payload += ", \"hum\": " + String(h);
  payload += "}";

  client.publish(MQTT_TOPIC, payload.c_str());

  Serial.print("Published: ");
  Serial.println(payload);

  delay(30000);
}

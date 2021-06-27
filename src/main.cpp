#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <DHT.h>
#include <DHT_U.h>
#include <ESPmDNS.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "config.h"

#define DHTPIN (17)
#define DHTTYPE DHT22
#define BUFFERSIZE (64)

DHT_Unified dht(DHTPIN, DHTTYPE);
constexpr uint32_t delayMS = 10000;

WiFiClient espClient;
PubSubClient client(espClient);  // lib required for mqtt

constexpr uint16_t mqttPort = 1883;
constexpr uint16_t otaPort = 3232;  // Port defaults to 3232

const char* mqttServer = "io.adafruit.com";  // mqtt server
const char* hostName = "gartenesp32";

const int AirValue = 3520;    // you need to replace this value with Value_1
const int WaterValue = 1280;  // you need to replace this value with Value_2
const int MoisturePin = 35;
int soilMoistureValue = 0;
int soilmoisturepercent = 0;

unsigned long myTime = 0;

char topicHumidity[BUFFERSIZE];
char topicWaterSoil[BUFFERSIZE];
char topicTemp[BUFFERSIZE];
char topicLog[BUFFERSIZE];

char bufferHumidity[BUFFERSIZE];
char bufferWaterSoil[BUFFERSIZE];
char bufferTemp[BUFFERSIZE];
char bufferLog[BUFFERSIZE];

void connectmqtt();
void reconnect();
void callback(char* topic, byte* payload, unsigned int length);
void readMoisture();
void prepareStrings();

void prepareStrings() {
  snprintf(topicHumidity, BUFFERSIZE, "%s%s", IO_USERNAME,
           "/f/don-pavlov.humidity");
  snprintf(topicWaterSoil, BUFFERSIZE, "%s%s", IO_USERNAME,
           "/f/don-pavlov.soilmoisture");
  snprintf(topicTemp, BUFFERSIZE, "%s%s", IO_USERNAME,
           "/f/don-pavlov.temperature");
  snprintf(topicLog, BUFFERSIZE, "%s%s", IO_USERNAME, "/f/don-pavlov.log");
}

void setup() {
  prepareStrings();
  Serial.begin(115200);
  Serial.println("Booting");
  std::fill(bufferTemp, bufferTemp + BUFFERSIZE, 0);
  std::fill(bufferHumidity, bufferHumidity + BUFFERSIZE, 0);
  std::fill(bufferWaterSoil, bufferWaterSoil + BUFFERSIZE, 0);
  std::fill(bufferLog, bufferLog + BUFFERSIZE, 0);

  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  ArduinoOTA.setPort(otaPort);

  ArduinoOTA.setHostname(hostName);

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else  // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS
        // using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial.println("End Failed");
      });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  dht.begin();
  sensor_t sensor;

  Serial.println(F("DHTxx Unified Sensor Example"));
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("°C"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("°C"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("%"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("%"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
}

void loop() {
  ArduinoOTA.handle();

  if (millis() - myTime > delayMS) {
    myTime = millis();
    // Get temperature event and print its value.
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
      Serial.println(F("Error reading temperature!"));
    } else {
      snprintf(bufferTemp, BUFFERSIZE, "Temperature: %.2f°C",
               event.temperature);
      Serial.println(bufferTemp);
      std::fill(bufferTemp, bufferTemp + BUFFERSIZE, 0);
      snprintf(bufferTemp, BUFFERSIZE, "%.2f", event.temperature);
      if (client.connected()) {
        client.publish(topicTemp, bufferTemp);
      }
      std::fill(bufferTemp, bufferTemp + BUFFERSIZE, 0);
    }
    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
      Serial.println(F("Error reading humidity!"));
    } else {
      snprintf(bufferHumidity, BUFFERSIZE, "Humidity: %.2f%%",
               event.relative_humidity);
      Serial.println(bufferHumidity);
      std::fill(bufferHumidity, bufferHumidity + BUFFERSIZE, 0);
      snprintf(bufferHumidity, BUFFERSIZE, "%.2f", event.relative_humidity);
      if (client.connected()) {
        client.publish(topicHumidity, bufferHumidity);
      }
      std::fill(bufferHumidity, bufferHumidity + BUFFERSIZE, 0);
    }

    readMoisture();

    if (!client.connected()) {
      reconnect();
    }

    client.loop();
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  if ((char)payload[0] == 'O' && (char)payload[1] == 'N')  // on
  {
    Serial.println("on");
  } else if ((char)payload[0] == 'O' && (char)payload[1] == 'F' &&
             (char)payload[2] == 'F')  // off
  {
    Serial.println(" off");
  }
  Serial.println();
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (client.connect(hostName, IO_USERNAME, IO_KEY)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(topicLog, "TempHumidMoist reconnected logging");
      // ... and resubscribe
      // client.subscribe("inTopic");

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void connectmqtt() {
  client.connect(hostName, IO_USERNAME,
                 IO_KEY);  // ESP will connect to mqtt broker with clientID
  {
    Serial.println("Connected to MQTT");
    // Once connected, publish an announcement...

    // ... and resubscribe
    // client.subscribe("inTopic"); //topic=Demo
    client.publish(topicLog, "TempHumidMoist started logging");

    if (!client.connected()) {
      reconnect();
    }
  }
}

void readMoisture() {
  soilMoistureValue = analogRead(MoisturePin);
  Serial.println(soilMoistureValue);
  soilmoisturepercent = map(soilMoistureValue, AirValue, WaterValue, 0, 100);

  snprintf(bufferWaterSoil, BUFFERSIZE, "%d", soilmoisturepercent);
  // snprintf(bufferLog, BUFFERSIZE, "SoilRaw: %d", soilMoistureValue);
  if (client.connected()) {
    client.publish(topicWaterSoil, bufferWaterSoil);
    // client.publish(topicLog, bufferLog);
  }
  std::fill(bufferWaterSoil, bufferWaterSoil + BUFFERSIZE, 0);
  // std::fill(bufferLog, bufferLog + BUFFERSIZE, 0);
}

#include <Arduino.h>

#include <soc/rtc.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <string>
#include <sstream>



const char *ssid = "Iphone";
const char *password = "vallin69140";
const char *postEndpoint = "http://172.20.10.5:8080/api/v1/esp32/";
bool wifiConnected = false;
const float tempFreq = 0.00;
const int connectionConfig = 1;
const int connectionFreq = 0;
const char *url_mqtt = "test.mosquitto.org";
const char *topic = "emqx/esp32-KebabKebab70";
const char *mqtt_id = "esp32";
const int mqtt_port = 1883;

#pragma region TEMPERATURE
  const int TEMPERATURE_INTERVAL = 5;
  const int WIFI_INTERVAL = 5;
  const int BUTTON_PIN = 1;
  const int TEMPERATURE_PIN = 34;
  
  #define M1_CALPOINT1_CELSIUS 23.0f
  #define M1_CALPOINT1_RAW 128253742.0f
  #define M1_CALPOINT2_CELSIUS -20.0f
  #define M1_CALPOINT2_RAW 114261758.0f

  #define M2_CALPOINT1_CELSIUS 23.0f
  #define M2_CALPOINT1_RAW 163600.0f
  #define M2_CALPOINT2_CELSIUS -20.0f
  #define M2_CALPOINT2_RAW 183660.0f
#pragma endregion TEMPERATURE

float readTemp1(bool printRaw = false)
{
  uint64_t value = 0;
  int rounds = 100;

  for (int i = 1; i <= rounds; i++)
  {
    value += rtc_clk_cal_ratio(RTC_CAL_RTC_MUX, 100);
    yield();
  }
  value /= (uint64_t)rounds;

  if (printRaw)
  {
    printf("%s: raw value is: %llu\r\n", __FUNCTION__, value);
  }

  return ((float)value - M1_CALPOINT1_RAW) * (M1_CALPOINT2_CELSIUS - M1_CALPOINT1_CELSIUS) / (M1_CALPOINT2_RAW - M1_CALPOINT1_RAW) + M1_CALPOINT1_CELSIUS;
}

float readTemp2(bool printRaw = false)
{
  uint64_t value = rtc_time_get();
  delay(100);
  value = (rtc_time_get() - value);

  if (printRaw)
  {
    printf("%s: raw value is: %llu\r\n", __FUNCTION__, value);
  }

  return ((float)value - M1_CALPOINT1_RAW) * (M1_CALPOINT2_CELSIUS - M1_CALPOINT1_CELSIUS) / (M1_CALPOINT2_RAW - M1_CALPOINT1_RAW) + M1_CALPOINT1_CELSIUS;
}




void sendDataOverWiFi(float tempFreq)
{
  WiFiClient client;
  HTTPClient http;

  // Requête HTTP POST avec la valeur de température
  http.begin(postEndpoint);
  http.addHeader("Content-Type", "application/json");

  // Construction du corps de la requête
  DynamicJsonDocument postDoc(2000);

  postDoc["value"] = tempFreq;
  String postBody;
  serializeJson(postDoc, postBody);
  Serial.print(postBody);
  // Envoi de la requête POST
  http.POST(postBody);
  String line = client.readStringUntil('\n');
  Serial.println(line);
}

void getConfig(float tempFreq, int connectionConfig, int connectionFreq)
{
  HTTPClient http;
  http.begin("http://172.20.10.5:8080/api/v1/esp32/config");
  int httpResponseCode = http.GET();

  if (httpResponseCode == 200)
  {
    String response = http.getString();

    DynamicJsonDocument doc(2000);
    deserializeJson(doc, response);
    tempFreq = doc["tempFreq"];
    connectionConfig = doc["connectionConfig"];
    connectionFreq = doc["connectionFreq"];
  }
}

void connectToWiFi()
{
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("Connected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
}


WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void sendDataByMqtt(float temperature) {

  if (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT broker...");
    if (mqttClient.connect(mqtt_id)) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.println(mqttClient.state());
      return;
    }
  }

  // Create a JSON document to hold the temperature value
  DynamicJsonDocument jsonDoc(128);
  jsonDoc["value"] = temperature;

  // Serialize the JSON document to a string
  String payload;
  serializeJson(jsonDoc, payload);

  // Publish the temperature payload to the MQTT topic
  mqttClient.publish(topic, payload.c_str());

  Serial.print("Published to MQTT topic: ");
  Serial.println(topic);
}


void setup()
{

  Serial.begin(115200);

  connectToWiFi();

  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
  mqttClient.setServer(url_mqtt, mqtt_port);
}

void loop()
{

#pragma region TEMPERATURE

  // Lecture de la température interne
  float temperature = readTemp1(false);
  // Serial.print("Température : ");
  // Serial.println(temperature);

  // Publish and subscribe
  char *value = "Température : ";

#pragma endregion TEMPERATURE
  if (connectionConfig == 1)
  {
    connectToWiFi();
    // WiFi.disconnect();
    getConfig(tempFreq, connectionConfig, connectionFreq);
    sendDataOverWiFi(temperature);
    delay(tempFreq);
  }
  else if (connectionConfig == 2)
  {

    sendDataByMqtt(temperature);
    delay(10000);
  }

  Serial.flush();
}
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <DHT20.h>
#include "Arduino_MQTT_Client.h"
#include "Server_Side_RPC.h"
#include "ThingsBoard.h"

// ---------- WiFi Configuration ----------
constexpr char WIFI_SSID[] = "ACLAB";
constexpr char WIFI_PASSWORD[] = "ACLAB2023";

// ---------- ThingsBoard Configuration ----------
#define ENCRYPTED false
const char* TOKEN = "mze9614291gw4wsthfrz";
const char* THINGSBOARD_SERVER = "app.coreiot.io";
#if ENCRYPTED
WiFiClientSecure espClient;
const uint16_t THINGSBOARD_PORT = 8883;
#else
WiFiClient espClient;
const uint16_t THINGSBOARD_PORT = 1883;
#endif

// ---------- MQTT & ThingsBoard ----------
constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 256U;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 256U;

constexpr uint8_t MAX_RPC_SUBSCRIPTIONS = 3U;
constexpr uint8_t MAX_RPC_RESPONSE = 5U;

Arduino_MQTT_Client mqttClient(espClient);
Server_Side_RPC<MAX_RPC_SUBSCRIPTIONS, MAX_RPC_RESPONSE> rpc;
const std::array<IAPI_Implementation*, 1U> apis = { &rpc };
ThingsBoard tb(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE, Default_Max_Stack_Size, apis);

// ---------- DHT20 Sensor ----------
DHT20 dht20;
QueueHandle_t sensorQueue;

struct SensorData {
  double temperature;
  double humidity;
};

// ---------- Tasks ----------
void TaskTemperature_Humidity(void *pvParameters) {
  while (1) {
    dht20.read();
    double temp = dht20.getTemperature();
    double humi = dht20.getHumidity();

    Serial.printf("Temp: %.2f *C, Humidity: %.2f %%\n", temp, humi);

    SensorData data = { temp, humi };
    xQueueSend(sensorQueue, &data, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void TaskSendData(void *pvParameters) {
  while (1) {
    // WiFi Connect
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      Serial.print("Connecting to WiFi");
      while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        vTaskDelay(pdMS_TO_TICKS(500));
      }
      Serial.println("\nWiFi connected");
    }

    // ThingsBoard Connect
    if (!tb.connected()) {
      Serial.println("Connecting to ThingsBoard...");
      if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
        Serial.println("Failed to connect to ThingsBoard!");
        vTaskDelay(pdMS_TO_TICKS(3000));
        continue;
      }
      Serial.println("Connected to ThingsBoard");
    }

    SensorData data;
    if (xQueueReceive(sensorQueue, &data, pdMS_TO_TICKS(1000)) == pdTRUE) {
      tb.sendTelemetryData("temperature", data.temperature);
      tb.sendTelemetryData("humidity", data.humidity);
      Serial.printf("Sent -> Temp: %.2f°C, Humi: %.2f%%\n", data.temperature, data.humidity);
    }

    tb.loop(); // Keep MQTT alive
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ---------- Setup & Loop ----------
void setup() {
  Serial.begin(115200);
  Wire.begin(11, 12); // SDA = GPIO8, SCL = GPIO9 (hoặc thay đổi nếu khác)
  dht20.begin();

  sensorQueue = xQueueCreate(5, sizeof(SensorData));
  xTaskCreatePinnedToCore(TaskTemperature_Humidity, "ReadTempHumi", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskSendData, "SendData", 4096, NULL, 1, NULL, 1);
}

void loop() {
  
}

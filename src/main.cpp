#define CONFIG_THINGSBOARD_ENABLE_DEBUG false
#include <Arduino.h>
#include <WiFiClient.h>
#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <DHT20.h>
#include <OTA_Firmware_Update.h>
#include <ThingsBoard.h>
#include <Shared_Attribute_Update.h>
#include <Attribute_Request.h>
#include <Espressif_Updater.h>

#define LED_PIN 48

#include "Server_Side_RPC.h"
//Shared Attributes Configuration
constexpr uint8_t MAX_ATTRIBUTES = 2U; //
constexpr std::array<const char*, MAX_ATTRIBUTES> 

SHARED_ATTRIBUTES = 
{
  "POWER",
  "ledState"
};

constexpr int16_t TELEMETRY_SEND_INTERVAL = 5000U;
uint32_t previousTelemetrySend; 
constexpr char CURRENT_FIRMWARE_TITLE[] = "ESP32_OTA";
constexpr char CURRENT_FIRMWARE_VERSION[] = "1.1";
// Maximum amount of retries we attempt to download each firmware chunck over MQTT
constexpr uint8_t FIRMWARE_FAILURE_RETRIES = 12U;

constexpr uint16_t FIRMWARE_PACKET_SIZE = 4096U;

constexpr char WIFI_SSID[] = "ACLAB";
constexpr char WIFI_PASSWORD[] = "ACLAB2023";
constexpr char TOKEN[] = "UPjAHGLnXtOwHomdjTJL";
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";

constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 512U;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 512U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;
constexpr uint64_t REQUEST_TIMEOUT_MICROSECONDS = 10000U * 1000U;
void requestTimedOut() {
  Serial.printf("Attribute request timed out did not receive a response in (%llu) microseconds. Ensure client is connected to the MQTT broker and that the keys actually exist on the target device\n", REQUEST_TIMEOUT_MICROSECONDS);
}
// Initialize underlying client, used to establish a connection
WiFiClient espClient;
// Initalize the Mqtt client instance
Arduino_MQTT_Client mqttClient(espClient);
// Initialize used apis
OTA_Firmware_Update<> ota;
Shared_Attribute_Update<1U, MAX_ATTRIBUTES> shared_update;
Attribute_Request<2U, MAX_ATTRIBUTES> attr_request;
const std::array<IAPI_Implementation*, 3U> apis = {
    &shared_update,
    &attr_request,
    &ota
};
// Initialize ThingsBoard instance with the maximum needed buffer size
ThingsBoard tb(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE, Default_Max_Stack_Size, apis);
// Initalize the Updater client instance used to flash binary to flash memory
Espressif_Updater<> updater;
// Statuses for updating
bool shared_update_subscribed = false;
bool currentFWSent = false;
bool updateRequestSent = false;
bool requestedShared = false;

DHT20 dht20;

void InitWiFi() {
  Serial.println("Connecting to AP ...");
  // Attempting to establish a connection to the given WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    // Delay 500ms until a connection has been successfully established
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}
bool reconnect() {
  // Check to ensure we aren't connected yet
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    return true;
  }

  // If we aren't establish a new connection to the given WiFi network
  InitWiFi();
  return true;
}
void update_starting_callback() {
}
void finished_callback(const bool & success) {
  if (success) {
    Serial.println("Done, Reboot now");
    esp_restart();
    return;
  }
  Serial.println("Downloading firmware failed");
}
void progress_callback(const size_t & current, const size_t & total) {
  Serial.printf("Progress %.2f%%\n", static_cast<float>(current * 100U) / total);
}
void processSharedAttributeUpdate(const JsonObjectConst &data) {
  //Info
  const size_t jsonSize = Helper::Measure_Json(data);
  char buffer[jsonSize];
  serializeJson(data, buffer, jsonSize);
  Serial.println(buffer);
}
void setup() {
  // Initalize serial connection for debugging
  Serial.begin(SERIAL_DEBUG_BAUD);
  delay(1000);
  InitWiFi();

  pinMode(LED_PIN, OUTPUT);
}
void processSharedAttributeRequest(const JsonObjectConst &data) {
  //Info
  const size_t jsonSize = Helper::Measure_Json(data);
  char buffer[jsonSize];
  serializeJson(data, buffer, jsonSize);
  Serial.println(buffer);
}
void loop() {
  delay(1000);
  if (!reconnect()) {
    return;
  }
  if (!tb.connected()) {
    Serial.printf("Connecting to: (%s) with token (%s)\n", THINGSBOARD_SERVER, TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println("Failed to connect");
      return;
    }
    if (!requestedShared) {
      Serial.println("Requesting shared attributes...");
      const Attribute_Request_Callback<MAX_ATTRIBUTES> sharedCallback(&processSharedAttributeRequest, REQUEST_TIMEOUT_MICROSECONDS, &requestTimedOut, SHARED_ATTRIBUTES);
      requestedShared = attr_request.Shared_Attributes_Request(sharedCallback);
      if (!requestedShared) {
        Serial.println("Failed to request shared attributes");
      }
    }

  if (!shared_update_subscribed){
      Serial.println("Subscribing for shared attribute updates...");
      const Shared_Attribute_Callback<MAX_ATTRIBUTES> callback(&processSharedAttributeUpdate, SHARED_ATTRIBUTES);
      if (!shared_update.Shared_Attributes_Subscribe(callback)) {
      Serial.println("Failed to subscribe for shared attribute updates");
      // continue;
      }
      Serial.println("Subscribe done");
      shared_update_subscribed = true;
    }
  }  
  if (!currentFWSent) {
    currentFWSent = ota.Firmware_Send_Info(CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION);
  }
  if (!updateRequestSent) {
    Serial.print(CURRENT_FIRMWARE_TITLE);
    Serial.println(CURRENT_FIRMWARE_VERSION);
    Serial.println("Firwmare Update ...");
    const OTA_Update_Callback callback(CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION, &updater, &finished_callback, &progress_callback, &update_starting_callback, FIRMWARE_FAILURE_RETRIES, FIRMWARE_PACKET_SIZE);
    updateRequestSent = ota.Start_Firmware_Update(callback);
    if(updateRequestSent) {
      delay(500);
      Serial.println("Firwmare Update Subscription...");
      updateRequestSent = ota.Subscribe_Firmware_Update(callback);
    }
  }
  if (millis() - previousTelemetrySend > TELEMETRY_SEND_INTERVAL)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }
  tb.loop();
}
#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#ifdef ESP32
#include <WiFi.h>
#include <WiFiClientSecure.h>
#endif // ESP32
#endif // ESP8266
#include <Arduino.h>
#include <Arduino_MQTT_Client.h>
#include <Server_Side_RPC.h>
#include <OTA_Firmware_Update.h>
#include <ThingsBoard.h>
#include <Espressif_Updater.h>
#include <DHT.h>
#include <esp_task_wdt.h>

#define DHTPIN 15
#define DHTTYPE DHT11
#define LED_PIN GPIO_NUM_2

#define ENCRYPTED false
constexpr char WIFI_SSID[] = "Minhkhoa";
constexpr char WIFI_PASSWORD[] = "minhkhoa2022";
constexpr char TOKEN[] = "TKWueK8HQ6KJubYkuNWs";
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 256U;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 256U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

// OTA Config
constexpr char CURRENT_FIRMWARE_TITLE[] = "OTA";
constexpr char CURRENT_FIRMWARE_VERSION[] = "1.0";
constexpr uint8_t FIRMWARE_FAILURE_RETRIES = 12U;
constexpr uint16_t FIRMWARE_PACKET_SIZE = 512U; // Giảm để tránh timeout

constexpr char BLINKING_INTERVAL_ATTR[] = "blinkingInterval";
constexpr char LED_MODE_ATTR[] = "ledMode";
constexpr char LED_STATE_ATTR[] = "ledState";
constexpr const char RPC_JSON_METHOD[] = "example_json";
constexpr const char RPC_TEMPERATURE_METHOD[] = "example_set_temperature";
constexpr const char RPC_SWITCH_METHOD[] = "setLedState";
constexpr const char RPC_TEMPERATURE_KEY[] = "temperature";
constexpr uint8_t MAX_RPC_SUBSCRIPTIONS = 3U;
constexpr uint8_t MAX_RPC_RESPONSE = 5U;

volatile bool attributesChanged = false;
volatile int ledMode = 0;
volatile bool ledState = false;
bool subscribed = false;

// Khởi tạo client
WiFiClient espClient;
Arduino_MQTT_Client mqttClient(espClient);
Server_Side_RPC<MAX_RPC_SUBSCRIPTIONS, MAX_RPC_RESPONSE> rpc;
OTA_Firmware_Update<> ota;
const std::array<IAPI_Implementation*, 2U> apis = { &rpc, &ota };
ThingsBoard tb(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE, Default_Max_Stack_Size, apis);
DHT dht(DHTPIN, DHTTYPE);
Espressif_Updater<> updater;

// OTA Status
bool currentFWSent = false;
bool updateRequestSent = false;

void InitWiFi() {
  Serial.println("Connecting to AP ...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(WiFi.localIP());
  Serial.print("WiFi RSSI: ");
  Serial.println(WiFi.RSSI());
  Serial.println("Connected to AP");
}

bool reconnect() {
  if (WiFi.status() == WL_CONNECTED && WiFi.RSSI() > -70) {
    return true;
  }
  InitWiFi();
  return true;
}

void processGetJson(const JsonVariantConst &data, JsonDocument &response) {
  Serial.println("Received the json RPC method");
  StaticJsonDocument<JSON_OBJECT_SIZE(4)> innerDoc;
  innerDoc["string"] = "exampleResponseString";
  innerDoc["int"] = 5;
  innerDoc["float"] = 5.0f;
  innerDoc["bool"] = true;
  response["json_data"] = innerDoc;
}

void processTemperatureChange(const JsonVariantConst &data, JsonDocument &response) {
  Serial.println("Received the set temperature RPC method");
  const float example_temperature = data[RPC_TEMPERATURE_KEY];
  Serial.print("Example temperature: ");
  Serial.println(example_temperature);
  response["string"] = "exampleResponseString";
  response["int"] = 5;
  response["float"] = 5.0f;
  response["double"] = 10.0;
  response["bool"] = true;
}

void processSwitchChange(const JsonVariantConst &data, JsonDocument &response) {
  Serial.println("Received RPC: setLedState");
  serializeJson(data, Serial);
  Serial.println();
  if (!data.isNull()) {
    bool newState = data.as<bool>();
    ledState = newState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    Serial.print("LED state changed to: ");
    Serial.println(ledState ? "ON" : "OFF");
  } else {
    Serial.println("Error: Invalid RPC data");
  }
}

void onSharedAttributeReceived(const JsonVariantConst &data) {
  Serial.println("Received shared attributes:");
  serializeJson(data, Serial);
  Serial.println();
  if (data.containsKey("fw_title")) {
    Serial.print("fw_title: ");
    Serial.println(data["fw_title"].as<String>());
  }
  if (data.containsKey("fw_version")) {
    Serial.print("fw_version: ");
    Serial.println(data["fw_version"].as<String>());
  }
}

void update_starting_callback() {
  Serial.println("Bắt đầu cập nhật OTA...");
  esp_task_wdt_init(30, true); // Tăng timeout WDT
}

void finished_callback(const bool &success) {
  esp_task_wdt_init(5, true); // Khôi phục WDT
  if (success) {
    Serial.println("Cập nhật thành công, khởi động lại...");
    esp_restart();
  } else {
    Serial.println("Cập nhật thất bại");
  }
}

void progress_callback(const size_t current, const size_t total) {
  Serial.printf("Tiến độ: %.2f%%\n", static_cast<float>(current * 100U) / total);
}

void controlLed(void *pvParameters) {
  pinMode(LED_PIN, OUTPUT);
  while (1) {
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    ledState != ledState;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void checkConnect(void *pvParameters) {
  while (1) {
    if (!reconnect()) {
      vTaskDelay(30000 / portTICK_PERIOD_MS);
      continue;
    }
    vTaskDelay(30000 / portTICK_PERIOD_MS);
  }
}

void tbConnect(void *pvParameters) {
  while (1) {
    if (!tb.connected()) {
      Serial.printf("Connecting to: (%s) with token (%s)\n", THINGSBOARD_SERVER, TOKEN);
      if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
        Serial.println("Failed to connect");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        continue;
      }
    }
    tb.loop(); // Chỉ gọi tb.loop() ở đây
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void tbSubscribed(void *pvParameters) {
  while (1) {
    if (!subscribed && tb.connected()) {
      Serial.println("Subscribing for RPC and attributes...");
      const std::array<RPC_Callback, MAX_RPC_SUBSCRIPTIONS> callbacks = {
        RPC_Callback{ RPC_JSON_METHOD, processGetJson },
        RPC_Callback{ RPC_TEMPERATURE_METHOD, processTemperatureChange },
        RPC_Callback{ RPC_SWITCH_METHOD, processSwitchChange }
      };
      if (!rpc.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
        Serial.println("Failed to subscribe for RPC");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        continue;
      }
      Serial.println("Subscribe done");
      subscribed = true;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Nhường CPU
  }
}

void sendData(void *pvParameters) {
  while (1) {
    dht.read();
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Failed to read from DHT sensor!");
      vTaskDelay(10000 / portTICK_PERIOD_MS);
      continue;
    }
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print("°C, Humidity: ");
    Serial.print(humidity);
    Serial.println("%");
    tb.sendTelemetryData("temperature", temperature);
    tb.sendTelemetryData("humidity", humidity);
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

void send_gas(void *pvParameters) {
  while (1) {
    int gas_value = analogRead(35);
    float gasPercent = map(gas_value, 4096, 0, 0, 100);
    Serial.print("GAS: ");
    Serial.println(gasPercent);
    tb.sendTelemetryData("Gas", gasPercent);
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

void otaUpdate(void *pvParameters) {
  while (1) {
    if (!tb.connected()) {
      Serial.println("OTA: Waiting for connection");
      vTaskDelay(5000 / portTICK_PERIOD_MS);
      continue;
    }
    Serial.printf("OTA: Free heap: %d bytes\n", ESP.getFreeHeap());
    if (!currentFWSent) {
      Serial.println("OTA: Sending firmware info...");
      currentFWSent = ota.Firmware_Send_Info(CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION);
      if (currentFWSent) {
        Serial.println("OTA: Sent firmware info");
      } else {
        Serial.println("OTA: Failed to send firmware info");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        continue;
      }
    }
    if (!updateRequestSent && currentFWSent && ESP.getFreeHeap() > 50000 && WiFi.RSSI() > -70) {
      Serial.println("OTA: Requesting update...");
      const OTA_Update_Callback callback(
        CURRENT_FIRMWARE_TITLE,
        CURRENT_FIRMWARE_VERSION,
        &updater,
        &finished_callback,
        &progress_callback,
        &update_starting_callback,
        FIRMWARE_FAILURE_RETRIES,
        FIRMWARE_PACKET_SIZE
      );
      updateRequestSent = ota.Start_Firmware_Update(callback);
      if (updateRequestSent) {
        Serial.println("OTA: Update requested");
      } else {
        Serial.println("OTA: Failed to request update");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        continue;
      }
    }
    vTaskDelay(60000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  pinMode(GPIO_NUM_2, OUTPUT);
  pinMode(15, INPUT);
  pinMode(35, INPUT);
  dht.begin();
  delay(2000); // Chờ DHT ổn định
  InitWiFi();
  esp_task_wdt_init(5, true); // Khởi tạo WDT
  xTaskCreate(checkConnect, "reconnect", 4096, NULL, 2, NULL);
  xTaskCreate(tbConnect, "tbConnect", 4096, NULL, 2, NULL);
  xTaskCreate(tbSubscribed, "tbSubscribed", 12288, NULL, 2, NULL); // Tăng stack
  xTaskCreate(sendData, "sendData", 4096, NULL, 2, NULL);
  //xTaskCreate(send_gas, "send_gas", 4096, NULL, 2, NULL);
  //xTaskCreate(controlLed, "controlLed", 4096, NULL, 2, NULL);
  xTaskCreate(otaUpdate, "otaUpdate", 12288, NULL, 2, NULL); // Tăng stack
}

void loop() {
}
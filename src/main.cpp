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
#include <ThingsBoard.h>
#include "DHT.h"
#define DHTPIN  15
#define DHTTYPE DHT11

// Whether the given script is using encryption or not,
// generally recommended as it increases security (communication with the server is not in clear text anymore),
// it does come with an overhead tough as having an encrypted session requires a lot of memory,
// which might not be avaialable on lower end devices.
#define ENCRYPTED false


constexpr char WIFI_SSID[] ="giangphaihocbai";
constexpr char WIFI_PASSWORD[] = "123456789";

// See https://thingsboard.io/docs/getting-started-guides/helloworld/
// to understand how to obtain an access token
constexpr char TOKEN[] = "";

// Thingsboard we want to establish a connection too
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";

// MQTT port used to communicate with the server, 1883 is the default unencrypted MQTT port,
// whereas 8883 would be the default encrypted SSL MQTT port
#if ENCRYPTED
constexpr uint16_t THINGSBOARD_PORT = 8883U;
#else
constexpr uint16_t THINGSBOARD_PORT = 1883U;
#endif

// Maximum size packets will ever be sent or received by the underlying MQTT client,
// if the size is to small messages might not be sent or received messages will be discarded
constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 256U;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 256U;

// Baud rate for the debugging serial connection.
// If the Serial output is mangled, ensure to change the monitor speed accordingly to this variable
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

constexpr char BLINKING_INTERVAL_ATTR[] = "blinkingInterval";
constexpr char LED_MODE_ATTR[] = "ledMode";
constexpr char LED_STATE_ATTR[] = "ledState";

volatile bool attributesChanged = false;
volatile int ledMode = 0;
volatile bool ledState = false;

#if ENCRYPTED
// See https://comodosslstore.com/resources/what-is-a-root-ca-certificate-and-how-do-i-download-it/
// on how to get the root certificate of the server we want to communicate with,
// this is needed to establish a secure connection and changes depending on the website.
constexpr char ROOT_CERT[] = R"(-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)";
#endif

constexpr const char RPC_JSON_METHOD[] = "example_json";
constexpr const char RPC_TEMPERATURE_METHOD[] = "example_set_temperature";
constexpr const char RPC_SWITCH_METHOD[] = "getValueButton";
constexpr const char RPC_TEMPERATURE_KEY[] = "temperature";
constexpr const char RPC_SWITCH_KEY[] = "switch";
constexpr uint8_t MAX_RPC_SUBSCRIPTIONS = 3U;
constexpr uint8_t MAX_RPC_RESPONSE = 5U;


// Initialize underlying client, used to establish a connection
#if ENCRYPTED
WiFiClientSecure espClient;
#else
WiFiClient espClient;
#endif
// Initalize the Mqtt client instance
Arduino_MQTT_Client mqttClient(espClient);
// Initialize used apis
Server_Side_RPC<MAX_RPC_SUBSCRIPTIONS, MAX_RPC_RESPONSE> rpc;
const std::array<IAPI_Implementation*, 1U> apis = {
    &rpc
};
// Initialize ThingsBoard instance with the maximum needed buffer size
ThingsBoard tb(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE, Default_Max_Stack_Size, apis);

//const Shared_Attribute_Callback attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
//const Attribute_Request_Callback attribute_shared_request_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
//DHT11
DHT dht(DHTPIN, DHTTYPE);

// Statuses for subscribing to rpc
bool subscribed = false;


/// @brief Initalizes WiFi connection,
// will endlessly delay until a connection has been successfully established
void InitWiFi() {
  Serial.println("Connecting to AP ...");
  // Attempting to establish a connection to the given WiFi network
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    // Delay 500ms until a connection has been successfully established
    delay(500);
    Serial.print(".");
  }
  Serial.println(WiFi.localIP());
  Serial.println("Connected to AP");
#if ENCRYPTED
  espClient.setCACert(ROOT_CERT);
#endif
}

/// @brief Reconnects the WiFi uses InitWiFi if the connection has been removed
/// @return Returns true as soon as a connection has been established again
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

/// @brief Processes function for RPC call "example_json"
/// JsonVariantConst is a JSON variant, that can be queried using operator[]
/// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
/// @param data Data containing the rpc data that was called and its current value
/// @param response Data containgin the response value, any number, string or json, that should be sent to the cloud. Useful for getMethods
void processGetJson(const JsonVariantConst &data, JsonDocument &response) {
  Serial.println("Received the json RPC method");

  // Size of the response document needs to be configured to the size of the innerDoc + 1.
  StaticJsonDocument<JSON_OBJECT_SIZE(4)> innerDoc;
  innerDoc["string"] = "exampleResponseString";
  innerDoc["int"] = 5;
  innerDoc["float"] = 5.0f;
  innerDoc["bool"] = true;
  response["json_data"] = innerDoc;
}

/// @brief Processes function for RPC call "example_set_temperature"
/// JsonVariantConst is a JSON variant, that can be queried using operator[]
/// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
/// @param data Data containing the rpc data that was called and its current value
/// @param response Data containgin the response value, any number, string or json, that should be sent to the cloud. Useful for getMethods
void processTemperatureChange(const JsonVariantConst &data, JsonDocument &response) {
  Serial.println("Received the set temperature RPC method");

  // Process data
  const float example_temperature = data[RPC_TEMPERATURE_KEY];

  Serial.print("Example temperature: ");
  Serial.println(example_temperature);

  // Ensure to only pass values do not store by copy, or if they do increase the MaxRPC template parameter accordingly to ensure that the value can be deserialized.RPC_Callback.
  // See https://arduinojson.org/v6/api/jsondocument/add/ for more information on which variables cause a copy to be created
  response["string"] = "exampleResponseString";
  response["int"] = 5;
  response["float"] = 5.0f;
  response["double"] = 10.0;
  response["bool"] = true;
}

/// @brief Processes function for RPC call "example_set_switch"
/// JsonVariantConst is a JSON variant, that can be queried using operator[]
/// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
/// @param data Data containing the rpc data that was called and its current value
/// @param response Data containgin the response value, any number, string or json, that should be sent to the cloud. Useful for getMethods
void processSwitchChange(const JsonVariantConst &data, JsonDocument &response) {
  Serial.println("Received the set switch method");

 // Đọc trạng thái switch từ dữ liệu RPC
  bool switch_state = data[RPC_SWITCH_KEY];

  Serial.print("Switch state: ");
  Serial.println(switch_state);

  // Điều khiển đèn LED dựa trên switch_state
 
  digitalWrite(GPIO_NUM_2, switch_state ? HIGH : LOW);

    // Phản hồi về trạng thái LED hiện tại
    response["switch"] = switch_state;
    Serial.println("aaaaaaaaaaaaaa");
}





void blink_led(void *pvParameters){
  pinMode(GPIO_NUM_2, OUTPUT);
  int ledState = 0;
  while(1){
    if(ledState == 0){
      digitalWrite(GPIO_NUM_2, HIGH);
    }
    else{
      digitalWrite(GPIO_NUM_2, LOW);
    }
    ledState = 1 - ledState;
    vTaskDelay(1000);
  }
}

void checkConnect(void *pvParameters){
  while(1){
    if (!reconnect()) {
      return;
    }
    vTaskDelay(30000);
  }
}

void tbConnect(void *pvParameters){
  while(1){
    tb.loop();
    if (!tb.connected()) {
      // Reconnect to the ThingsBoard server,
      // if a connection was disrupted or has not yet been established
      Serial.printf("Connecting to: (%s) with token (%s)\n", THINGSBOARD_SERVER, TOKEN);
      if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
        Serial.println("Failed to connect");
        return;
      }
    }
    vTaskDelay(100);
  }
}

void tbSubscribed(void *pvParameters){
  while (1){
    tb.loop();
    if (!subscribed) {
      Serial.println("Subscribing for RPC...");
      const std::array<RPC_Callback, MAX_RPC_SUBSCRIPTIONS> callbacks = {
        // Requires additional memory in the JsonDocument for the JsonDocument that will be copied into the response
        RPC_Callback{ RPC_JSON_METHOD,           processGetJson },
        // Requires additional memory in the JsonDocument for 5 key-value pairs that do not copy their value into the JsonDocument itself
        RPC_Callback{ RPC_TEMPERATURE_METHOD,    processTemperatureChange },
        // Internal size can be 0, because if we use the JsonDocument as a JsonVariant and then set the value we do not require additional memory
        RPC_Callback{ RPC_SWITCH_METHOD,         processSwitchChange }

      };
      
      // Perform a subscription. All consequent data processing will happen in
      // processTemperatureChange() and processSwitchChange() functions,
      // as denoted by callbacks array.
      if (!rpc.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
        Serial.println("Failed to subscribe for RPC");
        return;
      }
      
      Serial.println("Subscribe done");
      subscribed = true;
    }
    vTaskDelay(10);
  }
}

void sendData(void *pvParameters) {
  while(1){
    dht.read();
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
  
    if (isnan(humidity) || isnan(temperature)) {
        Serial.println("Failed to read from DHT sensor!");
        return;
    }
  
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print("°C, Humidity: ");
    Serial.print(humidity);
    Serial.println("%");
  
     tb.sendTelemetryData("temperature", temperature);
     tb.sendTelemetryData("humidity", humidity);
    vTaskDelay(5000);
  }
}

void send_gas(void *pvParameters){
  while(1){
    int gas_value = analogRead(35);
    float gasPercent = map(gas_value, 4096, 0, 0, 100);
    Serial.print("GAS: ");
    Serial.println(gasPercent);
    tb.sendTelemetryData("Gas", gasPercent);
    vTaskDelay(5000);
  }

}

void setup() {
  // Initalize serial connection for debugging
  Serial.begin(SERIAL_DEBUG_BAUD);
  pinMode(GPIO_NUM_2, OUTPUT);
  pinMode(15, INPUT);
  pinMode(35, INPUT);
  delay(1000);
  
  InitWiFi();
  xTaskCreate(blink_led, "led blink", 2048, NULL, 2, NULL);
  xTaskCreate(checkConnect, "reconnect", 2048, NULL, 2, NULL);
  xTaskCreate(tbConnect, "tbConnect", 2048, NULL, 2, NULL);
  xTaskCreate(tbSubscribed, "tbSubscribed", 2048, NULL, 2, NULL);
  xTaskCreate(sendData, "sendData", 2048, NULL, 2, NULL);
  //xTaskCreate(send_gas, "send_gas", 2048, NULL, 2, NULL);
}



void loop() {
}




// #include <Arduino.h>

// void setup(){
//   Serial.begin(115200);
// }
// void loop(){
//   Serial.print("hello");
//   delay(2000);
// }
// ------ Libraries ----------------

#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>



// ----------------------------- CONFIG ---------------------------------
#define RGB_PIN 5  // ESP32-C3: make sure this is a free GPIO
#define DHT_PIN 6
#define DHTTYPE DHT11
#define NEOPIXEL_TYPE (NEO_GRB + NEO_KHZ800)

#define ALARM_COLD 0.0
#define ALARM_HOT 30.0
#define WARN_COLD 10.0
#define WARN_HOT 25.0

// WiFi
const char* ssid = "devolo-875";
const char* pass = "FWHMGSHUNQKBJKLW";
//const char* ssid = "iPhone";
//const char* pass = "lukecheese";

// MQTT connection details
#define MQTT_HOST "broker.mqtt.cool"
#define MQTT_PORT 1883
#define MQTT_DEVICEID "lukeDHT6642"
#define MQTT_USER ""   // no need for authentication, for now
#define MQTT_TOKEN ""  // no need for authentication, for now
#define MQTT_TOPIC "lukeDHT6642/evt/status/fmt/json"
#define MQTT_TOPIC_DISPLAY "lukeDHT6642/cmd/display/fmt/json"

// MQTT objects
void callback(char* topic, byte* payload, unsigned int length);
WiFiClient wifiClient;
PubSubClient mqtt(MQTT_HOST, MQTT_PORT, callback, wifiClient);

// ----------------------------- GLOBALS --------------------------------
Adafruit_NeoPixel pixel(1, RGB_PIN, NEOPIXEL_TYPE);
DHT dht(DHT_PIN, DHTTYPE);

// Roomier JSON buffer; roomier char buffer
StaticJsonDocument<256> jsonDoc;
JsonObject payload = jsonDoc.to<JsonObject>();
JsonObject status = payload.createNestedObject("d");
static char msg[160];

float h = NAN;
float t = NAN;

// ----------------------------- HELPERS --------------------------------
void logln(const String& s) {
  // Use UART0 (same port you used in Option A)
  // C3 UART0: RX=GPIO20, TX=GPIO21
  Serial0.println(s);
}

// Print reasons if WiFi fails
void printWiFiStatus(){
  wl_status_t st = WiFi.status();
  Serial0.print("WiFi status: ");
  Serial0.println((int)st);
}

void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] : ");

  payload[length] = 0;  // ensure valid content is zero terminated so can treat as c-string
  Serial.println((char*)payload);
}

void setup() {
  // ---- Serial on UART0 (keep using the same cable/port you already opened)
  Serial0.begin(115200, SERIAL_8N1, 20, 21);
  delay(1000);
  logln("");
  logln("ESP32-C3 Sensor Application");

  // ---- WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  logln("Connecting to WiFi...");
  unsigned long start = millis();
  const unsigned long WIFI_TIMEOUT = 20000;  // 20s
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_TIMEOUT) {
    delay(500);
    Serial0.print(".");
  }
  Serial0.println("");

  if (WiFi.status() == WL_CONNECTED) {
    logln("WiFi connected: " + WiFi.localIP().toString());
  } else {
    logln("WiFi connect FAILED (continuing without network)");
  }

  // ---- Sensors / LED
  dht.begin();
  pixel.begin();
  pixel.setBrightness(40);               // tame brightness for a single LED
  pixel.show();                          // initialize to 'off'
  mqtt.setServer(MQTT_HOST, MQTT_PORT);  // ensure server is set

  // Connect to MQTT broker
  if (mqtt.connect(MQTT_DEVICEID, MQTT_USER, MQTT_TOKEN)) {
    Serial.println("MQTT Connected");
    mqtt.subscribe(MQTT_TOPIC_DISPLAY);
  } else {
    Serial.println("MQTT Failed to connect!");
    ESP.restart();
  }
}

void setLed(float tempC) {
  // Compute colors
  uint8_t b = (tempC < ALARM_COLD) ? 255 : ((tempC < WARN_COLD) ? 150 : 0);
  uint8_t r = (tempC >= ALARM_HOT) ? 255 : ((tempC > WARN_HOT) ? 150 : 0);
  uint8_t g = (tempC > ALARM_COLD) ? ((tempC <= WARN_HOT) ? 255 : ((tempC < ALARM_HOT) ? 150 : 0)) : 0;
  pixel.setPixelColor(0, r, g, b);
  pixel.show();
}

void loop() {

  mqtt.loop();
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt.connect(MQTT_DEVICEID, MQTT_USER, MQTT_TOKEN)) {
      Serial.println("MQTT Connected");
      mqtt.subscribe(MQTT_TOPIC_DISPLAY);
      mqtt.loop();
    } else {
      Serial.println("MQTT Failed to connect!");
      delay(5000);
    }
  }

  // DHT needs ~1-2s between reads; we wait 10s below
  h = dht.readHumidity();
  t = dht.readTemperature();  // Celsius

  if (isnan(h) || isnan(t)) {
    logln("Failed to read from DHT sensor!");
  } else {
    setLed(t);

    // Build JSON and print
    status["temp"] = t;
    status["humidity"] = h;
    size_t n = serializeJson(jsonDoc, msg, sizeof(msg));
    if (n == 0) {
      logln("JSON serialize overflow; increase buffer sizes.");
    } else {
      Serial0.println(msg);
      // Extra logging: show exactly what we publish and where
      Serial0.print("Publishing to ");
      Serial0.print(MQTT_TOPIC);
      Serial0.print(": ");
      Serial0.println(msg);
      Serial.print("Publishing to ");
      Serial.print(MQTT_TOPIC);
      Serial.print(": ");
      Serial.println(msg);

      if (!mqtt.publish(MQTT_TOPIC, msg)) {
        Serial.println("MQTT Publish failed");
      }
    }
  }

  // Pause - but keep polling MQTT for incoming messages
  for (int i = 0; i < 10; i++) {
    mqtt.loop();
    delay(1000);
  }
}

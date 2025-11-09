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
#define MQTT_TOPIC_INTERVAL "iot-2/type/ESP8266/id/dev01/cmd/interval/fmt/json"
#define MQTT_TOPIC_DISPLAY "lukeDHT6642/evt/status/fmt/json"

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

// reporting interval
volatile uint16_t g_publishInterval = 10;
// track last publish time so interval changes take effect immediately
volatile unsigned long g_lastPublishMs = 0;

// temp and humidity vars
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
  // Detailed logging of incoming MQTT message
  Serial0.print("Message arrived [");
  Serial0.print(topic);
  Serial0.print("] len=");
  Serial0.print(length);
  Serial0.print(" : ");

  // Copy payload to a safe, null-terminated buffer (avoid writing past provided memory)
  char buf[128];
  size_t copyLen = (length < sizeof(buf) - 1) ? length : (sizeof(buf) - 1);
  memcpy(buf, payload, copyLen);
  buf[copyLen] = '\0';
  if (copyLen != length) {
    Serial0.print("(truncated) ");
  }
  Serial0.println(buf);

  // Handle interval command topic
  if (strcmp(topic, MQTT_TOPIC_INTERVAL) == 0) {
    StaticJsonDocument<64> doc;
    DeserializationError err = deserializeJson(doc, buf);
    if (!err) {
      uint16_t v = g_publishInterval;
      if (doc.containsKey("Interval")) {
        v = doc["Interval"].as<uint16_t>();
      } else if (doc.containsKey("interval")) {
        v = doc["interval"].as<uint16_t>();
      } else if (doc.is<uint16_t>()) {
        // support bare numeric payload like: 5
        v = doc.as<uint16_t>();
      }
      // Clamp to sane bounds (2s .. 3600s)
      if (v < 2) v = 2;
      if (v > 3600) v = 3600;
      if (v != g_publishInterval) {
        g_publishInterval = v;
        // Apply starting now: reset last publish timestamp so the next send waits the new interval
        g_lastPublishMs = millis();
        Serial0.print("Updated publish interval to: ");
        Serial0.print(g_publishInterval);
        Serial0.println("s (takes effect now)");
      } else {
        Serial0.print("Interval unchanged (still ");
        Serial0.print(g_publishInterval);
        Serial0.println("s)");
      }
    } else {
      Serial0.print("Interval JSON error: ");
      Serial0.println(err.c_str());
    }
  }
}

void setup() {
  // ---- Serial0 on UART0 (keep using the same cable/port you already opened)
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
    Serial0.println("MQTT Connected");
    if (mqtt.subscribe(MQTT_TOPIC_DISPLAY)) {
      Serial0.print("Subscribed to status topic: ");
      Serial0.println(MQTT_TOPIC_DISPLAY);
    } else {
      Serial0.println("Failed to subscribe to status topic");
    }
    if (mqtt.subscribe(MQTT_TOPIC_INTERVAL)) {
      Serial0.print("Subscribed to interval topic: ");
      Serial0.println(MQTT_TOPIC_INTERVAL);
    } else {
      Serial0.println("Failed to subscribe to interval topic");
    }
  } else {
    Serial0.println("MQTT Failed to connect!");
    ESP.restart();
  }

  Serial0.print("Default publish interval: ");
  Serial0.print(g_publishInterval);
  Serial0.println("s");
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
  // use global last publish time so callback can reset schedule immediately
  mqtt.loop();

  // Reconnect if needed (non-blocking quick attempt)
  if (!mqtt.connected()) {
    Serial0.print("Attempting MQTT reconnect...");
    if (mqtt.connect(MQTT_DEVICEID, MQTT_USER, MQTT_TOKEN)) {
      Serial0.println("reconnected");
      mqtt.subscribe(MQTT_TOPIC_DISPLAY);
      mqtt.subscribe(MQTT_TOPIC_INTERVAL);
    } else {
      Serial0.println("failed");
    }
  }

  // Publish according to dynamic interval
  unsigned long now = millis();
  if (now - g_lastPublishMs >= (unsigned long)g_publishInterval * 1000UL) {
    g_lastPublishMs = now;

    h = dht.readHumidity();
    t = dht.readTemperature(); // Celsius

    if (isnan(h) || isnan(t)) {
      logln("Failed to read from DHT sensor!");
    } else {
      setLed(t);
      status["temp"] = t;
      status["humidity"] = h;
      size_t n = serializeJson(jsonDoc, msg, sizeof(msg));
      if (n == 0) {
        logln("JSON serialize overflow; increase buffer sizes.");
      } else {
        Serial0.print("Publishing to ");
        Serial0.print(MQTT_TOPIC);
        Serial0.print(": ");
        Serial0.println(msg);
        if (!mqtt.publish(MQTT_TOPIC, msg)) {
          Serial0.println("MQTT Publish failed");
        }
      }
    }
  }

  // Small delay to avoid a tight loop; still responsive to MQTT
  delay(50);
}

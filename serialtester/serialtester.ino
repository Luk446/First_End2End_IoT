// UART0: RX=GPIO20, TX=GPIO21 on ESP32-C3
void setup() {
  Serial0.begin(115200, SERIAL_8N1, 20, 21);
  delay(2000);                      // give the monitor time to attach
  Serial0.println("Booted on UART0");
}
#include <DHT.h>

#define DHTPIN 4        // GPIO 4 on ESP32-C3-Mini
#define DHTTYPE DHT11   // DHT 11

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nDHT11 test on ESP32-C3-Mini (GPIO 4)");
  dht.begin();
}

void loop() {
  // DHT11 can be read at most ~1 Hz; sample every 2s for safety
  float humidity = dht.readHumidity();
  float tempC    = dht.readTemperature();       // Celsius
  float tempF    = dht.readTemperature(true);   // Fahrenheit

  if (isnan(humidity) || isnan(tempC)) {
    Serial.println("Failed to read from DHT11 sensor. Check wiring/pull-up.");
  } else {
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print(" %\t");

    Serial.print("Temp: ");
    Serial.print(tempC);
    Serial.print(" °C (");
    Serial.print(tempF);
    Serial.println(" °F)");
  }

  delay(2000); // 2 seconds between reads
}

void loop() {
  Serial0.println("Some text");
  delay(1000);
  Serial0.println("Some text again");
}

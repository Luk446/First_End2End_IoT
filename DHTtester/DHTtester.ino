#include <DHT.h>

#define DHTPIN 6
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// UART0 default pins on ESP32-C3: RX=GPIO20, TX=GPIO21
void setup() {
  Serial0.begin(115200, SERIAL_8N1, 20, 21);
  delay(500);
  Serial0.println("\nDHT11 test on ESP32-C3-Mini (GPIO 4)");
  dht.begin();
  delay(1500); // let sensor stabilize
}

void loop() {
  float h  = dht.readHumidity();
  float tC = dht.readTemperature();
  float tF = dht.readTemperature(true);

  if (isnan(h) || isnan(tC)) {
    Serial0.println("Failed to read from DHT11. Check wiring/pull-up.");
  } else {
    Serial0.print("Humidity: "); Serial0.print(h); Serial0.print(" %\t");
    Serial0.print("Temp: "); Serial0.print(tC); Serial0.print(" °C (");
    Serial0.print(tF); Serial0.println(" °F)");
  }
  delay(2000);
}

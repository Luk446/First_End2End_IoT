// sketch_oct21a.ino
// Blink an LED connected to GPIO 5 on an ESP32

const int LED_PIN = 5;      // GPIO pin where the LED is connected
const unsigned long ON_MS  = 500;
const unsigned long OFF_MS = 500;

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
}

void loop() {
    digitalWrite(LED_PIN, HIGH);
    delay(ON_MS);
    digitalWrite(LED_PIN, LOW);
    delay(OFF_MS);
}
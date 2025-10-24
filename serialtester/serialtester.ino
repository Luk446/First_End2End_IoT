// UART0: RX=GPIO20, TX=GPIO21 on ESP32-C3
void setup() {
  Serial0.begin(115200, SERIAL_8N1, 20, 21);
  delay(2000);                      // give the monitor time to attach
  Serial0.println("Booted on UART0");
}

void loop() {
  Serial0.println("Some text");
  delay(1000);
  Serial0.println("Some text again");
}

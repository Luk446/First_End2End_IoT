# IoT Temperature Monitoring System

End-to-end IoT solution for DHT11 temperature/humidity monitoring using ESP32-C3, MQTT, and Node-RED.

## Software Components

### ESP32 Firmware
- **part2_1**: MQTT-enabled sensor with NeoPixel visual alerts (alarm/warning thresholds)
- **part3**: Extended version with dynamic interval control via MQTT commands
- **Libraries**: DHT11, Adafruit NeoPixel, ArduinoJson, PubSubClient

### Node-RED Flows
- **alflows.json**: Alert logic flows
- **firstmongoexport.json**: MongoDB integration
- **historicalchart.json**: Historical data visualization

## Key Features
- WiFi connectivity with auto-reconnect
- JSON-formatted MQTT telemetry
- Temperature-based LED color coding (blue=cold, green=normal, red=hot)
- Remote interval adjustment (2-3600s)
- Real-time dashboard and historical charts

## Architecture
ESP32-C3 → MQTT Broker (broker.mqtt.cool) → Node-RED → MongoDB/Dashboard

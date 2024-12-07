# ESP-NOW and MQTT Bridge with DHT Sensor Integration

## Description
This project integrates ESP-NOW and MQTT to build a robust wireless communication system for ESP32 nodes. The nodes gather temperature and humidity data using DHT sensors and send it to a bridge node via ESP-NOW. The bridge node forwards this data to an MQTT broker, enabling real-time monitoring and control.

The system includes:
- **Sensor Node**: Collects data using a DHT sensor and sends it to the bridge node.
- **Bridge Node**: Receives data from the sensor nodes via ESP-NOW and forwards it to an MQTT broker.
- **MQTT Broker**: Centralized platform for publishing and subscribing to data.

## Features
- Wireless communication using ESP-NOW.
- Temperature and humidity data collection from DHT11/DHT22 sensors.
- MQTT integration for centralized monitoring and control.
- Configurable parameters (sampling interval, communication interval, temperature thresholds) via MQTT.
- Heartbeat monitoring for node statuses.
- Deep sleep mode for energy efficiency.

## Components
### Hardware:
- ESP32 boards (1 for bridge, 1 or more for sensor nodes).
- DHT11/DHT22 sensor (connected to the sensor nodes).

### Software:
- Arduino IDE with required libraries:
  - [ESP-NOW](https://www.arduino.cc/reference/en/libraries/esp-now/)
  - [WiFi](https://www.arduino.cc/en/Reference/WiFi)
  - [DHT Sensor Library](https://github.com/adafruit/DHT-sensor-library)
  - [Preferences](https://www.arduino.cc/reference/en/libraries/preferences/)
  - [ArduinoJson](https://arduinojson.org/)
  - [PubSubClient](https://pubsubclient.knolleary.net/)

## Setup

### 1. Sensor Node Configuration
1. Connect the DHT sensor to the ESP32:
   - **VCC** to 3.3V.
   - **GND** to GND.
   - **Data Pin** to GPIO pin (e.g., GPIO4).
2. Flash the sensor node code (`sensor_node.ino`) onto the ESP32.
3. Update the bridge MAC address in the code:
   ```cpp
   uint8_t bridgeMAC[] = {0x08, 0xB6, 0x1F, 0x27, 0x67, 0x00};
   ```
4. Compile and upload the code.

### 2. Bridge Node Configuration
1. Flash the bridge node code (`bridge_node.ino`) onto the ESP32.
2. Update the Wi-Fi credentials:
   ```cpp
   const char* ssid = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   ```
3. Update the MQTT broker details:
   ```cpp
   #define MQTT_SERVER "broker.mqtt.cool"
   #define MQTT_PORT 1883
   ```
4. Compile and upload the code.

### 3. MQTT Broker Setup
1. Use a public MQTT broker (e.g., HiveMQ) or set up your own broker (e.g., Mosquitto).
2. Ensure the bridge node is subscribed to the correct topics:
   - Configuration: `sensor/config`
   - Data: `sensor/data`
   - Alerts: `sensor/alert`
   - Status: `sensor/status`

## Usage
1. Power on the bridge node and connect it to Wi-Fi.
2. Power on the sensor nodes. They will register with the bridge node via ESP-NOW.
3. Monitor the MQTT topics to view data, alerts, and node statuses.
4. Use the `sensor/config` topic to send JSON configuration commands to the nodes.

### Example MQTT Configuration Command
```json
{
  "mac": "08:B6:1F:27:67:00",
  "sampling_interval": 15000,
  "communication_interval": 15000,
  "temp_high": 35.0,
  "temp_low": 5.0
}
```

### Example MQTT Topics
- **Data**: `sensor/data` (e.g., `{"type":"data","temperature":25.30,"humidity":60.50}`)
- **Alerts**: `sensor/alert` (e.g., `{"type":"alert","temperature":40.00}`)
- **Status**: `sensor/status` (e.g., `{"type":"status","mac":"08:B6:1F:27:67:00","status":"online"}`)

## Deep Sleep
The sensor nodes enter deep sleep between data transmissions for energy efficiency. The sleep duration is determined by the `samplingInterval` parameter.

## Notes
- Ensure the bridge and sensor nodes use the same Wi-Fi channel.
- Adjust the maximum number of nodes in the bridge code if needed:
  ```cpp
  #define MAX_NODES 10
  ```

## Troubleshooting
- **DHT Sensor Issues**: Ensure proper connections and verify the sensor type (DHT11 or DHT22).
- **ESP-NOW Communication**: Confirm the MAC addresses and Wi-Fi channel match.
- **MQTT Connectivity**: Verify broker details and ensure the bridge node is connected to Wi-Fi.

## Future Improvements
- Add support for additional sensors.
- Implement secure MQTT communication using TLS.
- Enhance node removal logic with configurable timeouts.

## License
This project is open-source and available under the [MIT License](https://opensource.org/licenses/MIT).

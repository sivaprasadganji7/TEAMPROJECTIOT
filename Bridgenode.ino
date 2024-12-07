#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// MQTT Broker
#define MQTT_SERVER "broker.mqtt.cool"  // HiveMQ public broker
#define MQTT_PORT 1883                   // Port for non-secure connection

const char* mqtt_data_topic = "sensor/data";       // Topic for sensor data
const char* mqtt_alert_topic = "sensor/alert";    // Topic for alerts
const char* mqtt_config_topic = "sensor/config";  // Topic for configuration commands
const char* mqtt_status_topic = "sensor/status";  // Topic for node statuses

// Wi-Fi credentials
const char* ssid = "SIVAPRASAD";       
const char* password = "rama1234";    

WiFiClient espClient;
PubSubClient client(espClient);

// Structure to hold node information
struct NodeInfo {
  String mac;
  unsigned long lastHeartbeat;
};
#define MAX_NODES 10
NodeInfo nodes[MAX_NODES];
int nodeCount = 0;

// MQTT message callback function
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT Message arrived on topic: ");
  Serial.println(topic);

  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println("Message: " + message);

  // Handle configuration updates
  if (String(topic) == mqtt_config_topic) {
    DynamicJsonDocument configDoc(1024);
    DeserializationError error = deserializeJson(configDoc, message);
    if (error) {
      Serial.println("Failed to parse JSON configuration");
      return;
    }

    String nodeMac = configDoc["mac"].as<String>();
    unsigned long samplingInterval = configDoc["sampling_interval"].as<unsigned long>();
    unsigned long communicationInterval = configDoc["communication_interval"].as<unsigned long>();
    float tempHigh = configDoc["temp_high"].as<float>();
    float tempLow = configDoc["temp_low"].as<float>();

    // Find the node's MAC address
    int nodeIndex = -1;
    for (int i = 0; i < nodeCount; i++) {
      if (nodes[i].mac == nodeMac) {
        nodeIndex = i;
        break;
      }
    }

    if (nodeIndex == -1) {
      Serial.println("Node not found for configuration: " + nodeMac);
      return;
    }

    // Create configuration payload
    String configPayload = "{\"samplingInterval\":" + String(samplingInterval) + 
                           ",\"communicationInterval\":" + String(communicationInterval) + 
                           ",\"tempHigh\":" + String(tempHigh) + 
                           ",\"tempLow\":" + String(tempLow) + "}";
    
    // Convert MAC address string to uint8_t array
    uint8_t macArray[6];
    int macValues[6];
    sscanf(nodeMac.c_str(), "%x:%x:%x:%x:%x:%x", 
           &macValues[0], &macValues[1], &macValues[2], 
           &macValues[3], &macValues[4], &macValues[5]);
    for (int i = 0; i < 6; i++) {
      macArray[i] = macValues[i];
    }

    // Send configuration via ESP-NOW
    esp_err_t result = esp_now_send(macArray, (uint8_t*)configPayload.c_str(), configPayload.length() + 1);
    if (result == ESP_OK) {
      Serial.println("Configuration sent to node: " + nodeMac);
      String statusPayload = "{\"type\":\"status\",\"mac\":\"" + nodeMac + "\",\"status\":\"configured\"}";
      client.publish(mqtt_status_topic, statusPayload.c_str());
    } else {
      Serial.print("Failed to send configuration to node: ");
      Serial.println(result);
    }
  }
}

// Callback to handle received data via ESP-NOW
void onDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len) {
  // Extract MAC address of sender
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           recvInfo->src_addr[0], recvInfo->src_addr[1], recvInfo->src_addr[2],
           recvInfo->src_addr[3], recvInfo->src_addr[4], recvInfo->src_addr[5]);
  String senderMac = String(macStr);

  Serial.println("Received data from: " + senderMac);

  // Convert incoming data to a string
  String receivedData = String((char*)incomingData);
  Serial.println("Data: " + receivedData);

  // Parse JSON payload
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, receivedData);
  if (error) {
    Serial.println("Failed to parse JSON");
    return;
  }

  // Process the type of message
  String type = doc["type"].as<String>();

  if (type == "registration") {
    Serial.println("Processing registration...");
    String nodeMac = doc["mac"].as<String>();
    bool exists = false;

    for (int i = 0; i < nodeCount; i++) {
      if (nodes[i].mac == nodeMac) {
        exists = true;
        nodes[i].lastHeartbeat = millis();
        break;
      }
    }

    if (!exists && nodeCount < MAX_NODES) {
      nodes[nodeCount].mac = nodeMac;
      nodes[nodeCount].lastHeartbeat = millis();
      nodeCount++;
      Serial.println("Registered new node: " + nodeMac);

      String statusPayload = "{\"type\":\"status\",\"mac\":\"" + nodeMac + "\",\"status\":\"online\"}";
      client.publish(mqtt_status_topic, statusPayload.c_str());
    }
  } else if (type == "data") {
    Serial.println("Publishing sensor data to MQTT...");
    client.publish(mqtt_data_topic, receivedData.c_str());
  } else if (type == "alert") {
    Serial.println("Publishing alert to MQTT...");
    client.publish(mqtt_alert_topic, receivedData.c_str());
  } else if (type == "heartbeat") {
    Serial.println("Processing heartbeat...");
    String nodeMac = doc["mac"].as<String>();
    for (int i = 0; i < nodeCount; i++) {
      if (nodes[i].mac == nodeMac) {
        nodes[i].lastHeartbeat = millis();
        Serial.println("Heartbeat received from node: " + nodeMac);

        String statusPayload = "{\"type\":\"status\",\"mac\":\"" + nodeMac + "\",\"status\":\"online\"}";
        client.publish(mqtt_status_topic, statusPayload.c_str());
        break;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected");

  // Setup MQTT
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(mqttCallback);
  
  while (!client.connected()) {
    Serial.println("Connecting to MQTT broker...");
    if (client.connect("ESP32_Bridge")) {
      Serial.println("Connected to MQTT Broker");
      client.subscribe(mqtt_config_topic);
    } else {
      Serial.print("MQTT connection failed, retrying in 10 seconds...");
      Serial.println(client.state());
      delay(10000);
    }
  }

  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register ESP-NOW receive callback
  esp_now_register_recv_cb(onDataRecv);
}

void loop() {
  if (!client.connected()) {
    Serial.println("Reconnecting to MQTT Broker...");
    while (!client.connected()) {
      if (client.connect("ESP32_Bridge")) {
        Serial.println("Reconnected to MQTT Broker");
        client.subscribe(mqtt_config_topic);
      } else {
        Serial.print("MQTT connection failed, retrying in 10 seconds...");
        Serial.println(client.state());
        delay(10000);
      }
    }
  }

  client.loop();

  // Periodically check for node heartbeats
  unsigned long currentTime = millis();
  for (int i = 0; i < nodeCount; i++) {
    if (currentTime - nodes[i].lastHeartbeat > 30000) {  // 30 seconds timeout
      Serial.println("Node offline: " + nodes[i].mac);
      String statusPayload = "{\"type\":\"status\",\"mac\":\"" + nodes[i].mac + "\",\"status\":\"offline\"}";
      client.publish(mqtt_status_topic, statusPayload.c_str());
      for (int j = i; j < nodeCount - 1; j++) {
        nodes[j] = nodes[j + 1];
      }
      nodeCount--;
      i--;  // Adjust index after removal
    }
  }

  delay(5000);  // Check every 5 seconds
}

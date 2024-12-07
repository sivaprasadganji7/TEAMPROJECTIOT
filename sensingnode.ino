#include <esp_now.h>
#include <WiFi.h>
#include <DHT.h>
#include <Preferences.h>
#include <ArduinoJson.h>

// DHT Sensor setup
#define DHT_PIN 4           // Change to the pin you're using for the DHT sensor
#define DHT_TYPE DHT11      // Change to DHT22 if you're using a DHT22 sensor
DHT dht(DHT_PIN, DHT_TYPE);

// ESP-NOW bridge MAC address
uint8_t bridgeMAC[] = {0x08, 0xB6, 0x1F, 0x27, 0x67, 0x00};

// Preferences for storing configuration
Preferences preferences;

// Configuration defaults
unsigned long samplingInterval = 10000;       // Default: 10 seconds
unsigned long communicationInterval = 10000;  // Default: 10 seconds
float tempThresholdHigh = 30.0;                // Default: High-temperature alert threshold
float tempThresholdLow = 10.0;                 // Default: Low-temperature alert threshold

// Track the time for calculating Duty Cycle
unsigned long lastTransmissionTime = 0;        // Last time the data was sent
unsigned long totalSleepTime = 0;              // Total time spent in sleep
unsigned long totalTransmissionTime = 0;       // Total time spent in transmission

// Function to handle incoming ESP-NOW messages (configurations)
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  Serial.println("Received configuration from bridge.");

  // Parse JSON configuration
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, incomingData, len);
  if (error) {
    Serial.println("Failed to parse JSON configuration");
    return;
  }

  if (doc.containsKey("samplingInterval")) {
    samplingInterval = doc["samplingInterval"];
    Serial.println("Updated sampling interval: " + String(samplingInterval) + " ms");
    preferences.begin("nodeConfig", false);
    preferences.putULong("samplingInterval", samplingInterval);
    preferences.end();
  }

  if (doc.containsKey("communicationInterval")) {
    communicationInterval = doc["communicationInterval"];
    Serial.println("Updated communication interval: " + String(communicationInterval) + " ms");
    preferences.begin("nodeConfig", false);
    preferences.putULong("commInterval", communicationInterval);
    preferences.end();
  }

  if (doc.containsKey("tempHigh")) {
    tempThresholdHigh = doc["tempHigh"];
    Serial.println("Updated high temperature threshold: " + String(tempThresholdHigh) + " C");
    preferences.begin("nodeConfig", false);
    preferences.putFloat("tempHigh", tempThresholdHigh);
    preferences.end();
  }

  if (doc.containsKey("tempLow")) {
    tempThresholdLow = doc["tempLow"];
    Serial.println("Updated low temperature threshold: " + String(tempThresholdLow) + " C");
    preferences.begin("nodeConfig", false);
    preferences.putFloat("tempLow", tempThresholdLow);
    preferences.end();
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // Initialize the DHT sensor
  dht.begin();

  // Set ESP32 to Station mode for ESP-NOW
  WiFi.mode(WIFI_STA);

  // Initialize Preferences
  preferences.begin("nodeConfig", false);

  // Load configuration from non-volatile memory
  samplingInterval = preferences.getULong("samplingInterval", samplingInterval);
  communicationInterval = preferences.getULong("commInterval", communicationInterval);
  tempThresholdHigh = preferences.getFloat("tempHigh", tempThresholdHigh);
  tempThresholdLow = preferences.getFloat("tempLow", tempThresholdLow);
  preferences.end();

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback function for receiving configurations
  esp_now_register_recv_cb(onDataRecv);

  // Add bridge as a peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, bridgeMAC, sizeof(bridgeMAC));
  peerInfo.channel = 0;  // Default channel
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add bridge as peer");
    return;
  }

  Serial.println("ESP-NOW Initialized and bridge added as peer");

  // Send registration packet
  String registrationPayload = "{\"type\":\"registration\",\"mac\":\"" + String(WiFi.macAddress()) + "\"}";
  esp_now_send(bridgeMAC, (uint8_t *)registrationPayload.c_str(), registrationPayload.length() + 1);
  Serial.println("Sent registration: " + registrationPayload);
}

void loop() {
  // Record the start time of the transmission
  unsigned long transmissionStartTime = millis();

  // Read temperature and humidity from the DHT sensor
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // Check if the reading is valid
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Print temperature and humidity values for debugging
  Serial.println("Temperature: " + String(temperature) + "C, Humidity: " + String(humidity) + "%");

  // Check temperature thresholds and send alert if necessary
  if (temperature > tempThresholdHigh || temperature < tempThresholdLow) {
    String alertPayload = "{\"type\":\"alert\",\"temperature\":" + String(temperature) + "}";
    esp_now_send(bridgeMAC, (uint8_t *)alertPayload.c_str(), alertPayload.length() + 1);
    Serial.println("Alert sent: " + alertPayload);
  }

  // Prepare data as JSON
  String dataPayload = "{\"type\":\"data\",\"temperature\":" + String(temperature, 2) + ", \"humidity\":" + String(humidity, 2) + "}";
  esp_now_send(bridgeMAC, (uint8_t *)dataPayload.c_str(), dataPayload.length() + 1);  // Include null terminator
  Serial.println("Sent data: " + dataPayload);

  // Record the end time of the transmission
  unsigned long transmissionEndTime = millis();
  unsigned long transmissionDuration = transmissionEndTime - transmissionStartTime;
  totalTransmissionTime += transmissionDuration;

  // Calculate cycle time (including sleep time)
  totalSleepTime += samplingInterval;

  // Print the calculated duty cycle every cycle
  unsigned long cycleTime = totalTransmissionTime + totalSleepTime;
  float dutyCycle = (float)totalTransmissionTime / (float)cycleTime * 100;
  Serial.println("Duty Cycle: " + String(dutyCycle) + "%");

  // Send heartbeat
  String heartbeatPayload = "{\"type\":\"Device\",\"mac\":\"" + String(WiFi.macAddress()) + "\"}";
  esp_now_send(bridgeMAC, (uint8_t *)heartbeatPayload.c_str(), heartbeatPayload.length() + 1);
  Serial.println("Sent Device: " + heartbeatPayload);

  // Enter deep sleep for energy efficiency
  esp_sleep_enable_timer_wakeup(samplingInterval * 1000);  // Convert ms to microseconds
  esp_deep_sleep_start();
}

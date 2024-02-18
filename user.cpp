#if 1 // Change to 0 to disable this code (must enable ONE user*.cpp only!)

#include <WiFi.h>               // Include WiFi library for wireless connectivity
#include <PubSubClient.h>       // Include PubSubClient library for MQTT functionality
#include <ArduinoJson.h>

#define TFT_CS1 5
#define TFT_CS2 18

// This file provides a crude way to "drop in" user code to the eyes,
// allowing concurrent operations without having to maintain a bunch of
// special derivatives of the eye code (which is still undergoing a lot
// of development). Just replace the source code contents of THIS TAB ONLY,
// compile and upload to board. Shouldn't need to modify other eye code.

// User globals can go here, recommend declaring as static, e.g.:
// static int foo = 42;
// Network credentials
const char* ssid = "Picture-AP";         // WiFi SSID
const char* password = "ThePictureShow"; // WiFi Password

// MQTT broker settings
const char* mqtt_server = "192.168.168.1"; // MQTT broker address

// Topics
#define TOPIC_BUFFER_SIZE 64

const char* baseTopicFormat = "house/window/%d/picture/res/240x240/base64";

char topic1[TOPIC_BUFFER_SIZE];
char topic2[TOPIC_BUFFER_SIZE];

const char* topicCommandSend = "control/dev/send";

WiFiClient wifiClient;               // Instantiate WiFi client for internet connectivity
PubSubClient mqttClient(wifiClient); // Attach WiFi client to MQTT client for MQTT functionality

// Default windows
int window1 = 1;
int window2 = 2;

// Forward declarations
void setup_wifi();
void reconnect_wifi();
void reconnect_mqtt();
void formatTopic(int windowNumber, char* topicBuffer, size_t bufferSize);
void callback(char* topic, byte* payload, unsigned int length);
void generateClientID(char* clientId, const char* prefix);
void sendDeviceInfo();

// Called once near the end of the setup() function.
void user_setup(void) {
  setup_wifi();

  mqttClient.setBufferSize(51200);
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(callback);
  sendDeviceInfo();
}

// Called periodically during eye animation. This is invoked in the
// interval before starting drawing on the last eye so it won't exacerbate
// visible tearing in eye rendering.
// This function BLOCKS, it does NOT multitask with the eye animation code,
// and performance here will have a direct impact on overall refresh rates,
// so keep it simple. Avoid loops (e.g. if animating something like a servo
// or NeoPixels in response to some trigger) and instead rely on state
// machines or similar. Additionally, calls to this function are NOT time-
// constant -- eye rendering time can vary frame to frame, so animation or
// other over-time operations won't look very good using simple +/-
// increments, it's better to use millis() or micros() and work
// algebraically with elapsed times instead.
void user_loop(void) {
/*
  Suppose we have a global bool "animating" (meaning something is in
  motion) and global uint32_t's "startTime" (the initial time at which
  something triggered movement) and "transitionTime" (the total time
  over which movement should occur, expressed in microseconds).
  Maybe it's servos, maybe NeoPixels, or something different altogether.
  This function might resemble something like (pseudocode):

  if(!animating) {
    Not in motion, check sensor for trigger...
    if(read some sensor) {
      Motion is triggered! Record startTime, set transition
      to 1.5 seconds and set animating flag:
      startTime      = micros();
      transitionTime = 1500000;
      animating      = true;
      No motion actually takes place yet, that will begin on
      the next pass through this function.
    }
  } else {
    Currently in motion, ignore trigger and move things instead...
    uint32_t elapsed = millis() - startTime;
    if(elapsed < transitionTime) {
      Part way through motion...how far along?
      float ratio = (float)elapsed / (float)transitionTime;
      Do something here based on ratio, 0.0 = start, 1.0 = end
    } else {
      End of motion reached.
      Take whatever steps here to move into final position (1.0),
      and then clear the "animating" flag:
      animating = false;
    }
  }
*/

  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    reconnect_wifi(); // Attempt to reconnect to WiFi if disconnected
  }

  // Check MQTT connection
  if (!mqttClient.connected()) {
    reconnect_mqtt(); // Attempt to reconnect to MQTT if disconnected
  }

  // Process MQTT loop
  mqttClient.loop();

}

//####################################################################################################
// MQTT Topic Callback
//####################################################################################################
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println();

  // Use decodedBytes and decodedLength for further processing
  //Serial.print("Decoded Length: ");
  //Serial.println(decodedLength);

  // Check which topic the message is about and act accordingly
  int pin = TFT_CS1;
  if (strcmp(topic, topic1) == 0) {
    // Handle topic 1 message
    pin = TFT_CS1;
  } else if (strcmp(topic, topic2) == 0) {
    // Handle topic 2 message
    pin = TFT_CS2;
  } else {
    return;
  }

  // Convert payload to a null-terminated string
  char* base64String = new char[length + 1];
  memcpy(base64String, payload, length);
  base64String[length] = '\0';

  // Clean up
  delete[] base64String;
}

void setup_wifi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 30000) {  // 30 seconds
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to connect to WiFi. Please check your credentials");
    Serial.println("Rebooting...");
    ESP.restart();  // This will cause the ESP32 to reboot
  }

  Serial.println("");
  Serial.println("WiFi connected successfully");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect_wifi() {
  // Check if the device is connected to WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();

    // Attempt to reconnect
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {  // 10 seconds timeout
      delay(500);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi reconnected");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("\nFailed to reconnect to WiFi");
    }
  }
}

void reconnect_mqtt() {
  if (!mqttClient.connected()) {
    char clientID[20]; // Adjust size based on the expected length of your client ID
    generateClientID(clientID, "ESP");
    Serial.print("MQTT Client ID: ");
    Serial.println(clientID);

    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect(clientID)) {
      Serial.println("connected");

      // Format the topics to use
      formatTopic(window1, topic1, sizeof(topic1));
      formatTopic(window2, topic2, sizeof(topic2));

      // Subscribe to the topics now
      mqttClient.subscribe(topic1);
      mqttClient.subscribe(topic2);

    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void generateClientID(char* clientId, const char* prefix) {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  snprintf(clientId, 20, "%s_%02X%02X%02X", prefix, mac[3], mac[4], mac[5]);
}

void getDeviceId(char* buffer, size_t bufferSize) {
    uint64_t chipid = ESP.getEfuseMac();
    snprintf(buffer, bufferSize, "%08X%08X", (uint32_t)(chipid >> 32), (uint32_t)chipid);
}

void sendDeviceInfo() {
  // Get ESP32 Unique ID
  uint64_t chipid = ESP.getEfuseMac();
  String deviceId = String((uint32_t)(chipid >> 32), HEX) + String((uint32_t)chipid, HEX);

  // Construct topic with device ID
  String topic = String(topicCommandSend) + "/" + deviceId + "/info";
  char topicChar[topic.length() + 1];
  topic.toCharArray(topicChar, topic.length() + 1);

  // Network information
  String ipAddress = WiFi.localIP().toString();
  String macAddress = WiFi.macAddress();

  // Create JSON object
  StaticJsonDocument<256> doc;
  doc["chip_id"] = deviceId;
  doc["ip_address"] = ipAddress;
  doc["mac_address"] = macAddress;

  // Serialize JSON to String
  String payload;
  serializeJson(doc, payload);

  // Publish JSON payload
  mqttClient.publish(topicChar, payload.c_str());
}

void formatTopic(int windowNumber, char* topicBuffer, size_t bufferSize) {
    snprintf(topicBuffer, bufferSize, baseTopicFormat, windowNumber);
}


#endif // 0

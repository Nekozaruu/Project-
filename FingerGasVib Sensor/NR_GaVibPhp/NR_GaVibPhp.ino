#include <WiFi.h>
#include <Adafruit_Fingerprint.h>
#include <PubSubClient.h>

#define MQ135_PIN A5         // Define the pin for MQ135 gas sensor
#define VIBRATION_PIN 12     // Define the pin for vibration sensor
#define BUZZER_PIN 13        // Define the pin for the buzzer
#define GAS_THRESHOLD 50     // Define the gas threshold
#define SOLENOID_PIN 32      // Define the pin connected to the solenoid lock

const char* ssid = "Pena pemuji";
const char* password = "77777777";
const int FAN_PIN = 19;     // Define the pin for the 12V fan

#define FINGERPRINT_SENSOR_RX 16 // RX pin of fingerprint sensor connected to GPIO 16
#define FINGERPRINT_SENSOR_TX 17 // TX pin of fingerprint sensor connected to GPIO 17

const char* mqtt_server = "broker.hivemq.com"; // Change this to your MQTT broker address
const int mqtt_port = 1883; // Default MQTT port
const char* gas_topic = "gasValue"; // MQTT topic for gas value
const char* vibration_topic = "vibrationValue"; // MQTT topic for vibration value

Adafruit_Fingerprint finger = Adafruit_Fingerprint(&Serial2);

WiFiClient espClient;
PubSubClient client(espClient);

// Array to store registered fingerprint IDs
const uint16_t registeredIDs[] = {1, 2}; // Add more IDs as needed

unsigned long lastGasReadTime = 0;    // Variable to store the last time gas was read
unsigned long lastVibrationReadTime = 0;  // Variable to store the last time vibration was read
bool fanActive = false;               // Variable to track fan activation state

// Function declarations
void unlockDoor();
void lockDoor();
bool fingerprintTrigger();
void connectToWiFi();
void setup_mqtt();
void publish_gas_vibration(int gasValue, int vibrationValue);
int getFingerprintID();

void setup() {
  Serial.begin(115200);      // Initialize serial communication
  pinMode(MQ135_PIN, INPUT); // Set MQ135 pin as input
  pinMode(VIBRATION_PIN, INPUT); // Set vibration sensor pin as input
  pinMode(BUZZER_PIN, OUTPUT); // Set buzzer pin as output
  pinMode(FAN_PIN, OUTPUT);   // Set fan pin as output
  pinMode(SOLENOID_PIN, OUTPUT); // Set the solenoid pin as an output
  
  connectToWiFi(); // Connect to WiFi network
  Serial.println("Connected to WiFi");

  setup_mqtt(); // Connect to MQTT broker

  // Start the hardware serial port for the fingerprint sensor
  Serial2.begin(57600);

  if (finger.verifyPassword()) {
    Serial.println("Found fingerprint sensor!");
  } else {
    Serial.println("Did not find fingerprint sensor :(");
    while (1) { delay(1); }
  }
}

void loop() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection lost. Reconnecting...");
    connectToWiFi();
  }

  // Read analog value from MQ135 sensor at regular intervals
  unsigned long currentMillis = millis();
  if (currentMillis - lastGasReadTime >= 5000) {
    int gasValue = analogRead(MQ135_PIN);
    Serial.print("Gas Value: ");
    Serial.println(gasValue);
    if (gasValue > GAS_THRESHOLD) {
      Serial.println("Purifying Gas");
      if (!fanActive) {
        digitalWrite(FAN_PIN, HIGH); // Turn on the fan
        fanActive = true;
        delay(5000); //fan on 5 sec
      }
    } else {
      Serial.println("Clean Air");
      if (fanActive) {
        digitalWrite(FAN_PIN, LOW); // Turn off the fan
        fanActive = false;
      }
    }
    // Publish gas value to MQTT
    publish_gas_vibration(gasValue, digitalRead(VIBRATION_PIN));
    lastGasReadTime = currentMillis; // Update the last gas reading time
  }

  // Read digital value from vibration sensor at regular intervals
  if (currentMillis - lastVibrationReadTime >= 5000) {
    int vibrationValue = digitalRead(VIBRATION_PIN);
    if (vibrationValue == HIGH) {
      Serial.println("Vibration Detected");
      digitalWrite(BUZZER_PIN, HIGH); // Turn on the buzzer
      delay(3000); // Keep the buzzer active for 3 seconds
      digitalWrite(BUZZER_PIN, LOW); // Turn off the buzzer
    } else { 
      Serial.println("Vibration OFF");
    }
    lastVibrationReadTime = currentMillis; // Update the last vibration reading time
  }

  // Check for fingerprint detection
  if (fingerprintTrigger()) {
    unlockDoor(); // Unlock the door
    delay(5000); // Keep the door unlocked for 5 seconds (adjust as needed)
    lockDoor(); // Lock the door
  }

  // MQTT client loop
  if (!client.loop()) {
    client.connect("P-aangFyP");
  }

  // Delay to control the loop frequency
  delay(10);
}

void connectToWiFi() {
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 5) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
    attempts++;
  }
}

bool fingerprintTrigger() {
  Serial.println("Place your finger on the sensor...");
  delay(3000); // Delay for 3 seconds to allow time for the user to place their finger
  int detectedID = getFingerprintID();
  if (detectedID > 0) {
    for (int i = 0; i < sizeof(registeredIDs) / sizeof(registeredIDs[0]); i++) {
      if (detectedID == registeredIDs[i]) {
        Serial.println("Welcome Farhan!"); // Change the message as needed
        return true;
      }
    }
  } else if (detectedID == 0) {
    Serial.println("You're Thief!");
  } else {
    Serial.println("No fingerprint detected!");
  }
  return false;
}

int getFingerprintID() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.fingerFastSearch();
  if (p == FINGERPRINT_OK) {
    return finger.fingerID;
  } else if (p == FINGERPRINT_NOTFOUND) {
    return 0;
  } else {
    return -1;
  }
}

void unlockDoor() {
  digitalWrite(SOLENOID_PIN, HIGH); // Activate the solenoid to unlock the door
  Serial.println("Door unlocked");
}

void lockDoor() {
  digitalWrite(SOLENOID_PIN, LOW); // Deactivate the solenoid to lock the door
  Serial.println("Door locked");
}

void setup_mqtt() {
  client.setServer(mqtt_server, mqtt_port);
}

void publish_gas_vibration(int gasValue, int vibrationValue) {
  if (!client.connected()) {
    if (!client.connect("P-aangKacak")) {
      Serial.println("Failed to connect to MQTT broker");
      return;
    }
  }
  // Publish gas value to MQTT topic
  client.publish(gas_topic, String(gasValue).c_str());
  // Publish vibration value to MQTT topic
  client.publish(vibration_topic, String(vibrationValue).c_str());
}

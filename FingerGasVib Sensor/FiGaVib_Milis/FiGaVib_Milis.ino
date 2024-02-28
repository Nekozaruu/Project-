#include <WiFi.h>
#include <Adafruit_Fingerprint.h>

#define MQ135_PIN A5         // Define the pin for MQ135 gas sensor
#define VIBRATION_PIN 12     // Define the pin for vibration sensor
#define BUZZER_PIN 13        // Define the pin for the buzzer
#define GAS_THRESHOLD 50     // Define the gas threshold
#define SOLENOID_PIN 32      // Define the pin connected to the solenoid lock

const char* ssid = "@KKTMPJ STAFF";
const char* password = "kktmpj1234";
const int FAN_PIN = 19;     // Define the pin for the 12V fan

#define FINGERPRINT_SENSOR_RX 16 // RX pin of fingerprint sensor connected to GPIO 16
#define FINGERPRINT_SENSOR_TX 17 // TX pin of fingerprint sensor connected to GPIO 17

// Define the hardware serial port to use
#define FINGERPRINT_SERIAL Serial2 // Change to the appropriate Serial port you are using on your ESP32

Adafruit_Fingerprint finger = Adafruit_Fingerprint(&FINGERPRINT_SERIAL);

// Array to store registered fingerprint IDs
const uint16_t registeredIDs[] = {1, 2}; // Add more IDs as needed

WiFiServer server(80);
unsigned long lastVibrationTime = 0; // Variable to store the time when vibration was last detected
unsigned long lastGasTime = 0;       // Variable to store the time when gas was last read
bool fanActive = false;               // Variable to track fan activation state

// Function declarations
void unlockDoor();
void lockDoor();
bool fingerprintTrigger();
void connectToWiFi();

// Timing variables
unsigned long gasInterval = 5000; // 5 seconds
unsigned long vibrationInterval = 1000; // 1 second
unsigned long lastGasReadTime = 0;
unsigned long lastVibrationReadTime = 0;

void setup() {
  Serial.begin(115200);      // Initialize serial communication
  pinMode(MQ135_PIN, INPUT); // Set MQ135 pin as input
  pinMode(VIBRATION_PIN, INPUT); // Set vibration sensor pin as input
  pinMode(BUZZER_PIN, OUTPUT); // Set buzzer pin as output
  pinMode(FAN_PIN, OUTPUT);   // Set fan pin as output
  pinMode(SOLENOID_PIN, OUTPUT); // Set the solenoid pin as an output
  
  connectToWiFi(); // Connect to WiFi network
  Serial.println("Connected to WiFi");
  
  // Start the server
  server.begin();

  // Start the hardware serial port for the fingerprint sensor
  FINGERPRINT_SERIAL.begin(57600);

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

  // Read analog value from MQ135 sensor
  if (millis() - lastGasReadTime >= gasInterval) {
    int gasValue = analogRead(MQ135_PIN);
    Serial.print("Gas Value: ");
    Serial.println(gasValue);
    if (gasValue > GAS_THRESHOLD) {
      Serial.println("Purifying Gas");
      if (!fanActive) {
        digitalWrite(FAN_PIN, HIGH); // Turn on the fan
        fanActive = true;
      }
    } else {
      Serial.println("Clean Air");
      if (fanActive) {
        digitalWrite(FAN_PIN, LOW); // Turn off the fan
        fanActive = false;
      }
    }
    lastGasReadTime = millis(); // Update the last gas reading time
  }

  // Read digital value from vibration sensor
  if (millis() - lastVibrationReadTime >= vibrationInterval) {
    int vibrationValue = digitalRead(VIBRATION_PIN);
    if (vibrationValue == HIGH) {
      Serial.println("Vibration Detected");
      digitalWrite(BUZZER_PIN, HIGH); // Turn on the buzzer
    } else { 
      Serial.println("Vibration OFF");
      digitalWrite(BUZZER_PIN, LOW); // Turn off the buzzer
    }
    lastVibrationReadTime = millis(); // Update the last vibration reading time
  }

  // Check for fingerprint detection
  if (fingerprintTrigger()) {
    unlockDoor(); // Unlock the door
    delay(5000); // Keep the door unlocked for 5 seconds (adjust as needed)
    lockDoor(); // Lock the door
  }

  // Check if a client has connected
  WiFiClient client = server.available();
  if (client) {
    // Wait for data from client to close connection
    while (client.connected()) {
      if (client.available()) {
        client.stop();
      }
    }
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
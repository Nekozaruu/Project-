#include <Adafruit_Fingerprint.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <IOXhop_FirebaseESP32.h>
#include <ArduinoJson.h>

// Define the hardware serial port to use for the fingerprint sensor
#define RXD2 16
#define TXD2 17

#define SOLENOID_PIN 32   // Define the pin connected to the solenoid lock
#define VIBRATION_PIN 12  // Define the pin connected to the vibration sensor
#define BUZZER_PIN 13     // Define the pin connected to the buzzer

#define FIREBASE_HOST "maass3-8085d-default-rtdb.asia-southeast1.firebasedatabase.app"  //Masukkan link realtime database Firebase
#define FIREBASE_AUTH "jFPX6RUB9DfgvEEFGjV8pBdge1FVyrp269NMHVeB"

String switchlock;  //variable(switchlock) to read status LED in Firebase RTDB
String switchbuzz;  //variable(switchlock) to read status LED in Firebase RTDB

HardwareSerial mySerial(2);
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

const char* ssid = "Pena pemuji";
const char* password = "77777777";

const char* mqtt_server = "broker.hivemq.com";  // MQTT broker address
const int mqtt_port = 1883;                     // MQTT broker port

WiFiClient espClient;
PubSubClient client(espClient);

// MQTT topics
const char* vibration_topic = "vibrationValue";

void unlockDoor();
void lockDoor();
bool fingerprintTrigger();
uint8_t getFingerprintID();
void handleVibration();
void connectToWiFi();
void reconnectToMQTT();
void publishVibration(int vibrationValue);

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Fingerprint sensor test");

  // Start the hardware serial port for the fingerprint sensor
  mySerial.begin(57600, SERIAL_8N1, RXD2, TXD2);

  // Set the data rate for the sensor serial port
  finger.begin(57600);
  if (finger.status_reg == FINGERPRINT_OK) {
    Serial.println("Found fingerprint sensor!");
  } else {
    Serial.println("Couldn't find fingerprint sensor :(");
    while (1)
      ;
  }
  finger.getParameters();

  pinMode(SOLENOID_PIN, OUTPUT);  // Set the solenoid pin as an output
  pinMode(VIBRATION_PIN, INPUT);  // Set the vibration sensor pin as an input
  pinMode(BUZZER_PIN, OUTPUT);    // Set the buzzer pin as an output

  connectToWiFi();
  client.setServer(mqtt_server, mqtt_port);

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);  //Hosting Firebase

                                     
  Firebase.setString("Actuator/Doorlock1","ON");  //set inital status of Actuator
  Firebase.setString("Actuator/Buzzer","OFF");   //set inital status of Actuator
}

void loop() {
  // Ensure the MQTT connection remains active
  if (!client.connected()) {
    reconnectToMQTT();
  }
  client.loop();

  // Check for fingerprint detection
  if (fingerprintTrigger()) {
    unlockDoor();  // Unlock the door
    delay(3000);   // Keep the door unlocked for 3 seconds (adjust as needed)
    lockDoor();    // Lock the door after the delay
  }

  // Handle vibration sensor
  handleVibration();
}

// Function to detect fingerprint and trigger the door lock
bool fingerprintTrigger() {
  Serial.println("Place your finger on the sensor...");

  // Delay for 3 seconds before scanning the fingerprint
  delay(4000);

  uint8_t id = getFingerprintID();

  if (id == 1) {
    Serial.println("Welcome, Farhan");
    return true;  // Trigger the door to unlock
  } else if (id == 2) {
    Serial.println("Selamat Datang Pelajar");
    return true;  // Trigger the door to unlock
  } else if (id == 0) {
    Serial.println("No finger detected");
  }
  return false;  // Do not trigger the door to unlock
}

// Function to get fingerprint ID
uint8_t getFingerprintID() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK) {
    Serial.println("Unauthorized Access");
    return 0;
  }

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK) {
    Serial.println("Image conversion failed");
    return 0;
  }

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK) {
    Serial.println("Fingerprint not found");
    return 0;
  }

  // Fingerprint found!
  Serial.print("Found fingerprint ID #");
  Serial.print(finger.fingerID);
  Serial.println();
  return finger.fingerID;

   // Solenoid Control*
  switchlock = Firebase.getString("Actuator/Doorlock1");
  switchlock.toUpperCase(); //change rule to uppercase

  if(switchlock == "ON" || switchlock == "1"){
      digitalWrite(SOLENOID_PIN,HIGH);
      Serial.print("Doorlock Upper Status:");
      Serial.println(switchlock);
    }else if(switchlock == "OFF" || switchlock == "0"){
      digitalWrite(SOLENOID_PIN,LOW);
      Serial.print("Doorlock Upper Status:");
      Serial.println(switchlock);
    }

    // Buzzer Control*
    switchbuzz = Firebase.getString("Actuator/Buzzer");
    switchbuzz.toUpperCase(); //change rule to uppercase

  if(switchbuzz == "ON" || switchbuzz == "1"){
      digitalWrite(BUZZER_PIN,HIGH);
      Serial.print("Buzzer Status:");
      Serial.println(switchbuzz);
    }else if(switchbuzz == "OFF" || switchbuzz == "0"){
      digitalWrite(BUZZER_PIN,LOW);
      Serial.print("Buzzer Status:");
      Serial.println(switchbuzz);
    }
}

// Function to unlock the door
void unlockDoor() {
  digitalWrite(SOLENOID_PIN, LOW);  // Activate the solenoid to unlock the door
  Serial.println("Door unlocked");
}

// Function to lock the door
void lockDoor() {
  digitalWrite(SOLENOID_PIN, HIGH);  // Deactivate the solenoid to lock the door
  Serial.println("Door locked");
}

// Function to handle vibration detection and buzzer activation
void handleVibration() {
  static unsigned long lastVibrationTime = 0;  // Stores the last time vibration was detected
  const unsigned long buzzerDuration = 2000;   // Duration to keep the buzzer on in milliseconds

  int vibrationValue = digitalRead(VIBRATION_PIN);

  // If vibration is detected (HIGH)
  if (vibrationValue == HIGH) {
    Serial.println("Thief Alerts");

    // Update the last vibration time
    lastVibrationTime = millis();

    // Turn on the buzzer
    digitalWrite(BUZZER_PIN, HIGH);

    // Publish the vibration event to MQTT
    publishVibration(vibrationValue);
  }

  // Turn off the buzzer after the duration has passed
  if (millis() - lastVibrationTime > buzzerDuration) {
    digitalWrite(BUZZER_PIN, LOW);  // Turn off the buzzer
    Serial.println("Buzzer turned off");
  }
  Firebase.setFloat("Sensor/Vibration", vibrationValue);
  Firebase.getFloat("Sensor/Vibration");
}

// Function to connect to WiFi
void connectToWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
}

// Function to reconnect to MQTT
void reconnectToMQTT() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("P-aangKacak")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// Function to publish vibration data to MQTT
void publishVibration(int vibrationValue) {
  client.publish(vibration_topic, String(vibrationValue).c_str());
}

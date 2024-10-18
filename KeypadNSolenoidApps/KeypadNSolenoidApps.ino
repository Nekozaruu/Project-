#include <WiFi.h>
#include <Keypad.h>
#include <ArduinoJson.h>
#include <IOXhop_FirebaseESP32.h>

#define RELAY_PIN 19  // ESP32 pin GPIO19 connected to the relay
#define ROW_NUM 4     // keypad four rows
#define COLUMN_NUM 4  // keypad four columns (adjusted comment to match definition)

// Firebase Credentials
#define FIREBASE_HOST "maass3-8085d-default-rtdb.asia-southeast1.firebasedatabase.app" //Masukkan link realtime database Firebase
#define FIREBASE_AUTH "jFPX6RUB9DfgvEEFGjV8pBdge1FVyrp269NMHVeB" 


String switchkeylock;  //variable(switchlock) to read status LED in Firebase RTDB


// Define your WiFi credentials
const char* wifi_ssid = "Pena pemuji";
const char* wifi_password = "77777777";

// Define the keymap
char keys[ROW_NUM][COLUMN_NUM] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

// Connect keypad ROW0, ROW1, ROW2, ROW3 to these ESP32 pins
byte rowPins[ROW_NUM] = { 13, 12, 14, 27 };

// Connect keypad COL0, COL1, COL2, COL3 to these ESP32 pins
byte colPins[COLUMN_NUM] = { 26, 25, 33, 32 };

// Create the Keypad object
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROW_NUM, COLUMN_NUM);

// Define passwords
const String correct_password = "ABC";  // change your password here

String input_password;

void setup() {
  Serial.begin(115200);
  input_password.reserve(32);    // maximum input characters is 33, change if needed
  pinMode(RELAY_PIN, OUTPUT);    // initialize pin as an output
  digitalWrite(RELAY_PIN, LOW);  // lock the door

  // Connect to WiFi
  WiFi.begin(wifi_ssid, wifi_password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(2000);
    Serial.println("Tengah connecting lahh nii");
  }
  Serial.println();
  Serial.println("Connected to WiFi");

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);  // Hosting Firebase

    Firebase.setString("Actuator/Doorlock2","OFF");  //set inital status of Actuator
}

void loop() {
  char key = keypad.getKey();

  if (key) {
    Serial.println(key);

    if (key == '*') {
      input_password = "";  // reset the input password
      Serial.println("Input password reset.");
    } else if (key == '#') {
      if (input_password == correct_password) {
        Serial.println("The password is correct, unlocking the door in 5 seconds");
        digitalWrite(RELAY_PIN, HIGH);  // unlock the door for 5 seconds
        Serial.println("Relay set to HIGH (unlock).");
        delay(5000);
        digitalWrite(RELAY_PIN, LOW);  // lock the door
        Serial.println("Relay set to LOW (lock).");

        // Send data to Firebase
        Firebase.setString("Sensor/Key Passcode", "ABC");

      } else {
        Serial.println("The password is incorrect, try again");

        // Send data to Firebase for incorrect password
        Firebase.setString("Sensor/Key Passcode", "");
        Serial.println("Incorrect password set in Firebase.");

      }

      input_password = "";  // reset the input password
      Serial.println("Input password reset.");
    } else {
      input_password += key;  // append new character to input password string
      Serial.print("Current input password: ");
      Serial.println(input_password);
    }

     // Solenoid Control*
  switchkeylock = Firebase.getString("Actuator/Doorlock2");
  switchkeylock.toUpperCase(); //change rule to uppercase

  if(switchkeylock == "ON" || switchkeylock == "1"){
      digitalWrite(RELAY_PIN,HIGH);
      Serial.print("Doorlock Lower Status:");
      Serial.println(switchkeylock);
    }else if(switchkeylock == "OFF" || switchkeylock == "0"){
      digitalWrite(RELAY_PIN,LOW);
      Serial.print("Doorlock Lower Status:");
      Serial.println(switchkeylock);
    }
  }
}

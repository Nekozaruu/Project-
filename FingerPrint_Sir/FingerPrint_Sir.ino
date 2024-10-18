#include <Arduino.h>
#include <Adafruit_Fingerprint.h>

// Define the hardware serial port to use
#define RXD2 16
#define TXD2 17

HardwareSerial mySerial(2);

Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Fingerprint sensor test");
  // Start the hardware serial port for the fingerprint sensor
  // Set the serial pins for ESP32
  mySerial.begin(57600, SERIAL_8N1, RXD2, TXD2);
  
  // Set the data rate for the sensor serial port
  finger.begin(57600);
    if (finger.status_reg == FINGERPRINT_OK) {
    Serial.println("Found fingerprint sensor!");
  } else {
    Serial.println("Couldn't find fingerprint sensor :(");
    while (1);
  }
  finger.getParameters();
}

void loop() {
  getFingerprintID();
  Serial.println("Place your finger on the sensor...");
  if (finger.fingerID == 1) {
    Serial.println("Welcome, Farhan!"); // Change the message as needed
  }
  if (finger.fingerID == 2) {
    Serial.println("Welcome, Student!"); // Change the message as needed
  }
  delay(3000);
}

uint8_t getFingerprintID() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK) {
    Serial.println("No finger detected");
    return p;
  }

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK) {
    Serial.println("Image conversion failed");
    return p;
  }

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK) {
    Serial.println("Fingerprint not found");
    return p;
  }

  // Fingerprint found!
  Serial.print("Found fingerprint ID #");
  Serial.print(finger.fingerID);
  // Serial.print(" with confidence of ");
  // Serial.println(finger.confidence);
  return finger.fingerID;
}

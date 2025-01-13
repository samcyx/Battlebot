

#define joyX A0
#define joyY A1
#define SW 8
//#include "EdgeImpulse_FOMO_NO_PSRAM.j"
#include "Battlebots_inferencing.h"
#include <eloquent_esp32cam.h>
#include <eloquent_esp32cam/edgeimpulse/fomo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include "html_page.h"
#include <tuple>
// WiFi credentials
const char* ssid = "henesp";
const char* password = "12345678";

WebServer server(80);
Servo ESC; // ESC control instance

// Motor pin definitions

using eloq::camera;
using eloq::ei::fomo;

const int enA = 18;
const int enB = 19;
const int motor1_in1 = 3;  // Motor 1 forward
const int motor1_in2 = 4;  // Motor 1 reverse
const int motor2_in1 = 5;  // Motor 2 forward
const int motor2_in2 = 6;  // Motor 2 reverse
int motorSpeed1, motorSpeed2;
const int IN1 = 12;  // Motor 1 direction pin
const int IN2 = 15;  // Motor 1 direction pin
const int ENA = 13;  // Motor 1 speed

const int IN12 = 27;  // Motor 2 direction pin
const int IN22 = 26;  // Motor 2 direction pin
const int ENA2 = 25;  // Motor 2 speed
//in1 -> in2
// in3 -> in4
//FEEL FREE TO CHANGE PIN VALUES IN ORDER TO MATCH OUR CONFIGURATION WITH CIRCUIT BOARD
//https://lastminuteengineers.com/l293d-dc-motor-arduino-tutorial/


// Joystick values from HTML page
int xValue = 50;
int yValue = 50;
int sliderValue = 0; // Used for controlling the ESC
bool joystickEnabled = true;

void escControl() {
    int pwmVal = map(sliderValue, 0, 100, 0, 60); // Map slider (0-100) to ESC range (0-60)

    // Dead zone to prevent unintended small values affecting ESC
    if (pwmVal > 0 && pwmVal < 10) {
        pwmVal = 0;
    }

    ESC.write(pwmVal); // Write the PWM value to the ESC
}
void motorControl() {
    int motorSpeed = map(abs(yValue)-50, 50, 100, 0, 255); // Map yValue distance from 50 to motor speed

    if (yValue > 50) { // Move forward
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN12, HIGH);
        digitalWrite(IN22, LOW);
    } else if (yValue < 50) { // Move backward
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN12, LOW);
        digitalWrite(IN22, HIGH);
    } else { // Stop
        motorSpeed = 0;
    }
    analogWrite(ENA, motorSpeed); // Set speed based on mapped value
}

void handleRoot() {
    server.send(200, "text/html", htmlPage);
}

std::tuple<int, int> detectX() {
  // Code to run the detection (similar to your original code)
  if (!camera.capture().isOk()) {
    Serial.println(camera.exception.toString());
    return std::make_tuple(0, 0);
  }

  // run FOMO
  if (!fomo.run().isOk()) {
    Serial.println(fomo.exception.toString());
    return std::make_tuple(0, 0);
  }

  // how many objects were found?
  Serial.printf(
    "Found %d object(s) in %dms\n",
    fomo.count(),
    fomo.benchmark.millis());

  // if no object is detected, return
  if (!fomo.foundAnyObject())
    return std::make_tuple(0, 0);

  // if you expect to find a single object, use fomo.first


  int x = (int)fomo.first.x;
  int probability = (int)fomo.first.proba;
  return std::make_tuple(x, probability);
}
void handleUpdate() {
    if (server.hasArg("x") && server.hasArg("y") && server.hasArg("s")) {
        xValue = server.arg("x").toInt();
        yValue = server.arg("y").toInt();
        sliderValue = server.arg("s").toInt();
        
        Serial.printf("X: %d, Y: %d, Slider: %d\n", xValue, yValue, sliderValue);
        server.send(200, "text/plain", "OK");
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}


void control(int xValue, int yValue) {


  // Map joystick values to motor speeds (-255 to 255 for PWM control)
  int speed = map(yValue, 0, 1023, -255, 255);  // Forward/backward control
  int turn = map(xValue, 0, 1023, -255, 255);   // Left/right control

  // Calculate individual motor speeds for differential drive
  motorSpeed1 = speed + turn;  // Left motor speed

  motorSpeed2 = speed - turn;  // Right motor speed

  // Limit motor speed to max PWM values (-255 to 255)
  motorSpeed1 = constrain(motorSpeed1, -255, 255);
  motorSpeed2 = constrain(motorSpeed2, -255, 255);

  // Control left motor
  if (motorSpeed1 > 0) {
    digitalWrite(motor1_in1, HIGH);
    digitalWrite(motor1_in2, LOW);
  } else if (motorSpeed1 < 0) {
    digitalWrite(motor1_in1, LOW);
    digitalWrite(motor1_in2, HIGH);
  } else {
    digitalWrite(motor1_in1, LOW);
    digitalWrite(motor1_in2, LOW);
  }

  // Control right motor
  if (motorSpeed2 > 0) {
    digitalWrite(motor2_in1, HIGH);
    digitalWrite(motor2_in2, LOW);
  } else if (motorSpeed2 < 0) {
    digitalWrite(motor2_in1, LOW);
    digitalWrite(motor2_in2, HIGH);
  } else {
    digitalWrite(motor2_in1, LOW);
    digitalWrite(motor2_in2, LOW);
  }

  // Set motor speeds using PWM
  analogWrite(enA, abs(motorSpeed1));
  analogWrite(enB, abs(motorSpeed2));

  // Optional: Stop motors if the joystick button is pressed
  //if (digitalRead(SW) == LOW) {
  //  stopMotors();
  //}

  delay(10);  // Short delay to allow stable readings
}


void AIcontrol() {
  //Fill this in later based on x bounds for the camera...
  int xMin = 0, xMax = 96;
  int xValue, prob;
  std::tie(xValue, prob) = detectX();
  if (prob > 20)  //if probable detection
  {
    int yValue = 1023;
    xValue = map(xValue, xMin, xMax, -255, 255);
    control(xValue, yValue);
  } else  //if not probable, rotate in a fixed circle
  {
    idle();
  }
}

void idle() {
  digitalWrite(motor1_in1, HIGH);
  digitalWrite(motor1_in2, LOW);
  digitalWrite(motor2_in2, HIGH);
  digitalWrite(motor2_in1, LOW);
  analogWrite(enA, 255);
  analogWrite(enB, 255);
}




void stopMotors() {
  digitalWrite(motor1_in1, LOW);
  digitalWrite(motor1_in2, LOW);
  digitalWrite(motor2_in1, LOW);
  digitalWrite(motor2_in2, LOW);
}

void setup() {
  delay(3000);
  Serial.begin(115200);
  Serial.println("__EDGE IMPULSE FOMO (NO-PSRAM)__");
  Serial.begin(115200);

    // Setup WiFi
    WiFi.softAP(ssid, password);
    Serial.println("Access point started");
    Serial.printf("IP Address: %s\n", WiFi.softAPIP().toString().c_str());

    // Setup server routes
    server.on("/", handleRoot);
    server.on("/u", handleUpdate);
    server.begin();
    Serial.println("Server started");

    // Initialize ESC
    ESC.attach(32); // Connect ESC to pin 13
    ESC.write(0); // Stop ESC initially
    delay(1000); // Allow time for ESC initialization

    // Initialize motor pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
  // camera settings
  // replace with your own model!
  // Configure the camera pins for Freenove ESP32-S3 WROOM
  camera.pinout.freenove_s3();



  camera.brownout.disable();
  // NON-PSRAM FOMO only works on 96x96 (yolo) RGB565 images
  camera.resolution.yolo();
  camera.pixformat.rgb565();
  delay(3000);
  // init camera
  while (!camera.begin().isOk()) {
    Serial.println(camera.exception.toString());
    delay(3000);
  }


  Serial.println("Camera OK");
  Serial.println("Put object in front of camera");
}
void loop() {
  // Read joystick values
  if (!joyStickEnabled) {
    AIcontrol();
  } else {
    escControl();     // Control ESC based on slider input
    motorControl();   // Control motors based on joystick yValue
    server.handleClient(); // Handle web server requests
  }
}

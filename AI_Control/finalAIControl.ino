

#define joyX A0
#define joyY A1
#define SW 8
//#include "EdgeImpulse_FOMO_NO_PSRAM.j"
#include "Battlebots_inferencing.h"
#include <eloquent_esp32cam.h>
#include <eloquent_esp32cam/edgeimpulse/fomo.h>

using eloq::camera;
using eloq::ei::fomo;
#include <tuple>
const int enA = 18;
const int enB = 19;
const int motor1_in1 = 3;  // Motor 1 forward
const int motor1_in2 = 4;  // Motor 1 reverse
const int motor2_in1 = 5;  // Motor 2 forward
const int motor2_in2 = 6;  // Motor 2 reverse
int xValue, yValue;
int motorSpeed1, motorSpeed2;
//in1 -> in2
// in3 -> in4
//FEEL FREE TO CHANGE PIN VALUES IN ORDER TO MATCH OUR CONFIGURATION WITH CIRCUIT BOARD
//https://lastminuteengineers.com/l293d-dc-motor-arduino-tutorial/






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
  bool AI;
  if (AI) {
    AIcontrol();
  } else {
    xValue = analogRead(joyX);  // Left-right
    yValue = analogRead(joyY);  // Forward-backward
    control(xValue, yValue);
  }
}

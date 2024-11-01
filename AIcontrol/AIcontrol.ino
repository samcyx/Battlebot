/*
int in1 = 14;
int in3 15;
int in2 = 16;
int in4 = 17;

*/
#define joyX A0
#define joyY A1
#define SW 8
//#include "EdgeImpulse_FOMO_NO_PSRAM.j"  
#include <BattleBotDetection.h>
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

BattleBotDetection bbd;

void setup() {
  // Set motor pins as outputs
  pinMode(motor1_in1, OUTPUT);
  pinMode(motor1_in2, OUTPUT);
  pinMode(motor2_in1, OUTPUT);
  pinMode(motor2_in2, OUTPUT);
  bbd.begin();
  // Set PWM pins as output
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(SW, INPUT_PULLUP);

  // Initialize serial communication for debugging
  Serial.begin(9600);
/*
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  */
  //https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
}



void control(int xValue, int yValue)
{
   

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


void AIcontrol()
{
  //Fill this in later based on x bounds for the camera...
  int xMin = 0, xMax = 96;
  int xValue, prob;
  std::tie(xValue, prob) = bbd.detectX();
  if(prob > 20)//if probable detection
  {
    int yValue = 1023;
    xValue = map(xValue, xMin, xMax, -255, 255);
    control(xValue, yValue);
  }
  else//if not probable, rotate in a fixed circle
  {
    idle();
  }
}

void idle()
{
  digitalWrite(motor1_in1, HIGH);
  digitalWrite(motor1_in2, LOW);
  digitalWrite(motor2_in2, HIGH);
  digitalWrite(motor2_in1, LOW);
  analogWrite(enA, 255);
  analogWrite(enB, 255);

}

void loop() {
  // Read joystick values
  bool AI;
  if(AI){
    AIcontrol();
  }
  else{
    xValue = analogRead(joyX);  // Left-right
    yValue = analogRead(joyY);  // Forward-backward
    control(xValue, yValue);
  }
  
 
}


void stopMotors() {
  digitalWrite(motor1_in1, LOW);
  digitalWrite(motor1_in2, LOW);
  digitalWrite(motor2_in1, LOW);
  digitalWrite(motor2_in2, LOW);
}
/*
void forward() {
  // Turn on motor A & B
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(2000);
}
void backward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(2000);
}
void right() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}
void left() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}
void right() {
}

void joystickControl(int x, int y) {

  if (y > 512) {
    //forwards
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    x = map(x, 512, 1023, 0, 255);
    y = map(y, 512, 1023, 0, 255);
    analogWrite(enA, x);
    analogWrite(enB, y);
  } else if (y < 510) {
    //backwards
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    x = map(x, 0, 512, 255, 0);
    y = map(y, 0, 512, 255, 0);
    analogWrite(enA, x);
    analogWrite(enB, y);
  }
}
void directionControl() {
  // Set motors to maximum speed
  // For PWM maximum possible values are 0 to 255
  analogWrite(enA, 255);
  analogWrite(enB, 255);

  // Turn on motor A & B
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(2000);

  // Now change motor directions
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(2000);

  // Turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
void speedControl() {
  // Turn on motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Accelerate from zero to maximum speed
  for (int i = 0; i < 256; i++) {
    analogWrite(enA, i);
    analogWrite(enB, i);
    delay(20);
  }

  // Decelerate from maximum speed to zero
  for (int i = 255; i >= 0; --i) {
    analogWrite(enA, i);
    analogWrite(enB, i);
    delay(20);
  }

  // Now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
*/
void joyStickTest(int x, int y)
{
  int motorA, motorB;
  /*
  if (y > 512) {
    //forwards
    Serial.print("Forwards  ");
    y = map(y, 512, 1023, 0, 255);
 

    Serial.print("X: ");
    Serial.print(x);
    Serial.print("\tY: ");
    Serial.println(y);

  } else if (y < 510) {
    //backwards
    Serial.print("Backwards  ");

    y = map(y, 0, 512, 255, 0);
    Serial.print("X: ");
    Serial.print(x);
    Serial.print("\tY: ");
    Serial.println(y);
  }
  */
  if(x < 510)
  {
    //right
    //x = map(x, 0, 510, 255 , 0);
   // motorA = x;
    
  }
  else if(x > 510)
  {
    //left
  //  x = map(x, 510, 1023, 0 , 255);
    
  }
  motorA = x - y;
  motorB = x + y;
  Serial.print("Motor A: ");
  Serial.print(motorA);
  Serial.print("\tMotor B: ");
  Serial.println(motorB);
}

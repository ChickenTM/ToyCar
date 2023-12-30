#include <Arduino.h>
#include <Servo.h>

// Motor control pins for motor A
const int enA = 3;
const int in1 = 6;
const int in2 = 9;

// Motor control pins for motor B
const int enB = 4;
const int in3 = 10;
const int in4 = 20;

// LED pins
const int greenLedPin = 14;
const int redLedPin = 15;

// Servo motor pin
const int servoPin = 21;
Servo servo;

// Ultrasonic sensor pins
const int trigPin = 16;
const int echoPin = 17;

int motorSpeed = 0;
const int maxSpeed = 255;

// Function prototypes
void MoveForward();
void MoveBackward();
void SpeedUp();
void SlowDown();
void TurnLeft();
void TurnRight();
void ControlLED(int pin, bool state);
void BlinkLED(int pin);
void Stop();

void setup() {
  // Set pin modes
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(greenLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Attach servo motor
  servo.attach(servoPin);

  Serial.begin(9600);
}

void loop() {
  // Ultrasonic sensor readings
  long duration;
  int distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  if (distance > 100) {
    SpeedUp();
    MoveForward();
    ControlLED(greenLedPin, HIGH);
    ControlLED(redLedPin, LOW);
  } else if (distance >= 70 && distance <= 100) {
    SpeedUp();
    MoveForward();
    //ControlLED(greenLedPin, HIGH);
    ControlLED(redLedPin, LOW);
    BlinkLED(greenLedPin);
  } else if (distance >= 50 && distance < 70) {
    SpeedUp();
    MoveForward();
    ControlLED(greenLedPin, LOW);
    //ControlLED(redLedPin, HIGH);
    BlinkLED(redLedPin);
  } else if (distance >= 30 && distance < 50) {
    SlowDown();
    MoveForward();
    ControlLED(greenLedPin, LOW);
    //ControlLED(redLedPin, HIGH);
    BlinkLED(redLedPin);
  } else if (distance < 30) {
    ControlLED(greenLedPin, LOW);
    ControlLED(redLedPin, HIGH);
    Stop();
    MoveBackward();
    delay(500);
    Stop();
    
    // Turn the servo to the left
    servo.write(0);
    delay(1000);

    // Get ultrasonic reading on the left
    long leftDuration;
    int leftDistance;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    leftDuration = pulseIn(echoPin, HIGH);
    leftDistance = leftDuration * 0.034 / 2;

    // Turn the servo to the right
    servo.write(180);
    delay(1000);

    // Get ultrasonic reading on the right
    long rightDuration;
    int rightDistance;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    rightDuration = pulseIn(echoPin, HIGH);
    rightDistance = rightDuration * 0.034 / 2;

    // Reset servo position to straight
    servo.write(90);
    delay(1000);

    // Turn the car in the chosen direction
    if (leftDistance > rightDistance) {
      TurnLeft();
      delay(1000);
      Stop();
    } else {
      TurnRight();
      delay(1000);
      Stop();
    }

    delay(1000);
    MoveForward(); // Move forward after turning
  }
}

// Function to move both motors forward
void MoveForward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
}

// Function to move both motors backward
void MoveBackward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
}

// Function to gradually increase motor speed
void SpeedUp() {
  while (motorSpeed < maxSpeed) {
    motorSpeed++;
    analogWrite(enA, motorSpeed);
    analogWrite(enB, motorSpeed);
    delay(10);
  }
}

// Function to gradually decrease motor speed
void SlowDown() {
  while (motorSpeed > maxSpeed * 0.4) {
    motorSpeed--;
    analogWrite(enA, motorSpeed);
    analogWrite(enB, motorSpeed);
    delay(10);
  }
}

// Function to turn the car left
void TurnLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
}

// Function to turn the car right
void TurnRight() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
}

// Function to stop both motors
void Stop() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

// Function to control the state of an LED
void ControlLED(int pin, bool state) {
  digitalWrite(pin, state);
}

// Function to blink an LED
void BlinkLED(int pin) {
  digitalWrite(pin, HIGH);
  delay(500);
  digitalWrite(pin, LOW);
  delay(500);
}

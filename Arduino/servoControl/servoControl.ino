#include <Servo.h>

Servo myServo;  // create servo object

// Change this if you want a different pin
const int servoPin = 2;

// Range of motion (0–180° is standard for Servo library)
int angle = 0;

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);  // attach servo to pin 2
  Serial.println("DSS-P05 Servo Test on Teensy 4.1");
}

void loop() {
  // Sweep from 0 to 180 degrees
  for (angle = 0; angle <= 180; angle += 5) {
    myServo.write(angle);
    Serial.print("Angle: ");
    Serial.println(angle);
    delay(300);  // wait for servo to reach position
  }

  // Sweep back
  for (angle = 180; angle >= 0; angle -= 5) {
    myServo.write(angle);
    Serial.print("Angle: ");
    Serial.println(angle);
    delay(300);
  }
}
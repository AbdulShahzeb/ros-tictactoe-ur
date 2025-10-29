#include <Servo.h>

Servo myServo;
const int servoPin = 2;
int angle = 0;

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);
  Serial.println("DSS-P05 Servo Test on Teensy 4.1");
}

void loop() {
  for (angle = 0; angle <= 180; angle += 5) {
    myServo.write(angle);
    Serial.print("Angle: ");
    Serial.println(angle);
    delay(300);
  }

  for (angle = 180; angle >= 0; angle -= 5) {
    myServo.write(angle);
    Serial.print("Angle: ");
    Serial.println(angle);
    delay(300);
  }
}
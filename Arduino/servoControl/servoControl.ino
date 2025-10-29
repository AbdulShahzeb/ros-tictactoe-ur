#include <Servo.h>

Servo myServo;

const int servoPin = 2;
int currentAngle = 90;  // Start at middle position

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);
  myServo.write(currentAngle);
  
  Serial.println("DSS-P05 Servo Control Ready");
  Serial.println("Send angle (0-180) via serial");
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming int
    int targetAngle = Serial.parseInt();

    // Validate the angle
    if (targetAngle >= 0 && targetAngle <= 180) {
      myServo.write(targetAngle);
      currentAngle = targetAngle;
      Serial.print("OK:");
      Serial.println(currentAngle);
    } else {
      Serial.print("ERROR: Angle out of range (0-180): ");
      Serial.println(targetAngle);
    }

    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}
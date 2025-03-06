#include <Servo.h>

Servo myservo, myservo2;  // Create servo objects

void moveServo(Servo &servo, int start, int end, int delayTime) {
  int step = (start < end) ? 1 : -1;  // Determine step direction
  for (int pos = start; pos != end + step; pos += step) {
    servo.write(pos);
    delay(delayTime);
  }
}

void setup() {
  myservo.attach(17);   // Attach first servo to pin 9
  myservo2.attach(18); // Attach second servo to pin 10
}

void loop() {
  moveServo(myservo, 0, 90, 15);   // Move first servo from 0 to 90 degrees
  moveServo(myservo, 90, 0, 15);   // Move first servo from 90 to 0 degrees

  moveServo(myservo2, 0, 90, 15);  // Move second servo from 0 to 90 degrees
  moveServo(myservo2, 90, 0, 15);  // Move second servo from 90 to 0 degrees
}

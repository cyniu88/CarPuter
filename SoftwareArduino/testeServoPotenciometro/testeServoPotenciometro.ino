
 /* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0, throttle = 0;    // variable to store the servo position
int vetor[180];
void setup() {
  Serial.begin(38400);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  for (pos = 40; pos < 150; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(50);                       // waits 15ms for the servo to reach the position
    throttle = analogRead(A0);
    throttle = map(throttle, 0, 1023, 0, 179);
    vetor[pos] = throttle;
  }
  for (int i=40; i<150; i++) {
    Serial.print(i);
    Serial.print("\t");
    Serial.print(vetor[i]);
    Serial.println();
  }
  for (pos = 150; pos > 40; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(50);                       // waits 15ms for the servo to reach the position
    throttle = analogRead(A0);
    throttle = map(throttle, 0, 1023, 0, 179);
    vetor[pos] = throttle;
  }
    for (int i=150; i>40; i--) {
    Serial.print(i);
    Serial.print("\t");
    Serial.print(vetor[i]);
    Serial.println();
  }
}

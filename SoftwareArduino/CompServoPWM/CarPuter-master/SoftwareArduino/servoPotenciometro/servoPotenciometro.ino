
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
int vetor1[180], vetor2[180];

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  for (pos = 40; pos < 150; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(50);                       // waits 15ms for the servo to reach the position
    throttle = analogRead(A0);
    throttle = map(throttle, 0, 1023, 0, 179);
    vetor1[pos] = throttle;
  }
  
  for (pos = 150; pos > 40; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(50);                       // waits 15ms for the servo to reach the position
    throttle = analogRead(A0);
    throttle = map(throttle, 0, 1023, 0, 179);
    vetor2[pos] = throttle;
  }
  
  delay(2000);
  Serial.begin(38400);
  for (int i=0; i<180; i++) {
    Serial.print(i);
    Serial.print("\t");
    Serial.print(vetor1[i]);
    Serial.println();
  }  
  for (int i=180; i>0; i--) {
    Serial.print(i);
    Serial.print("\t");
    Serial.print(vetor2[i]);
    Serial.println();
  }
  
}

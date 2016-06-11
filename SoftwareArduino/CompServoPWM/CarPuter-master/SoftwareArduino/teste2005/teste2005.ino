#include <Servo.h>

Servo myServo;  
int targetAcel = 30;

void setup() {
  Serial.begin(38400);
  myServo.attach(9);
}

void loop() {  
  if (Serial.available() > 0) {
    delay(10);
    targetAcel = (Serial.read() - '0') * 100 + (Serial.read() - '0') * 10 + (Serial.read() - '0');
    
    if (targetAcel < 0) 
        targetAcel = 0;        
    if (targetAcel > 250) 
        targetAcel = 250;      
  }
  
  myServo.write(targetAcel);
  delay(300);
}

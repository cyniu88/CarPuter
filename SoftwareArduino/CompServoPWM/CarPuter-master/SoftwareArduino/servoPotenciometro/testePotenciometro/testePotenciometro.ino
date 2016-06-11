// Pot=175  ==>  Servo=147
// Pot=060  ==>  Servo=20

  #include <Servo.h> 
  #include <SoftwareSerial.h>
  #define SET 1

  SoftwareSerial mySerial(7, 8); // RX, TX
  Servo myServo;  // create servo object to control a servo 
  
  String check = "";
  char c;

  float ontem = millis();
  float lastError = 0;
  float kc = 0.1, ki = 0.05, kd = 0;
  unsigned int throttle = 0;
  int tolerancia = 2;
  int toleranciaki = 20;
  int soma = 0;
  int targetAcel = 100;
  float servo = 100;
  int potpin = 0;
  
  void setup() {
      Serial.begin(38400);    
      mySerial.begin(38400);
      mySerial.flush(); 
      Serial.flush();
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
  
      
      //t.update();
      //getThrottle();  

      throttle = analogRead(potpin);
      // Converte o valor pra ser usado no servo (valores entre 0 e 180) 
      throttle = map(throttle, 0, 1023, 0, 179);     

      servo = servo + (int)PID();
      
      if ((throttle > (targetAcel+toleranciaki)) || (throttle < (targetAcel-toleranciaki))) {
          soma = 0;
      }
  
      if (soma > 10) 
          soma = 10;
      if (soma < -10) 
          soma = -10;
                     
/*
      //if (throttle != -1) {
      //    LoopDelay = 250;
      if (throttle > (targetAcel+tolerancia))
          if (servo > 0) servo=servo-SET;
      if (throttle < (targetAcel-tolerancia))
          if (servo < 180) servo=servo+SET;
      //}
      //else LoopDelay = 300;
*/
      // Move o eixo do servo, de acordo com o angulo
      myServo.write((int)servo);                  
      // Aguarda o servo atingir a posição
      delay(50);  
  
// Debugging...  

      Serial.print("   targetAcel: ");
      Serial.print(targetAcel);
      Serial.print("   throttle: ");
      Serial.print(throttle);
      Serial.print("   Servo: ");
      Serial.print(servo);
      //Serial.print("   Delay: ");
      //Serial.print(LoopDelay);
      Serial.println();
      
      //delay(LoopDelay);
  }


  float PID() {
      float integral, proportional, derivative, dt;
      int error;
      
      dt = (millis() - ontem)/1000;
      ontem  = millis();
      
      error = targetAcel-throttle;
      
      soma = soma + (error * dt);
      
      proportional = error * kc;  
      integral = ki * soma;
      derivative = kd * ((error - lastError)/dt);
      
      lastError = error;
      
      Serial.print("   erro: ");
      Serial.print(error);
      Serial.print("   soma: ");
      Serial.print(soma);
      Serial.print("   propor: ");
      Serial.print(proportional);
      
      
      Serial.println();
      
      return proportional; //+ integral + derivative;
  }
 
  void resposta() {
    while (mySerial.available() > 0)  
        {
            c = mySerial.read();
            check += c;
        }
    Serial.print(check);
    Serial.write("\n"); 
    check.remove(0);    
 }
